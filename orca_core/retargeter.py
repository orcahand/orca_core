# dependencies for retargeter
from orca_core import OrcaHand
import time
import torch
import numpy as np
from torch.nn.functional import normalize
import os
import pytorch_kinematics as pk
from .utils import retarget_utils
from typing import Union
import yaml
from scipy.spatial.transform import Rotation
from .utils.yaml_utils import *
from .utils.load_utils import get_model_path
from typing import List, Dict


class Retargeter:
    """
    Retargeter class for the ORCA Hand
    """

    def __init__(
        self,
        model_path: str = None,
        include_wrist_and_tower: bool = False,

    ) -> None:
        
        self.model_path = get_model_path(model_path)
        
        self.config_path = os.path.join(self.model_path, "config.yaml")
        self.urdf_path = os.path.join(self.model_path, "urdf", "orcahand.urdf")
        self.mjco_path = os.path.join(self.model_path, "mujoco", "orcahand.xml")
        
        config = read_yaml(self.config_path)
        self.joint_ids: List[str] = config.get('joint_ids', [])
        self.joint_roms: Dict[str, List[float]] = config.get('joint_roms', {})
        
        assert (
            int(self.urdf_path is not None)
            + int(self.mjco_path is not None)
        ) > 1, "One or more of urdf_path or mjco_path should be provided"

        self.device: str = "cuda" if torch.cuda.is_available() else "cpu"
        
        self.retargeter_cfg_path = os.path.join(self.model_path, "retargeter.yaml")
        print(self.retargeter_cfg_path)
        self.retargeter_cfg = read_yaml(self.retargeter_cfg_path)
        if self.retargeter_cfg is None:
            raise ValueError(f"retargeter.yaml not found at {self.retargeter_cfg_path}")
        print(self.retargeter_cfg)
        self.lr = self.retargeter_cfg["lr"]
        self.use_scalar_distance_palm = self.retargeter_cfg["use_scalar_distance_palm"]
        self.loss_coeffs = torch.tensor(self.retargeter_cfg["loss_coeffs"]).to(self.device)            
        self.joint_regularizers = self.retargeter_cfg["joint_regularizers"]
        self.target_angles = None
        self.mano_adjustments = self.retargeter_cfg["mano_adjustments"]

        self.hand_scheme_path = os.path.join(self.model_path, "hand_scheme.yaml")
        self.hand_scheme = read_yaml(self.hand_scheme_path)
        if self.hand_scheme is None:
            raise ValueError(f"hand_scheme.yaml not found at {self.hand_scheme_path}") 
        
        self.include_wrist_and_tower = include_wrist_and_tower
        tendons_to_joints = self.hand_scheme["gc_tendons_to_joint_ids"]
        joints_to_tendons = {v: k for k, v in tendons_to_joints.items()}
        self.wrist_name = self.hand_scheme["wrist_name"]
        
        self.gc_tendons =  self.hand_scheme["gc_tendons"]
        self.n_tendons = len(self.gc_tendons)
        self.finger_to_tip = self.hand_scheme["finger_to_tip"]
        self.finger_to_base = self.hand_scheme["finger_to_base"]
        self.gc_limits_lower = []
        self.gc_limits_upper = []
        for tendon in self.gc_tendons:
            assert tendon in tendons_to_joints, f"{tendon} not found in tendons_to_joints"
            joint = tendons_to_joints[tendon]
            self.gc_limits_lower.append(hand.joint_roms[joint][0])
            self.gc_limits_upper.append(hand.joint_roms[joint][1])
            
        print(f"GC_LIMITS_LOWER: {self.gc_limits_lower}")
        print(f"GC_LIMITS_UPPER: {self.gc_limits_upper}")

        prev_cwd = os.getcwd()
        os.chdir(self.model_path)
        if self.urdf_path is not None:
            self.chain = pk.build_chain_from_urdf(open(hand.urdf_path).read()).to(
                device=self.device
            )
        elif self.mjco_path is not None:
            self.chain = pk.build_chain_from_mjcf(open(hand.mjco_path).read()).to(
                device=self.device
            )
        os.chdir(prev_cwd)

        self.gc_joints = torch.ones(self.n_tendons).to(self.device) * 16.0
        self.gc_joints.requires_grad_()
        
        self.tendon_names = list(self.gc_tendons.keys())

        self.regularizer_zeros = torch.zeros(self.n_tendons).to(self.device)
        self.regularizer_weights = torch.zeros(self.n_tendons).to(self.device)
        for joint_name, zero_value, weight in self.joint_regularizers:
            self.regularizer_zeros[self.tendon_names.index(joints_to_tendons[joint_name])] = zero_value
            self.regularizer_weights[self.tendon_names.index(joints_to_tendons[joint_name])] = weight

        # self.opt = torch.optim.Adam([self.gc_joints], lr=self.lr)
        self.opt = torch.optim.RMSprop([self.gc_joints], lr=self.lr)

        self.root = torch.zeros(1, 3).to(self.device)
        self.frames_we_care_about = None

        if self.use_scalar_distance_palm:
            self.use_scalar_distance = [False, True, True, True, True]
        else:
            self.use_scalar_distance = [False, False, False, False, False]

        self._sanity_check()
        
        _chain_transforms = self.chain.forward_kinematics(
            torch.zeros(self.chain.n_joints, device=self.chain.device)
        )
        self.model_center, self.model_rotation = (
            retarget_utils.get_hand_center_and_rotation(
                thumb_base=_chain_transforms[self.finger_to_base["thumb"]]
                .transform_points(self.root)
                .cpu()
                .numpy(),
                index_base=_chain_transforms[self.finger_to_base["index"]]
                .transform_points(self.root)
                .cpu()
                .numpy(),
                middle_base=_chain_transforms[self.finger_to_base["middle"]]
                .transform_points(self.root)
                .cpu()
                .numpy(),
                ring_base=_chain_transforms[self.finger_to_base["ring"]]
                .transform_points(self.root)
                .cpu()
                .numpy(),
                pinky_base=_chain_transforms[self.finger_to_base["pinky"]]
                .transform_points(self.root)
                .cpu()
                .numpy(),
                wrist=_chain_transforms[self.wrist_name]
                .transform_points(self.root)
                .cpu()
                .numpy(),
            )
        )
        
        print(f"Model center: {self.model_center}")
        print(f"Model rotation: {self.model_rotation}")

        assert np.allclose(
            (self.model_rotation @ self.model_rotation.T), (np.eye(3)), atol=1e-6
        ), "Model rotation matrix is not orthogonal"

    def _sanity_check(self):
        """
        Check if the chain and scheme configuration is correct
        """

        ## Check the tip and base frames exist
        for finger, tip in self.finger_to_tip.items():
            assert (
                tip in self.chain.get_link_names()
            ), f"Tip frame {tip} not found in the chain"
        for finger, base in self.finger_to_base.items():
            assert (
                base in self.chain.get_link_names()
            ), f"Base frame {base} not found in the chain"

        ## Check the base frame is fixed to the palm
        chain_transform1 = self.chain.forward_kinematics(
            torch.randn(self.chain.n_joints, device=self.chain.device)
        )
        chain_transform2 = self.chain.forward_kinematics(
            torch.randn(self.chain.n_joints, device=self.chain.device)
        )
        chain_transform3 = self.chain.forward_kinematics(
            torch.randn(self.chain.n_joints, device=self.chain.device)
        )
        for finger, base in self.finger_to_base.items():
            pass

    def retarget_finger_mano_joints(
        self,
        joints: np.array,
        warm: bool = True,
        opt_steps: int = 2,
        dynamic_keyvector_scaling: bool = False,
    ):
        """
        Process the MANO joints and update the finger joint angles
        joints: (21, 3)
        Over the 21 dims:
        0-4: thumb (from hand base)
        5-8: index
        9-12: middle
        13-16: ring
        17-20: pinky
        """

        # print(f"Retargeting: Warm: {warm} Opt steps: {opt_steps}")
        if self.frames_we_care_about is None:
            frames_names = []
            frames_names.append(self.finger_to_base["thumb"])
            frames_names.append(self.finger_to_base["pinky"])
            for finger, finger_tip in self.finger_to_tip.items():
                frames_names.append(finger_tip)
            self.frames_we_care_about = self.chain.get_frame_indices(*frames_names)

        start_time = time.time()
        if not warm:
            self.gc_joints = torch.ones(self.n_joints).to(self.device) * 16.0
            self.gc_joints.requires_grad_()

        assert joints.shape == (
            22,
            3,
        ), "The shape of the mano joints array should be (21, 3)"

        joints = torch.from_numpy(joints).to(self.device)

        mano_joints_dict = retarget_utils.get_mano_joints_dict(joints)

        mano_fingertips = {}
        for finger, finger_joints in list(mano_joints_dict.items())[1:]:
            mano_fingertips[finger] = finger_joints[[-1], :]

        mano_pps = {}
        for finger, finger_joints in list(mano_joints_dict.items())[1:]:
            mano_pps[finger] = finger_joints[[0], :]

        mano_palm = torch.mean(
            torch.cat([mano_pps["thumb"], mano_pps["pinky"]], dim=0).to(self.device),
            dim=0,
            keepdim=True,
        )

        keyvectors_mano = retarget_utils.get_keyvectors(mano_fingertips, mano_palm)
        # norms_mano = {k: torch.norm(v) for k, v in keyvectors_mano.items()}
        # print(f"keyvectors_mano: {norms_mano}")

        for step in range(opt_steps):
            chain_transforms = self.chain.forward_kinematics(
                (self.gc_joints / (180 / np.pi)),
                frame_indices=self.frames_we_care_about
            )
            fingertips = {}
            for finger, finger_tip in self.finger_to_tip.items():
                fingertips[finger] = chain_transforms[finger_tip].transform_points(
                    self.root
                )

            palm = (
                chain_transforms[self.finger_to_base["thumb"]].transform_points(
                    self.root
                )
                + chain_transforms[self.finger_to_base["pinky"]].transform_points(
                    self.root
                )
            ) / 2

            keyvectors_faive = retarget_utils.get_keyvectors(fingertips, palm)
            # norms_faive = {k: torch.norm(v) for k, v in keyvectors_faive.items()}
            # print(f"keyvectors_faive: {norms_faive}")

            loss = 0

            for i, (keyvector_faive, keyvector_mano) in enumerate(
                zip(keyvectors_faive.values(), keyvectors_mano.values())
            ):
                if not self.use_scalar_distance[i]:
                    loss += (
                        self.loss_coeffs[i]
                        * torch.norm(keyvector_mano - keyvector_faive) ** 2
                    )
                else:
                    loss += (
                        self.loss_coeffs[i]
                        * (torch.norm(keyvector_mano) - torch.norm(keyvector_faive))
                        ** 2
                    )
            
            # Regularize the joints to zero
            loss += torch.sum(
                self.regularizer_weights * (self.gc_joints - self.regularizer_zeros) ** 2
            )

            # print(f"step: {step} Loss: {loss}")
            self.scaling_factors_set = True
            self.opt.zero_grad()
            loss.backward()
            self.opt.step()
            with torch.no_grad():
                self.gc_joints[:] = torch.clamp(
                    self.gc_joints,
                    torch.tensor(self.gc_limits_lower).to(self.device),
                    torch.tensor(self.gc_limits_upper).to(self.device),
                )

        finger_joint_angles = self.gc_joints.detach().cpu().numpy()


        if self.include_wrist_and_tower == True:
            wrist_angle = retarget_utils.get_wrist_angle(joints)
            finger_joint_angles = np.insert(finger_joint_angles, 0, wrist_angle)
        else:
            wrist_angle = 0
        

        # print(f"Retarget time: {(time.time() - start_time) * 1000} ms")

        return finger_joint_angles, wrist_angle


    def adjust_mano_fingers(self, joints):


        # Assuming mano_adjustments is accessible within the class
        mano_adjustments = self.mano_adjustments

        # Get the joints per finger
        joints_dict = retarget_utils.get_mano_joints_dict(
            joints, include_wrist=True, batch_processing=False
        )

        # Initialize adjusted joints dictionary
        adjusted_joints_dict = {}

        # Process each finger
        for finger in ["thumb", "index", "middle", "ring", "pinky"]:
            # Original joints for the finger
            finger_joints = joints_dict[finger]  # Shape: (n_joints, 3)

            if  mano_adjustments.get(finger) is None:
                adjusted_joints_dict[finger] = finger_joints
                continue
            # Adjustments for the finger
            adjustments = mano_adjustments[finger]
            translation = adjustments.get("translation", np.zeros(3))  # (3,)
            rotation_angles = adjustments.get("rotation", np.zeros(3))  # (3,)
            scale = adjustments.get("scale", np.ones(3))  # (3,)

            # Scaling in the finger base frame
            x_base = finger_joints[0]  # Base joint position (3,)
            x_local = finger_joints - x_base  # Local coordinates (n_joints, 3)
            x_local_scaled = x_local * scale  # Apply scaling

            # Rotation around base joint in palm frame
            rot = Rotation.from_euler("xyz", rotation_angles, degrees=False)
            R_matrix = rot.as_matrix()  # Rotation matrix (3,3)
            x_local_rotated = x_local_scaled @ R_matrix.T  # Apply rotation
            finger_joints_rotated = x_base + x_local_rotated  # Rotated positions

            # Translation in palm frame
            finger_joints_adjusted = finger_joints_rotated + translation  # Adjusted positions

            # Store adjusted joints
            adjusted_joints_dict[finger] = finger_joints_adjusted

        # Keep the wrist as is
        adjusted_joints_dict["wrist"] = joints_dict["wrist"]
        adjusted_joints_dict["forearm"] = joints_dict["forearm"]

        # Concatenate adjusted joints
        joints = np.concatenate(
            [
                adjusted_joints_dict["forearm"].reshape(1, -1),
                adjusted_joints_dict["wrist"].reshape(1, -1),
                adjusted_joints_dict["thumb"],
                adjusted_joints_dict["index"],
                adjusted_joints_dict["middle"],
                adjusted_joints_dict["ring"],
                adjusted_joints_dict["pinky"],
            ],
            axis=0,
        )

        return joints


    def retarget(self, joints, debug_dict=None):
        normalized_joint_pos, mano_center_and_rot = (
            retarget_utils.normalize_points_to_hands_local(joints)
        )
        
        # TODO: Make the thumb rotate even more!
        normalized_joint_pos = (
            retarget_utils.correct_rokoko_offset(normalized_joint_pos, 
                                                 offset_angle=10, scaling_factor=2)
        )
        # rotate joints about z xis 15 degrees
        normalized_joint_pos = self.adjust_mano_fingers(normalized_joint_pos)
        # (model_joint_pos - model_center) @ model_rotation = normalized_joint_pos
        debug_dict["mano_center_and_rot"] = mano_center_and_rot
        debug_dict["model_center_and_rot"] = (self.model_center, self.model_rotation)
        normalized_joint_pos = (
            normalized_joint_pos @ self.model_rotation.T + self.model_center
        )
            
        self.target_angles, wrist_angle = self.retarget_finger_mano_joints(normalized_joint_pos)

        normalized_joint_pos =retarget_utils.rotate_points_around_y(normalized_joint_pos, wrist_angle)
        if debug_dict is not None:
            debug_dict["normalized_joint_pos"] = normalized_joint_pos

        return self.target_angles, debug_dict
    
    def get_hand_center_and_rotation(
        thumb_base, index_base, middle_base, ring_base, pinky_base, wrist=None
    ):
        """
        Get the center of the hand and the rotation matrix of the hand
        x axis is the direction from ring to index finger base
        y axis is the direction from wrist to middle finger base
        z axis is the dot product.
        """
        hand_center = (thumb_base + pinky_base) / 2
        if wrist is None:
            wrist = hand_center

        y_axis = middle_base - wrist
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = index_base - ring_base
        x_axis -= (x_axis @ y_axis.T) * y_axis
        x_axis = x_axis / np.linalg.norm(x_axis)
        z_axis = np.cross(x_axis, y_axis)
        z_axis = z_axis / np.linalg.norm(z_axis)
        rot_matrix = np.concatenate(
            (x_axis.reshape(1, 3), y_axis.reshape(1, 3), z_axis.reshape(1, 3)), axis=0
        ).T
        return hand_center, rot_matrix
