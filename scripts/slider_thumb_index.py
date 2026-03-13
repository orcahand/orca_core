import tkinter as tk
from tkinter import ttk
from orca_core import OrcaHand
import argparse

THUMB_INDEX_JOINTS = [
    "thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip",
    "index_abd", "index_mcp", "index_pip",
]

KEY_BINDINGS = ["q", "w", "e", "r", "t", "y", "u"]
KEY_STEP = 2.0


class ThumbIndexControlUI:
    def __init__(self, root, hand):
        self.hand = hand
        self.joint_roms = hand.joint_roms_dict
        self.all_joint_ids = hand.joint_ids
        self.joints = [j for j in THUMB_INDEX_JOINTS if j in self.all_joint_ids]
        self.joint_values = {joint: tk.DoubleVar() for joint in self.joints}
        self.sliders = {}

        current = self.hand.get_joint_pos(as_list=False)
        for joint in self.joints:
            if joint in current:
                self.joint_values[joint].set(current[joint])

        self.create_ui(root)
        self.bind_keys(root)

    def create_ui(self, root):
        root.title("Thumb & Index Control")
        root.geometry("500x400")

        torque_frame = ttk.Frame(root)
        torque_frame.pack(pady=10)

        ttk.Button(torque_frame, text="Enable Torque", command=self.enable_torque).pack(side=tk.LEFT, padx=5)
        ttk.Button(torque_frame, text="Disable Torque", command=self.disable_torque).pack(side=tk.LEFT, padx=5)

        sliders_frame = ttk.Frame(root)
        sliders_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        for i, joint in enumerate(self.joints):
            rom_min, rom_max = self.joint_roms[joint]
            frame = ttk.Frame(sliders_frame)
            frame.pack(fill=tk.X, pady=5, padx=10)

            key = KEY_BINDINGS[i] if i < len(KEY_BINDINGS) else ""
            key_label = f"[{key}]" if key else "   "
            label = ttk.Label(frame, text=f"{key_label} {joint}", width=20)
            label.pack(side=tk.LEFT)

            slider = ttk.Scale(
                frame,
                from_=rom_min,
                to=rom_max,
                orient=tk.HORIZONTAL,
                variable=self.joint_values[joint],
                command=lambda value, j=joint: self.update_joint(j, value),
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self.sliders[joint] = slider

            value_label = ttk.Label(frame, text=f"{self.joint_values[joint].get():.1f}", width=8)
            value_label.pack(side=tk.RIGHT)

            self.joint_values[joint].trace_add(
                "write", lambda *args, j=joint, lbl=value_label: self.update_label(j, lbl)
            )

        hint = ttk.Label(root, text=f"Keys [{KEY_BINDINGS[0]}]-[{KEY_BINDINGS[-1]}]: hold to increase, Shift+key to decrease")
        hint.pack(pady=5)

    def bind_keys(self, root):
        for i, joint in enumerate(self.joints):
            if i >= len(KEY_BINDINGS):
                break
            key = KEY_BINDINGS[i]
            root.bind(f"<KeyPress-{key}>", lambda e, j=joint: self.key_adjust(j, KEY_STEP))
            root.bind(f"<KeyPress-{key.upper()}>", lambda e, j=joint: self.key_adjust(j, -KEY_STEP))

    def key_adjust(self, joint, delta):
        rom_min, rom_max = self.joint_roms[joint]
        current = self.joint_values[joint].get()
        new_val = max(rom_min, min(rom_max, current + delta))
        self.joint_values[joint].set(new_val)
        self.update_joint(joint, new_val)

    def enable_torque(self):
        self.hand.enable_torque()
        print("Torque enabled.")
        current = self.hand.get_joint_pos(as_list=False)
        for joint in self.joints:
            if joint in current:
                self.joint_values[joint].set(current[joint])

    def disable_torque(self):
        self.hand.disable_torque()
        print("Torque disabled.")

    def update_joint(self, joint, value):
        try:
            self.hand.set_joint_pos({joint: float(value)})
        except Exception as e:
            print(f"Error updating {joint}: {e}")

    def update_label(self, joint, label):
        label.config(text=f"{self.joint_values[joint].get():.1f}")


def main():
    parser = argparse.ArgumentParser(description="Control thumb and index finger with sliders and keyboard.")
    parser.add_argument("model_path", type=str, nargs="?", default=None,
                        help="Path to the hand model directory")
    args = parser.parse_args()

    hand = OrcaHand(args.model_path)
    status = hand.connect()
    print(status)

    if not status[0]:
        print("Failed to connect to the hand.")
        return

    root = tk.Tk()
    ThumbIndexControlUI(root, hand)
    root.mainloop()


if __name__ == "__main__":
    main()
