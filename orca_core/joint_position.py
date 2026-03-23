from dataclasses import dataclass
from typing import ClassVar, Sequence

import numpy as np


@dataclass(frozen=True)
class OrcaJointPosition:
    """Immutable, typed container for ORCA hand joint positions.

    Stores a mapping from joint name to joint position value. The class
    is frozen so instances can be safely shared across threads without copying.

    A set of default joint names can be registered once per process via
    :meth:`register_joint_names`; subsequent construction helpers use this
    ordering when none is supplied explicitly.

    Attributes:
        data: Mapping from joint name to position.

    Example:
        >>> # Create a new joint position object from a dictionary of joint positions
        >>> pos = OrcaJointPosition({"index_mcp": 0.5, "thumb_pip": 1.2})  # set keys arbitrarily
        >>> pos.as_list()
        [0.5, 1.2]
        >>> # You can also pass in an array of joint positions using specific joint names
        >>> pos = OrcaJointPosition.from_ndarray([0.5, 1.2], joint_ids=["thumb_pip", "index_mcp"])
        >>> pos.as_dict()
        {'thumb_pip': 0.5, 'index_mcp': 1.2}
        >>> # Register default joint ordering for your model
        >>> OrcaJointPosition.register_joint_names(["index_mcp", "thumb_pip"])
        >>> # You can now pass in an array of joint positions without specifying the joint ids
        >>> pos = OrcaJointPosition.from_ndarray([0.5, 1.2])
    """
    data: dict[str, float]

    _default_joint_ids: ClassVar[tuple[str, ...] | None] = None


    def __post_init__(self) -> None:
        if self._default_joint_ids is not None:
            unknown = set(self.data) - set(self._default_joint_ids)
            if unknown:
                raise ValueError(f"Unknown joints for this hand version: {unknown}")

    @classmethod
    def register_joint_names(cls, joint_ids: Sequence[str]) -> None:
        """Register the canonical joint ordering for the active hand model.

        Args:
            joint_ids: Ordered sequence of joint name strings, e.g.
                ``["index_mcp", "index_pip", "thumb_mcp", ...]``.
        """
        cls._default_joint_ids = tuple(joint_ids)

    @classmethod
    def from_dict(cls, joint_pos: dict[str, float | None]) -> "OrcaJointPosition":
        """Construct from a mapping, silently dropping ``None`` values.

        Args:
            joint_pos: Mapping from joint name to position. Entries whose
                value is ``None`` are excluded from the resulting object.

        Returns:
            A new :class:`OrcaJointPosition` containing only the non-``None``
            entries of *joint_pos*.
        """
        return cls({k: v for k, v in joint_pos.items() if v is not None})

    @classmethod
    def from_ndarray(
        cls,
        joint_pos: np.ndarray,
        joint_ids: Sequence[str] | None = None,
    ) -> "OrcaJointPosition":
        """Construct from a 1-D NumPy array.

        Falls back to the registered default joint ordering when *joint_ids* is
        omitted. ``NaN`` values are treated as unset and excluded from the
        result, mirroring the behaviour of :meth:`from_dict` with ``None``.

        Args:
            joint_pos: 1-D array of joint positions in radians. Its length
                must match ``len(joint_ids)``.
            joint_ids: Ordered joint names corresponding to *joint_pos*. When
                ``None`` the class-level default registered via
                :meth:`register_joint_names` is used.

        Returns:
            A new :class:`OrcaJointPosition` with ``NaN`` entries omitted.

        Raises:
            ValueError: If no joint ordering is available (neither supplied nor
                registered), if *joint_pos* is not 1-D, or if its length does
                not match *joint_ids*.
        """
        # TODO(fracapuano): When given a batch of joint positions, return a batch of OrcaJointPosition objects.
        resolved_ids = joint_ids if joint_ids is not None else cls._default_joint_ids
        if resolved_ids is None:
            raise ValueError(
                "joint_ids are currently unset. Provide them alongside the input data, or call OrcaJointPosition.register_joint_names(joint_ids) first."
            )
        arr = np.asarray(joint_pos, dtype=np.float64)
        if arr.ndim != 1:
            raise ValueError("joint_pos must be a 1-D array.")
        if arr.shape[0] != len(resolved_ids):
            raise ValueError("Length of joint_pos must match the number of joint_ids.")

        return cls({joint: float(val) for joint, val in zip(resolved_ids, arr) if not np.isnan(val)})

    def __iter__(self):
        return iter(self.data.items())

    def as_dict(self) -> dict[str, float]:
        """Return a plain :class:`dict` copy of the joint positions.

        Returns:
            Dictionary mapping joint name to position in radians.
        """
        return dict(self.data)

    def as_array(self, joint_ids: Sequence[str]) -> np.ndarray:
        """Return positions as a NumPy array ordered by *joint_ids*.

        Missing joints (not present in :attr:`data`) are filled with
        ``np.nan`` so the caller can detect unset joints.

        Args:
            joint_ids: The desired joint ordering for the output array.

        Returns:
            Float64 array of length ``len(joint_ids)``.
        """
        return np.array([self.data.get(joint, np.nan) for joint in joint_ids], dtype=np.float64)

    def as_list(self, joint_ids: Sequence[str]) -> list[float | None]:
        """Return positions as a list ordered by *joint_ids*.

        Missing joints are represented as ``None``.

        Args:
            joint_ids: The desired joint ordering for the output list.

        Returns:
            List of length ``len(joint_ids)`` with ``None`` for missing joints.
        """
        return [self.data.get(joint) for joint in joint_ids]