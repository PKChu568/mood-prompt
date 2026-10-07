"""Stewart-platform geometry, loaded from Pollen Robotics' kinematics data.

The constants come from ``robot/reachy_mini_description/kinematics/
kinematics_data.json`` (vendored from Pollen's Reachy Mini SDK, Apache-2.0;
see the NOTICE.md beside it). Using their data -- rather than our own URDF
extraction -- means our IK applies head poses in exactly the frame
convention the mood dataset is authored against.

Per motor the data gives:
  T_motor_world   4x4 motor frame in the platform/world frame. The horn
                  rotates about this frame's local Z, sweeping in its XY
                  plane; the translation is the horn pivot.
  branch_position rod attachment point on the moving platform (head frame).
  solution        assembly-mode branch sign (0 -> +1, else -1).
  limits          (lower, upper) horn angle limits in radians.

Scalars: motor_arm_length (horn), rod_length, head_z_offset (added to a
target pose's Z before IK, per Pollen's convention).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from functools import cached_property
from pathlib import Path

import numpy as np

_DATA_PATH = (
    Path(__file__).resolve().parents[3]
    / "robot"
    / "reachy_mini_description"
    / "kinematics"
    / "kinematics_data.json"
)


@dataclass(frozen=True)
class StewartGeometry:
    """Stewart-platform geometry from Pollen's kinematics_data.json."""

    horn_length: float
    rod_length: float
    head_z_offset: float
    # Per motor (length 6):
    motor_world: np.ndarray  # (6, 4, 4) T_motor_world
    branch_position: np.ndarray  # (6, 3) platform attachment points
    solution: np.ndarray  # (6,) assembly-mode sign, +1 or -1
    horn_limits: np.ndarray  # (6, 2) lower/upper radians

    @cached_property
    def motor_world_inv(self) -> np.ndarray:
        """(6, 4, 4) inverse motor transforms (world -> motor frame)."""
        return np.array([np.linalg.inv(T) for T in self.motor_world])

    @classmethod
    def from_json(cls, path: Path = _DATA_PATH) -> "StewartGeometry":
        data = json.loads(Path(path).read_text())
        motors = data["motors"]
        return cls(
            horn_length=float(data["motor_arm_length"]),
            rod_length=float(data["rod_length"]),
            head_z_offset=float(data["head_z_offset"]),
            motor_world=np.array([m["T_motor_world"] for m in motors], dtype=float),
            branch_position=np.array(
                [m["branch_position"] for m in motors], dtype=float
            ),
            solution=np.array(
                [1.0 if m["solution"] else -1.0 for m in motors], dtype=float
            ),
            horn_limits=np.array([m["limits"] for m in motors], dtype=float),
        )


DEFAULT_GEOMETRY = StewartGeometry.from_json()
