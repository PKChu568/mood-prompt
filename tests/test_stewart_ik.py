"""Stewart IK/FK round-trip tests.

Geometry constants are UNVERIFIED against a real robot -- only against the
MJCF/URDF numbers (see kinematics/geometry.py). These tests check internal
consistency (IK then FK recovers the pose), not physical correctness.
"""

import numpy as np
import pytest

from mood_prompt.kinematics.stewart_ik import StewartIK, _params_to_pose


@pytest.fixture(scope="module")
def ik():
    return StewartIK()


def _pose(tx, ty, tz, rx, ry, rz):
    return _params_to_pose(np.array([tx, ty, tz, rx, ry, rz]))


# Small translations/rotations about the rest pose, in metres / radians.
SYNTHETIC_POSES = [
    _pose(0, 0, 0, 0, 0, 0),
    _pose(0.002, 0, 0, 0, 0, 0),
    _pose(0, -0.003, 0.001, 0, 0, 0),
    _pose(0, 0, 0, 0.03, 0, 0),
    _pose(0, 0, 0, 0, -0.02, 0),
    _pose(0, 0, 0, 0, 0, 0.05),
    _pose(0.001, 0.001, -0.001, 0.01, -0.01, 0.01),
    _pose(-0.002, 0.002, 0.0, 0.0, 0.02, -0.02),
]


@pytest.mark.parametrize("pose", SYNTHETIC_POSES)
def test_ik_fk_round_trip(ik, pose):
    angles = ik.inverse_kinematics(pose)
    recovered = ik.forward_kinematics(angles)
    # IK/FK are internally consistent to ~machine precision; measured worst
    # case over these poses is <1 um / <1 arcsec, so these tolerances leave
    # headroom without being vacuous.
    assert np.allclose(recovered[:3, 3], pose[:3, 3], atol=1e-5)
    # Rotation closeness via the relative rotation angle.
    rel = pose[:3, :3].T @ recovered[:3, :3]
    angle = np.arccos(np.clip((np.trace(rel) - 1) / 2, -1, 1))
    assert angle < np.deg2rad(0.001)


def test_identity_pose_gives_zero_angles(ik):
    angles = ik.inverse_kinematics(np.eye(4))
    assert np.allclose(angles, 0.0, atol=1e-9)
