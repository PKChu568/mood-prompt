"""Stewart IK/FK tests, using Pollen's vendored geometry.

Geometry constants come from Pollen's kinematics_data.json, so the IK/FK
now apply head poses in the same frame convention as the mood dataset.
These tests check internal consistency (IK then FK recovers the pose) and
that real mood trajectories solve within the horn joint limits.
"""

import numpy as np
import pytest

from mood_prompt.kinematics.stewart_ik import StewartIK, _params_to_pose
from mood_prompt.moods.library import load_trajectory


@pytest.fixture(scope="module")
def ik():
    return StewartIK()


def _pose(tx, ty, tz, rx, ry, rz):
    return _params_to_pose(np.array([tx, ty, tz, rx, ry, rz]))


SYNTHETIC_POSES = [
    _pose(0, 0, 0, 0, 0, 0),
    _pose(0.005, 0, 0, 0, 0, 0),
    _pose(0, -0.004, 0.003, 0, 0, 0),
    _pose(0, 0, 0, 0.10, 0, 0),
    _pose(0, 0, 0, 0, -0.08, 0),
    _pose(0, 0, 0, 0, 0, 0.12),
    _pose(0.003, 0.002, -0.002, 0.05, -0.05, 0.05),
    _pose(-0.004, 0.003, 0.004, 0.0, 0.08, -0.06),
]


@pytest.mark.parametrize("pose", SYNTHETIC_POSES)
def test_ik_fk_round_trip(ik, pose):
    angles = ik.inverse_kinematics(pose)
    recovered = ik.forward_kinematics(angles)
    assert np.allclose(recovered[:3, 3], pose[:3, 3], atol=1e-5)
    rel = pose[:3, :3].T @ recovered[:3, :3]
    angle = np.arccos(np.clip((np.trace(rel) - 1) / 2, -1, 1))
    assert angle < np.deg2rad(0.01)


def test_identity_pose_is_reachable_and_round_trips(ik):
    # Identity is the neutral head pose; horns sit at their physical rest
    # angle (not zero), and FK must recover identity.
    angles = ik.inverse_kinematics(np.eye(4))
    recovered = ik.forward_kinematics(angles)
    assert np.allclose(recovered, np.eye(4), atol=1e-5)


def test_clamp_bounds_to_joint_limits(ik):
    # A large tilt pushes some legs past their stops; clamp must bound them.
    pose = _pose(0.0, 0.0, 0.0, 0.0, 0.6, 0.0)
    angles = np.array(ik.inverse_kinematics(pose, clamp=True))
    lo = ik.g.horn_limits[:, 0]
    hi = ik.g.horn_limits[:, 1]
    assert np.all(angles >= lo - 1e-9)
    assert np.all(angles <= hi + 1e-9)


def test_mood_trajectory_solves_within_limits(ik):
    # The whole point of adopting Pollen's geometry: real mood head poses
    # must solve inside the Stewart workspace with no clamping and smooth,
    # continuous horn angles (no assembly-mode flips).
    traj = load_trajectory("curious1")
    prev = None
    sols = []
    for pose in traj.head:
        a = np.array(ik.inverse_kinematics(pose))  # no clamp: must be reachable
        lo = ik.g.horn_limits[:, 0]
        hi = ik.g.horn_limits[:, 1]
        assert np.all(a >= lo - 1e-6) and np.all(a <= hi + 1e-6)
        sols.append(a)
        prev = a
    jumps = np.abs(np.diff(np.array(sols), axis=0))
    assert jumps.max() < 0.3  # smooth; a branch flip would be multi-radian
