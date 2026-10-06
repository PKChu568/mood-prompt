"""Headless tests for the MuJoCo backend (no viewer, CI-safe)."""

import pytest

mujoco = pytest.importorskip("mujoco")

from mood_prompt.backend.mujoco_sim import MujocoBackend  # noqa: E402


@pytest.fixture
def backend():
    be = MujocoBackend("scene.xml")
    yield be
    be.close()


def test_model_has_nine_actuators(backend):
    assert backend.model.nu == 9


def test_set_wrong_joint_count_raises(backend):
    with pytest.raises(ValueError):
        backend.set_target_joints([0.0] * 6, [0.0, 0.0])  # 6 != 7 head joints


def test_targets_converge_after_stepping(backend):
    target_head = [0.2, 0.1, -0.1, 0.1, -0.1, 0.1, -0.1]
    target_antennas = [0.3, -0.3]
    backend.set_target_joints(target_head, target_antennas)
    for _ in range(2000):  # let the position actuators settle
        backend.step()

    head, antennas = backend.get_present_joints()
    for got, want in zip(head, target_head):
        assert got == pytest.approx(want, abs=0.05)
    for got, want in zip(antennas, target_antennas):
        assert got == pytest.approx(want, abs=0.05)
