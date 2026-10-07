"""HardwareBackend against the fake bus (no physical hardware).

Same pattern as test_mock_servo_bus, one level up: drive the backend's
full enable/set/read path through virtual servos speaking Protocol 2.0.
"""

import math

import numpy as np
import pytest

from mood_prompt.backend.hardware import (
    BODY_YAW_ID,
    CENTER_TICK,
    LEFT_ANTENNA_ID,
    RIGHT_ANTENNA_ID,
    STEWART_IDS,
    HardwareBackend,
)
from mood_prompt.driver.bus import DynamixelBus
from mood_prompt.protocol.control_table import TORQUE_ENABLE
from tests.mock_hardware.fake_dynamixel_servo import FakeBus, FakeServo

ALL_IDS = [BODY_YAW_ID, *STEWART_IDS, RIGHT_ANTENNA_ID, LEFT_ANTENNA_ID]


@pytest.fixture
def backend():
    fake = FakeBus([FakeServo(i) for i in ALL_IDS])
    be = HardwareBackend(fake)
    yield be
    be.close()


def test_nine_servos_present(backend):
    assert len(backend._head_servos) == 7
    assert len(backend._antenna_servos) == 2


def test_enable_sets_torque_on_all(backend):
    backend.enable()
    for servo in backend._all_servos:
        assert servo.torque_enable == 1
    backend.disable()
    for servo in backend._all_servos:
        assert servo.torque_enable == 0


def test_rest_reads_zero_radians(backend):
    # FakeServo seeds Present Position = 2048 (centre) -> 0 rad.
    head, antennas = backend.get_present_joints()
    assert head == pytest.approx([0.0] * 7, abs=1e-6)
    assert antennas == pytest.approx([0.0, 0.0], abs=1e-6)


def test_set_target_joints_round_trips(backend):
    head = [0.1, 0.2, -0.2, 0.3, -0.3, 0.1, -0.1]
    antennas = [0.5, -0.5]
    backend.set_target_joints(head, antennas)
    # FakeServo mirrors Goal Position into Present Position, so reading back
    # recovers the commanded angles within one tick of quantization.
    got_head, got_antennas = backend.get_present_joints()
    tick_rad = (2 * math.pi) / 4096
    assert got_head == pytest.approx(head, abs=tick_rad)
    assert got_antennas == pytest.approx(antennas, abs=tick_rad)


def test_set_target_pose_uses_ik(backend):
    # Identity pose is the neutral head pose; the Stewart horns sit at their
    # physical rest angle (~+/-0.626 rad), not zero. The backend should
    # command exactly the IK solution for each horn (body yaw stays 0).
    from mood_prompt.kinematics.stewart_ik import StewartIK

    pose = np.eye(4)
    expected_horns = StewartIK().inverse_kinematics(pose)
    backend.set_target_pose(pose, antennas=[0.0, 0.0])
    head, _ = backend.get_present_joints()
    tick_rad = (2 * math.pi) / 4096
    assert head[0] == pytest.approx(0.0, abs=2 * tick_rad)  # body yaw
    assert head[1:] == pytest.approx(expected_horns, abs=2 * tick_rad)


def test_wrong_joint_count_raises(backend):
    with pytest.raises(ValueError):
        backend.set_target_joints([0.0] * 6, [0.0, 0.0])
