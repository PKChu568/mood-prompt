"""End-to-end driver test over the fake bus (no physical hardware)."""

import pytest

from mood_prompt.driver.bus import BusError, DynamixelBus
from mood_prompt.driver.servo import Servo
from mood_prompt.protocol.control_table import GOAL_POSITION, PRESENT_POSITION
from tests.mock_hardware.fake_dynamixel_servo import FakeBus, FakeServo


@pytest.fixture
def bus():
    fake = FakeBus([FakeServo(1), FakeServo(2)])
    return DynamixelBus(fake)


def test_ping(bus):
    status = bus.ping(1)
    assert status.id == 1
    assert status.error == 0


def test_write_then_read_goal_position(bus):
    bus.write(1, GOAL_POSITION.address, (1500).to_bytes(4, "little"))
    raw = bus.read(1, PRESENT_POSITION.address, PRESENT_POSITION.size)
    assert int.from_bytes(raw, "little") == 1500


def test_servo_wrapper_round_trip(bus):
    servo = Servo(bus, 2)
    assert servo.present_position == 2048  # factory default
    servo.goal_position = 1000
    assert servo.present_position == 1000
    servo.torque_enable = 1
    assert servo.torque_enable == 1


def test_timeout_on_absent_servo(bus):
    with pytest.raises(BusError):
        bus.ping(99, retries=1)
