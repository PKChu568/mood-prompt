"""Hardware backend: drives the 9 Dynamixel servos over a real serial bus.

NOT YET VERIFIED AGAINST REAL HARDWARE -- no physical robot is available.
This is exercised only against the fake bus (tests/mock_hardware/), which
speaks real Protocol 2.0 bytes. Expect to revisit servo IDs, the tick<->
radian convention, and operating modes once a physical unit exists.

Servo layout (all on one TTL bus; IDs are our convention, the bus must be
re-IDed to match before use):

    id 1      body yaw      (XC330-M288-PG)
    id 2..7   Stewart 1..6  (XL330-M288-T)
    id 8      right antenna (XL330-M077-T)
    id 9      left antenna  (XL330-M077-T)

The head vector mirrors the sim backend: [yaw, stewart_1..6] (7 values);
antennas is [right, left] (2 values), matching Pollen's documented SDK
order. Units are radians.

Antenna rotation direction is unconfirmed against real hardware: the order
and joint sides match Pollen's convention and the MJCF, but the sign of
each antenna's rotation in sim looked questionable during manual testing.
Verify once a physical unit exists; do not flip it to satisfy the sim, as
that would desync from the authoritative [right, left] convention.
"""

from __future__ import annotations

import math

from mood_prompt.driver.bus import DynamixelBus
from mood_prompt.driver.servo import Servo
from mood_prompt.kinematics.stewart_ik import Pose, StewartIK

from .abstract import Backend

# Servo IDs by role (see module docstring).
BODY_YAW_ID = 1
STEWART_IDS = (2, 3, 4, 5, 6, 7)
RIGHT_ANTENNA_ID = 8
LEFT_ANTENNA_ID = 9

# XL330/XC330 position control: 4096 ticks/rev, centre (0 rad) at tick 2048.
TICKS_PER_REV = 4096
CENTER_TICK = 2048
_TICKS_PER_RAD = TICKS_PER_REV / (2 * math.pi)

# Position control (mode 3) for all joints.
_OPERATING_MODE_POSITION = 3


def _rad_to_ticks(rad: float) -> int:
    return int(round(CENTER_TICK + rad * _TICKS_PER_RAD))


def _ticks_to_rad(ticks: int) -> float:
    return (ticks - CENTER_TICK) / _TICKS_PER_RAD


class HardwareBackend(Backend):
    def __init__(self, port, baudrate: int = 1_000_000):
        self.bus = DynamixelBus(port, baudrate=baudrate)
        # Ordered head servos: body yaw first, then the 6 Stewart horns,
        # matching the sim backend's head vector layout.
        self._head_servos = [Servo(self.bus, BODY_YAW_ID)] + [
            Servo(self.bus, sid) for sid in STEWART_IDS
        ]
        self._antenna_servos = [
            Servo(self.bus, RIGHT_ANTENNA_ID),
            Servo(self.bus, LEFT_ANTENNA_ID),
        ]
        self._ik = StewartIK()

    @property
    def _all_servos(self) -> list[Servo]:
        return self._head_servos + self._antenna_servos

    def enable(self) -> None:
        for servo in self._all_servos:
            servo.operating_mode = _OPERATING_MODE_POSITION
            servo.torque_enable = 1

    def disable(self) -> None:
        for servo in self._all_servos:
            servo.torque_enable = 0

    def set_target_joints(self, head: list[float], antennas: list[float]) -> None:
        if len(head) != len(self._head_servos):
            raise ValueError(f"expected {len(self._head_servos)} head joints, got {len(head)}")
        if len(antennas) != len(self._antenna_servos):
            raise ValueError(
                f"expected {len(self._antenna_servos)} antenna joints, got {len(antennas)}"
            )
        for servo, rad in zip(self._head_servos, head):
            servo.goal_position = _rad_to_ticks(rad)
        for servo, rad in zip(self._antenna_servos, antennas):
            servo.goal_position = _rad_to_ticks(rad)

    def set_target_pose(
        self, pose: Pose, antennas: list[float], body_yaw: float = 0.0
    ) -> None:
        """Command a head pose (4x4) via Stewart IK, plus body yaw + antennas.

        IK yields the 6 horn angles; prepended with ``body_yaw`` they form
        the 7-element head vector consumed by ``set_target_joints``.
        """
        horn_angles = self._ik.inverse_kinematics(pose)
        self.set_target_joints([body_yaw, *horn_angles], antennas)

    def get_present_joints(self) -> tuple[list[float], list[float]]:
        head = [_ticks_to_rad(s.present_position) for s in self._head_servos]
        antennas = [_ticks_to_rad(s.present_position) for s in self._antenna_servos]
        return head, antennas

    def close(self) -> None:
        self.bus.close()
