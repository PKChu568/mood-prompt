"""Servo: named-register access on top of DynamixelBus + control_table."""

from __future__ import annotations

from mood_prompt.protocol import control_table as ct
from mood_prompt.protocol.control_table import Register

from .bus import DynamixelBus


class Servo:
    """A single servo addressed by id, exposing named registers.

    Reads/writes go through ``DynamixelBus`` using the ``(address, size)``
    tuples from ``control_table`` instead of raw numbers.
    """

    def __init__(self, bus: DynamixelBus, id: int):
        self._bus = bus
        self.id = id

    def _read(self, reg: Register) -> int:
        return int.from_bytes(self._bus.read(self.id, reg.address, reg.size), "little")

    def _write(self, reg: Register, value: int) -> None:
        self._bus.write(self.id, reg.address, value.to_bytes(reg.size, "little"))

    @property
    def torque_enable(self) -> int:
        return self._read(ct.TORQUE_ENABLE)

    @torque_enable.setter
    def torque_enable(self, value: int) -> None:
        self._write(ct.TORQUE_ENABLE, 1 if value else 0)

    @property
    def operating_mode(self) -> int:
        return self._read(ct.OPERATING_MODE)

    @operating_mode.setter
    def operating_mode(self, value: int) -> None:
        self._write(ct.OPERATING_MODE, value)

    @property
    def goal_position(self) -> int:
        return self._read(ct.GOAL_POSITION)

    @goal_position.setter
    def goal_position(self, value: int) -> None:
        self._write(ct.GOAL_POSITION, value)

    @property
    def present_position(self) -> int:
        return self._read(ct.PRESENT_POSITION)

    @property
    def present_current(self) -> int:
        return self._read(ct.PRESENT_CURRENT)

    @property
    def hardware_error_status(self) -> int:
        return self._read(ct.HARDWARE_ERROR_STATUS)
