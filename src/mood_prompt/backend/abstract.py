"""Backend interface implemented by both the sim and hardware backends.

Kept deliberately small. Expand only when the daemon actually needs more.
``head`` is the 7 actuated head joints (body yaw + 6 Stewart horns);
``antennas`` is the 2 antenna joints (right, left). Units are radians.
"""

from __future__ import annotations

from abc import ABC, abstractmethod


class Backend(ABC):
    @abstractmethod
    def enable(self) -> None:
        """Engage torque / make the robot hold targets."""

    @abstractmethod
    def disable(self) -> None:
        """Release torque."""

    @abstractmethod
    def set_target_joints(self, head: list[float], antennas: list[float]) -> None:
        """Command target joint angles (radians)."""

    @abstractmethod
    def get_present_joints(self) -> tuple[list[float], list[float]]:
        """Return current (head, antennas) joint angles (radians)."""

    @abstractmethod
    def close(self) -> None:
        """Release resources (serial port, viewer, ...)."""
