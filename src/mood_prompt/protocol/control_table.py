"""XL330-M288-T control-table register map.

Each entry is a ``Register(address, size_bytes)``. Reachy Mini's servos are
XL330-M288-T; these addresses are from the ROBOTIS XL330 control table.
Only the registers the driver actually touches are listed — extend as
needed.
"""

from __future__ import annotations

from typing import NamedTuple


class Register(NamedTuple):
    address: int
    size: int  # bytes


OPERATING_MODE = Register(11, 1)
TORQUE_ENABLE = Register(64, 1)
HARDWARE_ERROR_STATUS = Register(70, 1)
GOAL_CURRENT = Register(102, 2)
GOAL_POSITION = Register(116, 4)
PRESENT_CURRENT = Register(126, 2)
PRESENT_POSITION = Register(132, 4)
