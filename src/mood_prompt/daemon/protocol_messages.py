"""Message schemas for the daemon<->client WebSocket.

Plain dataclasses with to_json/from_json. Each message is a JSON object
with a "type" field plus its payload. Head poses are 4x4 matrices sent as
nested lists (JSON has no array type); antennas and body yaw are plain
numbers, matching the mood dataset and the backends' set_target_pose API.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field

import numpy as np


@dataclass
class SetTargetMsg:
    """Command an immediate head pose + antennas + body yaw."""

    head: list[list[float]]  # 4x4 homogeneous pose
    antennas: list[float]  # [right, left]
    body_yaw: float = 0.0
    type: str = field(default="set_target", init=False)

    def to_json(self) -> str:
        return json.dumps(asdict(self))

    @classmethod
    def from_dict(cls, d: dict) -> "SetTargetMsg":
        return cls(head=d["head"], antennas=d["antennas"], body_yaw=d.get("body_yaw", 0.0))

    @property
    def head_matrix(self) -> np.ndarray:
        return np.array(self.head, dtype=float)


@dataclass
class GetStateMsg:
    """Request the current robot state."""

    type: str = field(default="get_state", init=False)

    def to_json(self) -> str:
        return json.dumps({"type": self.type})

    @classmethod
    def from_dict(cls, d: dict) -> "GetStateMsg":
        return cls()


@dataclass
class StateMsg:
    """Current joint state, broadcast by the daemon."""

    head: list[float]  # 7 head joint angles [yaw, stewart_1..6]
    antennas: list[float]  # [right, left]
    type: str = field(default="state", init=False)

    def to_json(self) -> str:
        return json.dumps(asdict(self))

    @classmethod
    def from_dict(cls, d: dict) -> "StateMsg":
        return cls(head=d["head"], antennas=d["antennas"])


def parse(raw: str):
    """Decode a wire message into the matching dataclass by its "type"."""
    d = json.loads(raw)
    kind = d.get("type")
    if kind == "set_target":
        return SetTargetMsg.from_dict(d)
    if kind == "get_state":
        return GetStateMsg.from_dict(d)
    if kind == "state":
        return StateMsg.from_dict(d)
    raise ValueError(f"unknown message type: {kind!r}")
