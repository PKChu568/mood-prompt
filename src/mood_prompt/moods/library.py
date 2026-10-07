"""Load the Reachy Mini emotions library (metadata + motion trajectories).

Dataset lives in robot/reachy_mini_emotions_library/. ``metadata.jsonl`` has
one JSON object per line; each motion file is a JSON with ``time`` and
``set_target_data`` arrays, where each frame is a dict of:
    head          4x4 homogeneous head pose
    antennas      [right, left] angles (radians)
    body_yaw      body yaw angle (radians)
    check_collision  bool (ignored here)

This matches the HardwareBackend/MujocoBackend target API: head pose +
antennas + body yaw.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

_LIBRARY_DIR = (
    Path(__file__).resolve().parents[3]
    / "robot"
    / "reachy_mini_emotions_library"
)
_METADATA = _LIBRARY_DIR / "metadata.jsonl"


@dataclass(frozen=True)
class MoodEntry:
    title: str
    description: str
    motion_file: str
    file_name: str  # audio file


@dataclass
class Trajectory:
    time: np.ndarray  # (N,) seconds
    head: np.ndarray  # (N, 4, 4) homogeneous poses
    antennas: np.ndarray  # (N, 2) radians
    body_yaw: np.ndarray  # (N,) radians

    def __len__(self) -> int:
        return len(self.time)


def library_dir() -> Path:
    return _LIBRARY_DIR


def load_metadata() -> list[MoodEntry]:
    """Parse metadata.jsonl into MoodEntry records."""
    entries = []
    with open(_METADATA) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            obj = json.loads(line)
            entries.append(
                MoodEntry(
                    title=obj["title"],
                    description=obj["description"],
                    motion_file=obj["motion_file"],
                    file_name=obj["file_name"],
                )
            )
    return entries


def load_trajectory(name: str) -> Trajectory:
    """Load a motion trajectory by mood name or motion file name.

    ``name`` may be a title ("amazed1"), a motion file ("amazed1.json"), or
    a path. Raises FileNotFoundError if the file is missing.
    """
    filename = name if name.endswith(".json") else f"{name}.json"
    path = Path(name) if Path(name).is_absolute() else _LIBRARY_DIR / filename
    with open(path) as f:
        data = json.load(f)

    time = np.asarray(data["time"], dtype=float)
    frames = data["set_target_data"]
    head = np.asarray([frame["head"] for frame in frames], dtype=float)
    antennas = np.asarray([frame["antennas"] for frame in frames], dtype=float)
    body_yaw = np.asarray([frame["body_yaw"] for frame in frames], dtype=float)
    return Trajectory(time=time, head=head, antennas=antennas, body_yaw=body_yaw)
