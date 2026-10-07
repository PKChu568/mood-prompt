"""MiniClient: a thin synchronous WebSocket client for the daemon.

set_target sends one pose immediately. goto_target interpolates from the
current target to a new one over a duration, client-side (min-jerk on a
scalar s in [0,1], applied to translation, rotation, and antennas), which
matches the pattern Pollen's SDK uses. get_present_joints requests state.
"""

from __future__ import annotations

import time

import numpy as np
from websockets.sync.client import connect

from mood_prompt.daemon.protocol_messages import GetStateMsg, SetTargetMsg, parse


def _min_jerk(s: float) -> float:
    """Min-jerk easing: 0->0, 1->1, zero vel/accel at the ends."""
    return 10 * s**3 - 15 * s**4 + 6 * s**5


def _slerp_pose(a: np.ndarray, b: np.ndarray, s: float) -> np.ndarray:
    """Interpolate two 4x4 poses: lerp translation, slerp rotation."""
    out = np.eye(4)
    out[:3, 3] = (1 - s) * a[:3, 3] + s * b[:3, 3]
    # Rotation via the relative rotation's axis-angle, scaled by s.
    rel = a[:3, :3].T @ b[:3, :3]
    theta = np.arccos(np.clip((np.trace(rel) - 1) / 2, -1, 1))
    if theta < 1e-9:
        out[:3, :3] = a[:3, :3]
    else:
        axis = (
            np.array([rel[2, 1] - rel[1, 2], rel[0, 2] - rel[2, 0], rel[1, 0] - rel[0, 1]])
            / (2 * np.sin(theta))
        )
        k = axis
        kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        r = np.eye(3) + np.sin(theta * s) * kx + (1 - np.cos(theta * s)) * (kx @ kx)
        out[:3, :3] = a[:3, :3] @ r
    return out


class MiniClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 8765):
        # legacy=True: we manage the connection lifetime ourselves (close()
        # / context manager on MiniClient), rather than using connect() as a
        # one-shot context manager.
        self._ws = connect(f"ws://{host}:{port}", legacy=True)
        # Last commanded target (for goto interpolation start point).
        self._last_head = np.eye(4)
        self._last_antennas = [0.0, 0.0]
        self._last_body_yaw = 0.0

    def set_target(
        self, head: np.ndarray, antennas: list[float], body_yaw: float = 0.0
    ) -> None:
        head = np.asarray(head, dtype=float)
        msg = SetTargetMsg(head=head.tolist(), antennas=list(antennas), body_yaw=body_yaw)
        self._ws.send(msg.to_json())
        self._last_head = head
        self._last_antennas = list(antennas)
        self._last_body_yaw = body_yaw

    def goto_target(
        self,
        head: np.ndarray,
        antennas: list[float],
        duration: float,
        body_yaw: float = 0.0,
        rate_hz: float = 50.0,
    ) -> None:
        start_head = self._last_head.copy()
        start_ant = np.array(self._last_antennas)
        start_yaw = self._last_body_yaw
        target_head = np.asarray(head, dtype=float)
        target_ant = np.array(antennas)
        steps = max(1, int(duration * rate_hz))
        dt = duration / steps
        for i in range(1, steps + 1):
            s = _min_jerk(i / steps)
            h = _slerp_pose(start_head, target_head, s)
            a = (1 - s) * start_ant + s * target_ant
            y = (1 - s) * start_yaw + s * body_yaw
            self.set_target(h, a.tolist(), y)
            time.sleep(dt)

    def get_present_joints(self) -> tuple[list[float], list[float]]:
        self._ws.send(GetStateMsg().to_json())
        while True:
            msg = parse(self._ws.recv())
            if getattr(msg, "type", None) == "state":
                return msg.head, msg.antennas

    def close(self) -> None:
        self._ws.close()

    def __enter__(self) -> "MiniClient":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
