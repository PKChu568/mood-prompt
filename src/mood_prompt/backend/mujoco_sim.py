"""MuJoCo simulation backend.

Drives the 9 actuated joints of the Reachy Mini MJCF model: body yaw,
6 Stewart horns, and 2 antennas, in that actuator order. The model also
has 7 passive ball joints (each a quaternion in qpos), so present joint
values are read via each actuated joint's qpos address rather than by
slicing qpos directly.
"""

from __future__ import annotations

from pathlib import Path

import mujoco

from .abstract import Backend

# Package root -> robot/reachy_mini_description/mjcf
_MJCF_DIR = (
    Path(__file__).resolve().parents[3]
    / "robot"
    / "reachy_mini_description"
    / "mjcf"
)

# Actuator order as defined in reachy_mini.xml's <actuator> block.
_HEAD_JOINTS = ["yaw_body", "stewart_1", "stewart_2", "stewart_3", "stewart_4",
                "stewart_5", "stewart_6"]
_ANTENNA_JOINTS = ["right_antenna", "left_antenna"]


class MujocoBackend(Backend):
    def __init__(self, scene: str = "scene.xml"):
        path = Path(scene)
        if not path.is_absolute():
            # Accept a bare scene name, a name under scenes/, or scene.xml.
            candidate = _MJCF_DIR / scene
            if not candidate.exists():
                candidate = _MJCF_DIR / "scenes" / scene
            path = candidate
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)

        # Map each actuated joint to its qpos address for reading present pos.
        self._head_adr = [self._qpos_adr(n) for n in _HEAD_JOINTS]
        self._antenna_adr = [self._qpos_adr(n) for n in _ANTENNA_JOINTS]
        # Actuator ctrl indices follow the <actuator> definition order.
        self._n_head = len(_HEAD_JOINTS)
        mujoco.mj_forward(self.model, self.data)
        self._viewer = None

    def _qpos_adr(self, joint_name: str) -> int:
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid < 0:
            raise ValueError(f"joint not found in model: {joint_name}")
        return int(self.model.jnt_qposadr[jid])

    # Sim has no torque-enable concept the same way hardware does.
    def enable(self) -> None:
        pass

    def disable(self) -> None:
        pass

    def set_target_joints(self, head: list[float], antennas: list[float]) -> None:
        if len(head) != self._n_head:
            raise ValueError(f"expected {self._n_head} head joints, got {len(head)}")
        if len(antennas) != len(self._antenna_adr):
            raise ValueError(
                f"expected {len(self._antenna_adr)} antenna joints, got {len(antennas)}"
            )
        self.data.ctrl[: self._n_head] = head
        self.data.ctrl[self._n_head : self._n_head + len(antennas)] = antennas

    def get_present_joints(self) -> tuple[list[float], list[float]]:
        head = [float(self.data.qpos[a]) for a in self._head_adr]
        antennas = [float(self.data.qpos[a]) for a in self._antenna_adr]
        return head, antennas

    def step(self) -> None:
        """Advance physics by one timestep (and sync the viewer if open)."""
        mujoco.mj_step(self.model, self.data)
        if self._viewer is not None:
            self._viewer.sync()

    def step_and_render(self) -> None:
        """Open the passive viewer on first call, then step + sync."""
        if self._viewer is None:
            import mujoco.viewer

            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.step()

    def close(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
