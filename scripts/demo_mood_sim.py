"""Throwaway demo: play a mood trajectory through the MuJoCo sim (M3 + M6).

Bridges the moods library to the sim backend directly, no daemon yet
(that's M7). Loads a mood's per-frame head pose, runs it through Stewart
IK to get horn angles, and drives MujocoBackend in real time.

Head poses are applied exactly as Pollen's SDK does: the 4x4 pose as-is
(identity = neutral), with the head Z offset handled inside the IK. No
axis remapping or re-referencing. Angles are clamped to the horn limits.

Run: uv run --extra sim mjpython scripts/demo_mood_sim.py [mood_name]
(macOS needs mjpython, not python, for the MuJoCo viewer.)
"""

import sys
import time

from mood_prompt.backend.mujoco_sim import MujocoBackend
from mood_prompt.kinematics.stewart_ik import StewartIK
from mood_prompt.moods.library import load_trajectory


def main() -> None:
    mood = sys.argv[1] if len(sys.argv) > 1 else "curious1"
    traj = load_trajectory(mood)
    ik = StewartIK()
    backend = MujocoBackend("scene.xml")
    backend.enable()

    print(f"playing '{mood}': {len(traj)} frames, {traj.time[-1]:.1f}s")
    t0 = time.time()
    try:
        for i in range(len(traj)):
            horns = ik.inverse_kinematics(traj.head[i], clamp=True)
            head = [float(traj.body_yaw[i]), *horns]
            antennas = traj.antennas[i].tolist()
            # Hold each frame until its trajectory timestamp.
            target_t = float(traj.time[i])
            while time.time() - t0 < target_t:
                backend.set_target_joints(head, antennas)
                backend.step_and_render()
    finally:
        backend.close()


if __name__ == "__main__":
    main()
