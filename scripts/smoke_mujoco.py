"""Manual smoke test for the MuJoCo backend (T3.5).

Opens the passive viewer, nods the head and waggles the antennas for a
few seconds. Run: ``uv run --extra sim python scripts/smoke_mujoco.py``

Not automated (needs a display). Move/delete once scripts/run_daemon.py
exists in M7.
"""

import math
import time

from mood_prompt.backend.mujoco_sim import MujocoBackend


def main() -> None:
    backend = MujocoBackend("scene.xml")
    backend.enable()
    t0 = time.time()
    try:
        while time.time() - t0 < 8.0:
            phase = time.time() - t0
            # Small Stewart horn sweep + antenna waggle, all in radians.
            horn = 0.3 * math.sin(phase * 2.0)
            head = [0.0] + [horn] * 6
            antennas = [0.5 * math.sin(phase * 4.0), -0.5 * math.sin(phase * 4.0)]
            backend.set_target_joints(head, antennas)
            backend.step_and_render()
            time.sleep(backend.model.opt.timestep)
    finally:
        backend.close()


if __name__ == "__main__":
    main()
