"""Play a mood through a running daemon.

Start the daemon first (in another terminal):
    uv run python scripts/run_daemon.py --backend sim
Then:
    uv run python scripts/play_mood.py curious1

Loads the mood trajectory and streams each frame's head pose + antennas +
body yaw to the daemon via MiniClient, paced by the trajectory timestamps.
The daemon runs IK and drives the backend.
"""

import sys
import time

from mood_prompt.client.mini_client import MiniClient
from mood_prompt.moods.library import load_trajectory


def main() -> None:
    mood = sys.argv[1] if len(sys.argv) > 1 else "curious1"
    traj = load_trajectory(mood)
    print(f"playing '{mood}': {len(traj)} frames, {traj.time[-1]:.1f}s")

    with MiniClient() as client:
        t0 = time.time()
        for i in range(len(traj)):
            # Pace to the trajectory's own timestamps.
            target_t = float(traj.time[i])
            dt = target_t - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
            client.set_target(
                traj.head[i],
                traj.antennas[i].tolist(),
                body_yaw=float(traj.body_yaw[i]),
            )


if __name__ == "__main__":
    main()
