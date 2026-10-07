"""Play a mood through a running daemon, with its audio track.

Start the daemon first (in another terminal):
    uv run python scripts/run_daemon.py --backend sim
Then:
    uv run --extra audio python scripts/play_mood.py curious1
    uv run python scripts/play_mood.py curious1 --no-audio

Loads the mood trajectory and streams each frame's head pose + antennas +
body yaw to the daemon via MiniClient, paced by the trajectory timestamps.
The mood's audio clip starts at the same moment so sound and motion stay
in sync. The daemon runs IK and drives the backend.
"""

import argparse
import time

from mood_prompt.client.mini_client import MiniClient
from mood_prompt.moods import audio
from mood_prompt.moods.library import load_trajectory


def main() -> None:
    parser = argparse.ArgumentParser(description="Play a mood through the daemon")
    parser.add_argument("mood", nargs="?", default="curious1")
    parser.add_argument("--no-audio", action="store_true", help="motion only")
    args = parser.parse_args()

    traj = load_trajectory(args.mood)
    print(f"playing '{args.mood}': {len(traj)} frames, {traj.time[-1]:.1f}s")

    sound = None if args.no_audio else audio.audio_path(args.mood)

    with MiniClient() as client:
        t0 = time.time()
        # Start audio and motion together so they stay in sync.
        if sound is not None:
            try:
                audio.play(sound)
            except audio.AudioUnavailable as exc:
                print(f"(no audio: {exc})")
        try:
            for i in range(len(traj)):
                # Pace to the trajectory's own timestamps.
                dt = float(traj.time[i]) - (time.time() - t0)
                if dt > 0:
                    time.sleep(dt)
                client.set_target(
                    traj.head[i],
                    traj.antennas[i].tolist(),
                    body_yaw=float(traj.body_yaw[i]),
                )
        finally:
            audio.stop()


if __name__ == "__main__":
    main()
