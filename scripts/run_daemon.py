"""Thin wrapper around the daemon CLI.

    uv run python scripts/run_daemon.py --backend sim
    uv run python scripts/run_daemon.py --backend hardware --port /dev/ttyUSB0

(Equivalent to running `python -m mood_prompt.daemon.server`.)
"""

from mood_prompt.daemon.server import main

if __name__ == "__main__":
    main()
