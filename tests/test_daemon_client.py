"""End-to-end daemon<->client test over a real local WebSocket.

Runs the daemon (sim backend) in a background thread, connects a
MiniClient, sends a target pose, and reads state back. Exercises the full
message round-trip and the daemon's IK + backend call path.
"""

import threading
import time

import numpy as np
import pytest

pytest.importorskip("mujoco")
pytest.importorskip("websockets")

from mood_prompt.backend.mujoco_sim import MujocoBackend  # noqa: E402
from mood_prompt.client.mini_client import MiniClient  # noqa: E402
from mood_prompt.daemon.server import Daemon  # noqa: E402

# Each daemon instance keeps its background thread (and bound port) for the
# life of the process, so hand out a fresh port per test to avoid clashes.
_next_port = iter(range(8800, 8900))


@pytest.fixture
def port():
    p = next(_next_port)
    backend = MujocoBackend("scene.xml")
    d = Daemon(backend, port=p, step_sim=True)
    thread = threading.Thread(target=d.run, daemon=True)
    thread.start()
    time.sleep(0.5)  # let the server bind and start ticking
    yield p


def test_get_state_round_trip(port):
    with MiniClient(port=port) as client:
        head, antennas = client.get_present_joints()
    assert len(head) == 7
    assert len(antennas) == 2


def test_set_target_moves_backend(port):
    # Command a tilt; after the sim settles, present joints should change
    # from the neutral configuration.
    with MiniClient(port=port) as client:
        neutral_head, _ = client.get_present_joints()
        pose = np.eye(4)
        pose[:3, :3] = np.array(
            [[1, 0, 0], [0, np.cos(0.15), -np.sin(0.15)], [0, np.sin(0.15), np.cos(0.15)]]
        )
        client.set_target(pose, [0.3, -0.3])
        # The sim steps in real time, so the soft position actuators take a
        # few wall-clock seconds to approach the target; poll until they get
        # substantially there rather than guessing a fixed sleep.
        deadline = time.time() + 10.0
        moved_head = moved_antennas = None
        while time.time() < deadline:
            time.sleep(0.5)
            moved_head, moved_antennas = client.get_present_joints()
            if moved_antennas[0] > 0.15 and moved_antennas[1] < -0.15:
                break

    # The head left its neutral configuration, and the antennas moved
    # substantially toward their commanded targets (right +0.3, left -0.3).
    assert not np.allclose(moved_head, neutral_head, atol=1e-3)
    assert moved_antennas[0] > 0.15  # heading toward +0.3
    assert moved_antennas[1] < -0.15  # heading toward -0.3
