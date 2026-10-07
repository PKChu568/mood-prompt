# Architecture

Mood-Prompt is a from-scratch control stack for the Reachy Mini robot. It
mirrors the layered design Pollen Robotics uses, but every layer is built
here — our own Dynamixel driver, Stewart kinematics, backends, daemon, and
client — without depending on `pip install reachy_mini`.

## Layers

```
  app / script            scripts/play_mood.py, demo_mood_sim.py
        │  (head pose, antennas, body yaw)
        ▼
  client                  client/mini_client.py   MiniClient
        │  WebSocket (JSON messages, daemon/protocol_messages.py)
        ▼
  daemon                  daemon/server.py         Daemon
        │  runs Stewart IK on each head pose → joint angles
        ▼
  backend (pluggable)     backend/abstract.py      Backend (ABC)
        ├── MujocoBackend  backend/mujoco_sim.py    data.ctrl on the MJCF
        └── HardwareBackend backend/hardware.py     driver + IK over serial
                │
        ┌───────┴───────────────┐
        ▼                       ▼
  driver                   kinematics
  driver/bus.py            kinematics/stewart_ik.py   IK/FK
  driver/servo.py          kinematics/geometry.py     geometry constants
        │
        ▼
  protocol                 protocol/{crc,packet,instructions,control_table}.py
                           Dynamixel Protocol 2.0 wire format
```

A separate **moods** layer (`moods/library.py`, `moods/selector.py`) loads
the emotion dataset and feeds trajectories into the client; it sits beside
the app layer rather than inside the control path.

## The backend swap

The key design choice is that everything above the backend is identical in
simulation and on hardware. The daemon owns exactly one `Backend`, selected
at startup:

- `--backend sim` constructs a `MujocoBackend`, which writes joint targets
  into the MJCF model's `data.ctrl` and steps physics.
- `--backend hardware --port /dev/ttyUSB0` constructs a `HardwareBackend`,
  which drives the real Dynamixel servos through `driver/bus.py`.

Both implement the same small interface (`backend/abstract.py`):

```python
enable() / disable()
set_target_joints(head: list[float], antennas: list[float])
get_present_joints() -> (head, antennas)
close()
```

`head` is 7 joint angles `[body_yaw, stewart_1..6]`; `antennas` is
`[right, left]`; all radians. Because the daemon, client, moods, and scripts
only ever touch this interface (and head *poses*, which the daemon turns
into joints via IK), the hardware backend slots in without changing any
layer above it.

## Testing without hardware

No physical robot is required to exercise the full stack:

- **Protocol/driver:** `tests/mock_hardware/fake_dynamixel_servo.py` is a
  virtual bus that speaks real Protocol 2.0 bytes, so `DynamixelBus` and
  `HardwareBackend` run their complete request/response/timeout path
  against it (`tests/test_mock_servo_bus.py`, `tests/test_hardware_backend.py`).
- **Sim backend + daemon:** `tests/test_daemon_client.py` runs the daemon
  with `MujocoBackend` in a thread and drives it over a real local
  WebSocket.

What is **not** verified without hardware: servo IDs, the tick↔radian
convention, operating modes, the Stewart geometry's correspondence to a
real unit, and antenna rotation direction. These are flagged in the code
and in [protocol_reference.md](protocol_reference.md).

## Data flow for a mood

1. `play_mood.py` loads a trajectory (`moods/library.py`): per-frame head
   pose (4×4), antennas, body yaw.
2. For each frame it calls `MiniClient.set_target(head, antennas, body_yaw)`.
3. The client sends a `SetTargetMsg` over the WebSocket.
4. The daemon runs `StewartIK.inverse_kinematics` on the head pose to get
   6 horn angles, prepends body yaw, and calls `backend.set_target_joints`.
5. The backend applies the targets (sim: `data.ctrl` + physics step;
   hardware: Goal Position writes to each servo).
