# Mood-Prompt: from-scratch Reachy Mini control stack + mood/expression app

## Context

This repo will host software that runs *on* Reachy Mini (Pollen Robotics'
desktop companion robot) to drive mood/emotion-based expressive behavior,
with behavior cloning / imitation learning planned as a later phase.

**We are writing the entire control stack ourselves** — our own Dynamixel
serial driver, our own daemon process, our own backend abstraction (real
hardware + MuJoCo sim), our own Stewart-platform inverse kinematics, and
our own Python client/SDK — rather than depending on `pip install
reachy_mini`. We do not own the physical robot yet; that's not a blocker
because we design the same backend-swap abstraction pollen-robotics uses
(a `Backend` interface with a real-hardware implementation and a
MuJoCo-simulation implementation), so all development and testing happens
against sim now, and a hardware backend slots in later without touching
the app/daemon-client layer.

This plan is grounded in two rounds of direct research (not assumptions):
1. Reading pollen-robotics' actual SDK source (cloned to scratch,
   inspected directly) — confirmed the layered architecture: SDK talks to
   a daemon over WebSocket; daemon owns a pluggable `Backend`; the real
   backend drives motors via a 50Hz control loop; kinematics has
   analytical/NN/Placo engine options.
2. Protocol-level research on Dynamixel Protocol 2.0 and Stewart-platform
   IK math (see "Technical reference" below) — enough detail to actually
   implement a packet-level driver and closed-form IK, not just call a
   library.

**Correction to earlier research**: an earlier pass (based on
pollen-robotics' `robot/backend.py` source comments) reported that body
rotation and antennas use Feetech servos while the Stewart platform uses
Dynamixel. Fresh research against Pollen Robotics' official hardware
datasheet contradicts this: **all 9 servos are Dynamixel** — 6×
XL330-M288-T (Stewart platform), 1× XC330-M288-PG (body rotation), 2×
XL330-M077-T (antennas), all on one shared TTL bus. We're building for
one protocol (Dynamixel Protocol 2.0) accordingly. This is flagged as
**unverified against real hardware** — worth double-checking once we have
a physical unit, since two independent sources disagreed.

## Technical reference (for implementation)

### Dynamixel Protocol 2.0 wire format
- Packet: `0xFF 0xFF 0xFD 0x00 [ID] [LEN_L] [LEN_H] [INSTRUCTION] [PARAMS...] [CRC_L] [CRC_H]`
  (status packets insert an Error byte right after Instruction).
- CRC-16, polynomial `0x8005`, init 0, computed via lookup table over all
  bytes from header through last parameter. Verify against ROBOTIS's
  reference `update_crc()` table when implementing — "CRC-16/IBM" naming
  is ambiguous across sources and this must match exactly or every real
  servo will reject every packet.
- Instructions needed: PING `0x01`, READ `0x02`, WRITE `0x03`, SYNC_READ
  `0x82`, SYNC_WRITE `0x83`.
- XL330-M288-T control table (address, size, area): Operating Mode (11, 1,
  EEPROM), Torque Enable (64, 1, RAM), Goal Current (102, 2, RAM), Goal
  Position (116, 4, RAM), Present Current (126, 2, RAM), Present Position
  (132, 4, RAM), Hardware Error Status (70, 1, RAM, read-only).
- Factory defaults: baud 57,600, ID 1 (bus requires re-IDing 9 servos to
  unique addresses before use).
- Source: emanual.robotis.com/docs/en/dxl/protocol2/,
  .../docs/en/dxl/crc/, .../docs/en/dxl/x/xl330-m288/.

### Rotary Stewart platform inverse kinematics
MJCF body names (`stewart_link_rod`, `dc15_a01_horn_dummy`, ×6 each)
confirm this is a **rotary** Stewart platform — servo horn + fixed-length
rod, not linear actuators — so IK must solve for a horn rotation angle,
not a leg length.

Standard approach: for each leg *k*, compute the leg vector **l**_k = **T**
+ R·**p**_k − **b**_k from desired platform pose (**T**, R) and known
attachment geometry (platform point **p**_k, base point **b**_k, both
fixed constants extractable from the MJCF). For a rotary joint, reduce the
fixed-rod-length constraint to closed form via the identity a·sinθ+b·cosθ
= R·sin(θ+φ), yielding intermediate terms e_k, f_k, g_k from the leg
vector projected onto the horn's rotation plane:

```
α_k = asin(g_k / sqrt(e_k² + f_k²)) − atan2(f_k, e_k)
```

Reference derivation: Robert Eisele, "Inverse Kinematics of a Stewart
Platform" (raw.org/research/inverse-kinematics-of-a-stewart-platform),
with a working JS reference implementation at github.com/infusion/Stewart.
This matches the *shape* of pollen-robotics' own
`analytical_kinematics.py` (closed-form, no iteration) — we build our own
version of that same idea rather than porting theirs.

## Root structure

```
Mood-Prompt/
├── pyproject.toml              # uv project, Python >=3.11
├── uv.lock
├── README.md
├── .gitignore
├── .python-version             # 3.11
│
├── robot/                              # [existing] static assets, unchanged
│   ├── reachy_mini_description/        # URDF + MJCF + meshes (used by our sim backend)
│   └── reachy_mini_emotions_library/   # 84 mood trajectories + audio (used later, phase 2)
│
├── src/
│   └── mood_prompt/
│       ├── __init__.py
│       │
│       ├── protocol/                   # Dynamixel Protocol 2.0, from scratch
│       │   ├── __init__.py
│       │   ├── crc.py                  # CRC-16 table + compute function
│       │   ├── packet.py               # instruction/status packet encode+decode
│       │   ├── instructions.py         # opcode constants (PING/READ/WRITE/SYNC_*)
│       │   └── control_table.py        # register addr/size constants (XL330/XC330)
│       │
│       ├── driver/                     # serial bus I/O layer
│       │   ├── __init__.py
│       │   ├── bus.py                  # DynamixelBus: open serial port, send/recv packets
│       │   └── servo.py                # Servo: per-ID convenience (read/write named registers)
│       │
│       ├── kinematics/
│       │   ├── __init__.py
│       │   ├── geometry.py             # Stewart platform attachment-point constants
│       │   │                           # (extracted from the MJCF/URDF we already have)
│       │   └── stewart_ik.py           # closed-form IK: pose -> 6 horn angles (+ FK)
│       │
│       ├── backend/                    # pluggable backend, mirrors pollen's Backend abstraction
│       │   ├── __init__.py
│       │   ├── abstract.py             # Backend base class: enable/disable, goto, read state
│       │   ├── hardware.py             # HardwareBackend: drives driver/bus.py over real serial
│       │   └── mujoco_sim.py           # MujocoBackend: steps robot/reachy_mini_description/mjcf
│       │                               # directly via mujoco bindings (no protocol simulation —
│       │                               # confirmed approach: direct ctrl-array control in sim)
│       │
│       ├── daemon/
│       │   ├── __init__.py
│       │   ├── server.py               # our own daemon process: owns one Backend, exposes
│       │   │                           # a WebSocket control API
│       │   └── protocol_messages.py    # message schema for the daemon<->client WebSocket
│       │
│       ├── client/
│       │   ├── __init__.py
│       │   └── mini_client.py          # MiniClient: connects to daemon over WebSocket,
│       │                               # goto_target()/set_target()/play_move() — our own
│       │                               # equivalent of ReachyMini, talks our own protocol
│       │
│       └── moods/
│           ├── __init__.py
│           ├── library.py              # loads robot/reachy_mini_emotions_library/*.json
│           │                           # (schema already verified: time[] + set_target_data[])
│           └── selector.py             # mood string -> move name (stub lookup for now)
│
├── scripts/
│   ├── run_daemon.py            # `python scripts/run_daemon.py --backend sim|hardware`
│   ├── list_moods.py            # print all 84 moods + descriptions
│   └── play_mood.py             # connect to running daemon, play one named mood
│
├── tests/
│   ├── test_protocol_packet.py  # encode/decode round-trip against known byte sequences
│   │                             # from ROBOTIS docs (PING/WRITE examples) + CRC known-answer test
│   ├── test_mock_servo_bus.py   # DynamixelBus against a mock serial responder (see below):
│   │                             # full request/response loop without real hardware
│   ├── test_stewart_ik.py       # IK/FK round-trip: pose -> angles -> pose, within tolerance
│   └── test_moods_library.py    # data validation on the emotion JSON files
│
├── tests/mock_hardware/
│   ├── __init__.py
│   └── fake_dynamixel_servo.py  # virtual serial responder: speaks real Protocol 2.0 bytes,
│                                 # backed by an in-memory register table per servo ID
│                                 # (used via a virtual serial pair, e.g. `socat` or
│                                 # `pyserial`'s loop:// / os.openpty())
│
└── docs/
    ├── architecture.md          # layered architecture, our stack vs. pollen's, backend-swap
    └── protocol_reference.md    # the Dynamixel Protocol 2.0 + Stewart IK reference material
                                  # captured above, kept as a durable spec doc
```

### Why this shape

- **`protocol/`** is pure wire-format logic (packet bytes in/out, CRC),
  zero I/O — cleanly unit-testable against known byte sequences from
  ROBOTIS documentation without any serial port or hardware.
- **`driver/`** is the only layer that touches an actual serial port.
  Built on top of `protocol/`, tested against `tests/mock_hardware/` so
  the request/response loop (write packet, wait, parse status packet,
  handle timeout/retry) is validated end-to-end before any real hardware
  exists.
- **`kinematics/`** is isolated from both protocol and I/O — pure math,
  pose ↔ joint-angle conversion. `geometry.py` holds the attachment-point
  constants, which we extract from the MJCF/URDF geometry already sitting
  in `robot/reachy_mini_description/` (body/joint positions for the
  Stewart platform are defined there) rather than re-deriving from a CAD
  export.
- **`backend/`** mirrors pollen's own `Backend` abstraction on purpose —
  it's the proven shape for this exact problem (swap hardware for sim
  with zero change above this layer). `MujocoBackend` drives the sim via
  direct MuJoCo `ctrl` array writes against `robot/reachy_mini_description/mjcf/scene.xml`
  — no protocol simulation needed in sim mode, since there's no real bus
  to simulate when MuJoCo already owns the physics.
- **`daemon/` + `client/`** replace pollen's `daemon.py` +
  `reachy_mini.py` with our own equivalents: one process owns a `Backend`
  instance and exposes a WebSocket API; `MiniClient` is the thin client
  any app (including `moods/`) talks to. Keeping client and daemon
  separate (even though nothing stops them running in the same process
  early on) preserves the "swap hardware without touching app code"
  property from day one.
- **`moods/`** is unchanged in spirit from the earlier draft — a loader
  over the emotion library we already downloaded, and a stub
  mood→move-name selector that's the seam for later behavior-cloning
  work.
- **`tests/mock_hardware/fake_dynamixel_servo.py`** is the answer to "we
  have no physical robot": a virtual serial endpoint that actually speaks
  Protocol 2.0 bytes (real CRC, real packet structure) backed by an
  in-memory register table, so `driver/bus.py` gets exercised through its
  real request/response/timeout logic, not just mocked at the Python
  function-call level.
- **`docs/protocol_reference.md`** captures the wire-format and IK math
  from the research above as a durable spec doc — this was nontrivial to
  gather correctly (CRC variant ambiguity, rotary vs. linear Stewart
  distinction) and is exactly the kind of thing worth not re-deriving
  later.

## Implementation steps

1. **Bootstrap**: `uv init`, Python 3.11, add `pyserial`, `numpy`,
   `websockets` (for daemon/client), `mujoco` (sim backend only,
   optional-dependency group), `pytest`. No `reachy_mini` dependency.

2. **`protocol/`**: implement `crc.py` (table-based CRC-16, poly
   `0x8005`) and `packet.py` (pack/unpack instruction and status frames).
   Validate immediately against ROBOTIS's published example packets
   before building anything on top.

3. **`tests/mock_hardware/fake_dynamixel_servo.py`**: build the virtual
   servo responder early — parses incoming Protocol 2.0 packets, holds a
   register dict (Torque Enable, Goal/Present Position, Operating Mode,
   etc. per the control table), returns correct status packets. This
   unblocks testing `driver/` without hardware.

4. **`driver/bus.py` + `servo.py`**: implement the real serial I/O loop
   (open port, write packet, read response with timeout/retry) against
   `protocol/`. Test against the mock responder from step 3 via a virtual
   serial pair.

5. **`kinematics/geometry.py`**: extract Stewart platform attachment
   points, horn length, rod length from
   `robot/reachy_mini_description/urdf/robot.urdf` (has explicit joint
   origins) — write a small one-off script to pull these numbers out
   rather than hand-transcribing from the MJCF visually.

6. **`kinematics/stewart_ik.py`**: implement the closed-form rotary IK
   from the reference above, plus FK (angles → pose) for validation.
   Round-trip test: known pose → IK → angles → FK → should recover the
   original pose within tolerance.

7. **`backend/abstract.py`**: define the minimal `Backend` interface
   (`enable()`, `disable()`, `set_target_joints()`,
   `get_present_joints()`, `close()`) — kept intentionally small,
   expand only as `daemon/server.py` needs more.

8. **`backend/mujoco_sim.py`**: load
   `robot/reachy_mini_description/mjcf/scene.xml` via the `mujoco` Python
   bindings (already verified this loads cleanly — 9 actuators, 19
   bodies), step physics, read/write `ctrl` array directly per the
   confirmed direct-control approach.

9. **`backend/hardware.py`**: same `Backend` interface, backed by
   `driver/bus.py` + `kinematics/stewart_ik.py`. Cannot be validated
   against real hardware yet — write it against the spec, flag as
   hardware-unverified.

10. **`daemon/server.py` + `client/mini_client.py`**: minimal WebSocket
    protocol — daemon owns one `Backend` (chosen via `--backend sim|hardware`
    CLI flag), client sends `goto_target`/`set_target` messages, daemon
    applies them to the backend and streams state back.

11. **`moods/library.py` + `selector.py`**: as in the original draft —
    load the emotion JSON library, stub selector for mood→move lookup.

12. **`scripts/`**: `run_daemon.py`, `list_moods.py`, `play_mood.py` —
    operator CLIs for manual end-to-end testing against the sim backend.

13. **`docs/architecture.md`** and **`docs/protocol_reference.md`**:
    write up as described above.

## Verification

1. `uv sync` completes cleanly.
2. `uv run pytest tests/test_protocol_packet.py` — packet encode/decode
   matches known-good byte sequences from ROBOTIS docs; CRC known-answer
   test passes.
3. `uv run pytest tests/test_mock_servo_bus.py` — full write/read cycle
   against the fake servo responder succeeds, including a simulated
   timeout/retry case.
4. `uv run pytest tests/test_stewart_ik.py` — IK→FK round-trip recovers
   known poses within tolerance (e.g. 1mm / 0.1° on synthetic test poses,
   not yet validated against real robot geometry).
5. `uv run python scripts/run_daemon.py --backend sim` starts, loads the
   MJCF, opens a MuJoCo viewer window.
6. `uv run python scripts/list_moods.py` prints all 84 moods.
7. `uv run python scripts/play_mood.py cheerful1` — client connects to
   the running sim daemon, sends the mood's trajectory, visible motion in
   the MuJoCo viewer.
8. `uv run pytest tests/test_moods_library.py` — data validation on the
   emotion library passes.
9. Explicitly documented as **not yet verified**: `backend/hardware.py`
   and the real Dynamixel driver against actual servos — no physical
   robot available. This is the natural next milestone once hardware
   arrives, and the mock-hardware tests are the best available substitute
   until then.
