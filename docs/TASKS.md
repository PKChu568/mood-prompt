# Task plan

Companion to [PLAN.md](./PLAN.md). Breaks the 13 implementation steps
there into small, independently-committable tasks, grouped into
milestones. Tasks within a milestone are ordered by dependency — do them
top to bottom. Milestones themselves are also dependency-ordered (M1 →
M2 → ... → M7), except M6 (moods) and M4/M5 (kinematics/hardware
backend), which can run in parallel with the milestone before them once
M3 is done — noted inline below.

Each task is scoped to be one focused PR/commit: one file or a tight
cluster of files, one clear "done" condition.

---

## M0 — Bootstrap

- [x] **T0.1** `uv init`, set Python `>=3.11`, create `.python-version`.
- [x] **T0.2** Add core deps: `pyserial`, `numpy`. Add dev deps: `pytest`.
      Add optional-dependency group `sim` with `mujoco`. Add optional
      group `daemon` with `websockets`.
- [x] **T0.3** `uv sync` succeeds; commit `pyproject.toml` + `uv.lock`.

**Done when:** `uv run python -c "import numpy, serial"` works.

---

## M1 — Protocol layer (`src/mood_prompt/protocol/`)

Pure wire-format code. No I/O, no hardware, no MuJoCo. Fully testable
in isolation — do this first since everything else depends on it.

- [x] **T1.1** `protocol/crc.py` — CRC-16 lookup table (poly `0x8005`)
      + `compute_crc(data: bytes) -> int`. Write the known-answer test
      *alongside* this (`tests/test_protocol_packet.py::test_crc_known_value`)
      using a byte sequence from ROBOTIS docs before writing any other
      protocol code — this is the highest-risk piece of the whole driver
      (silent wrong-CRC bugs are hard to diagnose later).
- [x] **T1.2** `protocol/instructions.py` — opcode constants: `PING =
      0x01`, `READ = 0x02`, `WRITE = 0x03`, `SYNC_READ = 0x82`,
      `SYNC_WRITE = 0x83`. Just constants, no logic.
- [x] **T1.3** `protocol/control_table.py` — register constants for
      XL330-M288-T: `OPERATING_MODE = (11, 1)`, `TORQUE_ENABLE = (64,
      1)`, `GOAL_CURRENT = (102, 2)`, `GOAL_POSITION = (116, 4)`,
      `PRESENT_CURRENT = (126, 2)`, `PRESENT_POSITION = (132, 4)`,
      `HARDWARE_ERROR_STATUS = (70, 1)`. Use `(address, size_bytes)`
      tuples or a small dataclass — pick whichever, just be consistent.
- [x] **T1.4** `protocol/packet.py` — `encode_instruction_packet(id,
      instruction, params) -> bytes` (header + id + len + instruction +
      params + CRC).
- [x] **T1.5** `protocol/packet.py` — `decode_status_packet(data: bytes)
      -> StatusPacket` (id, error byte, params, CRC-validated). Raise a
      clear exception on CRC mismatch or malformed header.
- [x] **T1.6** `tests/test_protocol_packet.py` — round-trip tests:
      encode a PING packet, compare byte-for-byte against a known
      ROBOTIS example; encode/decode a WRITE(Goal Position) packet;
      decode a status packet with a deliberately corrupted CRC and
      assert it raises.

**Done when:** `uv run pytest tests/test_protocol_packet.py` passes, all
byte sequences verified against ROBOTIS documentation examples (not just
self-consistent round-trips).

---

## M2 — Mock hardware + driver (`tests/mock_hardware/`, `src/mood_prompt/driver/`)

Depends on M1. Build the fake servo *before* the real driver so the
driver has something to test against from its first commit.

- [x] **T2.1** `tests/mock_hardware/fake_dynamixel_servo.py` —
      `FakeServo` class: holds an in-memory register dict seeded with
      XL330 defaults (ID, baud, Torque Enable=0, Present Position=2048,
      etc.), `handle_packet(raw: bytes) -> bytes` that decodes an
      instruction packet (via `protocol/`), mutates/reads its register
      dict, and encodes+returns a status packet.
- [x] **T2.2** `tests/mock_hardware/fake_dynamixel_servo.py` —
      `FakeBus` class: holds multiple `FakeServo`s by ID, routes an
      incoming packet to the right servo (or broadcasts for `SYNC_*`),
      exposed as a `pyserial`-compatible object (`read()`/`write()`) so
      real driver code can talk to it without knowing it's fake. Use
      `serial.serial_for_url("loop://")` or a `socketpair`-backed pair —
      pick one approach and note it in a comment.
- [x] **T2.3** `driver/bus.py` — `DynamixelBus.__init__(port, baudrate)`
      opens a real `pyserial` port; `write_packet(bytes)`,
      `read_packet(timeout) -> bytes`.
- [x] **T2.4** `driver/bus.py` — `DynamixelBus.ping(id)`,
      `.read(id, address, size)`, `.write(id, address, data)` — build on
      T2.3 + `protocol/`, with timeout + a small retry count (matches
      the `allowed_retries` idea seen in pollen's driver).
- [x] **T2.5** `driver/servo.py` — `Servo` wrapper: takes a `bus` + `id`,
      exposes named properties (`torque_enable`, `goal_position`,
      `present_position`, ...) that read/write via `control_table.py`
      addresses instead of raw numbers.
- [x] **T2.6** `tests/test_mock_servo_bus.py` — instantiate `FakeBus`
      with a couple of `FakeServo`s, run `DynamixelBus` against it:
      ping, write Goal Position, read back Present Position, and one
      deliberate timeout case (servo ID not present on the bus).

**Done when:** `uv run pytest tests/test_mock_servo_bus.py` passes —
the real driver code path is exercised end-to-end with zero physical
hardware.

---

## M3 — Backend abstraction, sim first (`src/mood_prompt/backend/`)

Depends on M2 only for `abstract.py`'s shape; `mujoco_sim.py` itself
only depends on the MJCF assets already in `robot/` and has no
dependency on M1/M2. **Can start in parallel with M2** if useful, but
`abstract.py` should land first since both backends implement it.

- [x] **T3.1** `backend/abstract.py` — `Backend` ABC: `enable()`,
      `disable()`, `set_target_joints(head: list[float], antennas:
      list[float])`, `get_present_joints() -> tuple[list[float],
      list[float]]`, `close()`. Keep it exactly this small — expand
      later only when `daemon/server.py` actually needs more.
- [x] **T3.2** `backend/mujoco_sim.py` — `MujocoBackend.__init__(scene:
      str)` loads `robot/reachy_mini_description/mjcf/scene.xml` (or
      the named scene under `mjcf/scenes/`) via `mujoco.MjModel`.
- [x] **T3.3** `backend/mujoco_sim.py` — implement `set_target_joints`
      (write to `data.ctrl`), `get_present_joints` (read `data.qpos` for
      the 9 actuated joints), a `step()` method advancing physics by one
      timestep, and `enable()`/`disable()` as no-ops (sim has no torque
      concept the same way).
- [x] **T3.4** `backend/mujoco_sim.py` — minimal run loop or `step_and_render()`
      helper that opens the MuJoCo passive viewer (`mujoco.viewer.launch_passive`)
      so motion is visible during manual testing.
- [x] **T3.5** Manual smoke test (not automated yet — no daemon exists):
      a throwaway script that constructs `MujocoBackend`, sets a target,
      steps physics for a few seconds, confirms the viewer shows motion.
      Delete or move into `scripts/` once `scripts/run_daemon.py` exists
      in M7.

**Done when:** you can watch the MuJoCo viewer move the head/antennas in
response to `set_target_joints` calls, driven entirely by
`backend/mujoco_sim.py`.

---

## M4 — Kinematics (`src/mood_prompt/kinematics/`)

Depends on nothing except the MJCF/URDF files already in `robot/`. Can
run fully in parallel with M2/M3 — pure math, no I/O.

- [x] **T4.1** One-off extraction script (can live in `scripts/` or be
      throwaway): parse `robot/reachy_mini_description/urdf/robot.urdf`
      for the Stewart platform's joint origins, horn length, rod length,
      and base/platform attachment points. Print them as a Python dict
      literal.
- [x] **T4.2** `kinematics/geometry.py` — paste the extracted constants
      in as a `StewartGeometry` dataclass (6× base points, 6× platform
      points, horn length, rod length). This is data, not logic.
- [x] **T4.3** `kinematics/stewart_ik.py` — `forward_kinematics(horn_angles:
      list[float]) -> Pose` (4x4 matrix or position+quaternion — match
      whatever `moods/library.py`'s trajectory format expects, see M6).
- [x] **T4.4** `kinematics/stewart_ik.py` — `inverse_kinematics(pose:
      Pose) -> list[float]` (6 horn angles), implementing the closed-form
      `α_k = asin(g_k / sqrt(e_k²+f_k²)) − atan2(f_k, e_k)` from
      PLAN.md's technical reference.
- [x] **T4.5** `tests/test_stewart_ik.py` — round-trip test: pick 5-10
      synthetic poses (small translations/rotations from neutral), run
      IK → FK, assert recovered pose is within tolerance (start with a
      loose tolerance, e.g. 1mm/0.1°, tighten once passing).

**Done when:** `uv run pytest tests/test_stewart_ik.py` passes. Flag in
the test file docstring that geometry constants are unverified against
a real robot (only against the MJCF/URDF numbers).

---

## M5 — Hardware backend (`backend/hardware.py`)

Depends on M2 (driver) + M4 (kinematics) + M3 (for the `Backend`
interface shape).

- [x] **T5.1** `backend/hardware.py` — `HardwareBackend.__init__(port)`
      opens a `DynamixelBus`, constructs 9 `Servo` instances (6 Stewart +
      1 body yaw + 2 antennas) with their IDs.
- [x] **T5.2** `backend/hardware.py` — implement `enable()`/`disable()`
      (torque on/off across all 9 servos), `get_present_joints()` (read
      Present Position from all servos, convert raw ticks → radians).
- [x] **T5.3** `backend/hardware.py` — implement `set_target_joints()`:
      accept head pose or joint angles, run through
      `kinematics.stewart_ik` if given a pose, convert radians → raw
      ticks, write Goal Position to each Stewart servo + body yaw +
      antennas.
- [x] **T5.4** Extend `tests/mock_hardware/fake_dynamixel_servo.py`'s
      `FakeBus` (from T2.2) to represent all 9 servo IDs matching Reachy
      Mini's layout, and add a test instantiating `HardwareBackend`
      against it (same pattern as T2.6, one level up).

**Done when:** `HardwareBackend` passes the same mock-bus test pattern
as the raw driver. Explicitly documented as **not yet verified against
real hardware** — this is expected and fine; flag it in a comment at
the top of the file.

---

## M6 — Moods (`src/mood_prompt/moods/`)

Depends on nothing but the dataset already in `robot/reachy_mini_emotions_library/`.
Fully parallel with M2–M5 — pure data loading.

- [x] **T6.1** `moods/library.py` — `load_metadata() -> list[MoodEntry]`
      parsing `metadata.jsonl` (title, description, motion_file,
      file_name).
- [x] **T6.2** `moods/library.py` — `load_trajectory(name: str) ->
      Trajectory` parsing the named `.json` file's `time`/`set_target_data`
      arrays into numpy arrays.
- [x] **T6.3** `tests/test_moods_library.py` — for every entry in
      `metadata.jsonl`: assert the referenced `.json` and audio file
      exist, assert `len(time) == len(set_target_data)`, assert no NaNs.
- [x] **T6.4** `moods/selector.py` — `select_mood(text: str) -> str`:
      case-insensitive substring match against loaded titles/descriptions,
      returns the best-matching mood name (or a documented fallback like
      `"curious1"` if nothing matches). Explicitly a stub — docstring
      says so and points at PLAN.md's later behavior-cloning phase.

**Done when:** `uv run pytest tests/test_moods_library.py` passes on
all 81 entries (the dataset has 81 metadata records; the
plan's "84" was an estimate).

---

## M7 — Daemon + client + scripts (wiring it all together)

Depends on M3 (backend), and ideally M5/M6 too since the scripts exist
to exercise the whole stack, but the daemon/client plumbing itself only
strictly needs M3.

- [ ] **T7.1** `daemon/protocol_messages.py` — message schemas for the
      daemon↔client WebSocket: `SetTargetMsg`, `GotoTargetMsg`,
      `GetStateMsg`/`StateMsg`. Plain dataclasses + a `to_json`/`from_json`
      pair (or use `json.dumps`/`dataclasses.asdict` directly — no need
      for a heavy serialization library here).
- [ ] **T7.2** `daemon/server.py` — `Daemon` class: takes a `Backend`
      instance, opens a `websockets` server, on each incoming message
      calls the corresponding `Backend` method, on a timer broadcasts
      current state to connected clients.
- [ ] **T7.3** `daemon/server.py` — CLI entry (`if __name__ ==
      "__main__"` or a small `argparse`) supporting `--backend sim`
      (constructs `MujocoBackend`) and `--backend hardware --port
      /dev/ttyUSB0` (constructs `HardwareBackend`).
- [ ] **T7.4** `client/mini_client.py` — `MiniClient.__init__(host,
      port)` connects over WebSocket; `set_target(...)`,
      `goto_target(..., duration)` (client-side interpolation, matches
      the pattern seen in pollen's SDK — linear or min-jerk, pick one to
      start), `get_present_joints()`.
- [ ] **T7.5** `scripts/run_daemon.py` — thin CLI wrapper around
      `daemon/server.py`'s entry point (or just document running the
      module directly — decide based on how T7.3 ends up shaped).
- [ ] **T7.6** `scripts/list_moods.py` — uses `moods/library.py` to
      print all mood names + descriptions.
- [ ] **T7.7** `scripts/play_mood.py <mood_name>` — connects
      `MiniClient` to a running daemon, loads a trajectory via
      `moods/library.py`, steps through it calling `set_target` at the
      trajectory's timestamps.

**Done when:** the full manual flow works — start `run_daemon.py
--backend sim` in one terminal, run `play_mood.py cheerful1` in
another, see the MuJoCo viewer move.

---

## M8 — Docs

Can be done incrementally alongside any milestone above, or as a final
pass. Low-risk, no code dependencies.

- [ ] **T8.1** `docs/architecture.md` — the layered-architecture diagram
      and backend-swap explanation from the research conversation.
- [ ] **T8.2** `docs/protocol_reference.md` — lift the "Technical
      reference" section from `PLAN.md` into its own durable doc (CRC
      details, control table, Stewart IK derivation) so `PLAN.md` can
      stay focused on planning and this becomes the spec of record.

---

## Suggested working order

For a single developer going top to bottom with the least idle time
waiting on dependencies:

```
M0 → M1 → M2 ─┬─ M3 → (parallel: M4, M6) → M5 → M7 → M8
              └─ (M4, M6 can start anytime after M0, don't block on M2)
```

Practically: **M0, M1, M2 in strict order** (each depends on the last).
Then **M3, M4, M6 can be done in any order or interleaved** (independent
of each other). **M5 needs M2 + M4 done.** **M7 needs M3 done at
minimum**, benefits from M5/M6 being done too so there's something
meaningful to run end-to-end. **M8 anytime.**
