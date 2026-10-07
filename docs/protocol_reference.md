# Protocol & kinematics reference

The spec of record for the wire format and the Stewart kinematics, as
actually implemented. Planning lives in [PLAN.md](PLAN.md); this is the
durable reference.

## Dynamixel Protocol 2.0 wire format

Implemented in `protocol/`. Packet layout:

```
0xFF 0xFF 0xFD 0x00  [ID]  [LEN_L] [LEN_H]  [INSTR]  [PARAMS...]  [CRC_L] [CRC_H]
```

- Status packets use instruction byte `0x55` and insert an **error byte**
  right after it, before the parameters.
- `LEN` counts from the instruction/error byte through CRC_H inclusive,
  i.e. `len(stuffed_payload) + 2`.
- **Byte stuffing:** any `FF FF FD` sequence in the payload gets an extra
  `FD` inserted so it can't be mistaken for a header. Stuffed bytes are
  counted in `LEN` and the CRC. See `protocol/packet.py`.

### CRC-16

`protocol/crc.py`. Polynomial `0x8005`, initial value 0, MSB-first, no
input/output reflection, computed over every byte from the header through
the last parameter (little-endian on the wire: CRC_L then CRC_H).

The known-answer anchor (ROBOTIS PING to ID 1) is verified in the tests:

```
FF FF FD 00 01 03 00 01 19 4E   ->  CRC = 0x4E19
```

This is the highest-risk piece — a wrong CRC means every real servo
silently rejects every packet — so it is checked byte-for-byte against
ROBOTIS's documented examples, not just self-consistent round-trips.

### Instructions

`protocol/instructions.py`: PING `0x01`, READ `0x02`, WRITE `0x03`,
SYNC_READ `0x82`, SYNC_WRITE `0x83`.

### Control table (XL330-M288-T)

`protocol/control_table.py`, as `(address, size_bytes)`:

| Register | Address | Size | Notes |
|----------|---------|------|-------|
| Operating Mode | 11 | 1 | EEPROM |
| Torque Enable | 64 | 1 | RAM |
| Hardware Error Status | 70 | 1 | RAM, read-only |
| Goal Current | 102 | 2 | RAM |
| Goal Position | 116 | 4 | RAM |
| Present Current | 126 | 2 | RAM |
| Present Position | 132 | 4 | RAM |

Position control uses 4096 ticks/rev with tick 2048 = 0 rad (centre).

Source: emanual.robotis.com/docs/en/dxl/protocol2/, .../dxl/crc/,
.../dxl/x/xl330-m288/.

### Servo layout

All 9 servos share one TTL bus (6× XL330-M288-T Stewart, 1× XC330-M288-PG
body yaw, 2× XL330-M077-T antennas). Our ID convention (`backend/hardware.py`):

| ID | Role |
|----|------|
| 1 | body yaw |
| 2–7 | Stewart 1–6 |
| 8 | right antenna |
| 9 | left antenna |

**Unverified against hardware:** IDs, the tick↔radian convention,
operating modes, and antenna rotation direction. Antennas are ordered
`[right, left]`; the emotions dataset stores them `[left, right]` and
`moods/library.py` swaps them on load (confirmed by watching sim playback).

## Rotary Stewart platform kinematics

Implemented in `kinematics/`. Reachy Mini's platform is **rotary** (6-RSS):
each leg is a servo horn (length `h`) rotating about its motor frame's
local Z axis, joined by a fixed-length rod (length `d`) to an attachment
point on the moving platform. IK solves per leg for the horn angle.

### Geometry (authoritative source)

`kinematics/geometry.py` loads
`robot/reachy_mini_description/kinematics/kinematics_data.json`, vendored
from Pollen Robotics' SDK (Apache-2.0; see the NOTICE beside it). Per motor
it provides `T_motor_world` (motor frame in the platform frame),
`branch_position` (rod attachment on the platform), a `solution` sign (the
assembly-mode branch), and horn `limits`. Scalars: `motor_arm_length`
(0.04 m), `rod_length` (0.085 m), `head_z_offset` (0.177 m).

We use Pollen's data rather than our own URDF extraction so that IK applies
head poses in exactly the frame the mood dataset is authored against.

### Pose convention

A head pose is a 4×4 homogeneous transform with **identity = neutral**
(matching Pollen's `INIT_HEAD_POSE`). Before solving, `head_z_offset` is
added to the pose's Z translation. No axis remapping or re-referencing.

### Inverse kinematics (closed form)

For each leg, the rod attachment point is transformed into the motor's
local frame, where the horn tip is `(h·cos α, h·sin α, 0)`. The fixed-rod
constraint `|tip − q|² = d²` reduces to

```
e·cos α + f·sin α = g
```

with `e = 2hx`, `f = 2hy`, `g = x² + y² + z² + h² − d²` for the point
`(x, y, z)` in the motor frame. Solved as

```
α = atan2(f, e) + solution · acos(g / hypot(e, f))
```

where the per-leg `solution` sign (from the vendored data) selects the
correct assembly mode — this removes the branch-flipping ambiguity that a
bare `asin` form suffers from. Optional clamping bounds each angle to the
leg's horn limits.

> Note: an earlier version used the `asin(g/√(e²+f²)) − atan2(f,e)` form
> from PLAN.md with hand-extracted URDF geometry. That had a swapped
> sin/cos phase and a frame mismatch against the dataset; it was replaced
> by the above. The IK/FK round-trip is now exact and mood trajectories
> solve within the horn limits.

### Forward kinematics

No closed form exists for a Stewart platform's FK, so `forward_kinematics`
solves numerically (Gauss-Newton on the 6 rod-length residuals over the
6-DOF pose), seeded at the offset-applied neutral so it converges to the
physical assembly mode. IK→FK round-trips to machine precision.

Reference derivation shape: Robert Eisele, "Inverse Kinematics of a
Stewart Platform." Pollen's own `analytical_kinematics.py` (closed-form IK,
numerical FK) confirmed the overall approach and the data conventions.
