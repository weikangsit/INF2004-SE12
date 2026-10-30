# Data representation (Change Requirement 1) - whole firmware

How every piece of application data that crosses a task, interrupt or
network boundary is stored, with its range, the bits it needs and how it is
packed. MQTT payloads are specified field by field in
[mqtt_protocol.md](mqtt_protocol.md); Buddy 1's earlier evidence is in
[buddy1_data_representation.md](buddy1_data_representation.md).

Rules applied everywhere:

* Data shared between tasks, between an ISR and a task, in queue messages
  and in MQTT payloads uses only the bits its range needs.
* Data that leaves the processor is packed with explicit shifts and masks
  (`bitpack.c`, `proto.c`), never C bit-fields, because bit-field order is
  compiler-defined (Barr-C 5.5). RAM-only flag sets inside one module use
  `bool` / `uint8_t` bit-fields.
* Values used only inside one task's control or signal maths stay at full
  precision (float), as Tribunal 1 decided for Motion Control (Buddy 2).
* Shared packed data is accessed safely: a mutex for read-modify-write of
  the shared record, kernel event-flag calls for the fault mask, and single
  writers with one aligned load/store for everything else.

## 1. Shared vehicle state record (`vehicle_state.c`, mutex `state_mtx`)

Was 52 bytes of floats and bytes (`motion_state_t`); now 132 bits =
**17 bytes**, packed with `bitpack.c`, read as a whole snapshot.

| Field | Writer | Range / step | Bits | Was |
|---|---|---|---|---|
| speed left, right | Motion | +/-2047 mm/s, 1 mm/s | 12 + 12 signed | 2 x float |
| duty left, right | Motion | -100..100 % | 8 + 8 signed | 2 x float |
| distance | Motion | +/-131 m, 1 mm | 18 signed | float |
| pitch | IMU | +/-128 deg, 0.5 deg | 9 signed | float |
| roll | IMU | +/-180 deg, 1 deg | 9 signed | float |
| heading | IMU | 0..359.5 deg, 0.5 deg | 10 | float |
| turn rate | IMU | +/-255 deg/s | 9 signed | float |
| hump peak | IMU | 0..127.5 mm, 0.5 mm | 8 | float |
| motion class | IMU | 0..3 | 2 | uint8 |
| IMU valid | IMU | flag | 1 | (none) |
| magnetometer calibrated | IMU | flag | 1 | (none) |
| IMU time stamp | IMU | ms modulo 65536 | 16 | (none) |
| raw line sensors | Line | 2 x black/white | 2 | int8 pos + 2 bools |
| last barcode | Line | none / A..D | 3 | char |
| nav state | Nav | 9 states | 4 | uint8 |

The IMU time stamp lets consumers ignore IMU data older than 60 ms
(Tribunal 1 interface rule). Line position and "line seen" are derived from
the two raw bits, so they are no longer stored twice.

## 2. Inter-task messages

| Queue / channel | From -> to | Was | Now | Bits used |
|---|---|---|---|---|
| line reading (was `line_pos_q`) | Line -> Motion | 8-byte message, queue of 4 | one 32-bit word, newest value | 2 sensor bits + 16-bit stamp |
| `bar_cmd_q` | Line -> Nav | 1 byte `char` A..Z | 1 byte | 2 (letter A..D) |
| `motion_cmd_q` | Nav, Safety -> Motion | 20 bytes (enum, 3 floats, 2 bools) | 4 bytes | 32 (below) |
| `motion_done_q` (new, Version C) | Motion -> Nav | (shared `motion_evt_q`, 8 bytes) | 1 byte | 5 (seq 3 + result 2) |
| terrain events | IMU -> Nav | (shared `motion_evt_q`, 8 bytes) | 1 byte | 1 (hump start / end) |
| `obstacle_q` | Ultrasonic -> Nav | 24 bytes (enum + 5 values) | 1 byte | 3 (route / near) |
| `scan_req_q` | Nav -> Ultrasonic | 1 byte | 1 byte | 1 |
| `remote_cmd_q` | Comms -> Nav | 1 byte | 1 byte | 3 |
| `telemetry_q` | any -> Telemetry | 8 bytes | 1-4 bytes | per event kind |
| publish queue | Telemetry -> Comms | 49 bytes | 1 + 1..18 bytes | topic 1 bit + frame |
| fault mask | any -> Telemetry | separate flags | 16-bit event flag | 1 per fault |
| safety events | IMU, Nav, Comms -> Safety | shared `evt_flg` | own 4-bit flag | 1 per event |
| heartbeats | tasks -> Safety | 7 bits | 7 bits | 1 per task |
| barcode edges | ISR -> Line | 32-bit stamps | 32-bit stamps | 31-bit time + 1-bit level |

Motion command word (`motion_control.c`):

| Bits | Field | Range |
|---|---|---|
| 0-2 | command | stop, forward, backward, turn L, turn R, U-turn, self-test, motor test |
| 3-5 | sequence | 1..7 (0 = STOP, no completion) |
| 6 | follow line | flag |
| 7 | end turn on line | flag |
| 8-13 | speed | 0..63 cm/s |
| 14-22 | distance | 0..511 cm, 0 = until the next command |
| 23-31 | angle | 0..511 deg |

Message buffers (bytes of kernel memory, `depth x (4-byte header + message
rounded to 4)`):

| Buffer | Before | After |
|---|---|---|
| line position | 64 | 0 (single word) |
| barcode letters | 36 | 32 |
| motion commands | 112 | 32 |
| motion events -> done + terrain | 128 | 32 + 32 |
| obstacle results | 128 | 32 |
| scan requests | 18 | 32 |
| remote commands | 36 | 32 |
| telemetry events | 256 | 128 |
| publish queue | 1140 | 360 |
| **Total** | **1918** | **716 (-63 %)** |

## 3. RAM usage comparison

Firmware image (`arm-none-eabi-size`, same toolchain and SDK):

| | text | data | bss |
|---|---|---|---|
| Before (commit `4241f19`) | 358 616 | 1 184 | 82 616 |
| After | 362 452 | 1 184 | 82 984 |

Static RAM of the application modules went from 637 B to 991 B. This is
not shared data growing: each task's private working state (Motion, IMU and
Nav contexts, the 256-byte barcode capture buffer) moved from locals of the
task's endless loop, which lived on its fixed-size stack, to file-scope
statics (Barr-C 1.8a). The task stacks are unchanged, so the real total
grows by that amount. The data the requirement targets shrank: the shared
record from 52 to 17 bytes, queue buffers from 1 918 to 716 bytes, and the
largest messages from 49 to 19 bytes.

Reproduce: build each commit, then `arm-none-eabi-size` on the `.elf` and
on each `src/*.c.obj`.

## 4. Correctness evidence

`python web/test_proto.py` compiles the pure modules with the firmware's
flags and runs them in a Cortex-M0+ emulator:

* `proto.c` + `bitpack.c`: 408 telemetry and 698 event vectors, all
  commands and configs, against the Python reference; 300 random fields of
  1-31 bits (clamping, neighbouring bits untouched).
* `code39.c`: A-D in both directions at 15-45 cm/s with +/-10 % width
  jitter; rejects unclear wide/narrow ratios, letters outside A-D and short
  reads.
* `obstacle_profile.c`: cosine within 3/1000 for 0-180 deg; 400 random
  sweeps with missing echoes, profile and route equal to the reference.

Not yet measured on the car: PID and distance accuracy with the packed
shared record (Motion keeps float internally, so no change is expected),
heading and hump accuracy at the new resolutions, and task timing.
