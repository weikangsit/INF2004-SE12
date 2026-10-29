# Buddy 1 - Change Requirement 1 evidence (minimum-bit data)

Scope: the WiFi / MQTT link, telemetry and the data they share with other
tasks (`src/comms.c`, `src/telemetry.c`, `src/proto.c`, `src/fault.c`,
`src/barcode_map.c`, `web/`). The other buddies' modules keep their
current representation until they make their own change.

"Before" is commit `4241f19` on `main`. "After" is branch
`buddy1-comms-refactor`. Both were built with the same toolchain: Arm GCC
10.3.1, Pico SDK 2.2.0, the INF2004 micro T-Kernel template, `-O2`.

## 1. Data representation table

### MQTT payload fields

| Data | Valid range | Bits | Representation |
|---|---|---|---|
| Nav state | 9 states (0..8) | 4 | unsigned field, telemetry bits 0-3 |
| Motion class | 0..3 | 2 | unsigned field |
| Line sensors | 2 x black/white | 2 | one bit per digital sensor; also one 2-bit field (`ir_black`) in the shared state record instead of two `bool`s |
| Last barcode | none, A..D | 3 | 2-bit letter + valid bit |
| Faults | 16 conditions | 16 | one bit each in a shared mask (`fault.h`) |
| Stall reason | left / right wheel, time limit | 3 | bits of the FAULT event detail field |
| Stalled motion state | 5 driving states | 3 | FAULT event field |
| Status | latched, table source | 2 | one bit each |
| Wheel speed (x2) | +/-1.27 m/s, 1 cm/s | 8 | signed field. One encoder count per 10 ms is already 14 mm/s, so 1 cm/s steps lose nothing |
| Motor duty (x2) | -100..100 % | 8 | signed field |
| Pitch | +/-90 deg, 0.5 deg | 9 | signed field (hump target +/-5 mm) |
| Roll | +/-180 deg, 1 deg | 9 | signed field |
| Heading | 0..359 deg, 1 deg | 9 | unsigned field (target +/-10 deg) |
| Turn rate | +/-255 deg/s | 9 | signed field |
| Front range | 0..510 cm, no echo | 9 | unsigned field, 511 = invalid echo |
| Distance | +/-81.9 m, 1 cm | 14 | signed field |
| Hump peak | 0..127.5 mm, 0.5 mm | 8 | unsigned field |
| Uptime | 109 min, 100 ms | 16 | unsigned field (wraps) |
| Event kind | 10 kinds | 4 | first field of every event |
| Barcode action | 4 actions | 2 | 2 bits per letter in CONFIG |
| Barcode table | 4 letters | 12 | 4 valid bits + 4 x 2-bit action |
| Remote command | 6 commands | 3 | + 5-bit fixed tag for validation (one byte is the smallest MQTT payload unit) |
| Heartbeat sequence | gap detection | 8 | wraps every 256 s |

Wire fields are packed with explicit shifts and masks in `proto.c`. C
bit-fields are not used there, because their bit order is set by the
compiler (Barr-C 5.5).

### Data kept in RAM and shared between tasks

| Data | Before | After | Shared by | Safe access |
|---|---|---|---|---|
| Fault conditions | separate event-flag bits and per-module bools, no single mask | one 16-bit mask in a kernel event flag | every task -> telemetry | `tk_set_flg` / `tk_clr_flg` are atomic on both cores and from ISRs |
| Link state + flags | `enum` + 4 `bool` (5 B) | 1-byte bit-field struct `link_status_t` | CYW43 task writes; other tasks read `b_connected` | single writer; a byte read is one access |
| Inbound message state | 48 B topic string + `uint32_t` length | 1-byte bit-field `rx_status_t` (2-bit topic, 2-bit length, overflow bit) | CYW43 task only | not shared |
| Inbound buffer | 32 B | 2 B (largest valid inbound frame) | CYW43 task only | not shared |
| Unsent frame | 49 B (`sub[16]`, len, `data[32]`) | 19 B + 1 B length | CYW43 task only | not shared |
| Hook timers (x3) | 3 x `uint32_t` | 3 x `uint16_t` (intervals < 65 s, modular difference) | CYW43 task only | not shared |
| Last hook time | `uint32_t` | `uint32_t` (kept: a 16-bit value could wrap during a long outage and look recent) | read by every task | aligned 32-bit read is one access |
| Backoff delay | `uint32_t` ms | 2-bit step (500 ms << step) | CYW43 task only | not shared |
| Heartbeat sequence | `uint32_t` | `uint8_t` | CYW43 task only | not shared |
| Barcode table | 26 B + `uint32_t` mask + `bool` | one `uint16_t` (12-bit table + "from MQTT" bit) | comms writes, Nav and telemetry read | a halfword store is one access, so a reader never sees half a table |

## 2. RAM usage comparison

Static RAM (`.data` + `.bss`) per object file, from `arm-none-eabi-size`:

| Module | Before | After |
|---|---|---|
| `comms.c` | 170 B | 45 B |
| `barcode_map.c` | 31 B | 2 B |
| `telemetry.c` | 0 B | 0 B |
| `proto.c` (new) | - | 0 B |
| `fault.c` (new) | - | 4 B |
| **Buddy 1 static total** | **201 B** | **51 B (-75 %)** |

Kernel memory for Buddy 1's message buffers (`bufsz` passed to
`tk_cre_mbf`, allocated from the kernel memory pool):

| Buffer | Before | After |
|---|---|---|
| publish queue | 20 x (49 + 8) = 1140 B | 10 x 24 + 10 x 12 = 360 B |
| `telemetry_q` | 16 x (8 + 8) = 256 B | 16 x (4 + 8) = 192 B |
| `remote_cmd_q` | 4 x (1 + 8) = 36 B | 36 B (enums are already 1 byte on this ABI) |
| **Total** | **1432 B** | **588 B (-59 %)** |

The publish queue still holds 1 s of telemetry plus 10 events, the
Tribunal 1 requirement.

Whole firmware image (`arm-none-eabi-size robocar_g12.elf`):

| | text | data | bss |
|---|---|---|---|
| Before | 358 616 | 1 184 | 82 616 |
| After | 360 548 | 1 176 | 82 464 |

Static RAM drops by 160 B. The kernel memory pool is a fixed region, so the
844 B saved there shows up as free pool space rather than smaller `.bss`.
Flash grows by about 1.9 KB for the bit packer and the validation code.

How to reproduce:

```bash
cmake -S . -B build -G Ninja -DWIFI_PASSWORD=x && cmake --build build
arm-none-eabi-size build/robocar_g12.elf build/CMakeFiles/robocar_g12.dir/src/comms.c.obj
```

## 3. Message size comparison

| Message | Before | After |
|---|---|---|
| telemetry | 32 B | 18 B (-44 %) |
| event | 8 B | 1-4 B (STATE 1, BARCODE / HUMP / FAULT 2, XCHECK / BOOT 3, others 4) |
| config/barcode | 12 B | 2 B |
| config/get | 1 B | 0 B |
| config/ack | 5 B | 1 B |
| heartbeat | 5 B | 1 B |
| cmd | 1 B | 1 B (3 command bits + 5 check bits) |
| publish-queue message (inter-task) | 49 B fixed | 1 B header + 1-18 B |
| `telemetry_q` message (inter-task) | 8 B | 1-4 B |

Payload traffic at the steady 10 Hz telemetry and 1 Hz heartbeat rates
falls from 325 B/s to 181 B/s. Each MQTT PUBLISH also carries a fixed
header and the topic name, which did not change.

## 4. Correctness and performance

| Check | Result |
|---|---|
| `python web/test_proto.py` | layouts in `proto.c`, `proto.js` and `proto_ref.py` match; the compiled `proto.c` runs in a Cortex-M0+ emulator (Unicorn) for 408 telemetry vectors (every field at its limits, past its limits, and random), 635 event vectors across all 10 kinds, all 256 command bytes, and the config and ack cases. All agree with the reference codec, including clamping. A deliberately wrong field width is caught. |
| `python web/test_server.py` | the broker passes the 2-byte retained config, the 18-byte telemetry, the empty `config/get` and the 1-byte e-stop through unchanged, and keeps the retained table across a restart |
| Dashboard in a browser | a fake car published packed frames and the page showed every value correctly (speed, pitch, heading, range / no echo, distance, hump, line bits, barcode, fault names) plus every event kind. Clicking EMERGENCY STOP delivered `0x42`. |
| Build | no warnings from the project files with `-Wall -Wextra` |
| Timing | instructions executed (emulator count): `proto_pack_telem` 3 116, `proto_pack_event` 302-891, `proto_unpack_cmd` 17. At 125 MHz that is about 50 us per 100 ms telemetry period (< 0.1 % CPU) in the lowest-priority task, and under 8 us for an event in the caller's task. |

Not yet measured on the car (needs the hardware): end-to-end telemetry
latency (NFR4, 500 ms), e-stop receipt-to-brake time (FR7, 100 ms; the
receipt time is printed on the USB console), and recovery after pulling
WiFi mid-run.
