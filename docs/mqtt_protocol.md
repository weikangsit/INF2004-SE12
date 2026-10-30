# MQTT protocol - Group 12 robot car (Buddy 1)

All payloads are packed binary frames (Change Requirement 1). There is no
JSON or text. Every frame is a little-endian bit stream:

* the first field starts at bit 0 of byte 0;
* each field is written least significant bit first;
* a signed field is two's complement in its own width;
* the car clamps out-of-range values to the field range before packing.

The firmware packer is `src/proto.c`, the browser decoder is
`web/static/proto.js`, and `web/proto_ref.py` is a Python reference.
`web/test_proto.py` checks that all three have the same layouts, and runs
the compiled `proto.c` in a Cortex-M0+ emulator against the reference.

Broker: `web/server.py` (MQTT 3.1.1, TCP 1883 for the car, WebSocket
`/mqtt` on 8080 for the browser). Client id `robocar-g12`, keep-alive 10 s.

## Topics

| Topic (`robot/12/...`) | Direction | Bytes | QoS / retain | Rate |
|---|---|---|---|---|
| `telemetry` | car -> web | 18 | 0 / no | 10 Hz |
| `event` | car -> web | 1-4 | 0 / no | on change |
| `heartbeat` | car -> web | 1 | 0 / no | 1 Hz |
| `config/barcode` | web -> car | 3 | car subscribes QoS 1; web publishes **retained** | on change |
| `config/get` | car -> web | 0 | 0 / no | every 5 s until a table arrives |
| `config/ack` | car -> web | 1 | 0 / no | after each table |
| `cmd` | web -> car | 1 | car subscribes QoS 1 | on demand |

## telemetry (18 bytes = 144 bits)

| Bits | Field | Width | Signed | Unit / meaning |
|---|---|---|---|---|
| 0-3 | state | 4 | no | nav state 0..8 (INIT, READY, LINE_FOLLOW, BARCODE_ACT, HUMP_REPORT, OBSTACLE_AVOID, LINE_REACQUIRE, MISSION_DONE, FAULT) |
| 4-5 | motion class | 2 | no | 0 still, 1 moving, 2 turning, 3 hump |
| 6-7 | line sensors | 2 | no | raw digital readings, bit 0 left black, bit 1 right black (`motion_state_t.ir_black`, updated every 5 ms in every line-task state) |
| 8-10 | last barcode | 3 | no | bits 0-1 slot, bit 2 valid (0 = none yet). The slot is the table entry whose letter was read, which is also its action (00 straight, 01 left, 10 right, 11 U-turn); the web app names the letter from the table. |
| 11-26 | fault mask | 16 | no | bit n = `fault_code_t` n (below) |
| 27-28 | status | 2 | no | bit 0 safety latched, bit 1 barcode table came over MQTT |
| 29-36 | speed left | 8 | yes | cm/s (+/-1.27 m/s) |
| 37-44 | speed right | 8 | yes | cm/s |
| 45-52 | duty left | 8 | yes | percent -100..100 |
| 53-60 | duty right | 8 | yes | percent |
| 61-69 | pitch | 9 | yes | 0.5 deg (+/-90 deg) |
| 70-78 | roll | 9 | yes | 1 deg (+/-180 deg) |
| 79-87 | heading | 9 | no | 1 deg, 0..359 |
| 88-96 | turn rate | 9 | yes | deg/s (+/-255) |
| 97-105 | front range | 9 | no | cm 0..510, **511 = no valid echo** |
| 106-119 | distance | 14 | yes | cm (+/-81.9 m) |
| 120-127 | hump peak | 8 | no | 0.5 mm (0..127.5 mm) |
| 128-143 | uptime | 16 | no | 100 ms ticks (wraps after 109 min) |

### Fault mask bits (`src/fault.h`)

| Bit | Code | Meaning | Kind |
|---|---|---|---|
| 0 | FC_STALL | no encoder edges while driving | latched |
| 1 | FC_ENCODER | encoder self-test failed | latched |
| 2 | FC_IMU | IMU reads failing | live |
| 3 | FC_NO_ECHO | last front ping had no echo | live |
| 4 | FC_WIFI_LOSS | MQTT link down (never brakes the car) | live, warning |
| 5 | FC_COLLISION | IMU impact | latched |
| 6 | FC_OVERTILT | pitch or roll beyond 35 deg | latched |
| 7 | FC_LINE_LOST | line-lost timeout | latched |
| 8 | FC_ESTOP | emergency stop | latched |
| 9 | FC_HEARTBEAT | a supervised task missed its deadline (detail = task: 0 line, 1 motion, 2 IMU, 3 ultrasonic, 4 nav, 5 comms radio) | latched |
| 10 | FC_SENSOR | line sensor calibration failed | latched |
| 11 | FC_NO_ROUTE | no way around an obstacle; EVENT detail 1 = scan cannot proceed safely (too few valid echoes) | latched |
| 12 | FC_REACQUIRE | line not found after a bypass | latched |
| 13 | FC_SELFTEST | power-on self-test timed out | latched |
| 14 | FC_TELEM_LATE | telemetry task missed its deadline | warning, sent once |
| 15 | FC_SOFTWARE | impossible state reached | latched |

"Live" bits are refreshed every 100 ms by the telemetry task. Latched bits
stay set until the operator resets (`safety_clear()`).

## event (1-4 bytes)

Bits 0-3 hold the kind; the kind's fields follow from bit 4. The length is
fixed per kind, and the decoder rejects any other length.

| Kind | Name | Fields after the kind (width, s = signed) | Bytes |
|---|---|---|---|
| 1 | STATE | state 4 | 1 |
| 2 | BARCODE | slot 2, valid 1 (letter is in the table), read 1 (a letter was decoded; 0 = unreadable), edge count 6 | 2 |
| 3 | HUMP | height 8 (0.5 mm) | 2 |
| 4 | OBSTACLE | bearing 8 (deg), range 9 (cm, 511 = no echo), width 7 (cm) | 4 |
| 5 | FAULT | fault code 4, detail 4, motion state 3. Detail: missing heartbeat task, or for a stall bit 0 left wheel silent, bit 1 right wheel silent, bit 2 time limit. Motion state (stall only): 1 straight, 2 reverse, 3 turn left, 4 turn right, 5 U-turn | 2 |
| 6 | TURN_XCHECK | encoder turn 9 (deg), IMU turn 9 (deg) | 3 |
| 7 | SUMMARY | hump peak 8 (0.5 mm), distance 14s (cm) | 4 |
| 8 | ENC_CAL | result 3 (0 ok, bit0/bit1 encoder silent, 4 rotated), left 9s, right 9s | 4 |
| 9 | MOTOR_TEST | phase 2 (0..3 = L+, L-, R+, R-), left counts / 4 13s, right counts / 4 13s (+/-16 380 counts) | 4 |
| 10 | BOOT | hours 5, minutes 6, seconds 6 (firmware build time) | 3 |
| 11 | MAG_CAL | result 1 (0 calibrated, 1 failed: field barely changed) | 1 |
| 12 | DEADBAND | wheel 1 (0 left, 1 right), dead band 7 (percent, 100 = never moved), slope 8 (duty per m/s x100) | 3 |

Every EV_FAULT also sets its bit in the fault mask, and a failed ENC_CAL
sets FC_ENCODER.

## heartbeat (1 byte)

8-bit sequence number, incremented every second and wrapping. The web app
shows the car offline after 3 s without one.

## config/barcode (3 bytes, retained)

One entry per action n (0 straight, 1 left, 2 right, 3 U-turn), holding
the barcode letter (A..Z) that triggers it.

| Bits | Meaning |
|---|---|
| 0-3 | action has a letter (bit n = action n) |
| 4-23 | 5-bit letter per action, action n in bits 4+5n..8+5n: letter - 'A' (0 A .. 25 Z; 26-31 invalid) |

The car rejects a frame that is not 3 bytes long, has a letter code above
25, or gives one letter to two actions. The earlier 2-byte layouts (A..D
only) fail the length check, so a table still retained on the broker from
before is ignored rather than misread. A decoded letter that no action has
is logged and the car keeps following the line.

Barcode letters and Tribunal 1. The Tribunal specifies a barcode result
as a 2-bit symbol plus a valid bit, for 4 letters (A..D). A track uses at
most 4 letters, but they can be any of A..Z, so the 4 letters are chosen
in this table instead of being fixed. The Code 39 decoder recognises all
26 letters (digits and symbols are still rejected), then looks the letter
up here: a letter in the table becomes its 2-bit slot, any other letter is
dropped (the USB console prints it). From there on the 2-bit symbol +
valid bit of the Tribunal is used unchanged: the shared state record, the
`bar_cmd_q` message, telemetry bits 8-10 and the BARCODE event. Only this
CONFIG frame carries letters, so it needs 5 bits per letter (3 bytes).

The car starts with a built-in table (Week 6 FR2: left A, right B,
straight C, U-turn D = frame `2f 40 18`), so it can run without WiFi. A
received table replaces it completely. The car asks for one on
`config/get` (empty payload) every 5 s until a table arrives.

## config/ack (1 byte)

Bits 0-3 are the actions that have a letter in the table now in use; bit 4
is set if that table came over MQTT (clear = built-in table); bits 5-7 are
0.

## cmd (1 byte)

Bits 3-7 must be `01000` (0x40), and bits 0-2 hold the command. Any other
byte or length is ignored.

| Byte | Command | Accepted in |
|---|---|---|
| 0x40 | start | READY |
| 0x41 | stop (back to READY) | while driving |
| 0x42 | **e-stop** | any state |
| 0x43 | finish run | while driving |
| 0x44 | reset fault | FAULT, MISSION_DONE |
| 0x45 | bench motor test | READY, FAULT, MISSION_DONE |
| 0x46 | bench dead-band test | READY, FAULT, MISSION_DONE |

On an e-stop the MQTT link sets `EVT_ESTOP` itself, before passing the
command to the Navigation FSM. The Safety Monitor (priority 1) therefore
brakes on its next 50 ms check, without waiting for the Navigation task
(FR7, Tribunal 1). The link also logs the arrival time over USB, so the
time from receipt to brake can be measured.

## Buffers and overflow

| Buffer | Holds | Size | When full |
|---|---|---|---|
| publish queue (`comms.c`) | 1-byte topic + frame | 360 B: 1 s of telemetry (10 x 24 B) + 10 events (10 x 12 B) | the **oldest** frame is dropped (count logged on reconnect) |
| `telemetry_q` (events from all tasks) | packed event, 1-4 B | 16 slots (192 B) | the **oldest** event is dropped |
| `remote_cmd_q` | 1-byte command | 4 slots | the new command is dropped; an e-stop still reaches Safety through `EVT_ESTOP` |
| lwIP MQTT output ring | encoded packets | 2048 B | the frame stays queued and is retried on the next 2 ms poll |
| inbound payload | 2 B (largest valid frame) | - | longer payloads are discarded |

Publishing never blocks a control task: `comms_publish()` only copies into
the queue. Every lwIP call runs on the template's CYW43 task, through the
`mqtt_utk_start()` hook. When the link is down the hook stops being called;
after 500 ms without a call, `comms_is_online()` reports offline and
FC_WIFI_LOSS is set. Reconnection backs off 0.5 s, 1 s, 2 s, then 4 s.
