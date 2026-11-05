# Firmware modules (Tribunal 1 interface map)

All files are in `src/` and follow the Barr-C 2018 standard. Each module
creates and owns its kernel objects in `*_init()` and exposes functions;
there are no shared global variables (Barr-C 4.2.c). "SDK side" files use
Pico SDK hardware headers and no kernel calls; the others use the kernel
and no hardware headers (see `port.h`).

| Module | Owner | Task / side | Provides | Consumes |
|---|---|---|---|---|
| `comms` | Buddy 1 | CYW43 hook (template task) | publish queue, broker discovery (UDP beacon), link + radio liveness, remote commands, e-stop to Safety | telemetry frames |
| `telemetry` | Buddy 1 | Telemetry, prio 8 | events (`telemetry_post_event`), 10 Hz frame | shared record, fault mask |
| `proto`, `barcode_map` | Buddy 1 | library | packed MQTT payloads, letter (A-Z) per action table | - |
| `motion_control` | Buddy 2 | Motion, prio 3 | `motion_cmd_q`, `motion_done_q` (seq + result), speed/duty/distance | line reading, IMU heading, safety latch |
| `motor_driver` | Buddy 2 | SDK side | motor PWM, encoders, Safety motor-inhibit latch | GPIO ISR |
| `sensor_line` | Buddy 3 | Line / Barcode, prio 2 | newest 2-bit line reading (stamped), line events, black age | - |
| `barcode`, `bar_filter`, `code39` | Buddy 3 | in the Line task / libraries | table slot (0..3) of the decoded A-Z letter (`bar_cmd_q`) | raw barcode edges (GPIO ISR) |
| `imu_terrain` | Buddy 4 | IMU, prio 5 | attitude, heading, hump, class, valid / calibrated / stamp; hump events; collision / over-tilt to Safety | distance and speeds |
| `ultrasonic_scan`, `obstacle_profile` | Buddy 5 | Ultrasonic, prio 6 / library | front range (read by Nav every period), route suggestion, bypass check (never drives) | scan requests |
| `nav_fsm` | team | Nav, prio 4 | system state, all movement commands | every module above |
| `safety` | team | Safety, prio 1 | latch, heartbeat supervision, braking | heartbeats, fault events, radio liveness |
| `vehicle_state` | team | library | packed shared record (`state_mtx`) | - |
| `fault`, `systime`, `bitpack` | team | library | fault mask, time helpers, bit packing | - |
| `hal`, `hal_irq`, `i2c_drv`, `port` | team | SDK side / kernel side | GPIO ISR dispatcher, servo, LED, buttons, I2C | - |

## Tribunal 1 decisions implemented

| Decision | Where |
|---|---|
| Digital line sensors, 1 bit each, 4-combination steering | `sensor_line.c`, `motion_control.c` |
| Barcode edges only time-stamped in the ISR (bounded ring, every edge kept); chatter filtered and decoded in the task | `hal.c`, `bar_filter.c`, `barcode.c` |
| Ring overflow decided: the edge is dropped and counted, the read it belonged to is rejected, and the pin's interrupt pauses until the Line task has emptied the ring (a chattering sensor cannot starve the line reading) | `hal.c`, `barcode.c` |
| Barcode posted only after '*' start/stop check; wide/narrow ratio check per character, bars and spaces classified separately; both directions | `code39.c`, `barcode.c` |
| Speed changes during a read: bars measured by the sensor's travel (encoder counts stamped on every edge in the ISR), by time only when the car barely moved (hand swipe); a driving read ends after `BAR_QUIET_MM` of white | `hal.c`, `bar_filter.c`, `barcode.c` |
| Timestamp and decode logs saved: element widths per read (`debug/barcode`) and a sensor status report (BAR_STATUS) while `BAR_LOG` is 1 | `barcode.c`, web app |
| Newest line reading used, timestamped, older than 20 ms ignored | `sensor_line_read()` |
| IMU data timestamped, older than 60 ms ignored | `vehicle_state` stamp, `motion_control.c` |
| Safety latch blocks motor drive (inhibit flag checked around every PWM write); queued commands discarded while latched | `motor_driver.c`, `motion_control.c` |
| Every motion command has a time limit; completion with sequence number and result | `motion_control.c` (`motion_done_q`) |
| Encoders + PID kept; open-loop fallback if an encoder fails the self-test | `motion_control.c` |
| Missing echo is invalid, never a clear path; too few valid echoes, or no known clear side, stop with "scan cannot proceed safely"; a bypass check re-pings a missing echo once | `obstacle_profile.c`, `nav_fsm.c` |
| Servo mount fixed: the interim reverse-and-scan (60-120 deg) is removed; the car scans 15-165 deg where it stops and goes round on the side with more room | `nav_fsm.c`, `obstacle_profile.c`, `car_config.h` |
| Scan reports progress (heartbeat per ping); servo settle scales with travel, so a scan takes about 1.2-1.5 s | `ultrasonic_scan.c` |
| Per-task heartbeat deadlines; Telemetry warning only; WiFi loss never brakes; stalled radio task brakes | `safety.c`, `comms.c` |
| Safety woken by fault events (50 ms timeout instead of polling); remote e-stop goes straight to Safety | `safety.c`, `comms.c` |
| Binary MQTT payloads, minimum bits, documented layout | `proto.c`, `mqtt_protocol.md` |
| Line-lost timeout (neither sensor black for 3 s) only while line following | `nav_fsm.c` |

## Changes the team should record in the Tribunal decision log

* Right IR sensor on GP3 instead of GP2 (Wei Kang's hardware finding;
  Zoom log said "no pin change").
* Speeds scaled to 75 % (cruise 22 cm/s instead of NFR1's 30 cm/s).
* Line calibration failure keeps following (Nav still faults at INIT).
* Encoders are wired and used (Version C deferred this decision).
* Over-tilt must last 200 ms; humps need 10 mm of real climb; speed
  set-points ramp at 0.5 m/s^2 (all to stop motor jerks reading as
  tilt / hump / impact; NFR3's 100 ms now counts from the confirmed tilt).
* Magnetometer calibration: hold GP21 for 2 s in READY (car spins once).
* `line_pos_q` became a single stamped word (newest value only).
* Servo mount raised, so the Version C interim (reverse 40 cm, scan
  60-120 deg) is removed: the car scans 15-165 deg where it stopped. The
  scan plans CONTINUE, LEFT, RIGHT or UNSAFE and the car goes round on the
  side the scan chose; a blocked bypass leg stops with FC_NO_ROUTE detail 2.
* Obstacle detection while following reads the standby range every period
  instead of a one-shot message, so an obstacle that came into view during
  a barcode turn is still caught (Week 6 feedback).
* Step-and-check bypass (any obstacle size, team flow): 90 deg spot
  turns. Width: turn out, step BYPASS_STEP_CM, face forward and check
  (pings at PEEK_RIGHT_DEG 60, centre, PEEK_LEFT_DEG 120, hit under
  BYPASS_PEEK_CM; a missing echo is pinged again once); blocked -> step again. Length: drive along, turn
  toward the line and check; blocked -> turn back and step along again.
  The return leg is the measured sideways steps plus a margin. More
  than BYPASS_MAX_STEPS either way stops with FC_NO_ROUTE detail 3.

## Not yet verified on the car

Steering with all four sensor combinations, barcode reads at speed, PID and
distance/turn accuracy, servo settle time over 15-165 deg, heading with motors
running, hump height, scan time, e-stop receipt-to-brake time, and task
timing with everything running. `ws2812.pio.h` is generated by `pioasm` and
is not hand-edited.
