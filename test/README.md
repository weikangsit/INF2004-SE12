# Unit tests that run on the PC

## Run

    python run_unit_tests.py              all tests
    python run_unit_tests.py imu nav      only tests whose name contains these
    python run_unit_tests.py --cc C:/w64devkit/bin/gcc.exe
    set CAR_TEST_VERBOSE=1                also show the firmware's tm_printf text

You need a C compiler for the PC (gcc): MSYS2 or w64devkit on Windows. The
Pico ARM compiler cannot be used, because these tests run on the PC.
`python web/test_proto.py` (other tests) uses the ARM compiler and an emulator.

## What is where

| File | Tests | Firmware file under test |
|---|---|---|
| `test_line_steer.c` | UT-1 to UT-4 | `motion_control.c` `line_steer()` |
| `test_sensor_line.c` | UT-5, UT-6 | `sensor_line.c` `black_hold()`, `junction_seen()` |
| `test_imu.c` | UT-13, UT-14, UT-28, UT-29 | `imu_terrain.c` hump, impact and tilt checks |
| `test_nav.c` | UT-19, UT-20 | `nav_fsm.c` `bypass_peek_result()`, `bypass_return()` |
| `test_safety.c` | UT-26, UT-27 | `safety.c` `events_handle()`, `heartbeats_check()` |
| `test_comms.c` | UT-30, UT-31 | `comms.c` back-off and publish queue |

Test function names start with the test ID (for example `test_UT29_...`).

## How it works

* Each test file does `#include "<firmware>.c"`, so it can call static
  functions and read their private state.
* `stubs/tk/tkernel.h` and `stubs/stub_kernel.c` are a small fake of micro
  T-Kernel: event flags, message buffers (4-byte header, size rounded up to
  4, like the real one), a fake 1 MHz clock and `tm_printf()`.
* `stubs/stub_modules.c` has fake versions of the other modules. They record
  what the code asked for (`stub_modules.h`). The build leaves a fake out when
  the test builds the real module (`-DREAL_<NAME>`, set in `run_unit_tests.py`).
* `stubs/lwip/`, `stubs/cyw43_utk.h`, `stubs/mqtt_utk.h` are empty-shell
  headers so `comms.c` compiles.
* `unity/` is [Unity](https://github.com/ThrowTheSwitch/Unity) 2.7.0 (MIT).

## Notes

* The tests check the values in `src/car_config.h`. If you tune a value that a
  test names (for example `BYPASS_MAX_STEPS`), the test fails on purpose:
  change the plan and the test together.
* `CAR_SRC_DIR` points the build at a copy of `src/`. It was used to check
  that the tests fail when the firmware is broken: 26 deliberate changes
  (for example `LINE_BLACK_HOLD_MS`, `OVERTILT_HOLD_MS`, the drop-oldest line
  in `comms_publish()`) were each caught by at least one test.
