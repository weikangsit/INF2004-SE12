"""Build and run the Unity unit tests UT-1 to UT-6, UT-13, UT-14, UT-19, UT-20
and UT-26 to UT-31 of the test plan on the PC.

Each test file includes the firmware .c file it tests (so it can reach static
functions), and links fake kernel and module stubs from test/stubs/.

Run:   python run_unit_tests.py            (all tests)
       python run_unit_tests.py imu nav    (only tests whose name contains these)
       python run_unit_tests.py --cc C:/w64devkit/bin/gcc.exe
       set CAR_TEST_VERBOSE=1              (also print the firmware's tm_printf text)

Needs a C compiler for the PC (gcc). It is looked for in this order: --cc,
the CC environment variable, gcc on PATH, MSYS2 (C:/msys64/*/bin), w64devkit
(C:/w64devkit/bin or ~/w64devkit/bin). The Pico ARM compiler cannot be used:
these tests run on the PC, not on the Cortex-M0+.
"""
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = Path(os.environ.get("CAR_SRC_DIR", HERE.parent / "src"))   # CAR_SRC_DIR: test a copy
BUILD = HERE / "build"

COMMON_SOURCES = [
    HERE / "unity" / "unity.c",
    HERE / "stubs" / "stub_kernel.c",
    HERE / "stubs" / "stub_modules.c",
    SRC / "systime.c",
    SRC / "calc.c",
    SRC / "fault.c",
]

# name, test file, firmware modules built for real (no stub), extra firmware .c files
TESTS = [
    ("line_steer", "test_line_steer.c", ["MOTION"], []),
    ("sensor_line", "test_sensor_line.c", ["SENSOR_LINE"], []),
    ("imu", "test_imu.c", ["IMU"], []),
    ("nav", "test_nav.c", ["NAV"], []),
    ("safety", "test_safety.c", ["SAFETY"], []),
    ("comms", "test_comms.c", ["COMMS", "BARCODE_MAP"],
     ["proto.c", "bitpack.c", "barcode_map.c"]),
]


def find_cc(cli):
    cands = [cli, os.environ.get("CC"), shutil.which("gcc")]
    home = Path.home()
    cands += ["C:/msys64/mingw64/bin/gcc.exe", "C:/msys64/ucrt64/bin/gcc.exe",
              "C:/w64devkit/bin/gcc.exe", str(home / "w64devkit/bin/gcc.exe")]
    for c in cands:
        if c and (Path(c).exists() or shutil.which(c)):
            return c
    return None


def build(cc, name, test_file, real, extra):
    BUILD.mkdir(exist_ok=True)
    exe = BUILD / (name + ".exe")
    cmd = [cc, "-std=gnu11", "-g", "-O0", "-Wall", "-Wextra",
           "-Wno-unused-parameter", "-Wno-unused-function", "-Wno-unused-variable",
           "-Wno-unused-but-set-variable", "-Wno-sign-compare",
           "-I", str(HERE / "stubs"), "-I", str(HERE / "unity"), "-I", str(SRC),
           "-I", str(HERE)]
    cmd += ["-DREAL_" + r for r in real]
    cmd += ["-o", str(exe), str(HERE / test_file)]
    cmd += [str(s) for s in COMMON_SOURCES]
    cmd += [str(SRC / e) for e in extra]
    cmd += ["-lm"]
    res = subprocess.run(cmd, capture_output=True, text=True)
    return exe, res


def main():
    args = sys.argv[1:]
    cli = None
    if "--cc" in args:
        i = args.index("--cc")
        cli = args[i + 1]
        del args[i:i + 2]
    cc = find_cc(cli)
    if not cc:
        print("No C compiler for the PC found. Install gcc (MSYS2 or w64devkit) "
              "or pass --cc <path to gcc>.")
        return 2

    selected = [t for t in TESTS if not args or any(a in t[0] for a in args)]
    total = passed = failed = 0
    failed_builds = []
    results = []
    for name, test_file, real, extra in selected:
        exe, res = build(cc, name, test_file, real, extra)
        if res.returncode != 0:
            failed_builds.append(name)
            print(f"BUILD FAILED: {test_file}")
            print(res.stdout + res.stderr)
            continue
        run = subprocess.run([str(exe)], capture_output=True, text=True)
        for line in run.stdout.splitlines():
            m = re.match(r"^(.*?):(\d+):(\w+):(PASS|FAIL|IGNORE)(?::(.*))?$", line)
            if m:
                total += 1
                status, func, msg = m.group(4), m.group(3), m.group(5) or ""
                if status == "PASS":
                    passed += 1
                else:
                    failed += 1
                results.append((func, status, f"{Path(m.group(1)).name}:{m.group(2)} {msg}".strip()))
            elif line.strip() and not re.match(r"^-+$|^\d+ Tests|^(OK|FAIL)$", line.strip()):
                print("   ", line)
        if run.returncode != 0 and not any(r[1] == "FAIL" for r in results):
            print(f"{name}: exit code {run.returncode}")
            print(run.stdout + run.stderr)
            failed += 1

    print()
    for func, status, info in results:
        print(f"{status:5} {func}" + ("" if status == "PASS" else f"   <- {info}"))
    print()
    print(f"{passed} passed, {failed} failed, {total} run"
          + (f", build failed: {', '.join(failed_builds)}" if failed_builds else ""))
    return 1 if (failed or failed_builds) else 0


if __name__ == "__main__":
    sys.exit(main())
