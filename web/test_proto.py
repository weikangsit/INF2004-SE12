"""Checks that the car, the browser and the reference codec agree on every
packed payload (Change Requirement 1).

1. Layout parity: the bit tables in src/proto.c, web/static/proto.js and
   proto_ref.py must list the same fields, widths and signedness.
2. Firmware packer: src/proto.c and src/bitpack.c are compiled for the
   Cortex-M0+ with the firmware's flags and executed in the Unicorn
   emulator. Their output is compared with proto_ref for edge and random
   values (clamping included).
3. Other pure firmware modules run the same way: the Code 39 decoder
   (src/code39.c) on synthetic barcode edges, and the obstacle profile and
   route planner (src/obstacle_profile.c) against a Python reference.

Run:  python test_proto.py
Part 2 needs arm-none-eabi-gcc (PATH, ARM_GCC, or the Pico toolchains) and
`pip install unicorn`; it is skipped with a message if either is missing.
"""
import glob
import os
import random
import re
import shutil
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

import proto_ref as ref

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"
failures = []


def check(cond, msg):
    if not cond:
        failures.append(msg)
        print("FAIL", msg)


def report(start, msg):
    """Print PASS only if no check failed since failures had length start."""
    if len(failures) == start:
        print("PASS", msg)


# --------------------------------------------------------------- 1. parity
def c_fields(block):
    return [(int(w), 1 if s == "true" else 0)
            for w, s in re.findall(r"\{\s*(\d+)U,\s*(true|false)\s*\}", block)]


def parse_c_layouts():
    text = (SRC / "proto.c").read_text()
    telem = re.search(r"g_telem_layout\[[^\]]*\]\s*=\s*\{(.*?)\};", text, re.S).group(1)
    events = re.search(r"g_event_layout\[[^\]]*\]\s*=\s*\{(.*?)\};", text, re.S).group(1)
    barcode = re.search(r"g_barcode_layout\[[^\]]*\]\s*=\s*\{(.*?)\};", text, re.S).group(1)
    ev = {}
    for kind, line in enumerate(l for l in events.splitlines() if l.strip().startswith("{ ")):
        count = int(re.match(r"\s*\{\s*(\d+)U", line).group(1))
        ev[kind] = c_fields(line[line.index("{", line.index("{") + 1):])[:count]
    ev[2] = c_fields(barcode)
    return c_fields(telem), ev


def parse_js_layouts():
    text = (HERE / "static" / "proto.js").read_text()
    telem = re.search(r"TELEM_LAYOUT = \[(.*?)\];", text, re.S).group(1)
    to_fields = lambda s: [(int(w), int(sg)) for _, w, sg in
                           re.findall(r'\["(\w+)", (\d+), (\d)\]', s)]
    ev = {int(k): to_fields(body) for k, body in
          re.findall(r'(\d+): \["\w+", \[(.*?)\]\],', text)}
    return to_fields(telem), ev


def test_parity():
    start = len(failures)
    c_telem, c_ev = parse_c_layouts()
    js_telem, js_ev = parse_js_layouts()
    py_telem = [(w, s) for _, w, s in ref.TELEM_LAYOUT]
    py_ev = {k: [(w, s) for _, w, s in f] for k, (_, f) in ref.EVENT_LAYOUT.items()}
    check(sum(w for w, _ in c_telem) == 8 * ref.TELEM_LEN, "telemetry is not 144 bits")
    check(c_telem == py_telem, "telemetry layout: proto.c != proto_ref.py")
    check(js_telem == py_telem, "telemetry layout: proto.js != proto_ref.py")
    for kind in ref.EVENT_LAYOUT:
        check(c_ev.get(kind) == py_ev[kind], f"event {kind}: proto.c != proto_ref.py")
        check(js_ev.get(kind) == py_ev[kind], f"event {kind}: proto.js != proto_ref.py")
        check(ref.event_len(kind) <= 4, f"event {kind} longer than 4 bytes")
    report(start, "layout tables match in proto.c, proto.js and proto_ref.py")


# --------------------------------------------------------- 2. C on Cortex-M0+
HARNESS = r"""
#include <stddef.h>
#include "bitpack.h"
#include "code39.h"
#include "obstacle_profile.h"
#include "proto.h"
_Static_assert(sizeof(obstacle_ray_t) == 4, "ray size");
_Static_assert(sizeof(obstacle_limits_t) == 6, "limits size");
_Static_assert(sizeof(obstacle_profile_t) == 14, "profile size");
_Static_assert(offsetof(obstacle_profile_t, valid_rays) == 10, "valid");
/* proto_telem_t layout assumed by test_proto.py */
_Static_assert(sizeof(proto_telem_t) == 32, "proto_telem_t size");
_Static_assert(offsetof(proto_telem_t, status) == 6, "status");
_Static_assert(offsetof(proto_telem_t, speed_l_cms) == 8, "speed_l");
_Static_assert(offsetof(proto_telem_t, uptime_ticks) == 30, "uptime");
void * memset (void * p_dst, int value, size_t len)
{
    unsigned char * p_byte = p_dst;
    while (len-- > 0U) { *p_byte++ = (unsigned char) value; }
    return p_dst;
}
"""
TELEM_STRUCT = "<BBBBHBxhhhhhhHhHhHH"     # proto_telem_t on ARM EABI
CODE, RAM, RET = 0x10000000, 0x20000000, 0x30000000
DATA_OFFSET = 0xF000          # firmware statics, clear of the test buffers


def find_gcc():
    cands = [os.environ.get("ARM_GCC"), shutil.which("arm-none-eabi-gcc")]
    home = Path.home()
    cands += sorted(glob.glob(str(home / ".pico-sdk/toolchain/*/bin/arm-none-eabi-gcc*")))
    cands += glob.glob("C:/Program Files/Raspberry Pi/*/gcc-arm-none-eabi/bin/arm-none-eabi-gcc.exe")
    return next((c for c in cands if c and Path(c).exists()), None)


class Cpu:
    """Calls functions of the compiled proto.c in an emulated Cortex-M0+."""

    def __init__(self, gcc, workdir):
        from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
        from unicorn import arm_const as A
        self.A = A
        tool = lambda name: str(Path(gcc).with_name(Path(gcc).name.replace("gcc", name)))
        (workdir / "harness.c").write_text(HARNESS)
        elf = workdir / "proto_test.elf"
        subprocess.run([gcc, "-mcpu=cortex-m0plus", "-mthumb", "-mfloat-abi=soft", "-O2",
                        "-std=gnu11", "-ffreestanding", "-nostdlib", "-Wall", "-Wextra",
                        "-Werror", f"-I{SRC}", f"-Wl,-Ttext=0x{CODE:08x}",
                        # barcode_map.c's table lives in emulated RAM; the
                        # test calls barcode_map_init() instead of copying .data
                        f"-Wl,--section-start=.data=0x{RAM + DATA_OFFSET:08x}",
                        "-Wl,--entry=proto_pack_telem"] +
                       [str(SRC / f) for f in ("proto.c", "bitpack.c", "code39.c",
                                               "obstacle_profile.c", "barcode_map.c")] +
                       [str(workdir / "harness.c"), "-lgcc", "-o", str(elf)], check=True)
        binary = workdir / "proto_test.bin"
        subprocess.run([tool("objcopy"), "-O", "binary", "-R", ".data", "-R", ".bss",
                        str(elf), str(binary)], check=True)
        nm = subprocess.run([tool("nm"), str(elf)], check=True, capture_output=True, text=True)
        self.sym = {n: int(a, 16) for a, _, n in (l.split() for l in nm.stdout.splitlines()
                                                  if len(l.split()) == 3)}
        code = binary.read_bytes()
        assert len(code) < 0x10000, "code image too large: unexpected data section"
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.uc.mem_map(CODE, 0x10000)
        self.uc.mem_map(RAM, 0x10000)
        self.uc.mem_map(RET, 0x1000)
        self.uc.mem_write(CODE, code)

    def call(self, name, *args, stack_args=()):
        A, uc = self.A, self.uc
        sp = RAM + 0x10000 - 64
        for i, val in enumerate(stack_args):
            uc.mem_write(sp + 4 * i, struct.pack("<i", val))
        for reg, val in zip((A.UC_ARM_REG_R0, A.UC_ARM_REG_R1, A.UC_ARM_REG_R2,
                             A.UC_ARM_REG_R3), args):
            uc.reg_write(reg, val & 0xFFFFFFFF)
        uc.reg_write(A.UC_ARM_REG_SP, sp)
        uc.reg_write(A.UC_ARM_REG_LR, RET | 1)
        uc.emu_start(self.sym[name] | 1, RET, count=200000)
        return uc.reg_read(A.UC_ARM_REG_R0)

    def write(self, addr, data):
        self.uc.mem_write(addr, bytes(data))

    def read(self, addr, n):
        return bytes(self.uc.mem_read(addr, n))


def cdiv(a, b):
    """C integer division (truncates toward zero)."""
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b > 0) else -q


def event_fields(kind, code, v1, v2):
    """Python mirror of event_values() in proto.c."""
    if kind == 2:
        valid = 65 <= code < 69
        return [code - 65 if valid else 0, int(valid), int(v1 >= 0), v1 if v1 >= 0 else 0, v2]
    if kind == 3:
        return [cdiv(v1 + 2, 5)]
    if kind == 4:
        return [code, cdiv(v1, 10), cdiv(v2, 10)]
    if kind == 6:
        return [v1, v2]
    if kind == 7:
        return [cdiv(v1 + 2, 5), v2]
    if kind == 9:
        return [code - 1, cdiv(v1, 4), cdiv(v2, 4)]
    if kind == 10:
        return [cdiv(v1, 100), v1 - 100 * cdiv(v1, 100), v2]
    return [code, v1, v2][:len(ref.EVENT_LAYOUT[kind][1])]


def test_firmware(cpu):
    rng = random.Random(12)
    fmt_types = TELEM_STRUCT[1:].replace("x", "")
    c_range = {"B": (0, 255), "H": (0, 65535), "h": (-32768, 32767)}

    # Telemetry: limits, just past the limits, and random values.
    names = [n for n, _, _ in ref.TELEM_LAYOUT]
    vectors = []
    for (name, width, signed), t in zip(ref.TELEM_LAYOUT, fmt_types):
        lo, hi = ref.field_range(width, signed)
        tlo, thi = c_range[t]
        for v in (lo, hi, max(tlo, lo - 1), min(thi, hi + 1), tlo, thi):
            vec = {n: 0 for n in names}
            vec[name] = v
            vectors.append(vec)
    for _ in range(300):
        vectors.append({n: rng.randint(*c_range[t]) for n, t in zip(names, fmt_types)})
    bad = 0
    start = len(failures)
    for vec in vectors:
        cpu.write(RAM, struct.pack(TELEM_STRUCT, *[vec[n] for n in names]))
        n = cpu.call("proto_pack_telem", RAM, RAM + 0x100)
        frame = cpu.read(RAM + 0x100, ref.TELEM_LEN)
        want = {nm: ref.clamp(vec[nm], w, s) for nm, w, s in ref.TELEM_LAYOUT}
        if n != ref.TELEM_LEN or ref.decode_telem(frame) != want:
            bad += 1
    check(bad == 0, f"telemetry: {bad} of {len(vectors)} vectors differ")
    report(start, f"proto_pack_telem: {len(vectors)} vectors (limits, clamping, random)")

    # Events: every kind with edge and random generic inputs.
    count = 0
    start = len(failures)
    for kind in ref.EVENT_LAYOUT:
        cases = [(0, 0, 0), (255, 32767, 32767), (255, -32768, -32768)]
        cases += [(65 + i, i - 1, 7) for i in range(5)] if kind == 2 else []
        cases += [(rng.randint(0, 255), rng.randint(-32768, 32767),
                   rng.randint(-32768, 32767)) for _ in range(60)]
        for code, v1, v2 in cases:
            n = cpu.call("proto_pack_event", kind, code, v1, v2, stack_args=(RAM + 0x200,))
            got = cpu.read(RAM + 0x200, n)
            fields = ref.EVENT_LAYOUT[kind][1]
            want = ref.encode_event(kind, dict(zip([f[0] for f in fields],
                                                   event_fields(kind, code, v1, v2))))
            check(got == want, f"event {kind} {code},{v1},{v2}: {got.hex()} != {want.hex()}")
            count += 1
    check(cpu.call("proto_pack_event", 0, 0, 0, 0, stack_args=(RAM + 0x200,)) == 0,
          "unknown event kind 0 should give length 0")
    check(cpu.call("proto_pack_event", 13, 0, 0, 0, stack_args=(RAM + 0x200,)) == 0,
          "unknown event kind 13 should give length 0")
    report(start, f"proto_pack_event: {count} vectors across all {len(ref.EVENT_LAYOUT)} kinds")

    # Commands: every byte value; wrong lengths rejected.
    start = len(failures)
    for byte in range(256):
        cpu.write(RAM, [byte])
        cpu.write(RAM + 0x10, [0xEE])
        ok = cpu.call("proto_unpack_cmd", RAM, 1, RAM + 0x10) & 0xFF
        want = (byte & 0xF8) == 0x40 and (byte & 7) <= 6
        check(bool(ok) == want, f"cmd byte {byte:#04x}")
        if want:
            check(cpu.read(RAM + 0x10, 1)[0] == byte & 7, f"cmd id for {byte:#04x}")
    for length in (0, 2):
        check(cpu.call("proto_unpack_cmd", RAM, length, RAM + 0x10) & 0xFF == 0,
              f"cmd length {length} accepted")
    report(start, "proto_unpack_cmd: all 256 byte values and wrong lengths")

    # Config: reserved bits, length and one-letter-per-action checked, 12-bit
    # table returned. Every 12-bit word is tried.
    start = len(failures)
    words = list(range(0x1000, 0x2000)) + list(range(0, 0x10000, 37)) + [0x0C9F, 0x8000, 0xFFFF]
    for word in words:
        cpu.write(RAM, struct.pack("<H", word))
        ok = cpu.call("proto_unpack_config", RAM, 2, RAM + 0x10) & 0xFF
        check(bool(ok) == ref.config_valid(word), f"config {word:#06x}")
        if ref.config_valid(word):
            check(struct.unpack("<H", cpu.read(RAM + 0x10, 2))[0] == (word & 0x0FFF),
                  f"config table {word:#06x}")
    check(not ref.config_valid(0x0C9F), "old-format default table (bit 12 clear) rejected")
    check(cpu.call("proto_unpack_config", RAM, 3, RAM + 0x10) & 0xFF == 0, "config length 3")
    frame = ref.encode_config({0: "C", 1: "A", 2: "B", 3: "D"})
    check(frame == bytes.fromhex("2f1d"), "default table frame")
    dup = 0x1003 | (1 << 4) | (1 << 6)                # straight and left both 'B'
    check(not ref.config_valid(dup), "duplicate letter rejected by the reference")
    report(start, "proto_unpack_config: reserved bits, length and duplicate letters validated")

    start = len(failures)
    for table in (0, 0x0C9F, 0x0FFF, 0x0005):
        for from_mqtt in (0, 1):
            ack = cpu.call("proto_pack_cfg_ack", table, from_mqtt) & 0xFF
            check(ack == (table & 0xF) | (from_mqtt << 4), f"cfg ack {table:#x}/{from_mqtt}")
    report(start, "proto_pack_cfg_ack")


# ---------------------------------------------------- 3. other pure modules
C39 = {"*": "010010100", "A": "100001001", "B": "001001001", "C": "101001000",
       "D": "000011001", "P": "001010010"}


def c39_edges(text, narrow, wide, reverse=False, jitter=None):
    """Edge times (us) of a Code 39 symbol; bit 0 = level after the edge."""
    widths = []
    for n, ch in enumerate(text):
        widths += [wide if b == "1" else narrow for b in C39[ch]]
        if n < len(text) - 1:
            widths.append(narrow)                 # inter-character gap
    if reverse:
        widths.reverse()
    if jitter:
        widths = [max(1, int(w * jitter.uniform(0.9, 1.1))) for w in widths]
    t, edges = 100000, []
    for n, w in enumerate(widths):
        edges.append((t & ~1) | (1 - n % 2))       # black, white, black ...
        t += w
    edges.append((t & ~1) | 0)
    return edges


def test_code39(cpu):
    start = len(failures)
    rng = random.Random(3)
    cases = 0
    for idx, letter in enumerate("ABCD"):
        for rev in (False, True):
            for narrow in (3000, 10000, 20000):   # 15..45 cm/s over 3 mm
                for ratio in (2.0, 2.5, 3.0):
                    edges = c39_edges("*" + letter + "*", narrow, int(narrow * ratio),
                                      rev, rng)
                    cpu.write(RAM, struct.pack("<%dI" % len(edges), *edges))
                    got = cpu.call("code39_decode", RAM, len(edges), 15) & 0xFF
                    check(got == idx, f"code39 {letter} rev={rev} n={narrow} r={ratio}: {got}")
                    cases += 1
    bad = [("*A*", 3000, 3300), ("*P*", 3000, 7500), ("*A", 3000, 7500)]
    for text, narrow, wide in bad:          # unclear ratio, non A-D, too short
        edges = c39_edges(text, narrow, wide)
        cpu.write(RAM, struct.pack("<%dI" % len(edges), *edges))
        got = cpu.call("code39_decode", RAM, len(edges), 15) & 0xFF
        check(got == 0xFF, f"code39 must reject {text} {narrow}/{wide}: {got}")
        cases += 1
    report(start, f"code39_decode: {cases} cases (A-D both directions, 15-45 cm/s, "
                  "+/-10 % jitter, ratio/letter/length rejects)")


NO_ECHO = 0xFFFF


def ref_build(rays, lim):
    """Python reference of obstacle_profile_build()."""
    scan, half, _ = lim
    prof = dict(closest=NO_ECHO, closest_deg=90, width=0, valid=0, in_path=False,
                centre=False)
    xs = []
    left = max(range(len(rays)), key=lambda i: (rays[i][0], -i))
    right = min(range(len(rays)), key=lambda i: (rays[i][0], i))
    for ang, rng_cm in rays:
        if rng_cm == NO_ECHO:
            continue
        prof["valid"] += 1
        if abs(ang - 90) <= 15:
            prof["centre"] = True
        if rng_cm < scan:
            x = cdiv(rng_cm * ref_cos(ang), 1000)
            xs.append(x)
            if -half <= x <= half:
                prof["in_path"] = True
            if rng_cm < prof["closest"]:
                prof["closest"], prof["closest_deg"] = rng_cm, ang
    if xs:
        prof["width"] = max(xs) - min(xs)
    prof["left"], prof["right"] = rays[left][1], rays[right][1]
    return prof


def ref_sin(a):
    p = a * (180 - a)
    return cdiv(4 * p * 1000, 40500 - p)


def ref_cos(a):
    a = min(a, 180)
    return ref_sin(90 - a) if a <= 90 else -ref_sin(a - 90)


def ref_plan(p, lim, retry):
    side = lim[2]
    lk, rk = p["left"] != NO_ECHO, p["right"] != NO_ECHO
    lok, rok = lk and p["left"] >= side, rk and p["right"] >= side
    if p["valid"] == 0:
        return 5
    if not p["in_path"]:
        return 0 if p["centre"] else 5
    if lok or rok:
        if lok and rok:
            return 1 if p["left"] >= p["right"] else 2
        return 1 if lok else 2
    if not lk and not rk:
        return 5
    return 4 if retry else 3


def test_obstacle(cpu):
    import math
    start = len(failures)
    for ang in range(0, 181):
        got = struct.unpack("<i", struct.pack("<I", cpu.call("obstacle_profile_cos1000", ang)))[0]
        check(abs(got - 1000 * math.cos(math.radians(ang))) <= 3, f"cos {ang}: {got}")
    rng = random.Random(9)
    lim = (40, 12, 30)
    cpu.write(RAM + 0x400, struct.pack("<HHH", *lim))
    count = 0
    for trial in range(400):
        n = rng.randint(1, 12)
        rays = []
        for _ in range(n):
            ang = rng.randint(15, 165)
            r = rng.choice([NO_ECHO, rng.randint(2, 39), rng.randint(40, 300)])
            rays.append((ang, r))
        cpu.write(RAM, b"".join(struct.pack("<HH", a, r) for a, r in rays))
        cpu.call("obstacle_profile_build", RAM, n, RAM + 0x400, RAM + 0x500)
        c, cd, w, cl, cr, valid, inp, cen = struct.unpack("<HHHHHBBB", cpu.read(RAM + 0x500, 13))
        ref = ref_build(rays, lim)
        got = dict(closest=c, closest_deg=cd, width=w, valid=valid, in_path=bool(inp),
                   centre=bool(cen), left=cl, right=cr)
        check(got == ref, f"profile {rays}: {got} != {ref}")
        for retry in (0, 1):
            route = cpu.call("obstacle_profile_plan", RAM + 0x500, RAM + 0x400, retry) & 0xFF
            check(route == ref_plan(ref, lim, retry), f"plan {rays} retry={retry}: {route}")
        count += 1
    report(start, f"obstacle_profile: cos 0-180 deg, {count} random sweeps "
                  "(no-echo rays, profile and route vs reference)")


def test_bitpack(cpu):
    start = len(failures)
    rng = random.Random(5)
    for _ in range(300):
        width, signed = rng.randint(1, 31), rng.randint(0, 1)
        lo, hi = ref.field_range(width, signed)
        val = rng.choice([lo, hi, lo - 1, hi + 1, rng.randint(lo, hi)])
        val = max(-2**31, min(2**31 - 1, val))
        pos = rng.randint(0, 40)
        cpu.write(RAM + 0x300, [0xA5] * 12)          # dirty buffer: put must clear
        cpu.write(RAM + 0x600, struct.pack("<BB", width, signed))
        cpu.call("bitpack_put", RAM + 0x300, pos, val, RAM + 0x600)
        got = cpu.call("bitpack_get", RAM + 0x300, pos, RAM + 0x600)
        got = struct.unpack("<i", struct.pack("<I", got))[0]
        check(got == ref.clamp(val, width, signed), f"bitpack w={width} s={signed} {val}: {got}")
        before = cpu.read(RAM + 0x300, 12)
        for bit in list(range(0, pos)) + list(range(pos + width, 96)):
            check((before[bit >> 3] >> (bit & 7)) & 1 == (0xA5 >> (bit & 7)) & 1,
                  f"bitpack w={width} pos={pos} touched bit {bit}")
    report(start, "bitpack_put/get: 300 random fields (widths 1-31, clamping, "
                  "neighbouring bits untouched)")


def test_barcode_map(cpu):
    """barcode_map_get: which action a letter triggers, for the built-in
    table and for tables built by the reference encoder."""
    start = len(failures)
    names = {0: "none", 1: "straight", 2: "left", 3: "right", 4: "U-turn"}
    get = lambda letter: cpu.call("barcode_map_get", ord(letter)) & 0xFF
    cpu.call("barcode_map_init")
    for letter, want in (("A", 2), ("B", 3), ("C", 1), ("D", 4), ("E", 0), ("@", 0), ("a", 0)):
        check(get(letter) == want, f"built-in {letter} -> {names[get(letter)]}, want {names[want]}")
    rnd = random.Random(12)
    for _ in range(200):
        letters = rnd.sample(ref.LETTERS, 4)
        table = {n: (letters[n] if rnd.random() < 0.7 else None) for n in range(4)}
        frame = ref.encode_config(table)
        word = frame[0] | (frame[1] << 8)
        cpu.call("barcode_map_apply", word & 0x0FFF)
        for letter in ref.LETTERS:
            want = next((n + 1 for n in range(4) if table[n] == letter), 0)
            check(get(letter) == want, f"table {table}: {letter} -> {names[get(letter)]}")
    report(start, "barcode_map_get: built-in table and 200 random action -> letter tables")


def main():
    test_parity()
    gcc = find_gcc()
    try:
        import unicorn  # noqa: F401
    except ImportError:
        unicorn = None
    if not gcc or unicorn is None:
        print("SKIP firmware packer test:",
              "arm-none-eabi-gcc not found" if not gcc else "pip install unicorn")
    else:
        with tempfile.TemporaryDirectory() as tmp:
            cpu = Cpu(gcc, Path(tmp))
            test_firmware(cpu)
            test_bitpack(cpu)
            test_code39(cpu)
            test_obstacle(cpu)
            test_barcode_map(cpu)
    print("FAILED" if failures else "ALL PASS", f"({len(failures)} failures)")
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
