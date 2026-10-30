"""Python reference codec for the packed MQTT payloads.

Mirror of src/proto.c (car) and web/static/proto.js (browser). Used by
test_proto.py to check the firmware's packer and by test_server.py to build
realistic frames. Every payload is a little-endian bit stream: the first
field starts at bit 0 of byte 0, each field LSB first, signed fields are
two's complement in their own width.
"""

# (name, bits, signed) - same order as g_telem_layout in src/proto.c
TELEM_LAYOUT = [
    ("state", 4, 0), ("motionClass", 2, 0), ("lineBits", 2, 0), ("barcode", 3, 0),
    ("faultMask", 16, 0), ("status", 2, 0),
    ("speedL", 8, 1), ("speedR", 8, 1), ("dutyL", 8, 1), ("dutyR", 8, 1),
    ("pitch", 9, 1), ("roll", 9, 1), ("heading", 9, 0), ("turnRate", 9, 1),
    ("range", 9, 0), ("distance", 14, 1), ("hump", 8, 0), ("uptime", 16, 0),
]
TELEM_LEN = 18

# kind -> (name, fields after the 4-bit kind)
EVENT_LAYOUT = {
    1: ("STATE", [("state", 4, 0)]),
    2: ("BARCODE", [("slot", 2, 0), ("valid", 1, 0), ("read", 1, 0),
                    ("edges", 6, 0)]),
    3: ("HUMP", [("height", 8, 0)]),
    4: ("OBSTACLE", [("bearing", 8, 0), ("range", 9, 0), ("width", 7, 0)]),
    5: ("FAULT", [("code", 4, 0), ("detail", 4, 0), ("state", 3, 0)]),
    6: ("TURN_XCHECK", [("encoder", 9, 0), ("imu", 9, 0)]),
    7: ("SUMMARY", [("hump", 8, 0), ("distance", 14, 1)]),
    8: ("ENC_CAL", [("result", 3, 0), ("left", 9, 1), ("right", 9, 1)]),
    9: ("MOTOR_TEST", [("phase", 2, 0), ("left", 13, 1), ("right", 13, 1)]),
    10: ("BOOT", [("hours", 5, 0), ("minutes", 6, 0), ("seconds", 6, 0)]),
    11: ("MAG_CAL", [("result", 1, 0)]),
    12: ("DEADBAND", [("wheel", 1, 0), ("deadband", 7, 0), ("kff", 8, 0)]),
    13: ("BAR_STATUS", [("black", 1, 0), ("reads", 4, 0), ("irqs", 8, 0), ("kept", 8, 0)]),
}
EVENT_KIND_BITS = 4
LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
CMDS = {"start": 0, "stop": 1, "estop": 2, "finish": 3, "reset": 4, "motor_test": 5,
        "deadband_test": 6}
CMD_TAG = 0x40


def field_range(width, signed):
    """Smallest and largest value a field can hold."""
    if signed:
        return -(1 << (width - 1)), (1 << (width - 1)) - 1
    return 0, (1 << width) - 1


def clamp(value, width, signed):
    lo, hi = field_range(width, signed)
    return max(lo, min(hi, value))


def pack(fields, values, nbytes, start=0, out=None):
    """Pack values (clamped, like the C packer) into a zeroed bytearray."""
    out = out if out is not None else bytearray(nbytes)
    pos = start
    for (_, width, signed), value in zip(fields, values):
        raw = clamp(value, width, signed) % (1 << width)
        for bit in range(width):
            if (raw >> bit) & 1:
                out[pos >> 3] |= 1 << (pos & 7)
            pos += 1
    return out


def unpack(data, fields, start=0):
    out, pos = {}, start
    for name, width, signed in fields:
        raw = 0
        for bit in range(width):
            if (data[pos >> 3] >> (pos & 7)) & 1:
                raw |= 1 << bit
            pos += 1
        out[name] = raw - (1 << width) if signed and raw >> (width - 1) else raw
    return out


def encode_telem(values):
    """values: dict keyed by TELEM_LAYOUT names (missing = 0)."""
    return bytes(pack(TELEM_LAYOUT, [values.get(n, 0) for n, _, _ in TELEM_LAYOUT],
                      TELEM_LEN))


def decode_telem(data):
    assert len(data) == TELEM_LEN, len(data)
    return unpack(data, TELEM_LAYOUT)


def event_len(kind):
    bits = EVENT_KIND_BITS + sum(w for _, w, _ in EVENT_LAYOUT[kind][1])
    return (bits + 7) // 8


def encode_event(kind, values):
    fields = EVENT_LAYOUT[kind][1]
    out = pack([("kind", EVENT_KIND_BITS, 0)], [kind], event_len(kind))
    return bytes(pack(fields, [values.get(n, 0) for n, _, _ in fields], 0,
                      start=EVENT_KIND_BITS, out=out))


def decode_event(data):
    kind = unpack(data, [("kind", EVENT_KIND_BITS, 0)])["kind"]
    assert kind in EVENT_LAYOUT and len(data) == event_len(kind), (kind, len(data))
    return kind, unpack(data, EVENT_LAYOUT[kind][1], EVENT_KIND_BITS)


def encode_config(table):
    """table: {action code 0..3: "A".."Z" or None} -> 3-byte CONFIG frame.

    Action codes: 0 straight, 1 left, 2 right, 3 U-turn. A letter may be
    given to one action only.
    """
    word = 0
    used = set()
    for n in range(4):
        letter = table.get(n)
        if letter is not None:
            assert letter in LETTERS and letter not in used, letter
            used.add(letter)
            word |= (1 << n) | (LETTERS.index(letter) << (4 + 5 * n))
    return bytes([word & 0xFF, (word >> 8) & 0xFF, word >> 16])


def config_valid(word):
    """True if a 24-bit CONFIG word passes the car's checks."""
    letters = [(word >> (4 + 5 * n)) & 31 for n in range(4) if (word >> n) & 1]
    return all(c < len(LETTERS) for c in letters) and len(letters) == len(set(letters))


BARDBG_HEADER = [("seq", 3, 0), ("chunk", 3, 0), ("last", 1, 0), ("hasLetter", 1, 0),
                 ("letter", 5, 0), ("count", 4, 0)]


def encode_bar_debug(seq, chunk, last, letter, widths):
    """One debug/barcode frame. letter: 0..25 or None; widths: 250 us units."""
    count = min(len(widths), 10)
    fields = BARDBG_HEADER + [("w", 11, 0)] * count
    values = [seq, chunk, int(last), int(letter is not None), letter or 0, count] + list(widths[:count])
    return bytes(pack(fields, values, (17 + 11 * count + 7) // 8))


def encode_cmd(name):
    return bytes([CMD_TAG | CMDS[name]])
