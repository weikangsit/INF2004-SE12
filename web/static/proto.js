/*
 * proto.js - packed binary payload codec, mirror of src/proto.c on the car.
 * Every payload is a little-endian bit stream: the first field starts at
 * bit 0 of byte 0 and each field is stored least significant bit first;
 * signed fields are two's complement in their own width. The layout tables
 * below must match src/proto.c (checked by web/test_proto.py).
 * Works in the browser (window.Proto) and in Node (module.exports).
 */
(function (root) {
  "use strict";

  // [name, bits, signed] - same order as g_telem_layout in src/proto.c
  const TELEM_LAYOUT = [
    ["state", 4, 0], ["motionClass", 2, 0], ["lineBits", 2, 0], ["barcode", 3, 0],
    ["faultMask", 16, 0], ["status", 2, 0],
    ["speedL", 8, 1], ["speedR", 8, 1], ["dutyL", 8, 1], ["dutyR", 8, 1],
    ["pitch", 9, 1], ["roll", 9, 1], ["heading", 9, 0], ["turnRate", 9, 1],
    ["range", 9, 0], ["distance", 14, 1], ["hump", 8, 0], ["uptime", 16, 0],
  ];
  const TELEM_LEN = 18;

  // kind -> [name, fields after the 4-bit kind] - same as g_event_layout
  const EVENT_LAYOUT = {
    1: ["STATE", [["state", 4, 0]]],
    2: ["BARCODE", [["slot", 2, 0], ["valid", 1, 0], ["read", 1, 0], ["edges", 6, 0]]],
    3: ["HUMP", [["height", 8, 0]]],
    4: ["OBSTACLE", [["bearing", 8, 0], ["range", 9, 0], ["width", 7, 0]]],
    5: ["FAULT", [["code", 4, 0], ["detail", 4, 0], ["state", 3, 0]]],
    6: ["TURN_XCHECK", [["encoder", 9, 0], ["imu", 9, 0]]],
    7: ["SUMMARY", [["hump", 8, 0], ["distance", 14, 1]]],
    8: ["ENC_CAL", [["result", 3, 0], ["left", 9, 1], ["right", 9, 1]]],
    9: ["MOTOR_TEST", [["phase", 2, 0], ["left", 13, 1], ["right", 13, 1]]],
    10: ["BOOT", [["hours", 5, 0], ["minutes", 6, 0], ["seconds", 6, 0]]],
    11: ["MAG_CAL", [["result", 1, 0]]],
    12: ["DEADBAND", [["wheel", 1, 0], ["deadband", 7, 0], ["kff", 8, 0]]],
  };
  const EVENT_KIND_BITS = 4;

  const ACTIONS = ["STRAIGHT", "LEFT", "RIGHT", "U-TURN"];          // wire codes 00 01 10 11
  const STATES = ["INIT", "READY", "LINE_FOLLOW", "BARCODE_ACT", "HUMP_REPORT",
                  "OBSTACLE_AVOID", "LINE_REACQUIRE", "MISSION_DONE", "FAULT"];
  const MOTION = ["stationary", "moving", "turning", "hump"];
  // bit n of the fault mask = src/fault.h fault_code_t n
  const FAULTS = ["stall", "encoder", "IMU", "no echo", "WiFi loss", "collision", "over-tilt",
                  "line lost", "e-stop", "missed heartbeat", "line sensor", "no route",
                  "reacquire failed", "self-test", "telemetry late", "software"];
  const WARNINGS = (1 << 4) | (1 << 14);   // WiFi loss, telemetry late: reported only
  const HB_TASKS = ["line", "motion", "imu", "ultrasonic", "nav", "comms", "telemetry"];
  const FC_STALL = 0, FC_HEARTBEAT = 9, FC_NO_ROUTE = 11;
  // motion.c states that can stall (EV_FAULT stall: value2)
  const MOTION_STATES = ["stopped", "driving straight", "reversing", "turning left", "turning right", "U-turning"];
  // EV_FAULT stall detail bits (motion.c): which wheel had no encoder edges, or the time limit
  const stallReason = (d) => [d & 1 ? "LEFT wheel stopped" : "", d & 2 ? "RIGHT wheel stopped" : "",
                              d & 4 ? "time limit" : ""].filter(Boolean).join(", ");
  const CMDS = { start: 0, stop: 1, estop: 2, finish: 3, reset: 4, motor_test: 5, deadband_test: 6 };
  const COUNTS_PER_UNIT = 4;   // motor-test counts are sent divided by 4
  const CMD_TAG = 0x40;
  const LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  // firmware built-in table (barcode_map.c, Week 6 FR2), used until one arrives over MQTT:
  // action code (index into ACTIONS) -> letter
  const DEFAULT_TABLE = { 0: "C", 1: "A", 2: "B", 3: "D" };
  const RANGE_NO_ECHO = 511;
  const ST_LATCHED = 1, ST_CFG_MQTT = 2, LINE_LEFT = 1, LINE_RIGHT = 2, BAR_VALID = 4;

  /** Read `width` bits at bit `pos`, LSB first; two's complement if signed. */
  function getBits(u8, pos, width, signed) {
    let v = 0;
    for (let i = 0; i < width; i++, pos++) {
      if ((u8[pos >> 3] >> (pos & 7)) & 1) v += 2 ** i;
    }
    return signed && v >= 2 ** (width - 1) ? v - 2 ** width : v;
  }

  /** Unpack a field list starting at bit `pos` into an object. */
  function unpack(u8, pos, layout) {
    const out = {};
    for (const [name, width, signed] of layout) {
      out[name] = getBits(u8, pos, width, signed);
      pos += width;
    }
    return out;
  }

  const bitsOf = (layout) => layout.reduce((n, f) => n + f[1], 0);

  /** map: { 0: "A".."Z" | null, ... 3 } (action code -> letter) -> 3-byte CONFIG frame */
  function encodeConfig(map) {
    let word = 0, used = 0;
    for (let n = 0; n < ACTIONS.length; n++) {
      const letter = map[n];
      if (letter === null || letter === undefined || letter === "") continue;
      const code = LETTERS.indexOf(letter);
      if (code < 0 || letter.length !== 1) throw new RangeError("letter must be one of " + LETTERS);
      if ((used >> code) & 1) throw new RangeError("letter " + letter + " is given to two actions");
      used |= 1 << code;
      word |= (1 << n) | (code << (4 + 5 * n));
    }
    return Uint8Array.of(word & 0xFF, (word >> 8) & 0xFF, (word >> 16) & 0xFF);
  }

  function decodeConfig(u8) {
    if (u8.length !== 3) return null;                  // old 2-byte layouts are rejected
    const word = u8[0] | (u8[1] << 8) | (u8[2] << 16);
    const map = {};
    let used = 0;
    for (let n = 0; n < ACTIONS.length; n++) {
      if (!((word >> n) & 1)) { map[n] = null; continue; }
      const code = (word >> (4 + 5 * n)) & 31;
      if (code >= LETTERS.length || (used >> code) & 1) return null;   // the car rejects these too
      used |= 1 << code;
      map[n] = LETTERS[code];
    }
    return map;
  }

  /** Action code a letter triggers in a table, or null. */
  function actionFor(map, letter) {
    for (let n = 0; n < ACTIONS.length; n++) if (map[n] === letter) return n;
    return null;
  }

  /** Actions configured in a CONFIG frame (bit n = ACTIONS[n]). */
  const configMask = (u8) => u8[0] & 0x0F;

  function faultNames(mask) {
    return FAULTS.filter((_, n) => (mask >> n) & 1);
  }

  function decodeTelem(u8) {
    if (u8.length !== TELEM_LEN) return null;
    const r = unpack(u8, 0, TELEM_LAYOUT);
    return {
      state: STATES[r.state] || "?" + r.state,
      motion: MOTION[r.motionClass],
      // raw digital line sensors (1 bit each) and what Line/Motion make of them
      irLBlack: !!(r.lineBits & LINE_LEFT), irRBlack: !!(r.lineBits & LINE_RIGHT),
      lineSeen: (r.lineBits & (LINE_LEFT | LINE_RIGHT)) !== 0,
      linePos: r.lineBits === LINE_LEFT ? -1 : r.lineBits === LINE_RIGHT ? 1 : 0,
      // last barcode: 2-bit slot (= action) + valid; the letter comes from the table
      barcodeSlot: r.barcode & BAR_VALID ? r.barcode & 3 : null,
      faultMask: r.faultMask, faults: faultNames(r.faultMask),
      tripped: (r.faultMask & ~WARNINGS) !== 0,
      safetyLatched: !!(r.status & ST_LATCHED), cfgFromMqtt: !!(r.status & ST_CFG_MQTT),
      speedL: r.speedL / 100, speedR: r.speedR / 100,           // m/s
      dutyL: r.dutyL, dutyR: r.dutyR,                           // %
      pitch: r.pitch / 2, roll: r.roll,                         // deg
      yaw: r.heading, turnRate: r.turnRate,                     // deg, deg/s
      obstacle: r.range === RANGE_NO_ECHO ? null : r.range,     // cm
      distance: r.distance / 100,                               // m
      humpPeak: r.hump / 2,                                     // mm
      uptime: r.uptime / 10,                                    // s (wraps at 109 min)
      raw: r,
    };
  }

  /** table: the car's action -> letter table, to name the letter of a slot. */
  function eventText(kind, f, table) {
    switch (kind) {
      case 1: return "state -> " + (STATES[f.state] || f.state);
      case 2: return f.valid ? "barcode " + ((table && table[f.slot]) || "?") + " -> " + ACTIONS[f.slot]
                : f.read ? "barcode letter not in the table (the car's USB console shows which)"
                : "barcode unreadable (" + f.edges + " edges)";
      case 3: return "hump height " + (f.height / 2).toFixed(1) + " mm";
      case 4: return "obstacle at " + f.bearing + "\u00b0, " +
                (f.range === RANGE_NO_ECHO ? "no echo" : f.range + " cm") + ", width " + f.width + " cm";
      case 5: return "FAULT: " + (FAULTS[f.code] || f.code) +
                (f.code === FC_HEARTBEAT ? " (" + (HB_TASKS[f.detail] || f.detail) + ")" : "") +
                (f.code === FC_NO_ROUTE && f.detail === 1 ? " (scan cannot proceed safely: too few valid echoes)" : "") +
                (f.code === FC_STALL && f.detail ? " (" + stallReason(f.detail) + " while " +
                  (MOTION_STATES[f.state] || "state " + f.state) + ")" : "");
      case 6: return "turn check: encoder " + f.encoder + "\u00b0 vs IMU " + f.imu + "\u00b0";
      case 7: return "run summary: hump peak " + (f.hump / 2).toFixed(1) + " mm, distance " +
                (f.distance / 100).toFixed(2) + " m";
      case 8: return f.result === 0
                ? "encoder self-test ok: left counts " + (f.left > 0 ? "+" : "-") + ", right counts " + (f.right > 0 ? "+" : "-")
                : f.result === 4
                ? "self-test FAILED: car turned " + f.left + "\u00b0 when both wheels were driven forward - one motor is wired backwards (flip LEFT_FWD_ON_A or RIGHT_FWD_ON_A)"
                : "encoder self-test FAILED: " + ["", "left", "right", "both"][f.result & 3] +
                  " encoder silent (counts L " + f.left + ", R " + f.right + ")";
      case 9: {
        const ph = ["LEFT +", "LEFT -", "RIGHT +", "RIGHT -"][f.phase];
        const pin = f.phase < 2 ? "GP8/GP9" : "GP10/GP11";
        return "motor test " + ph + " (" + pin + ") -> encoder counts L " + f.left * COUNTS_PER_UNIT +
               ", R " + f.right * COUNTS_PER_UNIT;
      }
      case 10: return "car connected, firmware built " + [f.hours, f.minutes, f.seconds]
                 .map((x) => String(x).padStart(2, "0")).join(":");
      case 11: return f.result === 0 ? "magnetometer calibrated"
                 : "magnetometer calibration FAILED (field barely changed - spin the car fully)";
      case 12: {
        const side = f.wheel ? "R" : "L";
        return f.deadband >= 100 ? "dead-band test " + (f.wheel ? "RIGHT" : "LEFT") + ": wheel never moved (check motor / encoder)"
          : "dead-band test " + (f.wheel ? "RIGHT" : "LEFT") + ": dead band " + f.deadband + " %, slope " +
            (f.kff / 100).toFixed(2) + " duty per m/s -> set MOTOR_DEADBAND_" + side + " (" +
            (f.deadband / 100).toFixed(2) + "f) and PID_KFF_" + side + " (" + (f.kff / 100).toFixed(2) + "f)";
      }
      default: return "event " + kind;
    }
  }

  // debug/barcode: seq 3, chunk 3, last 1, letter valid 1, letter 5, count 4, then count x 11-bit widths
  const BARDBG_HEADER = [["seq", 3, 0], ["chunk", 3, 0], ["last", 1, 0], ["hasLetter", 1, 0],
                         ["letter", 5, 0], ["count", 4, 0]];
  const BARDBG_UNIT_MS = 0.1;

  /** One debug/barcode frame -> { seq, chunk, last, letter|null, widths (ms) }, or null. */
  function decodeBarDebug(u8) {
    if (u8.length < 3) return null;
    const h = unpack(u8, 0, BARDBG_HEADER);
    if (h.count > 10 || h.letter >= LETTERS.length || u8.length !== Math.ceil((17 + 11 * h.count) / 8)) return null;
    const widths = [];
    for (let n = 0; n < h.count; n++) widths.push(getBits(u8, 17 + 11 * n, 11, 0) * BARDBG_UNIT_MS);
    return { seq: h.seq, chunk: h.chunk, last: !!h.last, letter: h.hasLetter ? LETTERS[h.letter] : null, widths };
  }

  /** Text rows for one capture: one per frame (Code 39 character), the 3 widest
   *  elements in [brackets] and the ratio the decoder checks (narrowest wide /
   *  widest narrow, must be >= 1.5). */
  function barDebugRows(frames) {
    return frames.map((f, n) => {
      const el = f.widths.slice(0, 9);
      const wide = el.map((w, i) => [w, i]).sort((a, b) => b[0] - a[0]).slice(0, 3).map((p) => p[1]);
      const wides = el.filter((_, i) => wide.includes(i)), narrows = el.filter((_, i) => !wide.includes(i));
      const ratio = el.length === 9 ? Math.min(...wides) / Math.max(...narrows) : null;
      const cells = el.map((w, i) => (wide.includes(i) ? "[" + w.toFixed(1) + "]" : w.toFixed(1)));
      return "char " + (n + 1) + ": " + cells.join(" ") +
        (f.widths.length > 9 ? "  gap " + f.widths[9].toFixed(1) : "") +
        (ratio === null ? "  (incomplete)" : "  ratio " + ratio.toFixed(2));
    });
  }

  function decodeEvent(u8, table) {
    if (u8.length < 1) return null;
    const kind = getBits(u8, 0, EVENT_KIND_BITS, 0);
    const entry = EVENT_LAYOUT[kind];
    if (!entry || u8.length !== Math.ceil((EVENT_KIND_BITS + bitsOf(entry[1])) / 8)) return null;
    const f = unpack(u8, EVENT_KIND_BITS, entry[1]);
    return { kind: entry[0], fields: f, text: eventText(kind, f, table),
             fault: kind === 5 ? !(WARNINGS & (1 << f.code)) : (kind === 8 || kind === 11) && f.result !== 0 };
  }

  /** CFG_ACK: bits 0..3 letters in use, bit 4 table came over MQTT. */
  function decodeAck(u8) {
    if (u8.length !== 1 || (u8[0] & 0xE0)) return null;
    return { mask: u8[0] & 0x0F, fromMqtt: !!(u8[0] & 0x10) };
  }

  function decodeHb(u8) {
    return u8.length === 1 ? u8[0] : null;
  }

  function encodeCmd(name) {
    if (!(name in CMDS)) throw new RangeError("unknown command " + name);
    return Uint8Array.of(CMD_TAG | CMDS[name]);
  }

  /** "00000001 00110100 ..." for display */
  function bits(u8) {
    return Array.from(u8, (x) => x.toString(2).padStart(8, "0")).join(" ");
  }

  const api = { TELEM_LAYOUT, EVENT_LAYOUT, ACTIONS, STATES, FAULTS, LETTERS, CMDS, DEFAULT_TABLE,
                getBits, encodeConfig, decodeConfig, actionFor, configMask, decodeTelem, decodeEvent,
                decodeBarDebug, barDebugRows,
                decodeAck, decodeHb, encodeCmd, bits };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.Proto = api;
})(typeof self !== "undefined" ? self : this);
