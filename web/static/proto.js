/*
 * proto.js - binary payload codec, mirror of src/proto.h on the car.
 * All multi-byte fields little-endian. Works in the browser (window.Proto)
 * and in Node (module.exports) so it can be unit-tested.
 */
(function (root) {
  "use strict";

  const T = { CONFIG: 0x01, CFG_GET: 0x02, CFG_ACK: 0x03, TELEM: 0x10, EVENT: 0x20, HB: 0x30, CMD: 0x40 };
  const ACTIONS = ["STRAIGHT", "LEFT", "RIGHT", "U-TURN"];          // wire codes 00 01 10 11
  const STATES = ["INIT", "READY", "LINE_FOLLOW", "BARCODE_ACT", "HUMP_REPORT",
                  "OBSTACLE_AVOID", "LINE_REACQUIRE", "MISSION_DONE", "FAULT"];
  const MOTION = ["stationary", "moving", "turning", "hump"];
  const FAULTS = ["none", "stall", "collision", "over-tilt", "line lost", "e-stop",
                  "missed heartbeat", "self-test", "no route", "reacquire failed", "sensor"];
  const HB_TASKS = ["line", "motion", "imu", "ultrasonic", "nav", "comms", "telemetry"];
  const EVENTS = { 1: "STATE", 2: "BARCODE", 3: "HUMP", 4: "OBSTACLE", 5: "FAULT", 6: "TURN_XCHECK", 7: "SUMMARY", 8: "ENC_CAL", 9: "MOTOR_TEST", 10: "BOOT" };
  const CMDS = { start: 0, stop: 1, estop: 2, finish: 3, reset: 4, motor_test: 5 };
  const LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";

  /** map: { A: 0..3 | null, ... } -> 12-byte CONFIG frame */
  function encodeConfig(map) {
    const b = new Uint8Array(12);
    let mask = 0;
    b[0] = T.CONFIG;
    for (let n = 0; n < 26; n++) {
      const code = map[LETTERS[n]];
      if (code === null || code === undefined) continue;
      if (!(code >= 0 && code <= 3)) throw new RangeError("action code must be 0..3");
      mask |= 1 << n;
      const bit = 2 * n;                       // always even: never straddles a byte
      b[5 + (bit >> 3)] |= code << (bit & 7);
    }
    new DataView(b.buffer).setUint32(1, mask >>> 0, true);
    return b;
  }

  function decodeConfig(u8) {
    if (u8.length !== 12 || u8[0] !== T.CONFIG) return null;
    const mask = new DataView(u8.buffer, u8.byteOffset).getUint32(1, true);
    const map = {};
    for (let n = 0; n < 26; n++) {
      const bit = 2 * n;
      map[LETTERS[n]] = (mask >>> n) & 1 ? (u8[5 + (bit >> 3)] >> (bit & 7)) & 3 : null;
    }
    return map;
  }

  function decodeTelem(u8) {
    if (u8.length !== 32 || u8[0] !== T.TELEM) return null;
    const v = new DataView(u8.buffer, u8.byteOffset, 32);
    const flags = v.getUint8(2);
    const obstacle = v.getUint16(16, true);
    return {
      state: STATES[v.getUint8(1)] || "?" + v.getUint8(1),
      lineSeen: !!(flags & 1), safetyLatched: !!(flags & 2), imuOk: !!(flags & 4), cfgLoaded: !!(flags & 8),
      motion: MOTION[v.getUint8(3)] || "?",
      speedL: v.getInt16(4, true) / 1000, speedR: v.getInt16(6, true) / 1000,        // m/s
      pitch: v.getInt16(8, true) / 10, roll: v.getInt16(10, true) / 10,              // deg
      yaw: v.getUint16(12, true) / 10, turnRate: v.getInt16(14, true) / 10,          // deg, deg/s
      obstacle: obstacle === 0xFFFF ? null : obstacle / 10,                          // cm
      dutyL: v.getInt8(18), dutyR: v.getInt8(19),                                    // %
      distance: v.getInt32(20, true) / 1000,                                         // m
      linePos: v.getInt8(24),
      barcode: v.getUint8(25) ? String.fromCharCode(v.getUint8(25)) : "",
      humpPeak: v.getUint16(26, true) / 10,                                          // mm
      uptime: v.getUint32(28, true) / 1000,                                          // s
    };
  }

  function decodeEvent(u8) {
    if (u8.length !== 8 || u8[0] !== T.EVENT) return null;
    const v = new DataView(u8.buffer, u8.byteOffset, 8);
    const kind = v.getUint8(1), code = v.getUint8(2), v1 = v.getInt16(4, true), v2 = v.getInt16(6, true);
    let text;
    switch (kind) {
      case 1: text = "state -> " + (STATES[code] || code); break;
      case 2: text = code === 63 ? "barcode unreadable (" + v2 + " edges)"
                 : "barcode " + String.fromCharCode(code) + " -> " + (v1 < 0 ? "not configured" : ACTIONS[v1]); break;
      case 3: text = "hump height " + (v1 / 10).toFixed(1) + " mm"; break;
      case 4: text = "obstacle at " + code + "\u00b0, " + (v1 / 10).toFixed(1) + " cm, width " + (v2 / 10).toFixed(1) + " cm"; break;
      case 5: text = "FAULT: " + (FAULTS[code] || code) + (code === 6 ? " (" + (HB_TASKS[v1] || v1) + ")" : ""); break;
      case 6: text = "turn check: encoder " + v1 + "\u00b0 vs IMU " + v2 + "\u00b0"; break;
      case 7: text = "run summary: hump peak " + (v1 / 10).toFixed(1) + " mm, distance " + (v2 / 100).toFixed(2) + " m"; break;
      case 8: text = code === 0
                 ? "encoder self-test ok: left counts " + (v1 > 0 ? "+" : "-") + ", right counts " + (v2 > 0 ? "+" : "-")
                 : code === 4
                 ? "self-test FAILED: car turned " + v1 + "\u00b0 when both wheels were driven forward - one motor is wired backwards (flip LEFT_FWD_ON_A or RIGHT_FWD_ON_A)"
                 : "encoder self-test FAILED: " + ["", "left", "right", "both"][code & 3] + " encoder silent (counts L " + v1 + ", R " + v2 + ")";
               break;
      case 9: {
        const ph = ["", "LEFT +", "LEFT -", "RIGHT +", "RIGHT -"][code] || code;
        const pin = { 1: "GP8/GP9", 2: "GP8/GP9", 3: "GP10/GP11", 4: "GP10/GP11" }[code] || "";
        text = "motor test " + ph + " (" + pin + ") -> encoder counts L " + v1 + ", R " + v2; break;
      }
      case 10: text = "car connected, firmware built " + String(Math.floor(v1 / 100)).padStart(2, "0") + ":" +
                      String(v1 % 100).padStart(2, "0") + ":" + String(v2).padStart(2, "0"); break;
      default: text = "event " + kind;
    }
    return { kind: EVENTS[kind] || String(kind), code, v1, v2, text, fault: kind === 5 || (kind === 8 && code !== 0) };
  }

  function decodeAck(u8) {
    if (u8.length !== 5 || u8[0] !== T.CFG_ACK) return null;
    return new DataView(u8.buffer, u8.byteOffset).getUint32(1, true);
  }

  function decodeHb(u8) {
    if (u8.length !== 5 || u8[0] !== T.HB) return null;
    return new DataView(u8.buffer, u8.byteOffset).getUint32(1, true);
  }

  function encodeCmd(name) {
    if (!(name in CMDS)) throw new RangeError("unknown command " + name);
    return Uint8Array.of(T.CMD | CMDS[name]);
  }

  /** "00000001 00110100 ..." for display */
  function bits(u8) {
    return Array.from(u8, (x) => x.toString(2).padStart(8, "0")).join(" ");
  }

  const api = { T, ACTIONS, STATES, LETTERS, CMDS, encodeConfig, decodeConfig, decodeTelem,
                decodeEvent, decodeAck, decodeHb, encodeCmd, bits };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.Proto = api;
})(typeof self !== "undefined" ? self : this);
