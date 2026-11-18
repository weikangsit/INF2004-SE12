/* app.js - Group 12 robot car web app (binary MQTT only). */
(function () {
  "use strict";
  const P = window.Proto;
  const ROOT = "robot/12";
  const $ = (id) => document.getElementById(id);
  const mqtt = new MqttWs();

  /** Set an element's text if it exists (a missing id must not stop the page). */
  function setText(id, text) {
    const el = $(id);
    if (el) el.textContent = text;
    return el;
  }
  const num = (v, digits) => (Number.isFinite(v) ? v.toFixed(digits) : "–");

  // localStorage can throw (private window, storage disabled): never let it.
  const store = {
    get(key) {
      try {
        return localStorage.getItem(key);
      } catch (_) {
        return null;
      }
    },
    set(key, value) {
      try {
        localStorage.setItem(key, value);
      } catch (_) {
        /* the page still works, it just forgets */
      }
    },
  };

  let config = {}; // action code 0..3 -> letter "A".."Z" | null (what the UI shows)
  let lastHb = 0,
    lastTlm = 0,
    ackMask = null,
    sentMask = null;
  let carTable = P.DEFAULT_TABLE; // table the car is using: built-in until one came over MQTT
  let carState = ""; // nav state from the latest telemetry
  const BARCODE_STATES = ["LINE_FOLLOW", "HUMP_REPORT"]; // states that act on a barcode (nav_fsm.c)
  const CAR_TIMEOUT_MS = 3000; // no heartbeat for this long: car offline
  // Compass calibration, tracked here (no telemetry bit): the car keeps
  // its offsets only until power-off. true / false / null = unknown.
  let compassCal = null,
    lastUp = null, // car uptime (s) and wall time (ms) of the last frame
    lastUpAt = 0;
  const UPTIME_WRAP_S = 6553.6; // 16-bit uptime in 0.1 s
  const REBOOT_SLIP_S = 10; // uptime this far off the wall clock = reboot

  // ------------------------------------------------------------------ config grid
  // shown in this order; the value is the action's wire code (index into P.ACTIONS)
  const ACTION_ROWS = [
    [2, "Right"],
    [1, "Left"],
    [0, "Straight"],
    [3, "U-turn"],
  ];
  function buildGrid() {
    const box = $("letters");
    for (const [n, name] of ACTION_ROWS) {
      const d = document.createElement("div");
      d.className = "letter";
      d.id = "A-" + n;
      const sel = document.createElement("select");
      sel.setAttribute("aria-label", "Barcode letter for " + name);
      sel.add(new Option("—", ""));
      for (const L of P.LETTERS) sel.add(new Option(L, L));
      sel.onchange = () => {
        const L = sel.value === "" ? null : sel.value;
        // a letter can only trigger one action: take it away from any other action
        for (const [m] of ACTION_ROWS)
          if (m !== n && config[m] === L) config[m] = null;
        config[n] = L;
        renderConfig();
      };
      d.append(
        Object.assign(document.createElement("b"), { textContent: name }),
        sel,
      );
      box.append(d);
    }
  }
  function renderConfig() {
    let frame;
    try {
      frame = P.encodeConfig(config);
    } catch (e) {
      config = {};
      frame = P.encodeConfig(config);
    }
    for (const [n] of ACTION_ROWS) {
      const d = $("A-" + n),
        v = config[n];
      d.querySelector("select").value = v == null ? "" : v;
      d.className = "letter" + (v == null ? "" : " a" + n);
      if (ackMask !== null && v != null && !((ackMask >>> n) & 1))
        d.classList.add("unack");
    }
    setText("cfg-bits", P.bits(frame));
    store.set("g12-config-actions", JSON.stringify(config));
  }
  function sendConfig() {
    const frame = P.encodeConfig(config);
    if (!mqtt.publish(ROOT + "/config/barcode", frame, true)) {
      setText("cfg-state", "Not sent: connect to the broker first.");
      logEvent("Barcode table not sent: broker offline", { src: "you", sev: "warn" });
      return;
    }
    sentMask = P.configMask(frame);
    setText(
      "cfg-state",
      "Sent " + popcount(sentMask) + " actions, waiting for the car to confirm…",
    );
    logEvent("Barcode table sent (" + popcount(sentMask) + " actions)", { src: "you" });
  }
  const popcount = (m) => {
    let c = 0;
    for (; m; m >>>= 1) c += m & 1;
    return c;
  };

  // ------------------------------------------------------------------ telemetry
  const hist = { speed: [], pitch: [], obst: [] };
  const HIST = 150; // 15 s at 10 Hz
  function push(a, v) {
    a.push(v);
    if (a.length > HIST) a.shift();
  }

  function showCompass() {
    const el = setText(
      "t-cal",
      compassCal === true
        ? "Calibrated"
        : compassCal === false
          ? "Not calibrated"
          : "Unknown",
    );
    if (el) el.className = compassCal === true ? "ok" : compassCal === false ? "warn" : "";
  }

  // A reboot shows as an uptime that no longer follows the wall clock
  // (the 109 min wrap included); it clears the calibration.
  function compassTrack(uptime) {
    const now = Date.now();
    if (lastUp != null) {
      const elapsed = (now - lastUpAt) / 1000;
      const expected = (lastUp + elapsed) % UPTIME_WRAP_S;
      const off = Math.abs(uptime - expected);
      if (elapsed > UPTIME_WRAP_S / 2) compassCal = null; // away too long to tell
      else if (Math.min(off, UPTIME_WRAP_S - off) > REBOOT_SLIP_S) compassCal = false;
    }
    lastUp = uptime;
    lastUpAt = now;
    showCompass();
  }

  function showChips(t) {
    const box = $("t-chips");
    const chips = [];
    if (t.safetyLatched) chips.push(["Safety stop engaged", "bad"]);
    for (const name of t.faults) {
      const warnOnly = name === "WiFi loss" || name === "telemetry late";
      chips.push([name[0].toUpperCase() + name.slice(1), warnOnly ? "warn" : "bad"]);
    }
    if (!t.faults.length && !t.safetyLatched) chips.push(["No faults", ""]);
    chips.push([t.cfgFromMqtt ? "Barcode table from this page" : "Built-in barcode table", ""]);
    const key = chips.map((c) => c.join(":")).join("|");
    if (box.dataset.key === key) return; // 10 Hz: rebuild only on change
    box.dataset.key = key;
    box.replaceChildren(
      ...chips.map(([text, cls]) => {
        const li = document.createElement("li");
        li.textContent = text;
        if (cls) li.className = cls;
        return li;
      }),
    );
  }

  function showTelem(t, raw) {
    lastTlm = Date.now();
    compassTrack(t.uptime);
    const v = (t.speedL + t.speedR) / 2;

    // headline
    setText("t-state", STATE_TEXT[t.state] || t.state);
    document.body.dataset.state = t.state; // style.css colours FAULT
    carState = t.state;
    setText("t-motion", t.motion ? t.motion[0].toUpperCase() + t.motion.slice(1) : "");
    setText("t-up", "Up " + fmtTime(t.uptime));
    showChips(t);

    // line and barcode
    for (const [id, black] of [
      ["t-ir-l", t.irLBlack],
      ["t-ir-r", t.irRBlack],
    ]) {
      const el = setText(id, black ? "BLACK" : "white");
      if (el) el.classList.toggle("black", black);
    }
    const track = $("track");
    if (track) track.dataset.line = (t.irLBlack ? "l" : "") + (t.irRBlack ? "r" : "");
    setText(
      "t-line",
      t.linePos === -1
        ? "Line under the left sensor: steering left."
        : t.linePos === 1
          ? "Line under the right sensor: steering right."
          : t.lineSeen
            ? "Both sensors on black: junction."
            : "Line between the sensors: driving straight.",
    );
    carTable = t.cfgFromMqtt ? config : P.DEFAULT_TABLE;
    setText(
      "t-bc",
      t.barcodeSlot == null
        ? "None yet"
        : (carTable[t.barcodeSlot] || "?") + ", " + P.ACTIONS[t.barcodeSlot].toLowerCase(),
    );

    // obstacle
    setText("t-obst", t.obstacle == null ? "Clear" : num(t.obstacle, 0) + " cm");
    const bar = $("t-obst-bar");
    if (bar) {
      const pct = t.obstacle == null ? 0 : Math.max(0, Math.min(100, 100 - t.obstacle));
      bar.style.width = pct + "%";
      bar.style.background =
        t.obstacle != null && t.obstacle < 15
          ? "var(--bad)"
          : t.obstacle != null && t.obstacle < 30
            ? "var(--warn)"
            : "var(--ok)";
    }

    // motion
    setText("t-speed", num(v, 2) + " m/s");
    setText("t-speed-lr", "Left " + num(t.speedL, 2) + ", right " + num(t.speedR, 2));
    setText("t-duty", t.dutyL + " % / " + t.dutyR + " %");
    setText("t-dist", num(t.distance, 2) + " m");

    // heading and tilt
    setText("t-yaw", num(t.yaw, 0) + "° " + compass(t.yaw));
    setText("t-rate", "Turning " + num(t.turnRate, 0) + "°/s");
    setText("t-pitch", num(t.pitch, 1) + "°");
    setText("t-roll", "Roll " + num(t.roll, 0) + "°");
    setText(
      "t-lift",
      num(110 * Math.abs(Math.tan((t.roll * Math.PI) / 180)), 1) + " mm",
    );
    setText("t-hump", num(t.humpPeak, 1) + " mm");

    setText("tlm-bits", P.bits(raw));
    push(hist.speed, v);
    push(hist.pitch, t.pitch);
    push(hist.obst, t.obstacle == null ? NaN : t.obstacle);
  }
  const STATE_TEXT = {
    INIT: "Starting up",
    READY: "Ready",
    LINE_FOLLOW: "Following the line",
    BARCODE_ACT: "Acting on a barcode",
    HUMP_REPORT: "Crossing a hump",
    OBSTACLE_AVOID: "Going around an obstacle",
    LINE_REACQUIRE: "Finding the line",
    MISSION_DONE: "Run finished",
    FAULT: "Stopped: fault",
  };
  const compass = (d) =>
    Number.isFinite(d) ? ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][Math.round(d / 45) % 8] : "";
  const fmtTime = (s) =>
    Number.isFinite(s)
      ? Math.floor(s / 60) + ":" + String(Math.floor(s % 60)).padStart(2, "0")
      : "–";

  // ------------------------------------------------------------------ charts
  const css = getComputedStyle(document.documentElement);
  const INK = css.getPropertyValue("--ink").trim() || "#1a1e21";
  const FAINT = css.getPropertyValue("--faint").trim() || "#5b6468";
  const RULE = css.getPropertyValue("--rule-soft").trim() || "#e4e8e5";

  function chart(id, data) {
    const c = $(id);
    if (!c || !c.clientWidth) return;
    // Draw at the screen's pixel density so lines stay sharp.
    const dpr = window.devicePixelRatio || 1,
      w = c.clientWidth,
      h = c.clientHeight;
    if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) {
      c.width = Math.round(w * dpr);
      c.height = Math.round(h * dpr);
    }
    const g = c.getContext("2d");
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);
    const vals = data.filter((x) => Number.isFinite(x));
    if (vals.length < 2) {
      g.fillStyle = FAINT;
      g.font = "12px Segoe UI, sans-serif";
      g.fillText("No data yet", 0, h / 2);
      return;
    }
    let lo = Math.min(...vals),
      hi = Math.max(...vals);
    if (hi - lo < 1e-3) {
      hi += 1;
      lo -= 1;
    }
    const top = 8,
      bottom = h - 8,
      left = 34;
    const y = (x) => bottom - ((x - lo) / (hi - lo)) * (bottom - top);
    g.strokeStyle = RULE;
    g.lineWidth = 1;
    g.beginPath();
    for (const gy of [top, (top + bottom) / 2, bottom]) {
      g.moveTo(left, gy + 0.5);
      g.lineTo(w, gy + 0.5);
    }
    g.stroke();
    g.strokeStyle = INK;
    g.lineWidth = 1.75;
    g.lineJoin = "round";
    g.beginPath();
    let pen = false;
    data.forEach((x, i) => {
      if (!Number.isFinite(x)) {
        pen = false;
        return;
      }
      const px = left + (i / (HIST - 1)) * (w - left);
      pen ? g.lineTo(px, y(x)) : g.moveTo(px, y(x));
      pen = true;
    });
    g.stroke();
    g.fillStyle = FAINT;
    g.font = "11px Bahnschrift, Segoe UI, sans-serif";
    g.textBaseline = "middle";
    g.fillText(hi.toFixed(1), 0, top);
    g.fillText(lo.toFixed(1), 0, bottom);
  }

  // ------------------------------------------------------------------ events
  // Where an entry comes from: the filter buttons, in this order.
  const SOURCES = {
    nav: "Navigation",
    line: "Line and barcode",
    obstacle: "Obstacle",
    motion: "Motion",
    imu: "Heading and tilt",
    link: "Connection",
    you: "You",
  };
  const FILTERS = [["all", "All"], ["problems", "Problems"]].concat(
    Object.entries(SOURCES),
  );
  // event kind -> source; FAULT goes by its fault code (src/fault.h order)
  const EVENT_SRC = {
    STATE: "nav", BARCODE: "line", HUMP: "imu", OBSTACLE: "obstacle", TURN_XCHECK: "imu",
    SUMMARY: "nav", ENC_CAL: "motion", MOTOR_TEST: "motion", BOOT: "link", MAG_CAL: "imu",
    DEADBAND: "motion",
  };
  const FAULT_SRC = ["motion", "motion", "imu", "obstacle", "link", "imu", "imu", "line",
                     "nav", "link", "line", "obstacle", "obstacle", "nav", "link", "nav"];
  const LOG_MAX = 300;
  const REPEAT_MS = 10000; // the same line again within this: count it, do not add it
  let logFilter = "all";
  let problems = 0;

  function matches(li) {
    return (
      logFilter === "all" ||
      (logFilter === "problems" ? li.dataset.sev !== "" : li.dataset.src === logFilter)
    );
  }

  /**
   * Add one log line. opts.src: a SOURCES key; opts.sev: "fault", "warn"
   * or ""; opts.detail: extra lines shown when the line is opened.
   */
  function logEvent(text, opts = {}) {
    const src = opts.src || "link",
      sev = opts.sev || "",
      now = new Date();
    const log = $("log");
    const top = log.firstElementChild;
    if (
      !opts.detail && top && top.dataset.text === text && top.dataset.src === src &&
      now - Number(top.dataset.at) < REPEAT_MS
    ) {
      const n = Number(top.dataset.n) + 1;
      top.dataset.n = n;
      top.dataset.at = now.getTime();
      top.querySelector("time").textContent = now.toLocaleTimeString();
      let badge = top.querySelector(".times");
      if (!badge) {
        badge = Object.assign(document.createElement("span"), { className: "times" });
        top.querySelector(".msg").append(badge);
      }
      badge.textContent = "×" + n;
      return;
    }
    const li = document.createElement("li");
    li.className = [sev, src === "you" ? "you" : ""].filter(Boolean).join(" ");
    Object.assign(li.dataset, { src, sev, text, n: 1, at: now.getTime() });
    const tm = Object.assign(document.createElement("time"), {
      textContent: now.toLocaleTimeString(),
      dateTime: now.toISOString(),
    });
    const where = Object.assign(document.createElement("span"), {
      className: "src",
      textContent: SOURCES[src] || src,
    });
    const msg = Object.assign(document.createElement("span"), { className: "msg" });
    if (opts.detail) {
      // textContent only: no HTML from the car ever reaches the page
      const d = document.createElement("details");
      d.append(
        Object.assign(document.createElement("summary"), { textContent: text }),
        Object.assign(document.createElement("pre"), { textContent: opts.detail }),
      );
      msg.append(d);
    } else {
      msg.textContent = text;
    }
    li.append(tm, where, msg);
    li.hidden = !matches(li);
    log.prepend(li);
    if (sev) problems++;
    while (log.children.length > LOG_MAX) {
      if (log.lastElementChild.dataset.sev) problems--;
      log.lastElementChild.remove();
    }
    renderFilters();
  }

  function renderFilters() {
    const box = $("log-filter");
    if (!box.children.length) {
      for (const [key, label] of FILTERS) {
        const b = Object.assign(document.createElement("button"), {
          type: "button",
          textContent: label,
        });
        b.dataset.key = key;
        b.onclick = () => {
          logFilter = key;
          for (const li of $("log").children) li.hidden = !matches(li);
          renderFilters();
        };
        box.append(b);
      }
    }
    for (const b of box.children) {
      b.setAttribute("aria-pressed", String(b.dataset.key === logFilter));
      if (b.dataset.key === "problems") {
        b.classList.toggle("has-problems", problems > 0);
        let n = b.querySelector(".n");
        if (problems > 0) {
          if (!n) b.append((n = Object.assign(document.createElement("span"), { className: "n" })));
          n.textContent = problems;
        } else if (n) n.remove();
      }
    }
  }

  function saveLog() {
    const lines = [...$("log").children].reverse().map((li) => {
      const n = Number(li.dataset.n) > 1 ? " (x" + li.dataset.n + ")" : "";
      const detail = li.querySelector("pre");
      return (
        new Date(Number(li.dataset.at)).toLocaleTimeString() + "\t" +
        (SOURCES[li.dataset.src] || li.dataset.src) + "\t" +
        (li.dataset.sev ? li.dataset.sev.toUpperCase() + ": " : "") +
        li.dataset.text + n + (detail ? "\n" + detail.textContent : "")
      );
    });
    const blob = new Blob([lines.join("\n") + "\n"], { type: "text/plain" });
    const a = Object.assign(document.createElement("a"), {
      href: URL.createObjectURL(blob),
      download: "robocar-log-" + new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-") + ".txt",
    });
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  }

  // ------------------------------------------------------------------ MQTT
  // Reconnect on its own when the broker goes away (server restarted,
  // laptop asleep): 1 s, 2 s, 4 s ... 10 s apart. Not after the broker
  // refused us (wrong password): retrying cannot fix that, so the page
  // waits for Connect.
  const RETRY_MIN_MS = 1000,
    RETRY_MAX_MS = 10000;
  let retryTimer = null,
    retryMs = RETRY_MIN_MS,
    refused = false,
    brokerOnline = false;

  mqtt.onstatus = (state, why) => {
    if (state !== "online" && refused) return; // keep "wrong password" shown
    const was = brokerOnline;
    brokerOnline = state === "online";
    const p = $("broker-pill");
    p.className = "pill " + (brokerOnline ? "on" : "off");
    p.textContent = brokerOnline ? "Broker connected" : "Broker: " + why;
    if (brokerOnline) {
      retryMs = RETRY_MIN_MS;
      mqtt.subscribe(ROOT + "/#"); // retained table arrives first
      logEvent("Connected to the broker", { src: "link" });
    } else if (why === "wrong password" || why.startsWith("refused")) {
      refused = true;
      logEvent("Broker refused the connection: " + why, { src: "link", sev: "fault" });
    } else {
      if (was) logEvent("Lost the broker: " + why, { src: "link", sev: "warn" });
      if (!retryTimer) {
        // error + close: one retry
        retryTimer = setTimeout(() => {
          retryTimer = null;
          connect(true);
        }, retryMs);
        retryMs = Math.min(retryMs * 2, RETRY_MAX_MS);
      }
      p.textContent += ", retrying…";
    }
    showLinkState();
  };

  // A frame this page cannot decode almost always means the car runs a
  // different firmware version. Say so once per topic instead of dropping it.
  const badFrames = new Set();
  let badMsg = "";
  function badFrame(sub, u8, expected) {
    const msg =
      sub + " frame is " + u8.length + " bytes, this page expects " + expected +
      ": the car firmware and this page are different versions (rebuild and flash the car)";
    badMsg = msg;
    if (!badFrames.has(sub)) {
      badFrames.add(sub);
      logEvent(msg, { src: "link", sev: "fault" });
    }
  }

  // ------------------------------------------------------------------ barcode sensor status
  function showBarStatus(f) {
    const el = setText("t-bcs", f.black ? "BLACK" : "white");
    if (el) el.classList.toggle("black", !!f.black);
    const n = setText(
      "t-bcs-n",
      f.edges + " edges, " + f.glitches + " glitches removed, " + f.reads + " reads in 1 s" +
        (f.overflow ? ", edges lost (ring full)" : ""),
    );
    if (n) n.className = f.overflow ? "warn" : "";
  }

  // ------------------------------------------------------------------ barcode debug log
  const barDbg = new Map(); // seq -> frames received so far
  function barDebug(f) {
    const frames = barDbg.get(f.seq) || [];
    if (f.chunk === 0) frames.length = 0; // a new capture with a reused seq
    frames[f.chunk] = f;
    barDbg.set(f.seq, frames);
    if (!f.last) return;
    barDbg.delete(f.seq);
    const got = frames.filter(Boolean);
    const edges = got.reduce((n, x) => n + x.widths.length, 0) + 1;
    const slot = f.letter == null ? null : P.actionFor(carTable, f.letter);
    const head =
      "Barcode read: " + edges + " edges (a full *X* symbol has 30), " +
      (f.letter == null
        ? "no letter decoded"
        : "letter " + f.letter +
          (slot == null ? " (not in the table)" : ", " + P.ACTIONS[slot].toLowerCase())) +
      (got.length !== frames.length ? ", some frames lost" : "");
    const unit = f.distance
      ? "Element widths (mm of sensor travel, from the wheel encoders):"
      : "Element widths (ms; the car did not move enough to measure distance):";
    logEvent(head, {
      src: "line",
      sev: f.letter == null ? "warn" : "",
      detail: [unit].concat(P.barDebugRows(got)).join("\n"),
    });
  }

  /**
   * Log wording: the same state names as the headline, no "FAULT:" prefix
   * (the red row and the Problems filter already say it), ": " for "->".
   */
  function readable(text) {
    const state = /^state -> (\w+)$/.exec(text);
    if (state) return "State: " + (STATE_TEXT[state[1]] || state[1]);
    const t = text
      .replace(/^FAULT: /, "")
      .replace(/ -> ([A-Z][A-Z-]*)$/, (_, a) => ": " + a.toLowerCase())
      .replace(/ -> /g, ": ");
    return t[0].toUpperCase() + t.slice(1);
  }

  function showEvent(e) {
    if (e.kind === "MAG_CAL" && e.fields.result === 0) {
      compassCal = true; // a failed one keeps any earlier offsets
      showCompass();
    }
    if (e.kind === "BAR_STATUS") {
      showBarStatus(e.fields); // the line card, not the log (1 per second)
      return;
    }
    const src =
      e.kind === "FAULT" ? FAULT_SRC[e.fields.code] || "nav" : EVENT_SRC[e.kind] || "nav";
    let sev = e.fault ? "fault" : e.kind === "FAULT" ? "warn" : "";
    let text = readable(e.text);
    if (e.kind === "BARCODE" && e.fields.valid && carState && !BARCODE_STATES.includes(carState)) {
      text += " (ignored: the car acts on barcodes only while following the line, not in " +
        (STATE_TEXT[carState] || carState).toLowerCase() + ")";
      sev = sev || "warn";
    }
    logEvent(text, { src, sev });
  }

  function handleMessage(topic, u8, retained) {
    const sub = topic.slice(ROOT.length + 1);
    if (sub === "telemetry") {
      const t = P.decodeTelem(u8);
      if (t) showTelem(t, u8);
      else badFrame(sub, u8, 18);
    } else if (sub === "event") {
      const e = P.decodeEvent(u8, carTable);
      if (e) showEvent(e);
      else badFrame(sub, u8, "1-4");
    } else if (sub === "debug/barcode") {
      const f = P.decodeBarDebug(u8);
      if (f) barDebug(f);
      else badFrame(sub, u8, "3-16");
    } else if (sub === "heartbeat") {
      if (P.decodeHb(u8) !== null) lastHb = Date.now();
      else badFrame(sub, u8, 1);
    } else if (sub === "config/barcode") {
      const map = P.decodeConfig(u8);
      if (map && retained) {
        config = map;
        renderConfig();
      } // broker copy is the source of truth
    } else if (sub === "config/get") {
      if (u8.length === 0) {
        logEvent("Car asked for the barcode table", { src: "line" });
        if (Object.values(config).some((v) => v != null)) sendConfig();
      }
    } else if (sub === "config/ack") {
      const ack = P.decodeAck(u8);
      if (ack !== null) {
        const m = ack.mask;
        ackMask = m;
        const want = P.configMask(P.encodeConfig(config));
        setText(
          "cfg-state",
          !ack.fromMqtt
            ? "The car is using its built-in table."
            : m === want
              ? "The car confirmed all " + popcount(m) + " actions."
              : "The car has " + popcount(m) + " actions, different from this page.",
        );
        logEvent("Car confirmed the barcode table (" + popcount(m) + " actions)", { src: "line" });
        renderConfig();
      }
    }
  }

  // One broken frame or a page bug must not stop everything after it.
  const crashed = new Set();
  mqtt.onmessage = (topic, u8, retained) => {
    try {
      handleMessage(topic, u8, retained);
    } catch (err) {
      if (!crashed.has(topic)) {
        crashed.add(topic);
        logEvent("Could not show a message on " + topic + ": " + err.message, {
          src: "link",
          sev: "fault",
        });
      }
    }
  };

  function connect(auto = false) {
    if (!auto) {
      // Connect button / page load
      clearTimeout(retryTimer);
      retryTimer = null;
      retryMs = RETRY_MIN_MS;
    }
    refused = false;
    const id = "web-" + Math.random().toString(16).slice(2, 8);
    try {
      mqtt.connect($("url").value, {
        clientId: id,
        username: "web",
        password: $("pw").value,
      });
    } catch (err) {
      // a malformed address throws before any socket exists
      mqtt.onstatus("offline", "bad address (" + err.message + ")");
    }
    store.set("g12-url", $("url").value);
  }

  // Commands only work through the broker: say so on the buttons themselves.
  function showLinkState() {
    const carOnline = Date.now() - lastHb < CAR_TIMEOUT_MS;
    for (const b of document.querySelectorAll("[data-cmd]:not(.estop), #send-cfg")) {
      b.disabled = !brokerOnline;
      b.title = brokerOnline ? b.dataset.title || "" : "Connect to the broker first";
    }
    const stop = document.querySelector(".estop");
    stop.classList.toggle("unsent", !brokerOnline);
    setText(
      "estop-note",
      !brokerOnline
        ? "Not connected: press the car's start button (GP20) to stop it"
        : carOnline
          ? "Brakes now, in any state"
          : "Car not heard from: it may not receive this",
    );
  }

  // ------------------------------------------------------------------ wiring
  buildGrid();
  try {
    config = JSON.parse(store.get("g12-config-actions")) || {};
  } catch (_) {
    config = {};
  }
  renderConfig();
  showCompass();
  renderFilters();
  $("url").value =
    store.get("g12-url") ||
    (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/mqtt";
  $("conn").onsubmit = (e) => {
    e.preventDefault();
    connect();
  };
  $("send-cfg").onclick = sendConfig;
  $("default-btn").onclick = () => {
    config = Object.assign({}, P.DEFAULT_TABLE);
    renderConfig();
  };
  $("log-save").onclick = saveLog;
  $("log-clear").onclick = () => {
    $("log").replaceChildren();
    problems = 0;
    renderFilters();
  };
  const CMD_TEXT = {
    estop: "Emergency stop",
    start: "Start",
    stop: "Stop",
    finish: "Finish run",
    reset: "Reset fault",
    motor_test: "Motor test",
    deadband_test: "Dead-band test",
    mag_cal: "Calibrate compass",
  };
  document.querySelectorAll("[data-cmd]").forEach((b) => {
    b.dataset.title = b.title;
    b.onclick = () => {
      const name = CMD_TEXT[b.dataset.cmd] || b.dataset.cmd;
      if (mqtt.publish(ROOT + "/cmd", P.encodeCmd(b.dataset.cmd))) {
        logEvent(name + " sent", { src: "you", sev: b.dataset.cmd === "estop" ? "warn" : "" });
      } else {
        logEvent(name + " not sent: no broker connection", { src: "you", sev: "fault" });
      }
    };
  });

  setInterval(() => {
    const age = Date.now() - lastHb,
      p = $("car-pill"),
      online = age < CAR_TIMEOUT_MS;
    p.className = "pill " + (online ? "on" : "off");
    p.textContent = online ? "Car online" : "Car offline";
    const tlmAge = Date.now() - lastTlm;
    document.body.classList.toggle("stale", !lastTlm || tlmAge > CAR_TIMEOUT_MS);
    setText(
      "tlm-age",
      lastTlm
        ? tlmAge > CAR_TIMEOUT_MS
          ? "Last data " + Math.round(tlmAge / 1000) + " s ago"
          : ""
        : badMsg,
    );
    showLinkState();
    chart("c-speed", hist.speed);
    chart("c-pitch", hist.pitch);
    chart("c-obst", hist.obst);
  }, 250);

  connect();
})();
