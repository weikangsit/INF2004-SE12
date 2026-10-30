/* app.js - Group 12 robot car web app (binary MQTT only). */
(function () {
  "use strict";
  const P = window.Proto;
  const ROOT = "robot/12";
  const $ = (id) => document.getElementById(id);
  const mqtt = new MqttWs();

  let config = {};            // action code 0..3 -> letter "A".."Z" | null (what the UI shows)
  let lastHb = 0, lastTlm = 0, ackMask = null, sentMask = null;
  let carTable = P.DEFAULT_TABLE;   // table the car is using: built-in until one came over MQTT

  // ------------------------------------------------------------------ config grid
  // shown in this order; the value is the action's wire code (index into P.ACTIONS)
  const ACTION_ROWS = [[2, "Right"], [1, "Left"], [0, "Straight"], [3, "U-turn"]];
  function buildGrid() {
    const box = $("letters");
    for (const [n, name] of ACTION_ROWS) {
      const d = document.createElement("div");
      d.className = "letter"; d.id = "A-" + n;
      const sel = document.createElement("select");
      sel.setAttribute("aria-label", "Barcode letter for " + name);
      sel.add(new Option("\u2014", ""));
      for (const L of P.LETTERS) sel.add(new Option(L, L));
      sel.onchange = () => {
        const L = sel.value === "" ? null : sel.value;
        // a letter can only trigger one action: take it away from any other action
        for (const [m] of ACTION_ROWS) if (m !== n && config[m] === L) config[m] = null;
        config[n] = L;
        renderConfig();
      };
      d.append(Object.assign(document.createElement("b"), { textContent: name }), sel);
      box.append(d);
    }
  }
  function renderConfig() {
    let frame;
    try { frame = P.encodeConfig(config); } catch (e) { config = {}; frame = P.encodeConfig(config); }
    for (const [n] of ACTION_ROWS) {
      const d = $("A-" + n), v = config[n];
      d.querySelector("select").value = v == null ? "" : v;
      d.className = "letter" + (v == null ? "" : " a" + n);
      if (ackMask !== null && v != null && !((ackMask >>> n) & 1)) d.classList.add("unack");
    }
    $("cfg-bits").textContent = P.bits(frame);
    localStorage.setItem("g12-config-actions", JSON.stringify(config));
  }
  function sendConfig() {
    const frame = P.encodeConfig(config);
    sentMask = P.configMask(frame);
    mqtt.publish(ROOT + "/config/barcode", frame, true);
    $("cfg-state").textContent = "sent " + popcount(sentMask) + " actions, waiting for the car to confirm\u2026";
    logEvent("config sent (" + popcount(sentMask) + " actions)");
  }
  const popcount = (m) => { let c = 0; for (; m; m >>>= 1) c += m & 1; return c; };

  // ------------------------------------------------------------------ telemetry
  const hist = { speed: [], pitch: [], obst: [] };
  const HIST = 150;   // 15 s at 10 Hz
  function push(a, v) { a.push(v); if (a.length > HIST) a.shift(); }

  function showTelem(t, raw) {
    lastTlm = Date.now();
    const v = (t.speedL + t.speedR) / 2;
    $("t-speed").textContent = v.toFixed(2) + " m/s";
    $("t-speed-lr").textContent = "L " + t.speedL.toFixed(2) + "  R " + t.speedR.toFixed(2);
    $("t-pitch").textContent = t.pitch.toFixed(1) + "\u00b0";
    $("t-roll").textContent = "roll " + t.roll.toFixed(1) + "\u00b0";
    $("t-yaw").textContent = t.yaw.toFixed(1) + "\u00b0 " + compass(t.yaw);
    $("t-rate").textContent = "turn rate " + t.turnRate.toFixed(1) + "\u00b0/s";
    $("t-obst").textContent = t.obstacle == null ? "clear" : t.obstacle.toFixed(1) + " cm";
    const pct = t.obstacle == null ? 0 : Math.max(0, Math.min(100, 100 - t.obstacle));
    const bar = $("t-obst-bar");
    bar.style.width = pct + "%";
    bar.style.background = t.obstacle != null && t.obstacle < 12 ? "var(--bad)" : t.obstacle != null && t.obstacle < 30 ? "var(--warn)" : "var(--ok)";
    $("t-state").textContent = t.state;
    $("t-motion").textContent = t.motion;
    $("t-dist").textContent = t.distance.toFixed(2) + " m";
    $("t-duty").textContent = t.dutyL + "% / " + t.dutyR + "%";
    for (const [id, black] of [["t-ir-l", t.irLBlack], ["t-ir-r", t.irRBlack]]) {
      const el = $(id);
      el.textContent = black ? "BLACK" : "white";
      el.classList.toggle("black", black);
    }
    $("t-line").textContent = t.linePos === -1 ? "left black \u2192 steer left (slow)" : t.linePos === 1 ? "right black \u2192 steer right (slow)" : t.lineSeen ? "junction (both black)" : "centred \u2192 straight";
    carTable = t.cfgFromMqtt ? config : P.DEFAULT_TABLE;
    $("t-bc").textContent = t.barcodeSlot == null ? "\u2013"
      : (carTable[t.barcodeSlot] || "?") + " \u2192 " + P.ACTIONS[t.barcodeSlot];
    $("t-hump").textContent = t.humpPeak.toFixed(1) + " mm";
    $("t-up").textContent = fmtTime(t.uptime);
    $("t-flags").textContent = [t.safetyLatched ? "SAFETY STOP" : "safe",
                                t.cfgFromMqtt ? "table from web" : "built-in table"].join(" \u00b7 ");
    const ff = $("t-faults");
    ff.textContent = t.faults.length ? t.faults.join(", ") : "none";
    ff.className = t.tripped ? "bad" : (t.faults.length ? "warn" : "");
    $("tlm-bits").textContent = P.bits(raw);
    push(hist.speed, v); push(hist.pitch, t.pitch); push(hist.obst, t.obstacle == null ? NaN : t.obstacle);
  }
  const compass = (d) => ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][Math.round(d / 45) % 8];
  const actionName = (c) => (c == null ? "no action" : P.ACTIONS[c]);
  const fmtTime = (s) => Math.floor(s / 60) + ":" + String(Math.floor(s % 60)).padStart(2, "0");

  function chart(id, data, color) {
    const c = $(id), g = c.getContext("2d"), w = c.width, h = c.height;
    g.clearRect(0, 0, w, h);
    const vals = data.filter((x) => !isNaN(x));
    if (vals.length < 2) return;
    let lo = Math.min(...vals), hi = Math.max(...vals);
    if (hi - lo < 1e-3) { hi += 1; lo -= 1; }
    g.strokeStyle = "#2a3542"; g.beginPath(); g.moveTo(0, h / 2); g.lineTo(w, h / 2); g.stroke();
    g.strokeStyle = color; g.lineWidth = 2; g.beginPath();
    let pen = false;
    data.forEach((x, i) => {
      if (isNaN(x)) { pen = false; return; }
      const px = (i / (HIST - 1)) * w, py = h - 6 - ((x - lo) / (hi - lo)) * (h - 12);
      pen ? g.lineTo(px, py) : g.moveTo(px, py); pen = true;
    });
    g.stroke();
    g.fillStyle = "#8b98a5"; g.font = "11px sans-serif";
    g.fillText(hi.toFixed(1), 4, 12); g.fillText(lo.toFixed(1), 4, h - 4);
  }

  // ------------------------------------------------------------------ events
  function logEvent(text, fault = false) {
    const li = document.createElement("li");
    if (fault) li.className = "fault";
    const tm = document.createElement("time");
    tm.textContent = new Date().toLocaleTimeString();
    li.append(tm, document.createTextNode(text));        // textContent only: no HTML injection
    const log = $("log");
    log.prepend(li);
    while (log.children.length > 200) log.lastChild.remove();
  }

  // ------------------------------------------------------------------ MQTT
  mqtt.onstatus = (state, why) => {
    const p = $("broker-pill");
    p.className = "pill " + (state === "online" ? "on" : "off");
    p.textContent = "broker " + why;
    if (state === "online") mqtt.subscribe(ROOT + "/#");   // retained table arrives first
  };

  // A frame this page cannot decode almost always means the car runs a
  // different firmware version (e.g. 32-byte telemetry from before the
  // packed protocol). Say so once per topic instead of dropping it silently.
  const badFrames = new Set();
  let badMsg = "";
  function badFrame(sub, u8, expected) {
    const msg = sub + " frame is " + u8.length + " bytes, this page expects " + expected +
                " - car firmware and web app are different versions (rebuild and flash the car)";
    badMsg = "(" + msg + ")";
    if (!badFrames.has(sub)) { badFrames.add(sub); logEvent(msg, true); }
  }

  mqtt.onmessage = (topic, u8, retained) => {
    const sub = topic.slice(ROOT.length + 1);
    if (sub === "telemetry") {
      const t = P.decodeTelem(u8);
      if (t) showTelem(t, u8); else badFrame(sub, u8, 18);
    } else if (sub === "event") {
      const e = P.decodeEvent(u8, carTable);
      if (e) logEvent(e.text, e.fault); else badFrame(sub, u8, "1-4");
    } else if (sub === "heartbeat") {
      if (P.decodeHb(u8) !== null) lastHb = Date.now(); else badFrame(sub, u8, 1);
    } else if (sub === "config/barcode") {
      const map = P.decodeConfig(u8);
      if (map && retained) { config = map; renderConfig(); }   // broker copy is the source of truth
    } else if (sub === "config/get") {
      if (u8.length === 0) {
        logEvent("car asked for the barcode table");
        if (Object.values(config).some((v) => v != null)) sendConfig();
      }
    } else if (sub === "config/ack") {
      const ack = P.decodeAck(u8);
      if (ack !== null) {
        const m = ack.mask;
        ackMask = m;
        const want = P.configMask(P.encodeConfig(config));
        $("cfg-state").textContent = !ack.fromMqtt ? "car is using its built-in table"
          : m === want ? "car confirmed all " + popcount(m) + " actions"
          : "car has " + popcount(m) + " actions; differs from this page";
        logEvent("car confirmed table (" + popcount(m) + " actions)");
        renderConfig();
      }
    }
  };

  function connect() {
    const id = "web-" + Math.random().toString(16).slice(2, 8);
    mqtt.connect($("url").value, { clientId: id, username: "web", password: $("pw").value });
    localStorage.setItem("g12-url", $("url").value);
  }

  // ------------------------------------------------------------------ wiring
  buildGrid();
  try { config = JSON.parse(localStorage.getItem("g12-config-actions")) || {}; } catch (_) { config = {}; }
  renderConfig();
  $("url").value = localStorage.getItem("g12-url") ||
    (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/mqtt";
  $("conn").onsubmit = (e) => { e.preventDefault(); connect(); };
  $("send-cfg").onclick = sendConfig;
  $("default-btn").onclick = () => { config = Object.assign({}, P.DEFAULT_TABLE); renderConfig(); };
  document.querySelectorAll("[data-cmd]").forEach((b) => {
    b.onclick = () => { mqtt.publish(ROOT + "/cmd", P.encodeCmd(b.dataset.cmd)); logEvent("command: " + b.dataset.cmd); };
  });

  setInterval(() => {
    const age = Date.now() - lastHb, p = $("car-pill");
    p.className = "pill " + (age < 3000 ? "on" : "off");
    p.textContent = age < 3000 ? "car online" : "car offline";
    $("tlm-age").textContent = lastTlm ? "(" + ((Date.now() - lastTlm) / 1000).toFixed(1) + " s ago)" : badMsg;
    chart("c-speed", hist.speed, "#3b9eff");
    chart("c-pitch", hist.pitch, "#d29922");
    chart("c-obst", hist.obst, "#3fb950");
  }, 250);

  connect();
})();
