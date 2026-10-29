/* app.js - Group 12 robot car web app (binary MQTT only). */
(function () {
  "use strict";
  const P = window.Proto;
  const ROOT = "robot/12";
  const $ = (id) => document.getElementById(id);
  const mqtt = new MqttWs();

  let config = {};            // letter -> 0..3 | null (what the UI shows)
  let lastHb = 0, lastTlm = 0, ackMask = null, sentMask = null;

  // ------------------------------------------------------------------ config grid
  const OPTS = [["", "\u2014"], ["0", "1. Straight"], ["1", "2. Left"], ["2", "3. Right"], ["3", "4. U-turn"]];
  function buildGrid() {
    const box = $("letters");
    for (const L of P.LETTERS) {
      const d = document.createElement("div");
      d.className = "letter"; d.id = "L-" + L;
      const sel = document.createElement("select");
      sel.setAttribute("aria-label", "Action for letter " + L);
      for (const [v, t] of OPTS) sel.add(new Option(t, v));
      sel.onchange = () => { config[L] = sel.value === "" ? null : +sel.value; renderConfig(); };
      d.append(Object.assign(document.createElement("b"), { textContent: L }), sel);
      box.append(d);
    }
  }
  function renderConfig() {
    let frame;
    try { frame = P.encodeConfig(config); } catch (e) { return; }
    for (const L of P.LETTERS) {
      const d = $("L-" + L), v = config[L];
      d.querySelector("select").value = v == null ? "" : String(v);
      d.className = "letter" + (v == null ? "" : " a" + v);
      if (ackMask !== null && v != null && !((ackMask >>> (L.charCodeAt(0) - 65)) & 1)) d.classList.add("unack");
    }
    $("cfg-bits").textContent = P.bits(frame);
    localStorage.setItem("g12-config", JSON.stringify(config));
  }
  function sendConfig() {
    const frame = P.encodeConfig(config);
    sentMask = new DataView(frame.buffer).getUint32(1, true);
    mqtt.publish(ROOT + "/config/barcode", frame, true);
    $("cfg-state").textContent = "sent " + popcount(sentMask) + " letters, waiting for the car to confirm\u2026";
    logEvent("config sent (" + popcount(sentMask) + " letters)");
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
    $("t-line").textContent = !t.lineSeen && t.linePos === 0 ? "centred / none" : ["left", "centre", "right"][t.linePos + 1] + (t.lineSeen ? "" : " (searching)");
    $("t-bc").textContent = t.barcode ? t.barcode + " \u2192 " + actionName(config[t.barcode]) : "\u2013";
    $("t-hump").textContent = t.humpPeak.toFixed(1) + " mm";
    $("t-up").textContent = fmtTime(t.uptime);
    $("t-flags").textContent = [t.imuOk ? "IMU ok" : "IMU FAIL", t.safetyLatched ? "SAFETY STOP" : "safe",
                                t.cfgLoaded ? "table loaded" : "no table"].join(" \u00b7 ");
    $("tlm-bits").textContent = P.bits(raw);
    push(hist.speed, v); push(hist.pitch, t.pitch); push(hist.obst, t.obstacle == null ? NaN : t.obstacle);
  }
  const compass = (d) => ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][Math.round(d / 45) % 8];
  const actionName = (c) => (c == null ? "not configured" : P.ACTIONS[c]);
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

  mqtt.onmessage = (topic, u8, retained) => {
    const sub = topic.slice(ROOT.length + 1);
    if (sub === "telemetry") {
      const t = P.decodeTelem(u8);
      if (t) showTelem(t, u8);
    } else if (sub === "event") {
      const e = P.decodeEvent(u8);
      if (e) logEvent(e.text, e.fault);
    } else if (sub === "heartbeat") {
      if (P.decodeHb(u8) !== null) lastHb = Date.now();
    } else if (sub === "config/barcode") {
      const map = P.decodeConfig(u8);
      if (map && retained) { config = map; renderConfig(); }   // broker copy is the source of truth
    } else if (sub === "config/get") {
      if (u8.length === 1 && u8[0] === P.T.CFG_GET) {
        logEvent("car asked for the barcode table");
        if (Object.values(config).some((v) => v != null)) sendConfig();
      }
    } else if (sub === "config/ack") {
      const m = P.decodeAck(u8);
      if (m !== null) {
        ackMask = m;
        const want = new DataView(P.encodeConfig(config).buffer).getUint32(1, true);
        $("cfg-state").textContent = m === want ? "car confirmed all " + popcount(m) + " letters"
          : "car has " + popcount(m) + " letters; differs from this page";
        logEvent("car confirmed table (" + popcount(m) + " letters)");
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
  try { config = JSON.parse(localStorage.getItem("g12-config")) || {}; } catch (_) { config = {}; }
  renderConfig();
  $("url").value = localStorage.getItem("g12-url") ||
    (location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/mqtt";
  $("conn").onsubmit = (e) => { e.preventDefault(); connect(); };
  $("send-cfg").onclick = sendConfig;
  $("fill-btn").onclick = () => {
    const v = $("fill").value;
    for (const L of P.LETTERS) config[L] = v === "" ? null : +v;
    renderConfig();
  };
  document.querySelectorAll("[data-cmd]").forEach((b) => {
    b.onclick = () => { mqtt.publish(ROOT + "/cmd", P.encodeCmd(b.dataset.cmd)); logEvent("command: " + b.dataset.cmd); };
  });

  setInterval(() => {
    const age = Date.now() - lastHb, p = $("car-pill");
    p.className = "pill " + (age < 3000 ? "on" : "off");
    p.textContent = age < 3000 ? "car online" : "car offline";
    $("tlm-age").textContent = lastTlm ? "(" + ((Date.now() - lastTlm) / 1000).toFixed(1) + " s ago)" : "";
    chart("c-speed", hist.speed, "#3b9eff");
    chart("c-pitch", hist.pitch, "#d29922");
    chart("c-obst", hist.obst, "#3fb950");
  }, 250);

  connect();
})();
