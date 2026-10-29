/*
 * mqtt.js - minimal MQTT 3.1.1 client over WebSocket (binary frames).
 * Hand-written: CONNECT, PUBLISH (QoS 0), SUBSCRIBE, PINGREQ, DISCONNECT.
 */
(function (root) {
  "use strict";
  const enc = new TextEncoder(), dec = new TextDecoder();

  function varint(n) {
    const out = [];
    do { let b = n % 128; n = Math.floor(n / 128); if (n) b |= 0x80; out.push(b); } while (n);
    return out;
  }
  function str(s) { const b = enc.encode(s); return [b.length >> 8, b.length & 255, ...b]; }
  function pkt(first, body) { return Uint8Array.from([first, ...varint(body.length), ...body]); }

  class MqttWs {
    constructor() {
      this.ws = null; this.buf = new Uint8Array(0); this.pid = 1; this.ping = null;
      this.onmessage = () => {}; this.onstatus = () => {};
    }

    connect(url, { clientId, username = "", password = "", keepalive = 30 }) {
      this.close();
      this.ws = new WebSocket(url, "mqtt");
      this.ws.binaryType = "arraybuffer";
      this.ws.onopen = () => {
        let flags = 0x02;                                   // clean session
        const pay = [...str(clientId)];
        if (username) { flags |= 0x80; pay.push(...str(username)); }
        if (password) { flags |= 0x40; pay.push(...str(password)); }
        this._send(pkt(0x10, [...str("MQTT"), 4, flags, keepalive >> 8, keepalive & 255, ...pay]));
        this.ping = setInterval(() => this._send(Uint8Array.of(0xC0, 0)), keepalive * 500);
      };
      this.ws.onmessage = (e) => this._feed(new Uint8Array(e.data));
      this.ws.onclose = () => { clearInterval(this.ping); this.onstatus("offline", "connection closed"); };
      this.ws.onerror = () => this.onstatus("offline", "cannot reach broker");
    }

    close() {
      if (this.ws) {
        clearInterval(this.ping);
        try { this._send(Uint8Array.of(0xE0, 0)); } catch (_) { /* already closed */ }
        this.ws.onclose = null; this.ws.close(); this.ws = null;
      }
    }

    subscribe(filter) {
      const id = this.pid++ & 0xFFFF || 1;
      this._send(pkt(0x82, [id >> 8, id & 255, ...str(filter), 0]));
    }

    publish(topic, bytes, retain = false) {
      this._send(pkt(0x30 | (retain ? 1 : 0), [...str(topic), ...bytes]));
    }

    _send(u8) { if (this.ws && this.ws.readyState === 1) this.ws.send(u8); }

    _feed(chunk) {
      const b = new Uint8Array(this.buf.length + chunk.length);
      b.set(this.buf); b.set(chunk, this.buf.length);
      let i = 0;
      for (;;) {
        if (b.length - i < 2) break;
        let len = 0, mult = 1, j = i + 1, done = false;
        for (let k = 0; k < 4 && j < b.length; k++, j++) {
          len += (b[j] & 0x7F) * mult; mult *= 128;
          if (!(b[j] & 0x80)) { j++; done = true; break; }
        }
        if (!done || b.length - j < len) break;
        this._packet(b[i], b.subarray(j, j + len));
        i = j + len;
      }
      this.buf = b.slice(i);
    }

    _packet(first, body) {
      const type = first >> 4;
      if (type === 2) {                                     // CONNACK
        const rc = body[1];
        this.onstatus(rc === 0 ? "online" : "offline",
          rc === 0 ? "connected" : rc === 5 ? "wrong password" : "refused (" + rc + ")");
      } else if (type === 3) {                              // PUBLISH
        const tl = (body[0] << 8) | body[1];
        const topic = dec.decode(body.subarray(2, 2 + tl));
        const off = 2 + tl + (((first >> 1) & 3) ? 2 : 0);
        this.onmessage(topic, body.slice(off), !!(first & 1));
      }
    }
  }
  root.MqttWs = MqttWs;
})(self);
