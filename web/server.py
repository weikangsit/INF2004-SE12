#!/usr/bin/env python3
"""Group 12 robot car - MQTT 3.1.1 broker + web app server (stdlib only).

One process, two listeners:
  * MQTT over TCP        (default 127.0.0.1:1883) - for the Pico W car
  * HTTP + MQTT over WebSocket (default 127.0.0.1:8080) - web app at /,
    broker at ws://<host>:8080/mqtt (binary frames, subprotocol "mqtt")

Both listeners feed the same broker: QoS 0/1 publish (delivered at QoS 0),
+/# wildcards, retained messages (persisted to retained.bin so the car gets
its barcode table even when no browser is open), keep-alive, optional
password. Every payload is passed through untouched as bytes.

    python server.py                                  # local test only
    python server.py --mqtt-host 0.0.0.0 --password s3cret   # car on LAN

SECURITY: the MQTT port must be reachable by the car, so for real runs it
is bound to the LAN. Use --password (the car and web app must send it).
The web port stays on localhost unless you pass --web-host.
"""
from __future__ import annotations   # allows `str | None` hints on Python 3.7-3.9

import argparse
import asyncio
import base64
import hashlib
import hmac
import mimetypes
import os
import struct
import sys
import time
from pathlib import Path

STATIC = Path(__file__).resolve().parent / "static"
RETAIN_FILE = Path(__file__).resolve().parent / "retained.bin"
WS_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
MAX_PACKET = 64 * 1024

CONNECT, CONNACK, PUBLISH, PUBACK = 1, 2, 3, 4
SUBSCRIBE, SUBACK, UNSUBSCRIBE, UNSUBACK = 8, 9, 10, 11
PINGREQ, PINGRESP, DISCONNECT = 12, 13, 14


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


# --------------------------------------------------------------------------
# Wire helpers
# --------------------------------------------------------------------------
def enc_varint(n: int) -> bytes:
    out = bytearray()
    while True:
        b = n % 128
        n //= 128
        out.append(b | 0x80 if n else b)
        if not n:
            return bytes(out)


def enc_str(s: bytes) -> bytes:
    return struct.pack(">H", len(s)) + s


def packet(ptype: int, flags: int, body: bytes) -> bytes:
    return bytes([(ptype << 4) | flags]) + enc_varint(len(body)) + body


class Reader:
    """Cursor over a packet body."""

    def __init__(self, data: bytes):
        self.d, self.i = data, 0

    def u8(self) -> int:
        v = self.d[self.i]
        self.i += 1
        return v

    def u16(self) -> int:
        v = struct.unpack_from(">H", self.d, self.i)[0]
        self.i += 2
        return v

    def bin(self) -> bytes:
        n = self.u16()
        v = self.d[self.i:self.i + n]
        if len(v) != n:
            raise ValueError("truncated")
        self.i += n
        return v

    def rest(self) -> bytes:
        return self.d[self.i:]

    def more(self) -> bool:
        return self.i < len(self.d)


def topic_match(filt: str, topic: str) -> bool:
    f, t = filt.split("/"), topic.split("/")
    for i, part in enumerate(f):
        if part == "#":
            return True
        if i >= len(t) or (part != "+" and part != t[i]):
            return False
    return len(f) == len(t)


def valid_filter(f: str) -> bool:
    parts = f.split("/")
    for i, p in enumerate(parts):
        if "#" in p and (p != "#" or i != len(parts) - 1):
            return False
        if "+" in p and p != "+":
            return False
    return bool(f)


# --------------------------------------------------------------------------
# Transports: raw TCP and WebSocket expose the same read/write interface
# --------------------------------------------------------------------------
class TcpLink:
    def __init__(self, reader, writer):
        self.r, self.w = reader, writer

    async def readexactly(self, n: int) -> bytes:
        return await self.r.readexactly(n)

    async def send(self, data: bytes):
        self.w.write(data)
        await self.w.drain()

    def close(self):
        self.w.close()


class WsLink:
    """RFC 6455 binary-frame stream carrying MQTT bytes."""

    def __init__(self, reader, writer):
        self.r, self.w = reader, writer
        self.buf = bytearray()

    async def _frame(self):
        b0, b1 = await self.r.readexactly(2)
        op, masked, n = b0 & 0x0F, b1 & 0x80, b1 & 0x7F
        if n == 126:
            n = struct.unpack(">H", await self.r.readexactly(2))[0]
        elif n == 127:
            n = struct.unpack(">Q", await self.r.readexactly(8))[0]
        if n > MAX_PACKET or not masked:          # clients must mask
            raise ValueError("bad websocket frame")
        mask = await self.r.readexactly(4)
        data = bytearray(await self.r.readexactly(n))
        for i in range(n):
            data[i] ^= mask[i & 3]
        return op, bytes(data)

    async def readexactly(self, n: int) -> bytes:
        while len(self.buf) < n:
            op, data = await self._frame()
            if op in (0x0, 0x2):                  # continuation / binary
                self.buf += data
            elif op == 0x8:                       # close
                raise asyncio.IncompleteReadError(b"", n)
            elif op == 0x9:                       # ping -> pong
                await self._send_frame(0xA, data)
            elif op == 0x1:
                raise ValueError("text frames not allowed: binary only")
        out = bytes(self.buf[:n])
        del self.buf[:n]
        return out

    async def _send_frame(self, op: int, data: bytes):
        n = len(data)
        if n < 126:
            hdr = bytes([0x80 | op, n])
        elif n < 65536:
            hdr = bytes([0x80 | op, 126]) + struct.pack(">H", n)
        else:
            hdr = bytes([0x80 | op, 127]) + struct.pack(">Q", n)
        self.w.write(hdr + data)
        await self.w.drain()

    async def send(self, data: bytes):
        await self._send_frame(0x2, data)

    def close(self):
        self.w.close()


# --------------------------------------------------------------------------
# Broker core
# --------------------------------------------------------------------------
class Session:
    def __init__(self, link, peer):
        self.link, self.peer = link, peer
        self.client_id = ""
        self.subs: set[str] = set()
        self.alive = True
        self.lock = asyncio.Lock()

    async def send(self, data: bytes):
        if not self.alive:
            return
        async with self.lock:
            try:
                await self.link.send(data)
            except (ConnectionError, OSError):
                self.alive = False


class Broker:
    def __init__(self, password: str | None):
        self.password = password.encode() if password else None
        self.sessions: dict[str, Session] = {}
        self.retained: dict[str, bytes] = {}
        self._load_retained()

    # --- retained persistence (binary: u16 len, topic, u32 len, payload) --
    def _load_retained(self):
        try:
            d = RETAIN_FILE.read_bytes()
        except FileNotFoundError:
            return
        i = 0
        try:
            while i < len(d):
                (tl,) = struct.unpack_from("<H", d, i)
                topic = d[i + 2:i + 2 + tl].decode()
                i += 2 + tl
                (pl,) = struct.unpack_from("<I", d, i)
                self.retained[topic] = d[i + 4:i + 4 + pl]
                i += 4 + pl
        except (struct.error, UnicodeDecodeError):
            log("retained.bin is corrupt - ignored")
        log(f"loaded {len(self.retained)} retained message(s)")

    def _save_retained(self):
        out = bytearray()
        for t, p in self.retained.items():
            tb = t.encode()
            out += struct.pack("<H", len(tb)) + tb + struct.pack("<I", len(p)) + p
        tmp = RETAIN_FILE.with_suffix(".tmp")
        tmp.write_bytes(out)
        os.replace(tmp, RETAIN_FILE)

    # --- routing --------------------------------------------------------
    async def publish(self, topic: str, payload: bytes, retain: bool):
        if retain:
            if payload:
                self.retained[topic] = payload
            else:
                self.retained.pop(topic, None)    # empty retained = delete
            self._save_retained()
        pkt = packet(PUBLISH, 0, enc_str(topic.encode()) + payload)
        for s in list(self.sessions.values()):
            if any(topic_match(f, topic) for f in s.subs):
                await s.send(pkt)

    async def serve(self, link, peer: str):
        s = Session(link, peer)
        try:
            await self._run(s)
        except (asyncio.IncompleteReadError, ConnectionError, OSError):
            pass
        except (ValueError, IndexError, struct.error, UnicodeDecodeError) as e:
            log(f"{peer}: protocol error: {e}")
        finally:
            s.alive = False
            if s.client_id and self.sessions.get(s.client_id) is s:
                del self.sessions[s.client_id]
                log(f"{s.client_id} disconnected")
            link.close()

    async def _read_packet(self, link, timeout):
        b0 = (await asyncio.wait_for(link.readexactly(1), timeout))[0]
        mult, n = 1, 0
        for _ in range(4):
            b = (await link.readexactly(1))[0]
            n += (b & 0x7F) * mult
            if not b & 0x80:
                break
            mult *= 128
        else:
            raise ValueError("bad remaining length")
        if n > MAX_PACKET:
            raise ValueError("packet too large")
        body = await link.readexactly(n) if n else b""
        return b0 >> 4, b0 & 0x0F, body

    async def _run(self, s: Session):
        ptype, _, body = await self._read_packet(s.link, 10)
        if ptype != CONNECT:
            raise ValueError("first packet must be CONNECT")
        r = Reader(body)
        proto, level, flags, keepalive = r.bin(), r.u8(), r.u8(), r.u16()
        cid = r.bin().decode()
        if flags & 0x04:                      # will topic + message (ignored)
            r.bin(), r.bin()
        user = r.bin() if flags & 0x80 else b""
        pw = r.bin() if flags & 0x40 else b""
        if proto not in (b"MQTT", b"MQIsdp") or level not in (3, 4):
            await s.link.send(packet(CONNACK, 0, b"\x00\x01"))
            return
        if self.password and not hmac.compare_digest(pw, self.password):
            log(f"{s.peer}: bad password (user {user!r})")
            await s.link.send(packet(CONNACK, 0, b"\x00\x05"))
            return
        cid = cid or f"anon-{id(s):x}"
        old = self.sessions.get(cid)
        if old:                               # client takeover
            old.alive = False
            old.link.close()
        s.client_id = cid
        self.sessions[cid] = s
        await s.send(packet(CONNACK, 0, b"\x00\x00"))
        log(f"{cid} connected from {s.peer}")

        timeout = keepalive * 1.5 if keepalive else None
        while s.alive:
            ptype, flags, body = await self._read_packet(s.link, timeout)
            r = Reader(body)
            if ptype == PUBLISH:
                qos, retain = (flags >> 1) & 3, bool(flags & 1)
                topic = r.bin().decode()
                if "+" in topic or "#" in topic or not topic:
                    raise ValueError("bad publish topic")
                pid = r.u16() if qos else None
                await self.publish(topic, r.rest(), retain)
                if qos == 1:
                    await s.send(packet(PUBACK, 0, struct.pack(">H", pid)))
            elif ptype == SUBSCRIBE:
                pid, granted, new = r.u16(), bytearray(), []
                while r.more():
                    f = r.bin().decode()
                    r.u8()                    # requested QoS
                    if valid_filter(f):
                        s.subs.add(f)
                        new.append(f)
                        granted.append(0)     # delivered at QoS 0
                    else:
                        granted.append(0x80)
                await s.send(packet(SUBACK, 0, struct.pack(">H", pid) + granted))
                for t, p in list(self.retained.items()):
                    if any(topic_match(f, t) for f in new):
                        await s.send(packet(PUBLISH, 1, enc_str(t.encode()) + p))
            elif ptype == UNSUBSCRIBE:
                pid = r.u16()
                while r.more():
                    s.subs.discard(r.bin().decode())
                await s.send(packet(UNSUBACK, 0, struct.pack(">H", pid)))
            elif ptype == PINGREQ:
                await s.send(packet(PINGRESP, 0, b""))
            elif ptype == DISCONNECT:
                return
            elif ptype == PUBACK:
                pass
            else:
                raise ValueError(f"unsupported packet type {ptype}")


# --------------------------------------------------------------------------
# HTTP / WebSocket front end
# --------------------------------------------------------------------------
async def handle_http(broker: Broker, reader, writer):
    peer = "%s:%s" % writer.get_extra_info("peername")[:2]
    try:
        head = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), 10)
    except (asyncio.TimeoutError, asyncio.IncompleteReadError, asyncio.LimitOverrunError):
        writer.close()
        return
    lines = head.decode("latin-1").split("\r\n")
    try:
        method, path, _ = lines[0].split(" ", 2)
    except ValueError:
        writer.close()
        return
    hdr = {}
    for ln in lines[1:]:
        if ":" in ln:
            k, v = ln.split(":", 1)
            hdr[k.strip().lower()] = v.strip()

    if path == "/mqtt" and hdr.get("upgrade", "").lower() == "websocket":
        # Refuse cross-site pages talking to this broker from a browser
        origin, host = hdr.get("origin", ""), hdr.get("host", "")
        if origin and origin.split("://", 1)[-1] != host:
            writer.write(b"HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\n\r\n")
            await writer.drain()
            writer.close()
            return
        key = hdr.get("sec-websocket-key", "").encode()
        accept = base64.b64encode(hashlib.sha1(key + WS_GUID).digest()).decode()
        proto = "Sec-WebSocket-Protocol: mqtt\r\n" if "mqtt" in hdr.get(
            "sec-websocket-protocol", "") else ""
        writer.write((
            "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
            f"Connection: Upgrade\r\nSec-WebSocket-Accept: {accept}\r\n{proto}\r\n"
        ).encode())
        await writer.drain()
        await broker.serve(WsLink(reader, writer), "ws " + peer)
        return

    # static files
    rel = "index.html" if path in ("/", "") else path.split("?", 1)[0].lstrip("/")
    target = (STATIC / rel).resolve()
    if method != "GET" or STATIC not in target.parents or not target.is_file():
        writer.write(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n")
    else:
        data = target.read_bytes()
        ctype = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        if target.suffix == ".js":
            ctype = "text/javascript"
        writer.write((
            f"HTTP/1.1 200 OK\r\nContent-Type: {ctype}\r\nContent-Length: {len(data)}\r\n"
            "Cache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\n\r\n"
        ).encode() + data)
    await writer.drain()
    writer.close()


async def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--mqtt-host", default="127.0.0.1",
                    help="MQTT/TCP bind address; 0.0.0.0 so the car can reach it")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--web-host", default="127.0.0.1")
    ap.add_argument("--web-port", type=int, default=8080)
    ap.add_argument("--password", default=os.environ.get("ROBOCAR_MQTT_PASSWORD"),
                    help="require this MQTT password (or env ROBOCAR_MQTT_PASSWORD)")
    a = ap.parse_args()

    broker = Broker(a.password)
    tcp = await asyncio.start_server(
        lambda r, w: broker.serve(TcpLink(r, w), "%s:%s" % w.get_extra_info("peername")[:2]),
        a.mqtt_host, a.mqtt_port)
    web = await asyncio.start_server(lambda r, w: handle_http(broker, r, w),
                                     a.web_host, a.web_port)
    log(f"MQTT broker on {a.mqtt_host}:{a.mqtt_port}")
    log(f"web app on http://{a.web_host}:{a.web_port}/  (WebSocket /mqtt)")
    if a.mqtt_host not in ("127.0.0.1", "localhost", "::1") and not a.password:
        log("WARNING: broker is reachable from the network with NO password; "
            "anyone on the LAN can drive the car. Use --password.")
    async with tcp, web:
        await asyncio.gather(tcp.serve_forever(), web.serve_forever())


if __name__ == "__main__":
    if sys.version_info < (3, 8):
        sys.exit("Python 3.8+ required")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
