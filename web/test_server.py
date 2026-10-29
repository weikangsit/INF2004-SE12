"""End-to-end test: broker + web server, fake car (MQTT/TCP), fake browser
(MQTT/WebSocket). Stdlib only.  Run:  python test_server.py
"""
import base64
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time
import shutil
from pathlib import Path

import proto_ref

HERE = Path(__file__).resolve().parent
MQTT_PORT, WEB_PORT, PW = 18883, 18080, "testpw"


def varint(n):
    out = bytearray()
    while True:
        b, n = n % 128, n // 128
        out.append(b | 0x80 if n else b)
        if not n:
            return bytes(out)


def s(b):
    return struct.pack(">H", len(b)) + b


def pkt(first, body):
    return bytes([first]) + varint(len(body)) + body


def connect_pkt(cid, pw):
    flags = 0x02 | (0xC0 if pw is not None else 0)
    body = s(b"MQTT") + bytes([4, flags]) + struct.pack(">H", 30) + s(cid.encode())
    if pw is not None:
        body += s(b"car") + s(pw.encode())
    return pkt(0x10, body)


class Tcp:
    def __init__(self):
        self.k = socket.create_connection(("127.0.0.1", MQTT_PORT), timeout=3)

    def send(self, b):
        self.k.sendall(b)

    def recv(self, n):
        out = b""
        while len(out) < n:
            c = self.k.recv(n - len(out))
            if not c:
                raise EOFError
            out += c
        return out


class Ws(Tcp):
    def __init__(self, origin=None):
        self.k = socket.create_connection(("127.0.0.1", WEB_PORT), timeout=3)
        key = base64.b64encode(os.urandom(16)).decode()
        req = (f"GET /mqtt HTTP/1.1\r\nHost: 127.0.0.1:{WEB_PORT}\r\nUpgrade: websocket\r\n"
               f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n"
               "Sec-WebSocket-Protocol: mqtt\r\n")
        if origin:
            req += f"Origin: {origin}\r\n"
        self.k.sendall((req + "\r\n").encode())
        head = b""
        while b"\r\n\r\n" not in head:
            head += self.k.recv(1)
        self.status = int(head.split()[1])
        self.buf = b""

    def send(self, b):
        mask = os.urandom(4)
        n = len(b)
        hdr = bytes([0x82, 0x80 | n]) if n < 126 else bytes([0x82, 0x80 | 126]) + struct.pack(">H", n)
        self.k.sendall(hdr + mask + bytes(x ^ mask[i & 3] for i, x in enumerate(b)))

    def recv(self, n):
        while len(self.buf) < n:
            b0, b1 = Tcp.recv(self, 2)
            ln = b1 & 0x7F
            if ln == 126:
                ln = struct.unpack(">H", Tcp.recv(self, 2))[0]
            self.buf += Tcp.recv(self, ln)
        out, self.buf = self.buf[:n], self.buf[n:]
        return out


def read_pkt(c):
    b0 = c.recv(1)[0]
    n, mult = 0, 1
    while True:
        b = c.recv(1)[0]
        n += (b & 0x7F) * mult
        mult *= 128
        if not b & 0x80:
            break
    return b0, c.recv(n) if n else b""


def read_publish(c):
    while True:
        b0, body = read_pkt(c)
        if b0 >> 4 == 3:
            tl = struct.unpack(">H", body[:2])[0]
            return body[2:2 + tl].decode(), body[2 + tl:], bool(b0 & 1)


def mqtt_login(c, cid, pw=PW):
    c.send(connect_pkt(cid, pw))
    b0, body = read_pkt(c)
    assert b0 == 0x20, b0
    return body[1]


def sub(c, topic):
    c.send(pkt(0x82, struct.pack(">H", 1) + s(topic.encode()) + b"\x01"))
    b0, body = read_pkt(c)
    assert b0 >> 4 == 9 and body[2] == 0, body


def pub(c, topic, payload, retain=False):
    c.send(pkt(0x30 | int(retain), s(topic.encode()) + payload))


def main():
    work = Path(tempfile.mkdtemp(dir=os.environ.get("KIROCREW_SCRATCH")))
    shutil.copy(HERE / "server.py", work / "server.py")
    shutil.copytree(HERE / "static", work / "static")
    proc = subprocess.Popen([sys.executable, str(work / "server.py"), "--mqtt-port", str(MQTT_PORT),
                             "--web-port", str(WEB_PORT), "--password", PW],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    ok = True
    try:
        time.sleep(1.0)
        # 1. wrong password refused
        assert mqtt_login(Tcp(), "bad", "nope") == 5
        print("PASS wrong password refused")

        # 2. cross-site WebSocket refused
        assert Ws(origin="http://evil.example").status == 403
        print("PASS cross-origin WebSocket refused")

        # 3. static web app served
        h = socket.create_connection(("127.0.0.1", WEB_PORT))
        h.sendall(b"GET / HTTP/1.1\r\nHost: x\r\n\r\n")
        assert b"200 OK" in h.recv(200)
        print("PASS web app served")

        # 4. web publishes retained config; car subscribes later and gets it
        web = Ws(origin=f"http://127.0.0.1:{WEB_PORT}")
        assert web.status == 101 and mqtt_login(web, "web-1") == 0
        sub(web, "robot/12/#")
        cfg = {"A": 1, "B": 2, "C": 0, "D": 3}
        frame = proto_ref.encode_config(cfg)
        assert len(frame) == 2
        pub(web, "robot/12/config/barcode", frame, retain=True)
        read_publish(web)                        # own echo
        car = Tcp()
        assert mqtt_login(car, "robocar-g12") == 0
        sub(car, "robot/12/config/barcode")
        t, p, r = read_publish(car)
        assert t == "robot/12/config/barcode" and p == frame and r
        print("PASS car received retained 2-byte config:", p.hex())

        # 5. car telemetry (18 bytes) reaches the browser untouched
        tel = proto_ref.encode_telem({"state": 2, "motionClass": 1, "lineBits": 1,
                                      "barcode": 4 | 1, "faultMask": 1 << 4,
                                      "speedL": 30, "speedR": 31, "dutyL": 45,
                                      "dutyR": -12, "pitch": 9, "roll": -3,
                                      "heading": 123, "turnRate": 5, "range": 25,
                                      "distance": 150, "hump": 17, "uptime": 650})
        assert len(tel) == 18
        pub(car, "robot/12/telemetry", tel)
        t, p, _ = read_publish(web)
        assert t == "robot/12/telemetry" and p == tel
        print("PASS browser received 18-byte telemetry")

        # 5b. zero-length config/get and a 1-byte command pass through
        pub(car, "robot/12/config/get", b"")
        t, p, _ = read_publish(web)
        assert t == "robot/12/config/get" and p == b""
        sub(car, "robot/12/cmd")
        pub(web, "robot/12/cmd", proto_ref.encode_cmd("estop"))
        read_publish(web)                        # own echo
        t, p, _ = read_publish(car)
        assert t == "robot/12/cmd" and p == b"\x42"
        print("PASS empty config/get and 1-byte e-stop command delivered")
        (work / "telem.bin").write_bytes(tel)
        (work / "config.bin").write_bytes(frame)

        # 6. retained survives a broker restart (persisted binary file)
        proc.terminate(); proc.wait(5)
        proc = subprocess.Popen([sys.executable, str(work / "server.py"), "--mqtt-port", str(MQTT_PORT),
                                 "--web-port", str(WEB_PORT), "--password", PW],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        time.sleep(1.0)
        car = Tcp()
        assert mqtt_login(car, "robocar-g12") == 0
        sub(car, "robot/12/config/#")
        t, p, r = read_publish(car)
        assert p == frame and r
        print("PASS retained config persisted across restart")
        print("WORKDIR", work)
    except Exception as e:                       # noqa: BLE001
        ok = False
        print("FAIL", repr(e))
    finally:
        proc.terminate()
        out = proc.communicate(timeout=5)[0]
        if not ok:
            print(out)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
