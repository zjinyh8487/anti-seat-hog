# -*- coding: utf-8 -*-
"""
防占座监测 · 网页总览服务  v1.0
================================================
· 监听 UDP 4210 自动发现局域网里的所有 ESP32 座位
· 每秒读取每台的 /api，汇总给网页
· 打开 http://127.0.0.1:8899 就是那块「座位总览」大屏
· 每张卡片可以直接调用设备的 /tare /reset /stop（从服务器转发，网页不用管跨域）

只依赖 Python 标准库。用法：
    python dashboard_server.py            # 默认 8899 端口
    python dashboard_server.py 9000       # 换端口
额外的座位（广播收不到时）写进同目录 seats.txt，一行一个 IP 或 seat-01.local。
"""
import json
import os
import socket
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, "dashboard.html")
SEATS_TXT = os.path.join(HERE, "seats.txt")

BEACON_PORT = 4210
HTTP_PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8899
POLL_INTERVAL = 1.0
OFFLINE_AFTER = 6.0          # 秒：这么久没读到 /api 就算离线
FORGET_AFTER = 600.0         # 秒：这么久没广播也没数据，就从列表里清掉

_lock = threading.Lock()
_seats = {}                  # ip -> {"name":..., "seen": 广播时间, "state": {...}, "ok_at": 最后成功时间}
_extra = []                  # seats.txt 里手写的地址


def log(*a):
    print("[%s]" % time.strftime("%H:%M:%S"), *a, flush=True)


# ---------------------------------------------------------------- 发现
def load_extra():
    if not os.path.exists(SEATS_TXT):
        return []
    out = []
    with open(SEATS_TXT, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#"):
                out.append(line)
    return out


def discovery_loop():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("", BEACON_PORT))
    except OSError as e:
        log("UDP %d 绑定失败（可能另一个监测程序占着）：%s" % (BEACON_PORT, e))
        return
    log("正在监听广播 UDP %d …" % BEACON_PORT)
    while True:
        try:
            data, addr = s.recvfrom(2048)
            txt = data.decode("utf-8", "ignore").strip()
            parts = txt.split("|")
            if not parts or parts[0] != "SEAT" or len(parts) < 3:
                continue
            name, ip = parts[1], parts[2]
            if not ip or ip == "0.0.0.0":
                ip = addr[0]
            with _lock:
                rec = _seats.setdefault(ip, {"state": None, "ok_at": 0.0, "added": time.time()})
                rec["name"] = name
                rec["seen"] = time.time()
        except Exception:
            time.sleep(0.5)


# ---------------------------------------------------------------- 轮询
def fetch_one(ip):
    url = "http://%s/api" % ip
    try:
        with urllib.request.urlopen(url, timeout=2.5) as r:
            st = json.loads(r.read().decode("utf-8", "ignore"))
        now = time.time()
        with _lock:
            rec = _seats.setdefault(ip, {"name": st.get("device", ip), "added": now})
            rec["state"] = st
            rec["ok_at"] = now
            rec.setdefault("name", st.get("device", ip))
    except Exception:
        pass


def poll_loop():
    while True:
        with _lock:
            targets = list(_seats.keys()) + _extra
        for ip in dict.fromkeys(targets):
            threading.Thread(target=fetch_one, args=(ip,), daemon=True).start()
        time.sleep(POLL_INTERVAL)


# ---------------------------------------------------------------- 汇总
def snapshot():
    now = time.time()
    out = []
    with _lock:
        for ip, rec in list(_seats.items()):
            last = rec.get("ok_at") or rec.get("seen") or rec.get("added", now)
            if now - max(rec.get("ok_at", 0), rec.get("seen", 0)) > FORGET_AFTER:
                del _seats[ip]
                continue
            st = dict(rec.get("state") or {})
            st.setdefault("device", rec.get("name", ip))
            st["name"] = rec.get("name") or st.get("device", ip)
            st["ip"] = ip
            st["online"] = (now - rec.get("ok_at", 0)) < OFFLINE_AFTER
            if not st["online"]:
                st["state"] = "offline"
            out.append(st)
    out.sort(key=lambda s: (s.get("state") != "hog", not s.get("online"), s.get("name", "")))
    return out


# ---------------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):      # 静音，别刷屏
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path in ("/", "/index.html"):
            try:
                with open(PAGE, "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            except FileNotFoundError:
                self._send(500, "dashboard.html 不见了", "text/plain; charset=utf-8")
            return

        if u.path == "/state":
            self._send(200, json.dumps(snapshot(), ensure_ascii=False))
            return

        if u.path == "/cmd":
            q = parse_qs(u.query)
            ip = (q.get("ip") or [""])[0]
            act = (q.get("act") or [""])[0]
            if act not in ("tare", "reset", "stop", "mp3", "play") or not ip:
                self._send(400, json.dumps({"ok": False, "why": "bad params"}))
                return
            path = act
            if act == "play":
                try:
                    trk = max(1, min(3000, int((q.get("track") or ["1"])[0])))
                except ValueError:
                    trk = 1
                path = "mp3?t=%d" % trk
            try:
                with urllib.request.urlopen("http://%s/%s" % (ip, path), timeout=6) as r:
                    body = r.read().decode("utf-8", "ignore")
                log("→ %s /%s  %s" % (ip, path, body[:80]))
                self._send(200, json.dumps({"ok": True, "reply": body}, ensure_ascii=False))
            except Exception as e:
                self._send(200, json.dumps({"ok": False, "why": str(e)}, ensure_ascii=False))
            return

        self._send(404, json.dumps({"ok": False}))


def main():
    global _extra
    _extra = load_extra()
    if _extra:
        log("seats.txt 里额外指定了 %d 个地址：%s" % (len(_extra), ", ".join(_extra)))
    threading.Thread(target=discovery_loop, daemon=True).start()
    threading.Thread(target=poll_loop, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    log("座位总览已启动 → http://127.0.0.1:%d" % HTTP_PORT)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        log("退出")


if __name__ == "__main__":
    main()
