"""로봇(파이) 쪽 우체통: PC 스테이션(pc_station.py)이 여기에 연결한다 (PC 쪽은 서버를 안 엶).

GET  /frame.jpg   지금 카메라 사진 (X-Time = 찍은 시각)
POST /dets        PC 가 찾은 카드 {"ts","w","h","dets":[[이름,확신도,x1,y1,x2,y2],...]}
GET  /outbox      로봇 대사(가져가면 비움), 상태, 누를 비밀번호
POST /order       찾을 동물 순서 {"order": ["giraffe", ...]}
POST /result      키보드로 눌린 결과 {"ok": true/false, "typed": "122"}
"""
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

DIGIT_H = 0.054                 # 카드 숫자 칸 실제 높이 (m). 11/27 가로 카드 5.4cm (세로 카드는 7.2cm)


def pair(found):
    """[(이름, 확신도, (x1,y1,x2,y2))] -> {동물: 숫자}. 숫자 칸은 그림 바로 아래, 가로 가운데가 그림 안"""
    an = [f for f in found if not f[0].startswith("num")]
    nu = [f for f in found if f[0].startswith("num")]
    out = {}
    for c, _, (x1, y1, x2, y2) in an:
        best, bd, h = "", 1e9, y2 - y1
        for nc, _, (a1, b1, a2, b2) in nu:
            cx = (a1 + a2) / 2
            if x1 <= cx <= x2 and y2 - 0.2 * h <= b1 <= y2 + 0.8 * h and abs(b1 - y2) < bd:
                best, bd = nc[3:], abs(b1 - y2)
        if c not in out or best:
            out[c] = best
    return out


class Link:
    def __init__(self, port=8093):
        self.lock = threading.Lock()
        self.frame, self.frame_t = None, 0.0
        self.dets, self.dets_rx = None, 0.0
        self.order, self.result = None, None
        self.last_poll = time.time()             # PC 스테이션이 마지막으로 물어본 시각 (끊김 감지)
        self.polled = False
        self.obst, self.obst_rx = None, 0.0     # PC 가 카메라로 찾은 장애물 상자                      # 한 번이라도 연결됐나 (스테이션 없이 하는 시험은 끊김 검사 안 함)
        self.mission = None
        srv = ThreadingHTTPServer(("0.0.0.0", port), self._handler())
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print(f"[link] PC 스테이션 기다림 :{port}", flush=True)

    def _handler(self):
        link = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *x):
                pass

            def _send(self, body, ctype="application/json", extra=None):
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                for k, v in (extra or {}).items():
                    self.send_header(k, v)
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path.startswith("/frame.jpg"):
                    with link.lock:
                        f, t = link.frame, link.frame_t
                    if f is None:
                        self.send_response(503)
                        self.end_headers()
                        return
                    b = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
                    return self._send(b, "image/jpeg", {"X-Time": f"{t:.3f}"})
                if self.path.startswith("/outbox"):
                    link.last_poll, link.polled = time.time(), True
                    m = link.mission
                    o = {"say": [], "state": "", "code": ""}
                    if m is not None:
                        o["say"], m.outbox[:] = list(m.outbox), []
                        o["state"] = m.state
                        o["code"] = m.code if m.state == "PRESS" else ""
                        vc = getattr(m, "virtual_code", "")
                        if vc:
                            o["virtual"], m.virtual_code = vc, ""
                    return self._send(json.dumps(o, ensure_ascii=False).encode())
                self.send_response(404)
                self.end_headers()

            def do_POST(self):
                n = int(self.headers.get("Content-Length", 0))
                d = json.loads(self.rfile.read(n) or b"{}")
                with link.lock:
                    if self.path.startswith("/dets"):
                        link.dets, link.dets_rx = d, time.time()
                    elif self.path.startswith("/order"):
                        link.order = d.get("order")
                    elif self.path.startswith("/result"):
                        link.result = d
                    elif self.path.startswith("/obst"):
                        link.obst, link.obst_rx = d, time.time()
                self._send(b"{}")

        return H


class RemoteVision:
    """카메라는 파이에서 읽고(ArUco 도 여기서), 카드 찾기는 PC 가 보내 준 결과를 쓴다"""

    def __init__(self, cam_cfg, link):
        from vision import Aruco, Camera
        self.cam, self.aruco, self.link = Camera(cam_cfg), Aruco(), link
        self.fov = cam_cfg["fov_deg"]
        self.last_frame = None

    def perceive(self):
        f = self.cam.read()
        if f is None:
            return {"animals": [], "markers": [], "err": self.cam.err or "프레임 없음"}
        with self.link.lock:
            self.link.frame, self.link.frame_t = f, time.time()
            d, rx = self.link.dets, self.link.dets_rx
        self.last_frame = f
        out = {"markers": self.aruco.find(f), "animals": []}
        if d and time.time() - rx < 0.8:
            w = d["w"]
            fpx = (w / 2) / math.tan(math.radians(self.fov / 2))
            hgt = d.get("h", 480)
            # 잘린 카드 빼기: 좌우 끝에 닿은 것, 그리고 숫자 칸이 위/아래 끝에 닿은 것 (11/26: 아래가 잘린 4 가 1 로 읽힘)
            found = [(c, cf, (x1, y1, x2, y2)) for c, cf, x1, y1, x2, y2 in d["dets"]
                     if x1 > 4 and x2 < w - 4 and not (c.startswith("num") and (y1 < 4 or y2 > hgt - 4))]
            pr = pair(found)
            nums = {c[3:]: b for c, _, b in found if c.startswith("num")}
            for c, cf, (x1, y1, x2, y2) in found:
                if c.startswith("num"):
                    continue
                dg = pr.get(c, "")
                dist = fpx * DIGIT_H / max(1.0, nums[dg][3] - nums[dg][1]) if dg in nums else None
                out["animals"].append({"label": c, "digit": dg, "conf": cf, "cx": (x1 + x2) / 2 / w, "w": (x2 - x1) / w, "d": dist})
        return out


class KeyCheck:
    """문 스테이션 대신: PC 가 키보드로 받은 결과. open = 맞음, wrong = 틀림, closed = 아직"""

    def __init__(self, link):
        self.link = link

    def send_code(self, code):
        pass

    def status(self):
        with self.link.lock:
            r, self.link.result = self.link.result, None
        if r is None:
            return "closed"
        return "open" if r.get("ok") else "wrong"
