"""길잡이 시연 사이트 (11/28): 5단계를 하나씩 눌러서 보여 준다.
  1 보정   2 지도 생성(방 전체)   3 동물·탈출 위치 찾기(기억)   4 말한 동물 찾아가기   5 탈출

실행 (aihand 폴더):  .venv\\Scripts\\python tools\\demo\\demo.py   ->  http://127.0.0.1:8770
실제 실행은 테스트 실험실(tools/lab)의 로봇 테스트 코드를 그대로 쓴다 (같은 기록이 logs/lab 에 남음).
PC 는 항상 로봇(파이) 쪽으로 연결한다. 이 서버는 이 PC 브라우저(127.0.0.1)만 받는다.
"""
import json
import os
import sys
import threading
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
LAB = os.path.join(os.path.dirname(HERE), "lab")
sys.path.insert(0, LAB)
sys.path.insert(0, os.path.dirname(HERE))
import lab  # noqa: E402  (테스트 실행/기록 그대로)

PORT = 8770
ROBOT = "192.168.137.4"

STAGES = {
    "1": ("robot_demo", {"mode": "real", "stage": "calib"}),
    "2": ("robot_room", {"mode": "real", "secs": 300, "cover": "1", "camobst": "1", "spin": "1"}),
    "3": ("robot_demo", {"mode": "real", "stage": "find"}),
    "4": ("robot_demo", {"mode": "real", "stage": "visit"}),
    "5": ("robot_demo", {"mode": "real", "stage": "exit"}),
}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _cors(self):
        # 깃허브 페이지(https://...github.io)에서 이 노트북 서버(127.0.0.1:8770)를 부를 수 있게
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Private-Network", "true")

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.end_headers()

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self._cors()
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path == "/":
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                return self._send(200, f.read(), "text/html; charset=utf-8")
        if u.path.startswith("/img/"):                # 11/27 실제 결과 그림 (저장소 docs/demo/img)
            p = os.path.join(HERE, "..", "..", "..", "docs", "demo", "img", os.path.basename(u.path))
            if os.path.isfile(p):
                with open(p, "rb") as f:
                    return self._send(200, f.read(), "image/png")
            return self._send(404, {"error": "없음"})
        if u.path == "/api/poll":
            cur = lab.current
            n = int(q.get("since", ["0"])[0])
            if cur is None:
                return self._send(200, {"name": None, "running": False, "events": []})
            if q.get("name", [""])[0] != cur.name:
                n = 0
            return self._send(200, {"name": cur.name, "test": cur.test["id"], "params": cur.params,
                                    "running": cur.running, "events": cur.since(n)})
        if u.path == "/api/img":
            p = os.path.abspath(q.get("path", [""])[0])
            if p.startswith(lab.ROOT) and p.lower().endswith((".png", ".jpg")) and os.path.isfile(p):
                with open(p, "rb") as f:
                    return self._send(200, f.read(), "image/png" if p.endswith(".png") else "image/jpeg")
            return self._send(404, {"error": "없음"})
        if u.path == "/api/cam":                      # 로봇 카메라 (미션 중 :8093, 지도 주행 중 :8092)
            for port in (8093, 8092):
                try:
                    with urllib.request.urlopen(f"http://{ROBOT}:{port}/frame.jpg", timeout=1.2) as r:
                        return self._send(200, r.read(), "image/jpeg")
                except Exception:
                    continue
            return self._send(503, {"error": "카메라 없음"})
        self._send(404, {"error": "없음"})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(n) or b"{}")
        if self.path == "/api/run":
            st = str(body.get("stage"))
            if st not in STAGES:
                return self._send(400, {"ok": False, "msg": "단계 번호 오류"})
            test, params = STAGES[st]
            params = dict(params)
            if st == "3":
                params["cards"] = body.get("cards", "")
            if st == "4":
                params["order"] = body.get("order", "")
                params["voice"] = "1" if body.get("voice") else "0"
            ok, msg = lab.start_run(test, params)
            return self._send(200, {"ok": ok, "msg": msg})
        if self.path == "/api/stop":
            if lab.current:
                lab.current.stop.set()
            return self._send(200, {"ok": True})
        self._send(404, {"error": "없음"})


def main():
    ThreadingHTTPServer.allow_reuse_address = False
    try:
        srv = ThreadingHTTPServer(("127.0.0.1", PORT), H)
    except OSError:
        print(f"이미 켜져 있습니다: http://127.0.0.1:{PORT}")
        return
    url = f"http://127.0.0.1:{PORT}"
    print("길잡이 시연:", url)
    if "--no-browser" not in sys.argv:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        if lab.current:
            lab.current.stop.set()


if __name__ == "__main__":
    main()
