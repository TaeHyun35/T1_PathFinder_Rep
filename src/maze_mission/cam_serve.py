"""파이: 카메라 최신 사진을 HTTP 로 내보냄 (PC 가 받아서 YOLO/VLM 돌림). 바퀴/손 안 움직임.

  ~/handenv/bin/python cam_serve.py --port 8092
GET /frame.jpg -> 지금 사진
"""
import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8092)
    ap.add_argument("--index", type=int, default=-1)   # -1 = C270 자동으로 찾기 (켤 때마다 0 또는 8)
    a = ap.parse_args()
    if a.index < 0:
        from vision import find_c270
        a.index = find_c270(0)
    cap = cv2.VideoCapture(a.index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    last = {"jpg": None, "t": 0.0}

    def grab():
        while True:
            ok, f = cap.read()
            if ok:
                last["jpg"] = cv2.imencode(".jpg", f, [cv2.IMWRITE_JPEG_QUALITY, 92])[1].tobytes()
                last["t"] = time.time()
            else:
                time.sleep(0.05)

    threading.Thread(target=grab, daemon=True).start()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *x):
            pass

        def do_GET(self):
            b = last["jpg"]
            if b is None:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("X-Time", f"{last['t']:.3f}")   # 파이 시계로 찍은 시각 -> 라이다 기록과 맞춤
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

    print(f"카메라 서버 :{a.port}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", a.port), H).serve_forever()


if __name__ == "__main__":
    main()
