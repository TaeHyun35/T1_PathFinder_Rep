"""동물 찾기 서버 (노트북 GPU). 파이가 YOLO 를 직접 돌리기 느릴 때 쓴다.
로봇 config: "detect": {"mode": "remote", "url": "http://<노트북IP>:8090/detect"}

실행 (ultralytics 가 깔린 환경):
  .venv-ml\\Scripts\\python maze_mission\\vision_server.py
  python vision_server.py --model yolov8s.pt --port 8090
"""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="yolo11n.pt")
    ap.add_argument("--port", type=int, default=8090)
    a = ap.parse_args()
    model = YOLO(a.model)

    class H(BaseHTTPRequestHandler):
        def log_message(self, *x):
            pass

        def do_POST(self):
            q = parse_qs(urlparse(self.path).query)
            conf = float(q.get("conf", ["0.45"])[0])
            n = int(self.headers.get("Content-Length", 0))
            img = cv2.imdecode(np.frombuffer(self.rfile.read(n), np.uint8), cv2.IMREAD_COLOR)
            out = []
            if img is not None:
                h, w = img.shape[:2]
                r = model.predict(img, conf=conf, verbose=False, imgsz=640)[0]
                for b in r.boxes:
                    x1, y1, x2, y2 = b.xyxy[0].tolist()
                    out.append({"label": r.names[int(b.cls)], "conf": float(b.conf), "cx": (x1 + x2) / 2 / w, "w": (x2 - x1) / w})
            body = json.dumps(out).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    print(f"동물 찾기 서버 :{a.port} (모델 {a.model})")
    ThreadingHTTPServer(("0.0.0.0", a.port), H).serve_forever()


if __name__ == "__main__":
    main()
