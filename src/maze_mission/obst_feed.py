"""PC: 로봇 카메라 사진에서 장애물(옷걸이, 의자, 가방...)을 찾아 로봇에 보냄 (11/27 카메라+라이다 같이 쓰기).

  .venv-ml\\Scripts\\python maze_mission\\obst_feed.py               (지도 주행: cam_serve :8092 에서 사진, 로봇 :8093 으로 보냄)
  .venv-ml\\Scripts\\python maze_mission\\obst_feed.py --src 8093    (미션: link 에서 사진)
로봇 쪽은 받은 상자를 라이다 점처럼 바꿔서 피한다 (obstacles.cam_obstacles).
"""
import argparse
import json
import os
import time
import urllib.request

import cv2
import numpy as np
from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))
NAMES = ["clothes rack", "chair", "bag", "box", "trash can", "table leg", "computer case", "person", "stool", "cart", "blanket", "clothes", "drying rack", "shelf", "cable"]   # 11/27: 옷걸이는 "blanket"(위 옷) 으로 잡힘


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="192.168.137.4")
    ap.add_argument("--src", type=int, default=8092)
    ap.add_argument("--dst", type=int, default=8093)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--secs", type=float, default=900)
    a = ap.parse_args()
    m = YOLO(os.path.join(HERE, "..", "models", "yolo", "yolov8s-worldv2.pt"))
    m.set_classes(NAMES)
    url_in = f"http://{a.robot}:{a.src}/frame.jpg"
    url_out = f"http://{a.robot}:{a.dst}/obst"
    log = open(os.path.join(HERE, "logs", time.strftime("obst_%Y%m%d_%H%M%S.jsonl")), "w", encoding="utf-8")
    t0, last_ts, n, sent = time.time(), None, 0, 0
    print("장애물 찾기 시작", flush=True)
    while time.time() - t0 < a.secs:
        try:
            r = urllib.request.urlopen(url_in, timeout=2)
            ts = float(r.headers.get("X-Time", "0"))
            img = cv2.imdecode(np.frombuffer(r.read(), np.uint8), cv2.IMREAD_COLOR)
        except Exception:
            time.sleep(0.3)
            continue
        if img is None or ts == last_ts:
            time.sleep(0.05)
            continue
        last_ts = ts
        res = m.predict(img, conf=a.conf, verbose=False)[0]
        dets = [[NAMES[int(b.cls)], round(float(b.conf), 3), *[round(v, 1) for v in b.xyxy[0].tolist()]] for b in res.boxes]
        dets = [d for d in dets if d[0] != "person"]            # 사람(줄 들고 있는 사용자)은 피할 물체로 안 넣음
        rec = {"ts": ts, "w": img.shape[1], "h": img.shape[0], "dets": dets}
        log.write(json.dumps(rec) + "\n")
        log.flush()
        try:
            req = urllib.request.Request(url_out, data=json.dumps(rec).encode(), headers={"Content-Type": "application/json"})
            urllib.request.urlopen(req, timeout=1).read()
            sent += 1
        except Exception:
            pass
        n += 1
        if n % 50 == 0:
            print(f"  사진 {n}, 보냄 {sent}, 지금 {[d[0] for d in dets]}", flush=True)
        time.sleep(0.1)


if __name__ == "__main__":
    main()
