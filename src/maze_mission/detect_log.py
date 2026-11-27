"""주행하는 동안 로봇 카메라 사진을 받아 카드를 찾고 기록 (PC, GPU).

  .venv-ml\\Scripts\\python maze_mission\\detect_log.py --secs 200
-> logs/detect_<시각>.jsonl : {"ts": 파이 시계로 찍은 시각, "dets": [[이름, 확신도, x1, y1, x2, y2], ...]}
   logs/detect_<시각>/ : 카드가 보인 사진 일부 (기록용)
라이다 기록(roommap)의 "ts" 와 시각으로 맞춰서 지도에 카드 위치를 찍는다 (roommap.py build --det).
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
CLASSES = ["dog", "panda", "turtle", "rabbit", "giraffe", "elephant", "num1", "num2", "num3", "num4"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://192.168.137.4:8092/frame.jpg")
    ap.add_argument("--model", default=(os.path.join(HERE, "models", "animal_best.pt") if os.path.exists(os.path.join(HERE, "models", "animal_best.pt")) else "D:/aihand-ml/runs/animal_real/weights/best.pt"))
    ap.add_argument("--secs", type=float, default=200)
    ap.add_argument("--conf", type=float, default=0.5)
    a = ap.parse_args()
    m = YOLO(a.model)
    name = time.strftime("detect_%Y%m%d_%H%M%S")
    os.makedirs(os.path.join(HERE, "logs", name), exist_ok=True)
    f = open(os.path.join(HERE, "logs", name + ".jsonl"), "w", encoding="utf-8")
    t0, last_ts, n, saved = time.time(), None, 0, 0
    print("카드 찾기 기록 시작:", name, flush=True)
    while time.time() - t0 < a.secs:
        try:
            r = urllib.request.urlopen(a.url, timeout=2)
            ts = float(r.headers.get("X-Time", "0"))
            img = cv2.imdecode(np.frombuffer(r.read(), np.uint8), cv2.IMREAD_COLOR)
        except Exception:
            time.sleep(0.3)
            continue
        if ts == last_ts or img is None:
            time.sleep(0.05)
            continue
        last_ts = ts
        res = m.predict(img, conf=a.conf, verbose=False)[0]
        dets = [[CLASSES[int(b.cls)], round(float(b.conf), 3), *[round(v, 1) for v in b.xyxy[0].tolist()]] for b in res.boxes]
        f.write(json.dumps({"ts": ts, "w": img.shape[1], "h": img.shape[0], "dets": dets}) + "\n")
        f.flush()
        n += 1
        if dets and n % 5 == 0 and saved < 200:
            for c, cf, x1, y1, x2, y2 in dets:
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 200, 0), 2)
                cv2.putText(img, c, (int(x1), max(18, int(y1) - 5)), 0, 0.7, (0, 200, 0), 2)
            cv2.imwrite(os.path.join(HERE, "logs", name, f"{n:05d}.jpg"), img)
            saved += 1
        time.sleep(0.15)
    f.close()
    print(f"끝: 사진 {n}장", flush=True)


if __name__ == "__main__":
    main()
