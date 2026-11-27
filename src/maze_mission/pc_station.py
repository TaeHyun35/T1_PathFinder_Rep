"""PC 스테이션 (노트북/PC). 로봇(파이) 미션과 짝으로 돈다. PC 가 로봇에게 먼저 연결하는 방식 (PC 쪽 서버 없음).

하는 일
  1) 음성(또는 글)으로 찾을 동물 순서를 받아 로봇에 보냄
  2) 로봇 카메라 사진을 받아 카드 찾기 (YOLO, 11/26 실제 추가 학습 모델) -> 결과를 로봇에 보냄 + 기록 (지도에 찍기용)
  3) 로봇이 남긴 대사를 꺼내 스피커로 말하기 (윈도우 한국어 목소리 Heami)
  4) 블루투스 키보드로 눌린 키(F1~F4 = 1~4, 숫자키도 됨)를 모아 정답과 비교 -> 결과를 로봇에 보냄

실행 (.venv-ml):
  python maze_mission\\pc_station.py --voice                 마이크로 "기린, 강아지, 코끼리 순서로 찾아 줘"
  python maze_mission\\pc_station.py --order 기린,강아지,코끼리
키 입력을 받으려면 이 창이 앞에 있어야 한다 (로봇이 누르는 키가 이 창으로 들어옴). q = 끝
"""
import argparse
import json
import msvcrt
import os
import queue
import re
import subprocess
import threading
import time
import urllib.request

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CLASSES = ["dog", "panda", "turtle", "rabbit", "giraffe", "elephant", "num1", "num2", "num3", "num4"]
KO2EN = [("강아지", "dog"), ("멍멍이", "dog"), ("개", "dog"), ("판다", "panda"), ("거북이", "turtle"), ("거북", "turtle"),
         ("토끼", "rabbit"), ("기린", "giraffe"), ("코끼리", "elephant")]
FKEYS = {59: "1", 60: "2", 61: "3", 62: "4"}         # msvcrt: F1~F4 는 0 또는 224 다음에 이 값


def parse_order(text):
    """문장에서 동물 이름을 나온 순서대로 (중복 없이)"""
    hits = []
    for ko, en in KO2EN:
        for m in re.finditer(ko, text):
            hits.append((m.start(), len(ko), en))
    hits.sort(key=lambda h: (h[0], -h[1]))
    out, last_end = [], -1
    for pos, ln, en in hits:
        if pos < last_end:                  # "강아지" 안의 "개" 같은 겹침 무시
            continue
        last_end = pos + ln
        if en not in out:
            out.append(en)
    return out


def listen(secs=6.0):
    import sounddevice as sd
    import whisper
    print(f"\n[음성] {secs:.0f}초 동안 말해 주세요 (예: 기린, 강아지, 코끼리 순서로 찾아 줘)", flush=True)
    a = sd.rec(int(secs * 16000), samplerate=16000, channels=1, dtype="float32")
    sd.wait()
    m = whisper.load_model("small")
    r = m.transcribe(a[:, 0], language="ko", initial_prompt="강아지, 판다, 거북이, 토끼, 기린, 코끼리 순서로 찾아 줘.")
    text = r["text"].strip()
    print(f"[음성] 들은 말: {text}", flush=True)
    return text


class Speaker(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.q = queue.Queue()

    def run(self):
        while True:
            text = self.q.get()
            safe = text.replace("'", "''")
            ps = ("Add-Type -AssemblyName System.Speech; $s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
                  "try { $s.SelectVoice('Microsoft Heami Desktop') } catch {}; $s.Rate = 0; "
                  f"$s.Speak('{safe}'); $s.Dispose()")
            subprocess.run(["powershell", "-NoProfile", "-Command", ps], capture_output=True)


class Station:
    def __init__(self, a):
        self.a, self.base = a, f"http://{a.robot}:{a.port}"
        self.speaker = Speaker()
        self.speaker.start()
        self.state, self.code, self.buf = "", "", ""
        self.stop = False
        name = time.strftime("station_%Y%m%d_%H%M%S")
        self.det_log = open(os.path.join(HERE, "logs", name + "_detect.jsonl"), "w", encoding="utf-8")
        print("기록:", self.det_log.name, flush=True)

    def get(self, path, timeout=2):
        with urllib.request.urlopen(self.base + path, timeout=timeout) as r:
            return r.read(), r.headers

    def post(self, path, obj):
        req = urllib.request.Request(self.base + path, data=json.dumps(obj).encode(), headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=2) as r:
            return r.read()

    def detect_loop(self):
        from ultralytics import YOLO
        m = YOLO(self.a.model)
        last_ts = None
        while not self.stop:
            try:
                b, h = self.get("/frame.jpg")
                ts = float(h.get("X-Time", "0"))
                img = cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_COLOR)
                if img is None or ts == last_ts:
                    time.sleep(0.05)
                    continue
                last_ts = ts
                r = m.predict(img, conf=self.a.conf, verbose=False)[0]
                dets = [[CLASSES[int(x.cls)], round(float(x.conf), 3), *[round(v, 1) for v in x.xyxy[0].tolist()]] for x in r.boxes]
                rec = {"ts": ts, "w": img.shape[1], "h": img.shape[0], "dets": dets}
                self.post("/dets", rec)
                self.det_log.write(json.dumps(rec) + "\n")
                self.det_log.flush()
            except Exception:
                time.sleep(0.3)
            time.sleep(0.12)

    def outbox_loop(self):
        while not self.stop:
            try:
                o = json.loads(self.get("/outbox")[0])
                for line in o.get("say", []):
                    print(f"\n[로봇] {line}", flush=True)
                    self.speaker.q.put(line)
                if o.get("state") != self.state:
                    self.state = o.get("state", "")
                    print(f"\n[상태] {self.state}", flush=True)
                if o.get("code") and o["code"] != self.code:
                    self.code, self.buf = o["code"], ""
                    print(f"[비밀번호] 로봇이 누를 번호 {self.code} (키 입력을 기다림)", flush=True)
                if o.get("virtual"):                 # 가상 누르기: 로봇 대신 입력
                    print(f"[가상 누르기] {o['virtual']}", flush=True)
                    self.code = self.code or o["virtual"]
                    for ch in o["virtual"]:
                        self.key(ch)
            except Exception:
                pass
            time.sleep(0.3)

    def key(self, ch):
        if not self.code:
            print(f"\n(키 {ch} - 아직 누를 차례 아님)", flush=True)
            return
        self.buf += ch
        print(f"\r[키보드] 눌린 키: {self.buf}", end="", flush=True)
        if len(self.buf) >= len(self.code):
            ok = self.buf == (self.a.answer or self.code)          # 정답을 따로 주면(진행자) 그걸로, 아니면 로봇 번호와 비교
            print(f"\n[확인] 눌린 {self.buf} / 정답 {self.code} -> {'맞음' if ok else '틀림'}", flush=True)
            try:
                self.post("/result", {"ok": ok, "typed": self.buf})
            except Exception as e:
                print("결과 보내기 실패:", e)
            self.buf = ""
            if ok:
                self.code = ""

    def run(self):
        order = parse_order(listen() if self.a.voice else self.a.order)
        if not order:
            print("동물 이름을 못 알아들음 -> --order 기린,강아지,코끼리 로 다시", flush=True)
            return
        print("[순서]", order, flush=True)
        for t in (self.detect_loop, self.outbox_loop):
            threading.Thread(target=t, daemon=True).start()
        while True:
            try:
                self.post("/order", {"order": order})
                break
            except Exception:
                print("로봇 미션에 연결 중...", flush=True)
                time.sleep(1.0)
        print("로봇에 순서 보냄. 이 창을 앞에 두세요 (키 입력 받기). q = 끝", flush=True)
        while True:
            if msvcrt.kbhit():
                c = msvcrt.getwch()
                if c in ("\x00", "\xe0"):
                    k = ord(msvcrt.getwch())
                    if k in FKEYS:
                        self.key(FKEYS[k])
                elif c in "1234":
                    self.key(c)
                elif c.lower() == "q":
                    break
            time.sleep(0.01)
        self.stop = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="192.168.137.4")
    ap.add_argument("--port", type=int, default=8093)
    ap.add_argument("--voice", action="store_true")
    ap.add_argument("--order", default="")
    ap.add_argument("--model", default=(os.path.join(HERE, "models", "animal_best.pt") if os.path.exists(os.path.join(HERE, "models", "animal_best.pt")) else "D:/aihand-ml/runs/animal_real/weights/best.pt"))
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--answer", default="", help="진짜 정답 비밀번호 (진행자가 정함). 없으면 로봇이 만든 번호와 비교")
    a = ap.parse_args()
    if not a.voice and not a.order:
        ap.error("--voice 또는 --order 가 필요")
    Station(a).run()


if __name__ == "__main__":
    main()
