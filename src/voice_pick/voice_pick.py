"""말로 시키면 웹캠으로 물건 위치를 찾아 손목을 돌리고, 잡았다가 내려놓는다 (오병주 테스트 25·26).

순서 (한 회차):
  1) 물건 놓기 (6초)  ->  2) "공 잡아" / "스프레이 집어" 말하기 (3.5초 녹음)
  3) Whisper로 받아쓰기  ->  4) YOLO-World로 웹캠 화면에서 그 물건 찾기
  5) 화면 왼쪽/가운데/오른쪽 중 물건이 있는 칸으로 손목 돌리기 (150 / 90 / 30)
  6) 사람이 물건을 손바닥에 대 주면 꽉 쥐기 -> 3초 들고 있기 -> 손 펴서 내려놓기
  물건이 안 보이면 주먹을 쥔다. 말을 안 하면 움직이지 않는다.

실행 예:
  python voice_pick.py                 4회차 (micro:bit 포트 자동 찾기)
  python voice_pick.py --rounds 6      6회차
  python voice_pick.py --look          손목 30/90/150에서 손바닥이 어디를 향하는지만 확인
  python voice_pick.py --no-robot      로봇 없이 웹캠 + 음성만 시험 (보낼 명령은 화면과 터미널에 출력)
  python voice_pick.py --list-mics     마이크 목록 보기
  python voice_pick.py --mic "USB"     이름에 USB가 들어간 마이크 사용
"""
import argparse
import json
import os
import sys
import threading
import time

import cv2
import numpy as np
import sounddevice as sd
import torch
from PIL import Image, ImageDraw, ImageFont

# ---------------------------------------------------------------- 바꿔도 되는 설정
OPEN = [50, 130, 130, 130, 130]          # 엄지, 검지, 중지, 약지, 새끼 (손 편 상태)
FIST = [130, 50, 50, 50, 50]             # 주먹 (물건 없을 때)
FIST_TIGHT = [140, 30, 20, 20, 20]       # 꽉 쥐기 (물건 잡을 때, 테스트 18-2에서 찾은 값)
ZONES = [("왼쪽", 150), ("가운데", 90), ("오른쪽", 30)]   # 화면 칸 -> 손목 각도
OBJECTS = {"공": "ball", "스프레이": "bottle"}            # 말하는 이름 -> YOLO가 찾는 이름
HINT = "로봇 손 명령: 공 잡아, 스프레이 집어, 공 집어, 스프레이 잡아."   # Whisper 힌트
REC_SEC = 3.5            # 말하기 녹음 시간
MIN_LOUD = 0.03          # 이보다 소리가 작으면 "말 안 함"으로 본다
YOLO_CONF = 0.2          # YOLO 확신도 기준
FIND_FRAMES = 10         # 이 프레임 수 중 절반 이상 같은 칸이면 위치 확정
WHISPER_MODEL = "base"   # tiny / base / small
# ----------------------------------------------------------------

SR = 16000
WIN = "voice pick"
HERE = os.path.dirname(os.path.abspath(__file__))
DEV = "cuda" if torch.cuda.is_available() else "cpu"


def font(size):
    for p in ("C:/Windows/Fonts/malgunbd.ttf", "C:/Windows/Fonts/malgun.ttf",
              "/System/Library/Fonts/AppleSDGothicNeo.ttc", "/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf"):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


FONT = {s: font(s) for s in (22, 30)}


# ---------------------------------------------------------------- 로봇 손 (micro:bit)
class Hand:
    """micro:bit에 '엄지,검지,중지,약지,새끼,손목,이동시간ms' 한 줄을 보내고 OK를 받는다."""

    def __init__(self, port=None, dry=False):
        self.dry, self.last = dry, ""
        if dry:
            return
        import serial
        from serial.tools import list_ports
        if port is None:
            for p in list_ports.comports():
                text = f"{p.description} {p.manufacturer or ''} {p.hwid}".lower()
                if "mbed" in text or "0d28" in text or "micro:bit" in text or "microbit" in text:
                    port = p.device
                    break
        if port is None:
            sys.exit("micro:bit 포트를 못 찾았습니다. USB를 꽂았는지 확인하거나 --port COM4 처럼 직접 적어 주세요."
                     " 로봇 없이 해 보려면 --no-robot")
        print("micro:bit 포트:", port)
        self.ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(0.5)

    def send(self, fingers, wrist, move=300):
        line = ",".join(str(int(round(a))) for a in fingers) + f",{int(wrist)},{int(move)}"
        self.last = line
        if self.dry:
            return
        self.ser.reset_input_buffer()
        self.ser.write((line + "\n").encode())
        self.ser.readline()

    def close(self):
        if not self.dry:
            self.ser.close()


# ---------------------------------------------------------------- 웹캠 + YOLO
class Cam(threading.Thread):
    """웹캠을 계속 읽고 최신 프레임과 YOLO 결과를 들고 있는다."""

    def __init__(self, model, index=None):
        super().__init__(daemon=True)
        self.model, self.frame, self.dets, self.running, self.cap = model, None, [], True, None
        backend = cv2.CAP_MSMF if os.name == "nt" else cv2.CAP_ANY
        for i in ([index] if index is not None else range(4)):
            cap = cv2.VideoCapture(i, backend)
            if cap.isOpened() and cap.read()[0]:
                self.cap = cap
                print("웹캠 번호:", i)
                break
        if self.cap is None:
            sys.exit("웹캠을 못 찾았습니다. 다른 프로그램(줌, 카메라 앱)이 쓰고 있는지 확인해 주세요. --cam 1 처럼 번호를 바꿀 수도 있습니다.")

    def run(self):
        while self.running:
            ok, f = self.cap.read()
            if not ok:
                continue
            f = cv2.flip(f, 1)       # 거울처럼 좌우 반전: 화면 왼쪽 = 내 왼쪽
            r = self.model.predict(f, device=DEV, verbose=False, conf=YOLO_CONF)[0]
            self.dets = [(r.names[int(b.cls)], float(b.conf), b.xyxy[0].tolist()) for b in r.boxes]
            self.frame = f

    def find(self, label):
        """여러 프레임에서 label을 찾아 가장 많이 나온 칸을 고른다. 절반 미만이면 None."""
        votes = [0, 0, 0]
        for _ in range(FIND_FRAMES):
            best = max((d for d in self.dets if d[0] == label), key=lambda d: d[1], default=None)
            if best:
                cx = (best[2][0] + best[2][2]) / 2
                votes[min(2, int(cx / (self.frame.shape[1] / 3)))] += 1
            time.sleep(0.04)
        return (int(np.argmax(votes)), votes) if max(votes) >= FIND_FRAMES // 2 else (None, votes)


class Screen:
    def __init__(self, cam, hand, out):
        self.cam, self.hand, self.out, self.first = cam, hand, out, True

    def draw(self, lines, color, zone):
        f = self.cam.frame.copy()
        h, w = f.shape[:2]
        for k in (1, 2):
            cv2.line(f, (w * k // 3, 0), (w * k // 3, h), (255, 255, 255), 2)
        if zone is not None:
            cv2.rectangle(f, (w * zone // 3 + 3, 3), (w * (zone + 1) // 3 - 3, h - 3), (0, 255, 0), 4)
        for name, conf, (x1, y1, x2, y2) in self.cam.dets:
            cv2.rectangle(f, (int(x1), int(y1)), (int(x2), int(y2)), (0, 200, 255), 2)
            cv2.putText(f, f"{name} {conf:.2f}", (int(x1), int(y1) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        img = Image.fromarray(cv2.cvtColor(f, cv2.COLOR_BGR2RGB))
        d = ImageDraw.Draw(img)
        for i, (zn, wr) in enumerate(ZONES):
            d.text((w * i // 3 + 10, h - 40), f"{zn} (손목 {wr})", font=FONT[22], fill=(255, 255, 255),
                   stroke_width=2, stroke_fill=(0, 0, 0))
        if self.hand.dry and self.hand.last:
            d.text((10, h - 75), f"[로봇 없음] 보낼 명령: {self.hand.last}", font=FONT[22], fill=(255, 120, 255),
                   stroke_width=2, stroke_fill=(0, 0, 0))
        for i, t in enumerate(lines):
            d.text((10, 10 + 42 * i), t, font=FONT[30], fill=color, stroke_width=3, stroke_fill=(0, 0, 0))
        return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

    def hold(self, sec, lines, color=(255, 255, 0), zone=None, action=None):
        t0 = time.perf_counter()
        while (el := time.perf_counter() - t0) < sec:
            if action:
                action(el)
            v = self.draw(lines + [f"{sec - el:.1f}초"], color, zone)
            self.out.write(v)
            cv2.imshow(WIN, v)
            if self.first:
                cv2.moveWindow(WIN, 0, 0)
                cv2.setWindowProperty(WIN, cv2.WND_PROP_TOPMOST, 1)
                self.first = False
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):      # ESC 또는 q로 중단
                raise KeyboardInterrupt
            time.sleep(1 / 15)


# ---------------------------------------------------------------- 한 회차
def listen(asr, scr, title):
    audio = sd.rec(int(REC_SEC * SR), samplerate=SR, channels=1, dtype="float32")
    scr.hold(REC_SEC, [f"{title} 지금 말하세요!"], (0, 255, 255))
    sd.wait()
    a = audio[:, 0]
    loud = float(np.abs(a).max())
    if loud < MIN_LOUD:
        return None, f"(소리 작음 {loud:.3f})"
    text = asr.transcribe(a, language="ko", initial_prompt=HINT, fp16=(DEV == "cuda"))["text"].strip()
    found = [k for k in OBJECTS if k in text.replace(" ", "")]
    many = sum(text.count(v) for v in ("잡아", "집어")) >= 2     # 힌트 문장을 그대로 따라 쓴 경우
    if len(found) == 1 and not many:
        return found[0], text
    return None, (f"(힌트를 따라 쓴 것으로 보여 무시) {text}" if (len(found) > 1 or many) else text)


def one_round(rnd, total, hand, cam, scr, asr):
    title = f"[{rnd}/{total}]"
    scr.hold(6, [f"{title} 물건을 칸 안에 놓으세요", "다음: '공 잡아' 또는 '스프레이 집어'"])
    word, text = listen(asr, scr, title)
    t_said = time.perf_counter()
    if word is None:
        scr.hold(3, [f"{title} 들은 말: {text}", "명령을 못 알아들음 -> 그대로 대기"], (0, 0, 255))
        return {"round": rnd, "heard": text, "result": "명령 못 알아들음"}
    zone, votes = cam.find(OBJECTS[word])
    rec = {"round": rnd, "heard": text, "object": word, "votes": votes, "find_s": round(time.perf_counter() - t_said, 2)}
    if zone is None:
        hand.send(FIST, 90, 400)
        scr.hold(4, [f"{title} 들은 말: {text}", f"'{word}' 안 보임 -> 주먹"], (0, 0, 255))
        hand.send(OPEN, 90, 400)
        rec["result"] = "없음 -> 주먹"
        return rec
    zn, wr = ZONES[zone]
    hand.send(OPEN, wr, 600)
    scr.hold(5, [f"{title} 들은 말: {text}", f"'{word}' {zn} 칸 -> 손목 {wr}", "물건을 손바닥에 대 주세요"], (0, 255, 0), zone)
    scr.hold(1.2, [f"{title} 쥐는 중... 다 쥐면 손을 떼세요"], (0, 200, 255), zone,
             lambda el: hand.send([o + (c - o) * min(1, el) for o, c in zip(OPEN, FIST_TIGHT)], wr, 100))
    scr.hold(3, [f"{title} 쥐고 있기"], (0, 255, 0), zone)
    hand.send(OPEN, wr, 400)
    scr.hold(2.5, [f"{title} 내려놓기 (물건을 받아 주세요)"], (255, 255, 0), zone)
    hand.send(OPEN, 90, 600)
    rec.update(zone=zn, wrist=wr, result=f"{zn} 칸 -> 손목 {wr} -> 잡고 내려놓음")
    return rec


def look(hand):
    """손목 30 / 90 / 150에서 손바닥 방향 확인 (테스트 25 1단계)."""
    hand.send(OPEN, 90, 400)
    for w in (30, 90, 150):
        print(f"손목 {w}: 손바닥이 어디를 향하는지 보세요 (5초)")
        hand.send(OPEN, w, 800)
        time.sleep(5)
    hand.send(OPEN, 90, 800)
    print("끝. 30이 오른쪽, 150이 왼쪽이 아니면 voice_pick.py 위쪽의 ZONES 각도를 서로 바꿔 주세요.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rounds", type=int, default=4)
    ap.add_argument("--port", default=None, help="micro:bit 포트 (예: COM4). 없으면 자동")
    ap.add_argument("--cam", type=int, default=None, help="웹캠 번호. 없으면 0부터 찾음")
    ap.add_argument("--mic", default=None, help="마이크 이름 일부 (예: USB). 없으면 기본 마이크")
    ap.add_argument("--no-robot", action="store_true", help="로봇 없이 웹캠 + 음성만")
    ap.add_argument("--look", action="store_true", help="손목 방향만 확인")
    ap.add_argument("--list-mics", action="store_true")
    a = ap.parse_args()

    if a.list_mics:
        for i, d in enumerate(sd.query_devices()):
            if d["max_input_channels"] > 0:
                print(i, d["name"])
        return
    hand = Hand(a.port, dry=a.no_robot)
    if a.look:
        look(hand)
        hand.close()
        return
    if a.mic:
        ids = [i for i, d in enumerate(sd.query_devices()) if d["max_input_channels"] > 0 and a.mic.lower() in d["name"].lower()]
        if not ids:
            sys.exit(f"'{a.mic}'가 들어간 마이크가 없습니다. --list-mics 로 이름을 확인해 주세요.")
        sd.default.device = (ids[0], None)
    print("계산 장치:", DEV, "(cuda = 그래픽카드, cpu = 느리지만 동작)")

    import whisper
    from ultralytics import YOLO
    os.makedirs(os.path.join(HERE, "captures"), exist_ok=True)
    print("모델 불러오는 중... (처음 한 번은 자동으로 내려받아서 몇 분 걸릴 수 있음)")
    yolo = YOLO(os.path.join(HERE, "yolov8s-worldv2.pt") if os.path.exists(os.path.join(HERE, "yolov8s-worldv2.pt")) else "yolov8s-worldv2.pt")
    yolo.set_classes(sorted(set(OBJECTS.values())))
    asr = whisper.load_model(WHISPER_MODEL, device=DEV)
    asr.transcribe(np.zeros(SR, np.float32), language="ko", fp16=(DEV == "cuda"))   # 첫 실행 느림 방지
    cam = Cam(yolo, a.cam)
    cam.start()
    while cam.frame is None:
        time.sleep(0.05)
    stamp = time.strftime("%m%d_%H%M%S")
    out = cv2.VideoWriter(os.path.join(HERE, "captures", f"voice_pick_{stamp}.avi"), cv2.VideoWriter_fourcc(*"MJPG"), 15,
                          (cam.frame.shape[1], cam.frame.shape[0]))
    scr = Screen(cam, hand, out)
    log = []
    try:
        hand.send(OPEN, 90, 400)
        for rnd in range(1, a.rounds + 1):
            rec = one_round(rnd, a.rounds, hand, cam, scr, asr)
            log.append(rec)
            print(rec)
    except KeyboardInterrupt:
        print("중단")
    finally:
        hand.send(OPEN, 90, 400)
        hand.close()
        cam.running = False
        out.release()
        cv2.destroyAllWindows()
        path = os.path.join(HERE, "captures", f"voice_pick_{stamp}.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(log, f, ensure_ascii=False, indent=1)
        print("기록 저장:", path)


if __name__ == "__main__":
    main()
