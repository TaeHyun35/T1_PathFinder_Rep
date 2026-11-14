"""AiHand 손 조종 (노트북에서 실행, micro:bit USB 연결).

micro:bit에는 microbit-serial-range.hex가 올라가 있어야 한다.

쓰는 법:
  python hand.py              명령을 한 줄씩 입력
  python hand.py --port COM5  포트 직접 정하기

명령:
  o            손 펴기
  f            주먹
  g            꽉 쥐기 (공·스프레이 잡을 때 값)
  w 90         손목 각도 (30 오른쪽 / 90 정면 / 150 왼쪽)
  t            받침대 빼기 시험: 손 펴기 -> 3초 뒤 꽉 쥐기 -> 받침대 빼기 안내 -> 5초 들고 있기 -> 펴기
  r            손목 흔들기 시험: 공 쥔 채로 손목 30 -> 150 -> 90 (차 흔들림 보기)
  p 50,130,130,130,130,90    각도 6개 직접 (엄지,검지,중지,약지,새끼,손목)
  q            끝내기
"""
import argparse
import sys
import time

OPEN = [50, 130, 130, 130, 130]
FIST = [130, 50, 50, 50, 50]
TIGHT = [140, 30, 20, 20, 20]


class Hand:
    def __init__(self, port=None):
        import serial
        from serial.tools import list_ports
        if port is None:
            for p in list_ports.comports():
                text = f"{p.description} {p.manufacturer or ''} {p.hwid}".lower()
                if "mbed" in text or "0d28" in text or "micro:bit" in text or "microbit" in text:
                    port = p.device
                    break
        if port is None:
            ports = ", ".join(p.device for p in list_ports.comports()) or "없음"
            sys.exit(f"micro:bit 포트를 못 찾았습니다 (보이는 포트: {ports}). --port COM5 처럼 적어 주세요.")
        print("micro:bit 포트:", port)
        self.ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(0.5)
        self.fingers, self.wrist = list(OPEN), 90

    def send(self, fingers=None, wrist=None, move=400):
        if fingers is not None:
            self.fingers = list(fingers)
        if wrist is not None:
            self.wrist = int(wrist)
        line = ",".join(str(int(a)) for a in self.fingers) + f",{self.wrist},{int(move)}"
        self.ser.reset_input_buffer()
        self.ser.write((line + "\n").encode())
        ok = self.ser.readline().decode(errors="replace").strip()
        print(f"  -> {line}   ({ok or '응답 없음'})")


def countdown(msg, sec):
    for i in range(sec, 0, -1):
        print(f"  {msg} {i}...", flush=True)
        time.sleep(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    a = ap.parse_args()
    h = Hand(a.port)
    h.send(OPEN, 90)
    print(__doc__.split("명령:")[1])
    while True:
        try:
            c = input("명령> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not c:
            continue
        k, *rest = c.split(maxsplit=1)
        k = k.lower()
        try:
            if k == "q":
                break
            elif k == "o":
                h.send(OPEN)
            elif k == "f":
                h.send(FIST)
            elif k == "g":
                h.send(TIGHT, move=600)
            elif k == "w":
                h.send(wrist=int(rest[0]), move=600)
            elif k == "p":
                v = [int(x) for x in rest[0].replace(" ", "").split(",")]
                h.send(v[:5], v[5] if len(v) > 5 else None)
            elif k == "t":
                h.send(OPEN)
                countdown("공을 받침대 위, 손바닥 아래에 맞추세요. 쥐기까지", 3)
                h.send(TIGHT, move=600)
                time.sleep(1.0)
                print("  지금 받침대를 빼세요!")
                countdown("들고 있기", 5)
                ans = input("  공이 손에 남았나요? (y/n, 메모는 뒤에): ")
                print(f"  기록: {time.strftime('%H:%M:%S')} 받침대빼기 {ans}")
                h.send(OPEN)
            elif k == "r":
                h.send(TIGHT, move=600)
                time.sleep(1)
                for w in (30, 150, 90):
                    h.send(wrist=w, move=800)
                    time.sleep(1.5)
            else:
                print("모르는 명령입니다.")
        except (IndexError, ValueError):
            print("형식이 틀렸습니다. 예: w 90 / p 50,130,130,130,130,90")
    h.send(OPEN, 90)


if __name__ == "__main__":
    main()
