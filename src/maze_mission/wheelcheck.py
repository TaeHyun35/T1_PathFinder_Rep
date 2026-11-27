"""바퀴 하나씩 점검 (로봇을 들어서 바퀴가 바닥에 안 닿게 하고 실행).

  python3 wheelcheck.py --power 40
각 바퀴: 앞으로 3초 -> 1초 쉼 -> 뒤로 3초. 순서 FL(왼앞) RL(왼뒤) FR(오른앞) RR(오른뒤).
멈춤: ~/stop.py 또는 Ctrl+C
"""
import argparse
import os
import time

from common import load_config
from drive import ESTOP, RaspbotCar

NAMES = {"FL": "왼쪽 앞", "RL": "왼쪽 뒤", "FR": "오른쪽 앞", "RR": "오른쪽 뒤"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--power", type=int, default=40)
    ap.add_argument("--secs", type=float, default=3.0)
    a = ap.parse_args()
    car = RaspbotCar(load_config()["drive"])
    try:
        for n in ("FL", "RL", "FR", "RR"):
            for sgn, word in ((1, "앞으로"), (-1, "뒤로")):
                print(f"{NAMES[n]} 바퀴 {word} (힘 {a.power})", flush=True)
                t0 = time.time()
                while time.time() - t0 < a.secs:
                    if os.path.exists(ESTOP):
                        raise KeyboardInterrupt
                    car._wheel(n, sgn * a.power)
                    time.sleep(0.05)
                car._wheel(n, 0)
                time.sleep(1.0)
    except KeyboardInterrupt:
        print("멈춤")
    finally:
        car.stop()


if __name__ == "__main__":
    main()
