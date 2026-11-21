"""라즈봇 V2 바퀴 (메카넘). 팀 car.py 와 같은 Raspbot_Lib 방식.

drive(vx, vy, w): vx 앞 m/s, vy 왼쪽 m/s, w 왼쪽 회전 rad/s
-> 잰 속도(speed_fwd / speed_side / speed_rot_deg)로 바퀴 세기로 바꾼다.
"""
import math
import sys


class RaspbotCar:
    def __init__(self, cfg):
        self.cfg = cfg
        try:
            import Raspbot_Lib
            self.bot = Raspbot_Lib.Raspbot()
        except Exception as e:
            sys.exit(f"Raspbot_Lib 를 못 불러옴 ({e}). 라즈봇 파이에서 실행하세요. 바퀴 없이 시험하려면 --no-motor")
        self.cmd = (0.0, 0.0, 0.0)

    def _wheel(self, name, p):
        p = max(-255, min(255, int(p)))
        if self.cfg["invert"][name]:
            p = -p
        mid = self.cfg["motor"][name]
        if hasattr(self.bot, "Ctrl_Muto"):
            self.bot.Ctrl_Muto(mid, p)
        else:
            self.bot.Ctrl_Car(mid, 0 if p >= 0 else 1, abs(p))

    def drive(self, vx, vy, w):
        c = self.cfg
        self.cmd = (vx, vy, w)
        px = vx / c["speed_fwd"] * c["power_ref"]
        py = vy / c["speed_side"] * c["power_ref"]
        pw = math.degrees(w) / c["speed_rot_deg"] * c["power_ref"]
        if c.get("flip_side"):
            py = -py
        fl, rl = px - py - pw, px + py - pw
        fr, rr = px + py + pw, px - py + pw
        m = max(abs(fl), abs(rl), abs(fr), abs(rr), 1e-9)
        k = min(1.0, c["power_max"] / m)
        for n, s in (("FL", fl), ("RL", rl), ("FR", fr), ("RR", rr)):
            self._wheel(n, s * k)

    def stop(self):
        self.cmd = (0.0, 0.0, 0.0)
        for n in ("FL", "RL", "FR", "RR"):
            self._wheel(n, 0)


class NoCar:
    """바퀴 없이 시험 (명령만 기록)"""

    def __init__(self, cfg=None):
        self.cmd = (0.0, 0.0, 0.0)

    def drive(self, vx, vy, w):
        self.cmd = (vx, vy, w)

    def stop(self):
        self.cmd = (0.0, 0.0, 0.0)
