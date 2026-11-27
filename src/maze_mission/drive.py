"""라즈봇 V2 바퀴 (메카넘). 팀 car.py 와 같은 Raspbot_Lib 방식.

drive(vx, vy, w): vx 앞 m/s, vy 왼쪽 m/s, w 왼쪽 회전 rad/s
-> 잰 속도(speed_fwd / speed_side / speed_rot_deg)로 바퀴 세기로 바꾼다.
"""
import math
import os
import sys

# 비상 정지: 이 파일이 생기면 모든 프로그램의 바퀴가 멈추고 프로그램이 끝난다.
#   파이에서  touch /tmp/robot_stop   (또는 python3 ~/stop.py)
ESTOP = "/tmp/robot_stop"


class EStop(KeyboardInterrupt):
    pass


class RaspbotCar:
    def __init__(self, cfg):
        self.cfg = cfg
        try:
            os.remove(ESTOP)                     # 새로 시작할 때만 지운다
        except OSError:
            pass
        try:
            import Raspbot_Lib
            self.bot = Raspbot_Lib.Raspbot()
        except Exception as e:
            sys.exit(f"Raspbot_Lib 를 못 불러옴 ({e}). 라즈봇 파이에서 실행하세요. 바퀴 없이 시험하려면 --no-motor")
        self.cmd = (0.0, 0.0, 0.0)
        self.out = (0.0, 0.0, 0.0)               # 실제로 보낸 속도 (서서히 따라감)
        self.t_last = None

    def _ramp(self, vx, vy, w):
        """손이 무거워서 급하게 서거나 출발하면 앞으로 고꾸라진다 -> 속도를 조금씩만 바꾼다"""
        import time
        now = time.time()
        dt = 0.1 if self.t_last is None else min(0.3, now - self.t_last)
        self.t_last = now
        acc = self.cfg.get("accel", 1.5)                     # m/s^2
        acc_w = math.radians(self.cfg.get("accel_rot_deg", 600))
        def step(cur, tgt, a):
            return cur + max(-a * dt, min(a * dt, tgt - cur))
        o = self.out
        self.out = (step(o[0], vx, acc), step(o[1], vy, acc), step(o[2], w, acc_w))
        return self.out

    def soft_stop(self, max_s=1.0):
        """서서히 멈춤 (보통 멈출 때). 급할 때는 stop()"""
        import time
        t0 = time.time()
        while any(abs(v) > 1e-3 for v in self.out) and time.time() - t0 < max_s:
            self.drive(0.0, 0.0, 0.0)
            time.sleep(0.03)
        self.stop()

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
        if os.path.exists(ESTOP):
            self.stop()
            raise EStop("비상 정지 파일 있음")
        if not all(math.isfinite(v) for v in (vx, vy, w)):
            return self.stop()                   # 계산이 꼬이면(nan) 무조건 멈춤
        c = self.cfg
        self.cmd = (vx, vy, w)
        vx, vy, w = self._ramp(vx, vy, w)
        # 앞으로 갈 때 저절로 도는 버릇 보정 (실측: 왼쪽으로 약 10도/초 @ 0.76m/s)
        w = w - math.radians(c.get("yaw_trim_deg", 0.0)) * vx
        px = vx / c["speed_fwd"] * c["power_ref"]
        py = vy / c["speed_side"] * c["power_ref"]
        # 11/26: 무게(손 + 건전지 상자) 때문에 바퀴 힘이 약 30 아래면 앞뒤/옆으로 안 움직임.
        # 느린 명령은 '끊어서 가기': 최소 힘으로 짧게 가고 쉬기를 0.2초마다 반복 -> 평균 속도는 명령대로
        mp = c.get("min_lin_power", 0)
        if mp > 0:
            import time as _t
            ph = (_t.time() % 0.2) / 0.2
            for nm in ("px", "py"):
                v = px if nm == "px" else py
                if abs(v) < 8:                     # 아주 작은 명령은 무시 (부풀리면 툭툭 밀림)
                    v = 0.0
                elif abs(v) < mp:
                    v = math.copysign(mp, v) if ph < abs(v) / mp else 0.0
                if nm == "px":
                    px = v
                else:
                    py = v
        pw = math.degrees(w) / c["speed_rot_deg"] * c["power_ref"]
        mr = c.get("min_rot_power", 0)           # 회전도 약하면 안 돎 -> 끊어서 돌기 (11/26)
        if mr > 0:
            import time as _t
            ph = (_t.time() % 0.2) / 0.2
            if abs(pw) < 2:
                pw = 0.0
            elif abs(pw) < mr:
                pw = math.copysign(mr, pw) if ph < abs(pw) / mr else 0.0
        if c.get("flip_side"):
            py = -py
        b = getattr(self, "boost", 1.0)          # 11/27: 명령했는데 안 움직이면 힘 더 (Boost)
        if b != 1.0:
            px, py, pw = px * b, py * b, pw * b
        fl, rl = px - py - pw, px + py - pw
        fr, rr = px + py + pw, px - py + pw
        m = max(abs(fl), abs(rl), abs(fr), abs(rr), 1e-9)
        k = min(1.0, c["power_max"] / m)
        for n, s in (("FL", fl), ("RL", rl), ("FR", fr), ("RR", rr)):
            self._wheel(n, s * k)

    def stop(self):
        self.cmd = self.out = (0.0, 0.0, 0.0)
        self.t_last = None
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

    def soft_stop(self, max_s=1.0):
        self.stop()


class Boost:
    """11/27: 가라고(돌라고) 했는데 1.5초 동안 거의 안 움직이면 바퀴 힘을 0.5초마다 25%씩 더 줌 (최대 2배).
    다시 움직이면 천천히 원래대로. car.boost 에 넣어서 씀.
      bst = Boost(); car.boost = bst.update(cmd, pose, now)"""

    def __init__(self, max_b=2.0):
        self.max_b, self.b, self.hist, self.t_up = max_b, 1.0, [], 0.0

    def update(self, cmd, pose, now):
        self.hist = [h for h in self.hist if now - h[0] <= 1.5] + [(now, cmd, pose)]
        h0 = self.hist[0]
        want_move = all(abs(c[0]) + abs(c[1]) > 0.05 or abs(c[2]) > 0.3 for _, c, _ in self.hist)
        if now - h0[0] < 1.2 or not want_move:
            if not want_move:
                self.b = max(1.0, self.b - 0.05)
            return self.b
        moved = math.hypot(pose[0] - h0[2][0], pose[1] - h0[2][1])
        rot = abs(math.atan2(math.sin(pose[2] - h0[2][2]), math.cos(pose[2] - h0[2][2])))
        lin = any(abs(c[0]) + abs(c[1]) > 0.05 for _, c, _ in self.hist)
        # 11/27 21:20: 제자리 회전에 힘을 올리면 너무 빨리 돌아 라이다 방향 맞추기가 놓침(실패 3.6% -> 18%) -> 회전만일 땐 안 올림
        stuck = lin and moved < 0.02
        if stuck and now - self.t_up > 0.5:
            self.b = min(self.max_b, self.b + 0.25)
            self.t_up = now
        elif not stuck:
            self.b = 1.0                                  # 움직이기 시작하면 바로 원래 힘 (천천히 내리면 그동안 너무 빨리 감)
        return self.b
