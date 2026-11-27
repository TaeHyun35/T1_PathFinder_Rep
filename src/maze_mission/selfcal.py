"""바닥 자가 보정 (파이). 라이다로 실제 움직임을 재서 이 바닥의 속도 비율/최소 속도/미끄러짐을 구한다.

  python3 selfcal.py --floor wood            전부 (약 10분)
  python3 selfcal.py --floor wood --only fwd,brake
결과: logs/floor_<바닥>_<시각>.json, 그리고 config.json 의 drive.speed_fwd / speed_side 를 이 바닥에 맞춤 (--no-save 면 안 바꿈)

안전: 뒤에는 물건이 붙어 있다고 보고 출발점보다 뒤로는 안 감 (돌아올 때 5cm 앞에서 멈춤).
      제자리 회전 전에는 25cm 앞으로 나가서 돈다 (뒷모서리/손이 벽을 긁지 않게).
      앞 손끝과 물체 사이가 15cm 보다 가까워지면 그 시험을 건너뜀. 비상: ~/stop.py
"""
import argparse
import json
import math
import os
import time

from common import HERE, load_config, save_config_patch
from heading import ScanOdom

FINGER = 0.315


class Rig:
    def __init__(self, cfg):
        from drive import RaspbotCar
        from lidar import RPLidar
        self.lid = RPLidar(cfg["lidar"])
        self.lid.wait_ready()
        t0 = time.time()                                    # 라이다가 제대로 보일 때까지 (막 켜지면 첫 바퀴가 비어 있음, 11/26)
        while time.time() - t0 < 15:
            s = self.lid.latest()
            if s is not None and sum(1 for p in s.pts if p[1] > 0) > 80 and s.ray(0, 6) == s.ray(0, 6):
                break
            time.sleep(0.3)
        else:
            raise SystemExit("라이다가 주변을 제대로 못 봄 (점이 너무 적음) -> 출발 안 함")
        time.sleep(0.5)
        self.cfg = cfg
        self.car = RaspbotCar(dict(cfg["drive"], accel=0.5, accel_rot_deg=120))
        self.odo = ScanOdom(self.lid.latest())          # 출발점 = (0,0,0)
        self.last = self.lid.latest()
        self.tp = time.time()

    def tick(self, cmd):
        """명령을 보내고 새 스캔이 왔으면 자세를 갱신. -> (시각, 자세)"""
        self.car.drive(*cmd)
        s = self.lid.latest()
        now = time.time()
        if s is not self.last:
            dt, self.tp, self.last = now - self.tp, now, s
            o = self.car.out
            self.odo.update(s, (o[0] * dt, o[1] * dt, o[2] * dt))
        return now, self.odo.pose

    def settle(self, secs=0.8):
        t0 = time.time()
        while time.time() - t0 < secs:
            self.tick((0, 0, 0))
            time.sleep(0.03)
        return self.odo.pose

    def front(self):
        return self.last.ray(0, 6)

    def side(self, left):
        return self.last.sector_min(90 if left else -90, 25)

    def run(self, cmd, secs, hard=False):
        """cmd 를 secs 초 -> 샘플 [(t, x, y, th)]. hard=True 면 바로 정지. 라이다 맞추기가 실패하면 self.bad = True"""
        samp, t0 = [], time.time()
        f0 = self.odo.fails
        while time.time() - t0 < secs:
            if cmd[0] > 0 and self.front() - FINGER < 0.15:
                print("    손끝 앞이 가까움 -> 이 시험 중단", flush=True)
                break
            t, p = self.tick(cmd)
            samp.append((t - t0, *p))
            time.sleep(0.03)
        t_stop, p_stop = time.time() - t0, self.odo.pose
        if hard:
            self.car.stop()
        else:
            self.car.soft_stop()
        p_end = self.settle(1.2)
        self.bad = self.odo.fails - f0 > 2
        if self.bad:
            print(f"    라이다 맞추기 {self.odo.fails - f0}번 실패 -> 이 측정은 버림", flush=True)
        return samp, t_stop, p_stop, p_end

    def goto(self, gx, gy=None, gth=None, v=0.10, timeout=12):
        """출발점 기준 (gx, gy, gth) 로 천천히 (라이다로 재며). 뒤로는 gx 보다 더 안 감"""
        p = self.odo.pose
        gy = p[1] if gy is None else gy
        gth = p[2] if gth is None else gth
        t0 = time.time()
        while time.time() - t0 < timeout:
            _, p = self.tick(self.car.cmd)
            ex, ey, eth = gx - p[0], gy - p[1], math.atan2(math.sin(gth - p[2]), math.cos(gth - p[2]))
            if abs(ex) < 0.015 and abs(ey) < 0.015 and abs(eth) < math.radians(2):
                break
            c, s = math.cos(p[2]), math.sin(p[2])
            rx, ry = c * ex + s * ey, -s * ex + c * ey
            vx = max(-v, min(v, 1.5 * rx))
            # 11/26: 옆 오차 작으면 옆으로 안 고침 (최소 힘 때문에 작은 옆 명령도 크게 툭툭 밀려 대각선이 됨)
            vy = max(-v, min(v, 1.5 * ry)) if abs(ry) > 0.03 else 0.0
            if vy and abs(rx) > 0.03:              # 옆으로 고칠 땐 앞뒤를 멈추고 옆으로만
                vx = 0.0
            w = max(-0.5, min(0.5, 2.5 * eth))
            if abs(eth) > math.radians(2) and abs(w) < math.radians(15):
                w = math.copysign(math.radians(15), eth)
            self.car.drive(vx, vy, w)
            time.sleep(0.03)
        self.car.soft_stop()
        return self.settle(0.6)

    def close(self):
        self.car.stop()
        self.lid.close()


def steady_rate(samp, k, t_from=0.7):
    """샘플에서 t_from 초 이후 구간의 기울기 (k=1 x, 2 y, 3 th)"""
    s = [q for q in samp if q[0] >= t_from]
    if len(s) < 3:
        return float("nan")
    n = len(s)
    mt = sum(q[0] for q in s) / n
    mv = sum(q[k] for q in s) / n
    num = sum((q[0] - mt) * (q[k] - mv) for q in s)
    den = sum((q[0] - mt) ** 2 for q in s) or 1e-9
    return num / den


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--floor", default="wood")
    ap.add_argument("--only", default="fwd,brake,side,rot")
    ap.add_argument("--no-save", action="store_true")
    a = ap.parse_args()
    cfg = load_config()
    rig = Rig(cfg)
    out = {"floor": a.floor, "time": time.strftime("%Y-%m-%d %H:%M:%S"), "fwd": [], "brake": [], "side": [], "rot": []}
    only = a.only.split(",")
    home_x = 0.05                                   # 돌아올 때 출발점보다 5cm 앞 (뒤에 물건이 붙어 있음)
    print(f"시작: 앞 {rig.front() * 100:.0f}cm, 왼쪽 {rig.side(True) * 100:.0f}cm, 오른쪽 {rig.side(False) * 100:.0f}cm", flush=True)
    try:
        if "fwd" in only:
            print("1) 느린 직진: 명령 속도별 실제 속도", flush=True)
            for v in (0.05, 0.08, 0.10, 0.15, 0.20):
                samp, *_ , p_end = rig.run((v, 0, 0), 2.5)
                real = steady_rate(samp, 1) if not rig.bad else float("nan")
                yaw = math.degrees(p_end[2])
                out["fwd"].append({"cmd": v, "real": round(real, 4), "dist": round(p_end[0], 3), "yaw_deg": round(yaw, 1)})
                print(f"   명령 {v * 100:4.0f}cm/s -> 실제 {real * 100:5.1f}cm/s (간 거리 {p_end[0] * 100:.1f}cm, 틀어짐 {yaw:+.1f}도)", flush=True)
                rig.goto(home_x, 0.0, 0.0)
        if "brake" in only:
            print("2) 제동: 멈추라고 한 뒤 더 미끄러진 거리", flush=True)
            for v in (0.20, 0.30, 0.40):
                for hard in (False, True):
                    samp, t_stop, p_stop, p_end = rig.run((v, 0, 0), 1.2, hard=hard)
                    v_at = steady_rate(samp, 1, t_from=0.6)
                    slide = p_end[0] - p_stop[0]
                    out["brake"].append({"cmd": v, "hard": hard, "v_at_stop": round(v_at, 3), "slide": round(slide, 4)})
                    print(f"   {v * 100:.0f}cm/s {'바로 서기' if hard else '부드럽게'}: 멈출 때 {v_at * 100:.0f}cm/s, 미끄러짐 {slide * 100:.1f}cm", flush=True)
                    rig.goto(home_x, 0.0, 0.0)
        if "side" in only:
            print("3) 옆으로: 실제 옆 속도와 새는 정도", flush=True)
            rig.goto(0.20, 0.0, 0.0)                    # 뒤 물건에서 떨어져서
            for v in (0.05, 0.10, 0.15):
                for left in (True, False):
                    need = v * 2.0 + 0.1
                    sd = rig.side(left)
                    if not math.isnan(sd) and sd - 0.28 < need:
                        print(f"   {'왼쪽' if left else '오른쪽'} 공간 {sd * 100:.0f}cm 모자람 -> 건너뜀", flush=True)
                        continue
                    samp, *_, p_end = rig.run((0, v if left else -v, 0), 2.0)
                    real = abs(steady_rate(samp, 2))
                    out["side"].append({"cmd": v, "left": left, "real": round(real, 4), "fwd_leak": round(p_end[0] - 0.20, 3),
                                        "yaw_deg": round(math.degrees(p_end[2]), 1)})
                    print(f"   {'왼쪽' if left else '오른쪽'} {v * 100:.0f}cm/s -> 실제 {real * 100:.1f}cm/s, 앞뒤로 샘 {(p_end[0] - 0.20) * 100:+.1f}cm, "
                          f"틀어짐 {math.degrees(p_end[2]):+.1f}도", flush=True)
                    rig.goto(0.20, 0.0, 0.0)
        if "rot" in only:
            print("4) 회전: 명령 각속도별 실제 각속도", flush=True)
            rig.goto(0.25, 0.0, 0.0)                    # 앞으로 나와서 돌기
            for wd in (20, 40, 80, 150):
                for sgn in (1, -1):
                    samp, *_ = rig.run((0, 0, sgn * math.radians(wd)), min(2.0, 120.0 / wd + 0.7))
                    real = abs(math.degrees(steady_rate(samp, 3, t_from=0.5)))
                    out["rot"].append({"cmd": wd, "dir": "left" if sgn > 0 else "right", "real": round(real, 1)})
                    print(f"   {'왼쪽' if sgn > 0 else '오른쪽'} {wd}도/초 -> 실제 {real:.0f}도/초", flush=True)
                    rig.goto(0.25, 0.0, 0.0)
        rig.goto(home_x, 0.0, 0.0)
    except KeyboardInterrupt:
        print("중간에 멈춤", flush=True)
    finally:
        rig.close()

    # 이 바닥에 맞춘 값
    rec = {}
    f = [r for r in out["fwd"] if 0.08 <= r["cmd"] <= 0.15 and r["real"] == r["real"] and r["real"] > 0.01]
    if f:
        ratio = sum(r["real"] / r["cmd"] for r in f) / len(f)
        rec["speed_fwd"] = round(cfg["drive"]["speed_fwd"] * ratio, 3)
        out["fwd_ratio"] = round(ratio, 3)
    s = [r for r in out["side"] if r["cmd"] >= 0.10 and r["real"] > 0.01]
    if s:
        ratio = sum(r["real"] / r["cmd"] for r in s) / len(s)
        rec["speed_side"] = round(cfg["drive"]["speed_side"] * ratio, 3)
        out["side_ratio"] = round(ratio, 3)
    soft = [r["slide"] for r in out["brake"] if not r["hard"]]
    if soft:
        out["brake_slide_max"] = round(max(soft), 3)
    out["recommend"] = rec
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    p = os.path.join(HERE, "logs", f"floor_{a.floor}_{time.strftime('%Y%m%d_%H%M%S')}.json")
    json.dump(out, open(p, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("저장:", p, flush=True)
    print("이 바닥에 맞춘 값:", rec, flush=True)
    # 이상한 값은 저장 안 함 (11/26: 배터리 약할 때 잰 0.21 이 저장돼 힘이 4배가 될 뻔함)
    bad = [k for k, r in (("fwd", out.get("fwd_ratio")), ("side", out.get("side_ratio"))) if r is not None and not 0.6 <= r <= 1.5]
    if bad or len(f) < 2:
        print(f"측정값이 이상하거나 모자람 ({bad or '직진 측정 2개 미만'}) -> 설정에 저장 안 함. 배터리/바퀴 확인 후 다시", flush=True)
        rec = {}
    if rec and not a.no_save:
        save_config_patch({"drive": rec, "floor": {"name": a.floor, "brake_slide": out.get("brake_slide_max")}})
        print("config.json 에 저장함", flush=True)


if __name__ == "__main__":
    main()
