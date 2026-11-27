"""집에서 하는 시험 (미로, 피코 없이). 라즈베리파이에서 실행. 결과는 logs/hometest_*.json 에 쌓인다.

  python3 hometest.py lidar               라이다 방향별 응답률 + 앞이 가려졌는지 판정
  python3 hometest.py straight            앞으로 가기: 라이다로 실제 간 거리 자동 측정 -> speed_fwd
  python3 hometest.py side                옆으로 가기: 같은 방식 -> speed_side
  python3 hometest.py rot                 제자리 회전: 라이다 모양 비교로 돈 각도 자동 측정 -> speed_rot_deg
  python3 hometest.py stopwall            벽으로 천천히 가다가 40cm 에서 멈추기 (멈춘 거리 오차)
  python3 hometest.py follow --secs 20    방 벽을 오른쪽에 두고 따라가기 (벽 거리 유지 오차)
  python3 hometest.py keys --n 10         손으로 F1~F4 를 10번씩 눌러 키보드 입력을 자동 확인
  python3 hometest.py keys --code 4213    정해진 순서로 누르기
  python3 hometest.py see                 동물 사진/마커 인식률 표 (조건마다 20장)

공통: --no-save (config 에 저장 안 함). 급할 때 Ctrl+C -> 바퀴 멈춤.
바퀴 시험은 충전 케이블을 뽑고, 주변 1m 를 비우고 한다.
"""
import argparse
import glob
import json
import math
import os
import statistics
import struct
import sys
import time

from common import HERE, load_config, save_config_patch, wrap_deg

RESULTS = []


def record(name, **kw):
    kw.update(test=name, time=time.strftime("%Y-%m-%d %H:%M:%S"))
    RESULTS.append(kw)
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    path = os.path.join(HERE, "logs", time.strftime("hometest_%Y%m%d.jsonl"))
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(kw, ensure_ascii=False) + "\n")


def ask_yes(msg):
    return input(msg + " (y/엔터=아니오) ").strip().lower() == "y"


# ------------------------------------------------------------------ 라이다 도우미
def fresh_scans(lid, k):
    out, last = [], None
    t0 = time.time()
    while len(out) < k and time.time() - t0 < 10:
        s = lid.latest()
        if s is not None and s is not last:
            out.append(s)
            last = s
        time.sleep(0.03)
    return out


def profile(scans):
    """1도 칸마다 거리 중간값 (360칸, 모르면 nan). 인덱스 = 로봇기준 각도 + 180"""
    bins = [[] for _ in range(360)]
    for s in scans:
        for a, d in s.pts:
            if d > 0:
                bins[int(round(a + 180)) % 360].append(d)
    return [statistics.median(b) if b else math.nan for b in bins]


def scan_xy(scans, rmax=5.0):
    """여러 스캔의 점들을 (x 앞, y 왼쪽) 로. 3cm 칸마다 한 점"""
    import numpy as np
    pts = [(d * math.cos(math.radians(a)), d * math.sin(math.radians(a)))
           for s in scans for a, d in s.pts if 0.15 < d < rmax]
    p = np.array(pts)
    _, idx = np.unique(np.round(p / 0.03), axis=0, return_index=True)
    return p[np.sort(idx)]


def icp_motion(before, after, guess=(0.0, 0.0, 0.0)):
    """출발 전/후 스캔으로 로봇이 실제로 움직인 양 (앞 m, 왼쪽 m, 왼쪽회전 도).
    이동과 회전을 같이 맞추므로 멀리 가도 각도가 맞다. 맞춘 점 비율도 같이 준다."""
    import numpy as np
    ref, src = scan_xy(before), scan_xy(after)
    tx, ty, th = guess[0], guess[1], math.radians(guess[2])
    good = 0
    for it in range(40):
        c, s = math.cos(th), math.sin(th)
        q = src @ np.array([[c, s], [-s, c]]) + (tx, ty)       # 후 스캔 -> 전 좌표
        d2 = ((q[:, None, :] - ref[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        dist = np.sqrt(d2[np.arange(len(q)), j])
        thr = max(0.05, 0.4 * 0.85 ** it)
        k = dist < thr
        good = k.sum()
        if good < 20:
            break
        A, B = q[k], ref[j[k]]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        dth = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        R = np.array([[math.cos(dth), -math.sin(dth)], [math.sin(dth), math.cos(dth)]])
        t = cb - R @ ca
        # 지금 변환에 이번 보정을 얹는다
        nt = R @ np.array([tx, ty]) + t
        tx, ty, th = nt[0], nt[1], th + dth
        if abs(dth) < 1e-5 and np.hypot(*t) < 1e-5:
            break
    return tx, ty, math.degrees(th), good / max(1, len(src))


def dir_dist(prof, center, half=4):
    v = [prof[(int(center) + 180 + k) % 360] for k in range(-half, half + 1)]
    v = [x for x in v if not math.isnan(x)]
    return statistics.median(v) if len(v) >= 3 else math.nan


def rotation_between(p_before, p_after, max_deg=179):
    """두 모양을 돌려 맞춰서 로봇이 왼쪽(+)으로 몇 도 돌았는지. (점수 낮을수록 잘 맞음)"""
    best, best_s = None, 0
    for s in range(-max_deg, max_deg + 1):
        err = n = 0
        for i in range(360):
            a, b = p_before[i], p_after[(i - s) % 360]
            if not (math.isnan(a) or math.isnan(b)):
                err += min(abs(a - b), 0.5)
                n += 1
        if n > 90:
            sc = err / n
            if best is None or sc < best:
                best, best_s = sc, s
    return best_s, best


# ------------------------------------------------------------------ 바퀴 시험
def setup_drive(cfg, need_car=True):
    from lidar import RPLidar
    lid = RPLidar(cfg["lidar"])
    lid.wait_ready()
    car = None
    if need_car:
        from drive import RaspbotCar
        car = RaspbotCar(cfg["drive"])
    return lid, car


def timed(car, vx, vy, w, sec):
    try:
        t0 = time.time()
        while time.time() - t0 < sec:            # 계속 다시 보내야 비상 정지 파일을 바로 본다
            car.drive(vx, vy, w)
            time.sleep(0.05)
        car.soft_stop()                          # 서서히 서기: 출발 때 덜 간 만큼 설 때 더 가서 총 거리 = 속도 x 시간
    finally:
        car.stop()
    time.sleep(0.6)                                  # 멈추고 흔들림 가라앉기


def timed_hold(car, lid, v, sec):
    """앞으로 sec 초. 가는 동안 라이다로 방향/옆 샘을 재서 출발 때 직선을 따라가게 계속 고친다"""
    from heading import LineHold, ScanOdom
    last = lid.latest()
    odo, ctl = ScanOdom(last), LineHold(v)
    cmd, t0 = (v, 0.0, 0.0), time.time()
    tp, worst = t0, 0.0
    slow = 0.5                                     # 마지막 0.5초는 속도를 줄이면서도 방향은 계속 고친다
    try:
        while True:
            el = time.time() - t0
            if el >= sec + slow:
                break
            ctl.v = v * max(0.0, min(1.0, (sec + slow - el) / slow))
            s = lid.latest()
            if s is not last:
                now = time.time()
                dt, tp, last = now - tp, now, s
                pose = odo.update(s, (cmd[0] * dt, cmd[1] * dt, cmd[2] * dt))
                worst = max(worst, abs(math.degrees(pose[2])))
                cmd = ctl(pose, dt)
            car.drive(*cmd)
            time.sleep(0.03)
        car.soft_stop()
    finally:
        car.stop()
    time.sleep(0.6)
    print(f"    (가는 중 라이다로 본 최대 틀어짐 {worst:.1f}도, 맞춤 실패 {odo.fails}번)")


def test_lidar(cfg, a):
    from lidar import RPLidar
    c = dict(cfg["lidar"], min_range=0.0, ignore=[])
    lid = RPLidar(c)
    lid.wait_ready(15)
    scans = fresh_scans(lid, 15)
    lid.close()
    rows = []
    print(f"회전 {lid.hz:.1f}Hz, 스캔 {len(scans)}번. 각도는 로봇 기준 (앞 0, 왼쪽 +)")
    for lo in range(-180, 180, 30):
        cov = statistics.mean(s.coverage(lo + 15, 15) for s in scans)
        near = [s.sector_min(lo + 15, 15) for s in scans]
        near = [x for x in near if not math.isnan(x)]
        nd = min(near) if near else math.nan
        rows.append({"from": lo, "to": lo + 30, "coverage": round(cov, 2), "nearest_m": None if math.isnan(nd) else round(nd, 2)})
        print(f"  {lo:+4d}~{lo + 30:+4d}도  응답 {cov:4.0%} {'#' * int(cov * 10):<10}  가장 가까운 {nd:5.2f}m{'   <- 거의 안 보임' if cov < 0.3 else ''}")
    front = statistics.mean(s.coverage(0, 30) for s in scans)
    verdict = "앞쪽 정상" if front >= 0.5 else "앞쪽이 가려짐 (손/카메라) -> 라이다를 더 높이 올려야 주행 시험 가능"
    print(f"\n앞 ±30도 응답률 {front:.0%} -> {verdict}")
    print("확인: 상자를 로봇 정면 50cm 에 두고 다시 돌렸을 때 '0~30도' 가장 가까운 값이 0.5 근처면 앞 방향(front_offset)도 맞음")
    record("lidar", hz=round(lid.hz, 1), front_coverage=round(front, 2), sectors=rows, verdict=verdict)


def test_straight(cfg, a, side=False):
    lid, car = setup_drive(cfg)
    c = cfg["drive"]
    v = a.speed or (c["speed_side"] if side else c["speed_fwd"])
    sec = a.secs or 1.5
    if a.hold:
        print("라이다로 방향 유지하면서 직진")
    axis = (90, -90) if side else (0, 180)         # 재는 방향: 가는 쪽 / 반대쪽
    print(f"{'왼쪽 옆으로' if side else '앞으로'} 세기 {c['power_ref']} 로 {sec}초. 가는 방향이나 반대쪽에 3m 안쪽 벽이 있어야 함.")
    res = []
    for trial in range(a.n):
        s0 = fresh_scans(lid, 5)
        p0 = profile(s0)
        d_go0, d_back0 = dir_dist(p0, axis[0]), dir_dist(p0, axis[1])
        if not ask_yes(f"[{trial + 1}/{a.n}] 출발할까요?"):
            break
        if a.hold and not side:
            timed_hold(car, lid, v, sec)
        else:
            timed(car, 0 if side else v, v if side else 0, 0, sec)
        s1 = fresh_scans(lid, 5)
        p1 = profile(s1)
        d_go1, d_back1 = dir_dist(p1, axis[0]), dir_dist(p1, axis[1])
        guess = (0, v * sec, 0) if side else (v * sec, 0, 0)
        fx, fy, yaw, fit = icp_motion(s0, s1, guess)            # 이동+회전을 같이 맞춤
        along, across = (fy, fx) if side else (fx, fy)
        old = [x for x in (d_go0 - d_go1, d_back1 - d_back0) if not math.isnan(x)]
        if fit >= 0.2:
            m = along
            how = f"ICP 맞춤 {fit:.0%}"
        elif old:
            m, yaw, across = statistics.mean(old), math.nan, math.nan
            how = "ICP 실패 -> 벽 거리로만 잼 (각도 모름)"
        else:
            print("  주변 모양이 안 맞고 앞뒤 벽도 안 보여서 못 잼")
            continue
        res.append(m / sec)
        print(f"  간 거리 {m * 100:.1f}cm, 옆으로 샘 {across * 100:+.1f}cm, 틀어진 각도 {yaw:+.1f}도 (왼쪽 +)"
              f" -> 속도 {m / sec:.3f} m/s  [{how}; 벽거리법 {statistics.mean(old) * 100 if old else math.nan:.1f}cm]")
        record("side" if side else "straight", sec=sec, moved_m=round(m, 4), across_m=None if math.isnan(across) else round(across, 4),
               yaw_deg=None if math.isnan(yaw) else round(yaw, 1), fit=round(fit, 2), speed=round(m / sec, 4))
    car.stop()
    lid.close()
    if res:
        sp = round(statistics.median(res), 4)
        key = "speed_side" if side else "speed_fwd"
        print(f"\n{key}: 지금 {c[key]} -> 잰 값 {sp}  ({len(res)}번 중간값)")
        if not a.no_save and ask_yes("config.json 에 저장할까요?"):
            save_config_patch({"drive": {key: sp}})


def test_rot(cfg, a):
    lid, car = setup_drive(cfg)
    c = cfg["drive"]
    sec = a.secs or 1.0
    w = math.radians(c["speed_rot_deg"])
    print(f"제자리 회전 세기 {c['power_ref']} 로 {sec}초 (예상 {c['speed_rot_deg'] * sec:.0f}도). 주변에 가구/벽이 보이는 곳에서.")
    res = []
    for trial in range(a.n):
        sgn = 1 if trial % 2 == 0 else -1
        p0 = profile(fresh_scans(lid, 5))
        if not ask_yes(f"[{trial + 1}/{a.n}] {'왼쪽' if sgn > 0 else '오른쪽'}으로 돌까요?"):
            break
        timed(car, 0, 0, sgn * w, sec)
        p1 = profile(fresh_scans(lid, 5))
        rot, score = rotation_between(p0, p1)
        res.append(abs(rot) / sec)
        print(f"  돈 각도 {rot:+d}도 (맞춤 오차 {score:.3f}m) -> {abs(rot) / sec:.0f} 도/초")
        record("rot", sec=sec, dir="left" if sgn > 0 else "right", rot_deg=rot, match_err=round(score, 3))
    lid.close()
    if res:
        sp = round(statistics.median(res), 1)
        print(f"\nspeed_rot_deg: 지금 {c['speed_rot_deg']} -> 잰 값 {sp}")
        print("  (돈 각도가 170도 넘게 나오면 시간을 줄여서 다시: --secs 0.6)")
        if not a.no_save and ask_yes("config.json 에 저장할까요?"):
            save_config_patch({"drive": {"speed_rot_deg": sp}})


def test_stopwall(cfg, a):
    lid, car = setup_drive(cfg)
    target = a.target
    for v in (0.10, 0.15):
        p = profile(fresh_scans(lid, 3))
        d0 = dir_dist(p, 0)
        if math.isnan(d0):
            print("앞이 안 보임 (손이 가리거나 벽이 너무 멂). 라이다를 올리거나 벽 1~2m 앞에 놓고 다시")
            break
        if not ask_yes(f"속도 {v}m/s 로 앞 벽({d0:.2f}m)까지 가서 {target}m 에서 멈춥니다. 출발?"):
            break
        t0 = time.time()
        try:
            while time.time() - t0 < 15:
                s = lid.latest()
                f = s.sector_min(0, 15)
                if math.isnan(f) or f <= target:
                    break
                car.drive(v, 0, 0)
                time.sleep(0.05)
        finally:
            car.stop()
        time.sleep(0.8)
        d1 = dir_dist(profile(fresh_scans(lid, 3)), 0)
        print(f"  멈춘 거리 {d1:.3f}m (목표 {target}) -> {'더 감' if d1 < target else '덜 감'} {abs(d1 - target) * 100:.1f}cm")
        record("stopwall", speed=v, target=target, stopped_m=round(d1, 3), err_cm=round((d1 - target) * 100, 1))
        if not ask_yes("뒤로 50cm 물러날까요?"):
            break
        timed(car, -0.12, 0, 0, 0.5 / 0.12)
    lid.close()


def test_brake(cfg, a):
    """제동 거리: 속도 v 로 가다가 '멈춰' 명령 순간부터 실제로 몇 cm 더 밀려 가는지.
    soft = 평소 멈춤(서서히), hard = 비상 정지(바로). 매번 라이다로 출발점까지 되돌아온다."""
    from heading import LineHold, ScanOdom
    lid, car = setup_drive(cfg)
    speeds = [float(x) for x in (a.speeds or "0.2,0.3,0.4").split(",")]
    run_s = a.secs or 1.2
    print(f"속도 {speeds} 마다 서서히/바로 멈춤. {run_s}초 가다가 멈춤 -> 밀린 거리 측정 -> 출발점으로 후진")
    rows = []
    for v in speeds:
        for mode in ("soft", "hard"):
            if not ask_yes(f"[{v} m/s, {'서서히' if mode == 'soft' else '바로'} 멈춤] 출발?"):
                lid.close()
                return
            last = lid.latest()
            odo, ctl = ScanOdom(last), LineHold(v)
            t0 = tp = t_scan = time.time()
            phase, x_cmd, t_cmd, dt = "run", None, None, 0.0
            try:
                while True:
                    now = time.time()
                    s = lid.latest()
                    if s is not last:
                        dt, tp, last, t_scan = now - tp, now, s, now
                        o = car.out
                        odo.update(s, (o[0] * dt, o[1] * dt, o[2] * dt))
                    if phase == "run" and now - t0 >= run_s:
                        phase, t_cmd = "stop", now
                        x_cmd = odo.pose[0] + car.out[0] * (now - t_scan)   # 마지막 스캔 뒤 간 만큼 보탬
                        if mode == "hard":
                            car.stop()
                    if phase == "run":
                        car.drive(*ctl(odo.pose, max(dt, 0.03)))
                    elif mode == "soft":
                        car.drive(0.0, 0.0, 0.0)                         # 서서히 (가속 제한으로 줄어듦)
                    else:
                        car.stop()
                    if phase == "stop" and now - t_cmd > 1.5:
                        break
                    time.sleep(0.03)
            finally:
                car.stop()
            for s in fresh_scans(lid, 3):                                # 멈춘 뒤 자세 다시 확인
                odo.update(s)
            slide = odo.pose[0] - x_cmd
            yaw = math.degrees(odo.pose[2])
            print(f"  {v} m/s {mode:4s}: 멈춰 명령 뒤 {slide * 100:5.1f}cm 더 감 (전체 {odo.pose[0] * 100:.1f}cm, 각도 {yaw:+.1f}도, 맞춤실패 {odo.fails})")
            rows.append((v, mode, slide))
            record("brake", speed=v, mode=mode, slide_m=round(slide, 4), total_m=round(odo.pose[0], 4), yaw_deg=round(yaw, 1), fails=odo.fails)
            # 출발점으로 되돌아가기 (라이다로 위치 보며 후진)
            back, t1 = LineHold(-0.2), time.time()
            last, tp = lid.latest(), time.time()
            t_max = max(0.0, odo.pose[0]) / 0.2 + 1.0        # 라이다가 이상해도 간 만큼만 후진
            try:
                while odo.pose[0] > 0.02 and time.time() - t1 < t_max:
                    s = lid.latest()
                    if s is not last:
                        now = time.time()
                        dt, tp, last = now - tp, now, s
                        o = car.out
                        odo.update(s, (o[0] * dt, o[1] * dt, o[2] * dt))
                    car.drive(*back(odo.pose, 0.05))
                    time.sleep(0.03)
                car.soft_stop()
            finally:
                car.stop()
            time.sleep(0.6)
    lid.close()
    print("\n정리 (멈춰 명령 뒤 더 간 거리):")
    for v, mode, sl in rows:
        print(f"  {v:.1f} m/s  {'서서히' if mode == 'soft' else '바로  '}  {sl * 100:5.1f}cm   (이론: 서서히 = v^2/2a = {v * v / (2 * cfg['drive'].get('accel', 1.5)) * 100:.1f}cm)")


def test_follow(cfg, a):
    from mission import follow
    lid, car = setup_drive(cfg)
    side = cfg["wall"]["side"]
    print(f"방 벽을 {'오른쪽' if side == 'right' else '왼쪽'}에 두고 벽에서 약 {cfg['wall']['d_ref']}m 에 놓으세요. {a.secs or 20}초 따라갑니다.")
    if not ask_yes("출발?"):
        return
    errs, tags, t0 = [], {}, time.time()
    s_side = -90 if side == "right" else 90
    try:
        while time.time() - t0 < (a.secs or 20):
            s = lid.latest()
            vx, vy, w, tag = follow(s, cfg, side)
            car.drive(vx, vy, w)
            tags[tag] = tags.get(tag, 0) + 1
            d = s.ray(s_side + (1 if side == "right" else -1) * cfg["wall"].get("skew_deg", 0))
            if tag == "벽따라" and not math.isnan(d):
                errs.append(d - cfg["wall"]["d_ref"])
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        car.stop()
    lid.close()
    if errs:
        mae = statistics.mean(abs(e) for e in errs)
        print(f"벽 거리 오차: 평균 {mae * 100:.1f}cm, 가장 큰 {max(abs(e) for e in errs) * 100:.1f}cm")
    print("상태별 횟수:", tags)
    record("follow", secs=a.secs or 20, mean_abs_err_cm=round(statistics.mean(abs(e) for e in errs) * 100, 1) if errs else None, tags=tags)


# ------------------------------------------------------------------ 키보드 (파이에 수신기를 꽂았을 때 자동 확인)
F_CODES = {59: "1", 60: "2", 61: "3", 62: "4"}           # 리눅스 KEY_F1..F4


class KeyEvents:
    """/dev/input 키보드 이벤트를 직접 읽음 (설치 필요 없음). 권한: input 그룹 또는 sudo"""

    def __init__(self):
        devs = []
        try:
            block = open("/proc/bus/input/devices").read().split("\n\n")
        except OSError:
            block = []
        for b in block:
            if "kbd" in b and "EV=" in b:
                name = next((l.split("=", 1)[1].strip('"') for l in b.splitlines() if l.startswith("N: Name=")), "?")
                for h in b.split("Handlers=")[-1].split():
                    if h.startswith("event"):
                        devs.append(("/dev/input/" + h, name))
        self.fds = []
        for path, name in devs:
            try:
                self.fds.append((os.open(path, os.O_RDONLY | os.O_NONBLOCK), name))
            except PermissionError:
                print(f"  {path} ({name}) 읽기 권한 없음 -> sudo usermod -aG input $USER 후 다시 로그인, 또는 sudo 로 실행")
        if not self.fds:
            raise RuntimeError("키보드 장치를 못 찾음 (수신기가 파이에 꽂혀 있는지 확인)")
        print("키보드 장치:", ", ".join(n for _, n in self.fds))
        self.fmt = "llHHi"
        self.size = struct.calcsize(self.fmt)

    def drain(self):
        keys = []
        for fd, _ in self.fds:
            while True:
                try:
                    data = os.read(fd, self.size * 64)
                except BlockingIOError:
                    break
                if not data:
                    break
                for i in range(0, len(data) - self.size + 1, self.size):
                    _, _, typ, code, val = struct.unpack(self.fmt, data[i:i + self.size])
                    if typ == 1 and val == 1:              # 키 누름
                        keys.append(F_CODES.get(code, f"코드{code}"))
        return keys


def test_keys(cfg, a):
    from hand import open_hand
    hc = cfg["hand"]
    try:
        kb = None if a.manual else KeyEvents()
    except RuntimeError as e:
        print(e, "-> 손으로 확인하는 방식(--manual)으로 진행")
        kb = None
    h = open_hand(hc)
    names = {2: "검지", 3: "중지", 4: "약지", 5: "새끼"}

    def one(digit):
        f = hc["key_finger"][digit]
        if kb:
            kb.drain()
        h.press(f)
        time.sleep(0.4)
        if kb:
            got = kb.drain()
        else:
            got = [digit] if ask_yes(f"  F{digit} 눌렸나요?") else []
        n_t = got.count(digit)
        kind = "ok" if (n_t == 1 and len(got) == 1) else "miss" if not got else "double" if n_t > 1 and len(got) == n_t else "wrong"
        return f, got, kind

    try:
        if a.code:
            res = []
            for trial in range(a.n):
                got_all = []
                for d in a.code:
                    _, got, _ = one(d)
                    got_all += got
                ok = "".join(got_all) == a.code
                res.append(ok)
                print(f"  [{trial + 1}] 목표 {a.code} -> 들어온 {''.join(got_all) or '(없음)'} {'O' if ok else 'X'}")
                record("keys_code", code=a.code, got="".join(got_all), ok=ok)
            print(f"\n비밀번호 {a.code}: {sum(res)}/{len(res)} 성공")
        else:
            table = {}
            for d in sorted(hc["key_finger"]):
                f = hc["key_finger"][d]
                cnt = {"ok": 0, "miss": 0, "double": 0, "wrong": 0}
                print(f"F{d} ({names.get(f, f)}, {hc['press_angle'][str(f)]}도) {a.n}번")
                for i in range(a.n):
                    _, got, kind = one(d)
                    cnt[kind] += 1
                    if kind != "ok":
                        print(f"    {i + 1}번째: {kind} {got}")
                table[d] = cnt
                record("keys_finger", key=f"F{d}", finger=f, angle=hc["press_angle"][str(f)], **cnt)
            print("\n키   손가락  성공  안눌림  두번  다른키")
            for d, c in table.items():
                f = hc["key_finger"][d]
                print(f"F{d}   {names.get(f, f):4s}  {c['ok']:3d}   {c['miss']:4d}   {c['double']:3d}   {c['wrong']:4d}")
            print("안눌림이 많으면 그 손가락 각도를 5도 더 굽히고(숫자 작게), 다른키가 많으면 차 위치(옆)를 조정")
    finally:
        h.close()


# ------------------------------------------------------------------ 인식
def test_see(cfg, a):
    from vision import Vision
    v = Vision(cfg["camera"], cfg["detect"], list(cfg["animals"].keys()))
    ko = {k: x["ko"] for k, x in cfg["animals"].items()}
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    print("조건을 적고 엔터 (예: 고양이 1m 정면 / 마커0 50cm 30도). 빈 줄 = 끝")
    while True:
        cond = input("\n조건> ").strip()
        if not cond:
            break
        hits, marks, confs, t0 = {}, {}, {}, time.time()
        for _ in range(a.frames):
            r = v.perceive()
            for an in r["animals"]:
                hits[an["label"]] = hits.get(an["label"], 0) + 1
                confs.setdefault(an["label"], []).append(an["conf"])
            for m in r["markers"]:
                marks[m["id"]] = marks.get(m["id"], 0) + 1
            last = r
            time.sleep(0.05)
        ms = (time.time() - t0) / a.frames * 1000
        img = os.path.join(HERE, "logs", time.strftime("see_%H%M%S.jpg"))
        v.save(img, last)
        an_txt = ", ".join(f"{ko.get(k, k)} {n}/{a.frames} (확신 {statistics.mean(confs[k]):.2f})" for k, n in hits.items()) or "동물 없음"
        mk_txt = ", ".join(f"마커{k} {n}/{a.frames}" for k, n in marks.items()) or "마커 없음"
        print(f"  {an_txt} | {mk_txt} | 한 장 {ms:.0f}ms | 사진 {img}")
        record("see", cond=cond, frames=a.frames, animals=hits, markers={str(k): n for k, n in marks.items()}, ms=round(ms))


def main():
    ap = argparse.ArgumentParser(description="집에서 하는 시험")
    ap.add_argument("test", choices=["lidar", "straight", "side", "rot", "stopwall", "brake", "follow", "keys", "see"])
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--secs", type=float, default=0)
    ap.add_argument("--target", type=float, default=0.40)
    ap.add_argument("--code")
    ap.add_argument("--manual", action="store_true")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--config")
    ap.add_argument("--speeds", help="brake 시험 속도들, 예: 0.2,0.3,0.4")
    ap.add_argument("--hold", action="store_true", help="직진할 때 라이다로 방향 유지")
    ap.add_argument("--speed", type=float, default=0, help="직진/옆 시험 속도 m/s (기본: 설정값)")
    ap.add_argument("--trim", type=float, help="직진 틀어짐 보정값을 이번만 바꿈 (0 = 보정 끔, 비교 영상용)")
    a = ap.parse_args()
    cfg = load_config(a.config)
    if a.trim is not None:
        cfg["drive"]["yaw_trim_deg"] = a.trim
        print(f"이번만 틀어짐 보정 {a.trim}도 (저장 안 함)")
    if a.trim == 0:                                  # 보정 전 = 예전처럼 급출발/급정거도 같이
        cfg["drive"]["accel"], cfg["drive"]["accel_rot_deg"] = 100.0, 100000.0
        print("이번만 서서히 서기도 끔 (예전처럼 급정거)")
    if a.test == "keys" and "--n" not in sys.argv:
        a.n = 10
    {"lidar": test_lidar, "straight": test_straight, "side": lambda c, x: test_straight(c, x, side=True), "rot": test_rot,
     "stopwall": test_stopwall, "brake": test_brake, "follow": test_follow, "keys": test_keys, "see": test_see}[a.test](cfg, a)


if __name__ == "__main__":
    main()
