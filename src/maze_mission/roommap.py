"""자율주행으로 방 지도 만들기.

1) 파이에서:  python3 roommap.py run --secs 90 --yes
   - 먼저 제자리에서 조금 돌아 보고 바퀴 방향이 맞는지 라이다로 확인 (틀리면 바로 멈춤)
   - 그다음 오른쪽 벽을 따라 혼자 돌아다니며 라이다 스캔을 전부 logs/roommap_*.jsonl 에 기록
   - 멈춤: 시간 끝 / Ctrl+C / ssh 끊김 / 5초 넘게 앞이 안 보임
2) 컴퓨터에서: python roommap.py build logs/roommap_xxx.jsonl
   - 스캔끼리 모양을 맞춰(ICP) 이어 붙여서 지도 그림(png)을 만든다

시뮬로 먼저 확인: python roommap.py run --sim --secs 120  ->  python roommap.py build <나온 파일>
"""
import argparse
import json
import math
import os
import signal
import sys
import time

from common import HERE, load_config


# ------------------------------------------------------------------ 1) 주행 + 기록
def run(cfg, a):
    from hometest import profile, rotation_between
    from mission import follow

    if a.sim:
        from sim import SimCar, SimClock, SimLidar, World
        world = World(cfg, seed=a.seed)
        lid, car, clock = SimLidar(world), SimCar(world), SimClock(world)
    else:
        from drive import RaspbotCar
        from lidar import RPLidar
        world = None
        lid = RPLidar(cfg["lidar"])
        lid.wait_ready()
        # 부드럽게 출발/멈춤: 갑자기 힘이 걸리면 손 무게로 앞으로 쏠려 뒷바퀴가 들림 (11/26 카드 시험)
        car = RaspbotCar(dict(cfg["drive"], accel=0.5, accel_rot_deg=120))

        class clock:
            now = staticmethod(time.time)
            sleep = staticmethod(time.sleep)

    def bail(*_):
        raise KeyboardInterrupt
    if not a.sim:
        signal.signal(signal.SIGTERM, bail)
        signal.signal(signal.SIGHUP, bail)          # ssh 가 끊겨도 바퀴 멈춤

    # 11/27: 힘 30(직진)/20(회전) 아래는 끊어서 가서 툭툭 느림 -> 끊기지 않는 속도로 (지도/카드는 이 속도에서도 문제없음)
    cfg["wall"]["w_turn_deg"] = 45.0
    cfg["wall"]["w_arc_deg"] = 30.0
    cfg["wall"]["v"] = a.speed
    cfg["wall"]["v_arc"] = a.speed * 0.9

    def scans(k):
        out, last, t0 = [], None, clock.now()
        while len(out) < k and clock.now() - t0 < 5:
            s = lid.latest()
            if s is not None and (a.sim or s is not last):
                out.append(s)
                last = s
            clock.sleep(0.05)
        return out

    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    path = os.path.join(HERE, "logs", time.strftime("roommap_%Y%m%d_%H%M%S") + ("_sim" if a.sim else "") + ".jsonl")
    f = open(path, "w", encoding="utf-8")
    tags, n, stuck = {}, 0, 0
    try:
        # 바퀴 방향 확인: 왼쪽으로 조금 돌고, 라이다로 실제로 왼쪽으로 돌았는지 본다
        if not a.yes and input("제자리에서 왼쪽으로 조금 돌아 봅니다. 출발? (y) ").strip().lower() != "y":
            return
        # 라이다로 재면서 왼쪽 30도까지 돌기 (약한 힘에선 바퀴가 걸렸다 풀렸다 해서 시간으로는 매번 다름 -> 11/26)
        from heading import ScanOdom as _SO
        last0 = lid.latest(); o0 = _SO(last0); t0 = tp0 = clock.now(); w = math.radians(40)
        scan_first = last0                                 # 11/27 밤: 출발 자리 라이다 사진 (테이프 네모 방향 다시 맞추기용)
        while clock.now() - t0 < 5.0 and math.degrees(o0.pose[2]) < 30:
            car.drive(0, 0, w)
            s0 = lid.latest()
            if s0 is not last0:
                now0 = clock.now(); dt0 = now0 - tp0; tp0 = now0; last0 = s0
                o0.update(s0, (0, 0, w * dt0))
            clock.sleep(0.03)
        car.soft_stop()
        clock.sleep(0.6)
        rot = round(math.degrees(o0.pose[2]))
        print(f"방향 확인: 왼쪽으로 돌기 -> 라이다로 잰 회전 {rot:+d}도 ({clock.now() - t0:.1f}초, 맞춤 실패 {o0.fails}번)")
        if rot < 15:
            print("왼쪽으로 안 돎 (바퀴 방향이 이상하거나 바퀴가 안 돎). 멈춥니다.")
            return
        # 11/27: 시작 자리 둘레를 먼저 한 바퀴 보기 (시작 옆에 붙은 카드를 놓치지 않게, 첫 지도도 넓게)
        s0 = lid.latest()
        around = min([d for d in (s0.sector_min(a_, 30) for a_ in range(-150, 181, 60)) if d == d] or [9])
        if a.spin and around > 0.42:                      # 손끝이 도는 반지름(약 33cm) + 여유
            t1 = clock.now()
            sw = 45                    # 11/27 밤: 테이프 미로는 천천히 (처음 방향이 틀리면 테이프 네모 전체가 돌아감)
            while clock.now() - t1 < (20.0 if a.vwalls else 14.0) and math.degrees(o0.pose[2]) < 30 + 360:
                car.drive(0, 0, math.radians(sw))
                s1 = lid.latest()
                if s1 is not last0:
                    now1 = clock.now(); dt1 = now1 - tp0; tp0 = now1; last0 = s1
                    o0.update(s1, (0, 0, math.radians(sw) * dt1))
                clock.sleep(0.03)
            car.soft_stop()
            clock.sleep(0.5)
            print(f"시작 자리 한 바퀴 보기: {math.degrees(o0.pose[2]) - rot:.0f}도 ({clock.now() - t1:.1f}초)")
        elif a.spin:
            print(f"시작 자리가 좁아서(가장 가까운 물체 {around * 100:.0f}cm) 한 바퀴 보기는 건너뜀")
        f.write(json.dumps({"meta": {"speed": a.speed, "side": cfg["wall"]["side"], "check_rot_deg": rot, "sim": a.sim}}) + "\n")

        from heading import ScanOdom
        from obstacles import ScanMemory
        mem = ScanMemory() if cfg["wall"].get("memory", False) else None   # 11/27 시뮬: 낮은 상자엔 효과 없고 1판 더 꼬임 -> 기본 끔
        # 11/27: 카메라가 본 장애물(옷걸이, 의자, 가방...)을 라이다 점처럼 기억해서 피함 (PC obst_feed.py 가 보내 줌)
        camlink, cam_mem = None, None
        if a.camobst and not a.sim:
            from link import Link
            camlink = Link(8093)
            cam_mem = ScanMemory(keep_s=25.0, max_r=1.8, only_extra=True)
        cam_seen = 0
        t_start, last, t_print = clock.now(), None, 0
        odo, t_prev, cmd = None, clock.now(), (0.0, 0.0, 0.0)
        hist = []                  # (t, x, y, 누적 회전)
        escape_until, escapes, turn_acc = 0.0, 0, 0.0
        open_since = None          # 열린 공간에서 돌기 시작한 시각 (2초 넘으면 직진해서 벽 찾기)
        side = 1 if cfg["wall"]["side"] == "right" else -1
        travel, last_xy, lap_done = 0.0, None, False    # 11/27: 한 바퀴 돌아 처음 벽에 붙은 곳으로 오면 지도 완성 -> 출발점으로
        wall0 = None                                     # 처음 벽따라 시작한 자리 (x, y, 그때까지 간 거리, 그때 누적 회전)
        blind_since, blind_dir = None, 0
        ex = None
        # 11/27: 한 바퀴 뒤 '카메라로 가까이 안 본 벽' / '안 가 본 길' 찾아가기 (침대-책상 사잇길을 안 들어가던 문제)
        from frontier import Grid, Nav
        grid, nav, phase, t_cover = Grid(), None, "wall", None
        from drive import Boost
        bst = Boost()
        vw, SMALL = None, {}
        if a.vwalls:
            from vwalls import VirtualWalls
            vw = VirtualWalls.load(a.vwalls)
            # 11/27 밤: 한 바퀴 도는 동안 쌓인 방향 오차(시뮬 15도)를 없애려고, 출발 사진과 지금 사진을 직접 맞춤
            from heading import icp_xy, to_xy
            g = o0.pose
            best = None
            for dth in (0.0, -10.0, 10.0, -20.0, 20.0):
                gx = (g[0], g[1], math.atan2(math.sin(g[2]), math.cos(g[2])) + math.radians(dth))
                r_ = icp_xy(to_xy(scan_first), to_xy(lid.latest()), gx)
                if best is None or r_[3] > best[3]:
                    best = r_
            if best[3] >= 0.5:
                print(f"출발 사진과 다시 맞춤: 돈 각도 {math.degrees(g[2]) % 360:.0f}도 -> {math.degrees(best[2]) % 360:.0f}도 (맞춤 {best[3]:.2f})", flush=True)
                o0.pose = (best[0], best[1], best[2])
            else:
                print(f"출발 사진과 다시 맞추기 실패 (맞춤 {best[3]:.2f}) -> 쌓인 값 그대로", flush=True)
            vw.rebase(*o0.pose)                      # 방향 확인/한 바퀴 동안 돈 만큼 (주행거리계는 그 뒤에 0 에서 시작)
            f.write(json.dumps({"vw": {"file": os.path.basename(a.vwalls), "rebase": list(o0.pose),
                                       "walls": json.load(open(a.vwalls, encoding="utf-8"))}}, ensure_ascii=False) + chr(10))
            SMALL.update(lap_min_travel=2.5, far_min=0.5)
            cfg["wall"]["no_strafe"] = True          # 11/27 밤: 왼쪽 뒷바퀴 헛돎 -> 옆/비스듬히 안 감, 힘 올리기 끔
            print(f"테이프 미로(가상 벽) {a.vwalls}: 벽 {len(vw.segs)}개, 출발 뒤 돈 각도 {math.degrees(o0.pose[2]):.0f}도", flush=True)
        while clock.now() - t_start < a.secs:
            s = lid.latest()
            if s is None or (s is last and not a.sim):
                clock.sleep(0.02)
                continue
            last = s
            now = clock.now()
            dt, t_prev = now - t_prev, now
            if odo is None:
                odo = ScanOdom(s)
                pose = odo.pose
            else:
                pth = odo.pose[2]
                pose = odo.update(s, (cmd[0] * dt, cmd[1] * dt, cmd[2] * dt))
                turn_acc += abs(pose[2] - pth)
            hist.append((now, pose[0], pose[1], turn_acc))
            if last_xy is not None:
                travel += math.hypot(pose[0] - last_xy[0], pose[1] - last_xy[1])
            last_xy = (pose[0], pose[1])
            hist = [h for h in hist if now - h[0] <= 20.0]
            if ex is None:
                from explore import Explorer
                ex = Explorer(cfg, **SMALL)
            s_in = mem.update(s, pose) if mem is not None else s
            if cam_mem is not None:
                from obstacles import cam_obstacles
                extra = []
                if camlink.obst and time.time() - camlink.obst_rx < 0.8:
                    extra = cam_obstacles(camlink.obst["dets"], 0.21, s, cfg["camera"].get("tilt_up_deg", 0.0))
                    if extra:
                        cam_seen += 1
                s_in = cam_mem.update(s_in, pose, extra)
            if a.poles:
                from obstacles import pole_rings, _Scan
                pr = pole_rings(s)
                if pr:
                    s_in = _Scan(s_in, list(s_in.pts) + pr)
            s_in = ex.with_marks(s_in, pose, now)
            if vw is not None:
                s_in = vw.add(s_in, pose)                # 11/27 밤: 테이프 미로 = 가상 벽
            grid.add(s_in, pose)
            if phase == "wall":
                (vx, vy, wz), tag, ev = ex.step(s_in, pose, now)
            else:
                (vx, vy, wz), tag, ev = nav.step(s_in, pose, now)
            if ev and phase != "wall" and "걸림 ->" in ev:
                ex.add_mark(pose, now)          # 지도 보며 갈 때 막힌 곳도 기억
            if ev:
                print(f"  {now - t_start:5.1f}초  {ev}", flush=True)
            if phase == "wall" and ex.lap_done and a.lap:
                print(f"  {now - t_start:5.1f}초  -> 한 바퀴 완료", flush=True)
                lap_done = True
                if not a.cover:
                    print(f"  {now - t_start:5.1f}초  -> 지도를 완성했습니다", flush=True)
                    break
                phase, t_cover = "cover", now
                nav = Nav(grid, cfg, max_r=a.cover_r, v=min(0.20, a.speed))
                print(f"  {now - t_start:5.1f}초  -> 아직 안 본 곳 찾아가기 (최대 {a.cover_secs:.0f}초)", flush=True)
                continue
            if phase == "cover" and (nav.done or now - t_cover > a.cover_secs):
                print(f"  {now - t_start:5.1f}초  -> 지도를 완성했습니다 (찾아간 곳 {nav.reached}군데)", flush=True)
                if not a.home:
                    break
                phase = "home"
                nav.go_to((0.0, 0.0))
                print(f"  {now - t_start:5.1f}초  -> 출발점으로 길 따라 돌아갑니다", flush=True)
                continue
            if phase == "home" and nav.done:
                break
            if vw is not None and abs(wz) > math.radians(a.max_w):
                wz = math.copysign(math.radians(a.max_w), wz)     # 11/27 밤: 빨리 돌면 라이다 방향 맞추기를 놓침 (실패 11%)
            cmd = (vx, vy, wz)
            if not a.sim and not cfg["wall"].get("no_strafe"):
                b_old = getattr(car, "boost", 1.0)
                car.boost = bst.update(cmd, pose, now)
                if car.boost > 1.0 and b_old == 1.0:
                    print(f"  {now - t_start:5.1f}초  안 움직임 -> 바퀴 힘 올림", flush=True)
            car.drive(vx, vy, wz)
            tags[tag] = tags.get(tag, 0) + 1
            stuck = stuck + 1 if "정지" in tag else 0
            rec = {"t": round(clock.now() - t_start, 3), "ts": round(time.time(), 3), "cmd": [round(vx, 3), round(vy, 3), round(wz, 3)],
                   "odo": [round(pose[0], 3), round(pose[1], 3), round(pose[2], 4)],
                   "a": [round(p[0], 2) for p in s.pts if p[1] > 0], "d": [round(p[1], 3) for p in s.pts if p[1] > 0]}
            if world is not None:
                rec["gt"] = [round(world.x, 4), round(world.y, 4), round(world.th, 4)]
            f.write(json.dumps(rec) + "\n")
            n += 1
            el = clock.now() - t_start
            if el - t_print >= 5:
                t_print = el
                print(f"  {el:5.1f}초  스캔 {n}  지금: {tag}")
            if stuck > 50:
                print("5초 넘게 앞이 안 보여서 멈춥니다")
                break
            clock.sleep(0.1)
        if lap_done and a.home and odo is not None:
            car.soft_stop()
            clock.sleep(0.5)
            go_home(car, lid, odo, clock, f, t_start, cfg)
    except KeyboardInterrupt:
        print("\n중간에 멈춤")
    finally:
        try:
            car.soft_stop()                     # 손 무게 때문에 급정거하면 앞으로 고꾸라짐
        except BaseException:
            pass
        car.stop()
        f.close()
        lid.close()
    print("상태별 횟수:", tags)
    print("카메라 장애물 반영 횟수:", locals().get("cam_seen", 0))
    print("빠져나오기 횟수:", locals().get("escapes", 0))
    print("기록:", path, f"(스캔 {n}개)")


def go_home(car, lid, odo, clock, f, t_start, cfg, v=0.20):
    """출발점(0,0)으로: 라이다 주행거리계로 방향을 돌리고 똑바로. 손 앞에 물체가 있으면 멈춤"""
    from obstacles import hand_corridor
    print("  출발점으로 돌아갑니다", flush=True)
    last, tp = lid.latest(), clock.now()
    cmd = (0.0, 0.0, 0.0)
    t0 = clock.now()
    while clock.now() - t0 < 40:
        s = lid.latest()
        if s is not last:
            now = clock.now()
            dt = now - tp
            tp, last = now, s
            pose = odo.update(s, (cmd[0] * dt, cmd[1] * dt, cmd[2] * dt))
            f.write(json.dumps({"t": round(now - t_start, 3), "ts": round(time.time(), 3), "cmd": [round(c, 3) for c in cmd],
                                "odo": [round(pose[0], 3), round(pose[1], 3), round(pose[2], 4)],
                                "a": [round(p[0], 2) for p in s.pts if p[1] > 0], "d": [round(p[1], 3) for p in s.pts if p[1] > 0]}) + "\n")
            x, y, th = pose
            dist = math.hypot(x, y)
            if dist < 0.10:
                print(f"  출발점 도착 (남은 거리 {dist * 100:.0f}cm)", flush=True)
                break
            want = math.atan2(-y, -x)
            e = math.atan2(math.sin(want - th), math.cos(want - th))
            if abs(e) > math.radians(10):                      # 먼저 방향 돌리기
                cmd = (0.0, 0.0, math.copysign(math.radians(45), e))
            else:
                hc = hand_corridor(s, cfg["wall"].get("finger_m", 0.315), 0.08, cfg["wall"].get("finger_margin", 0.08))
                if hc is not None:
                    print(f"  손 앞에 물체 ({hc * 100:.0f}cm) -> 여기서 멈춤 (출발점까지 {dist * 100:.0f}cm)", flush=True)
                    break
                cmd = (min(v, 0.08 + dist), 0.0, max(-0.5, min(0.5, 2.5 * e)))
        car.drive(*cmd)
        clock.sleep(0.03)
    car.soft_stop()


# ------------------------------------------------------------------ 2) 지도 만들기 (컴퓨터)
def build(path, out=None, det=None):
    import numpy as np
    from scipy.spatial import cKDTree

    meta, recs = {}, []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            r = json.loads(line)
            if "meta" in r:
                meta = r["meta"]
            elif len(r["a"]) > 30:
                recs.append(r)
    if not recs:
        sys.exit("스캔이 없음")

    def to_xy(r, rmax=6.0):
        a = np.radians(np.array(r["a"]))
        d = np.array(r["d"])
        k = d < rmax
        p = np.stack([d[k] * np.cos(a[k]), d[k] * np.sin(a[k])], 1)
        q = np.unique(np.round(p / 0.03), axis=0, return_index=True)[1]    # 3cm 칸마다 한 점
        return p[np.sort(q)]

    def T(x, y, th):
        c, s = math.cos(th), math.sin(th)
        return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])

    def apply(M, p):
        return p @ M[:2, :2].T + M[:2, 2]

    def icp(src, tree, ref, M):
        good = 0
        for it in range(30):
            thr = max(0.06, 0.35 * (0.85 ** it))
            q = apply(M, src)
            dist, idx = tree.query(q)
            k = dist < thr
            good = int(k.sum())
            if good < 25:
                return M, good, False
            A, B = q[k], ref[idx[k]]
            ca, cb = A.mean(0), B.mean(0)
            U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[1] *= -1
                R = Vt.T @ U.T
            D = np.eye(3)
            D[:2, :2], D[:2, 2] = R, cb - R @ ca
            M = D @ M
            if abs(D[0, 2]) + abs(D[1, 2]) < 1e-4 and abs(math.atan2(R[1, 0], R[0, 0])) < 1e-4:
                break
        return M, good, good > 0.35 * len(src)

    M = np.eye(3)
    keys = []                    # (pose, 세계 좌표 점들)
    poses, fails = [], 0
    prev = None
    for r in recs:
        src = to_xy(r)
        if prev is not None and "odo" in r and "odo" in prev:
            # 주행 중 라이다로 잰 움직임으로 예측 (명령값은 느린 회전에서 실제와 2배 넘게 다름)
            M = M @ (np.linalg.inv(T(*prev["odo"])) @ T(*r["odo"]))
        elif prev is not None:     # 명령 속도로 대충 예측 (진짜 위치는 ICP 가 맞춘다)
            dt = max(0.0, min(0.5, r["t"] - prev["t"]))
            vx, vy, w = prev["cmd"]
            M = M @ T(vx * dt, vy * dt, w * dt)
        if keys:
            # 최근 것만이 아니라 지금 위치 근처(2.5m)의 지도 전체와 맞춤 -> 같은 곳에 다시 오면 어긋남이 바로잡힘
            near = [k[1] for k in keys if np.hypot(*(k[0][:2, 2] - M[:2, 2])) < 2.5]
            ref = np.concatenate(near if near else [k[1] for k in keys[-12:]])
            ref = ref[np.unique(np.round(ref / 0.03), axis=0, return_index=True)[1]]
            M2, good, ok = icp(src, cKDTree(ref), ref, M)
            if ok:
                M = M2
            else:
                fails += 1
        if not keys or np.hypot(*(M[:2, 2] - keys[-1][0][:2, 2])) > 0.08 or \
                abs(math.atan2(M[1, 0], M[0, 0]) - math.atan2(keys[-1][0][1, 0], keys[-1][0][0, 0])) > math.radians(8):
            keys.append((M.copy(), apply(M, src)))
        poses.append((M[0, 2], M[1, 2], math.atan2(M[1, 0], M[0, 0])))
        prev = r

    pts = np.concatenate([k[1] for k in keys])
    P = np.array(poses)
    path_len = float(np.sum(np.hypot(*np.diff(P[:, :2], axis=0).T)))

    # 시뮬이면 정답과 비교 (처음 위치/방향을 맞춘 뒤 위치 오차)
    gt_err = None
    if "gt" in recs[0]:
        G = np.array([r["gt"] for r in recs])
        g0 = T(*G[0])
        Gl = np.array([np.linalg.inv(g0) @ np.array([g[0], g[1], 1.0]) for g in G])[:, :2]
        e = np.hypot(*(P[:, :2] - Gl).T)
        gt_err = (float(e.mean()), float(e.max()))

    cards = place_cards(det, recs, poses) if det else []

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(pts[:, 0], pts[:, 1], s=1.2, c="#333333", linewidths=0)
    ax.plot(P[:, 0], P[:, 1], color="#2a6fdb", lw=1.6, label="로봇이 간 길")
    ax.plot(P[0, 0], P[0, 1], "o", color="#1a9e55", ms=8, label="출발")
    ax.plot(P[-1, 0], P[-1, 1], "s", color="#d64545", ms=7, label="끝")
    for c in cards:
        ax.plot(c["x"], c["y"], "*", color="#e08a00", ms=16, mec="black", mew=0.6)
        ax.annotate(f"{c['ko']} {c['digit'] or '?'}", (c["x"], c["y"]), textcoords="offset points", xytext=(8, 6),
                    fontsize=12, fontweight="bold", color="#b35c00")
    if cards:
        ax.plot([], [], "*", color="#e08a00", ms=12, mec="black", label="찾은 카드 (이름 숫자)")
    ax.set_aspect("equal")
    ax.grid(True, color="#dddddd", lw=0.5)
    ax.set_xlabel("m")
    ax.set_ylabel("m")
    title = f"방 지도  (스캔 {len(recs)}개, 간 거리 {path_len:.1f}m, 맞춤 실패 {fails}번)"
    if gt_err:
        title += f"\n시뮬 정답과 위치 오차: 평균 {gt_err[0] * 100:.1f}cm, 최대 {gt_err[1] * 100:.1f}cm"
    ax.set_title(title)
    ax.legend(loc="upper right")
    out = out or os.path.splitext(path)[0] + ".png"
    fig.savefig(out, dpi=130, bbox_inches="tight")
    for c in cards:
        print(f"  카드 {c['ko']} {c['digit'] or '?'}: 지도 위치 ({c['x']:+.2f}, {c['y']:+.2f}) m, {c['n']}번 봄, 퍼짐 {c['spread'] * 100:.0f}cm, 거리 재기 {c['how']}")
    print(title.replace("\n", " / "))
    print("그림:", out)
    return out, gt_err, fails


CARD_KO = {"dog": "강아지", "panda": "판다", "turtle": "거북이", "rabbit": "토끼", "giraffe": "기린", "elephant": "코끼리"}
CAM_X = 0.21           # 라이다 중심 -> 카메라 (앞으로 m, 11/26 실측)
CAM_HFOV = 48.0        # 카메라 가로 시야 (도)
DIGIT_H = 0.054        # 카드 숫자 칸 실제 높이 (m). 11/27 가로 카드 5.4cm (세로 카드는 7.2cm)


def place_cards(det_path, recs, poses, min_seen=3):
    """카메라로 본 카드를 지도 좌표로: 로봇 위치/방향 + 화면에서의 방향 + 라이다로 잰 거리. 여러 번 본 것을 모아 중간값"""
    import numpy as np
    sys.path.insert(0, os.path.join(HERE, "animals"))
    from pair import pair
    ts = np.array([r.get("ts", -1e9) for r in recs])
    obs = {}
    for line in open(det_path, encoding="utf-8"):
        d = json.loads(line)
        if not d["dets"]:
            continue
        i = int(np.argmin(np.abs(ts - d["ts"])))
        if abs(ts[i] - d["ts"]) > 0.25:
            continue
        w = d["w"]
        f = (w / 2) / math.tan(math.radians(CAM_HFOV / 2))
        found = [(c, cf, (x1, y1, x2, y2)) for c, cf, x1, y1, x2, y2 in d["dets"] if x1 > 4 and x2 < w - 4]   # 잘린 카드 제외
        pr = pair(found)
        nums = {c[3:]: b for c, _, b in found if c.startswith("num")}
        x, y, th = poses[i]
        r = recs[i]
        a = np.radians(np.array(r["a"]))
        dd = np.array(r["d"])
        P = np.stack([dd * np.cos(a), dd * np.sin(a)], 1)               # 로봇 기준 라이다 점
        for c, cf, (x1, y1, x2, y2) in found:
            if c.startswith("num"):
                continue
            b = -math.atan(((x1 + x2) / 2 - w / 2) / f)                    # 화면 오른쪽 = 오른쪽(-)
            u = np.array([math.cos(b), math.sin(b)])
            q = P - np.array([CAM_X, 0.0])
            along = q @ u
            perp = np.abs(q[:, 0] * u[1] - q[:, 1] * u[0])
            k = (along > 0.1) & (perp < 0.05)
            how = "라이다"
            d_size = None
            if pr.get(c) and pr[c] in nums:                                # 숫자 칸 크기로 잰 거리 (카드 자체의 거리)
                bh = nums[pr[c]][3] - nums[pr[c]][1]
                d_size = f * DIGIT_H / max(bh, 1)
            if k.any():
                dist = float(along[k].min())
                # 11/27: 카드가 책장 안/침대 옆처럼 라이다 높이(27cm)의 물체보다 안쪽이나 바깥에 있으면 라이다가 다른 것을 잼
                if d_size is not None and abs(dist - d_size) > 0.25:
                    dist, how = d_size, "크기"
            elif d_size is not None:
                dist, how = d_size, "크기"
            else:
                continue
            lx, ly = CAM_X + dist * u[0], dist * u[1]
            cs, sn = math.cos(th), math.sin(th)
            obs.setdefault(c, []).append((x + cs * lx - sn * ly, y + sn * lx + cs * ly, pr.get(c, ""), how))
    out = []
    for c, L in obs.items():
        if len(L) < min_seen:
            continue
        X = np.array([[o[0], o[1]] for o in L])
        mx, my = np.median(X[:, 0]), np.median(X[:, 1])
        spread = float(np.median(np.hypot(X[:, 0] - mx, X[:, 1] - my)))
        digits = [o[2] for o in L if o[2]]
        dg = max(set(digits), key=digits.count) if digits else ""
        hows = [o[3] for o in L]
        out.append({"label": c, "ko": CARD_KO.get(c, c), "x": float(mx), "y": float(my), "n": len(L), "digit": dg,
                    "spread": spread, "how": f"라이다 {hows.count('라이다')} / 크기 {hows.count('크기')}"})
    return out


def main():
    ap = argparse.ArgumentParser(description="자율주행 방 지도")
    ap.add_argument("cmd", choices=["run", "build"])
    ap.add_argument("log", nargs="?")
    ap.add_argument("--secs", type=float, default=90)
    ap.add_argument("--speed", type=float, default=0.22, help="벽 따라갈 때 속도 m/s (11/27 기본 0.22: 끊기지 않는 가장 느린 속도 근처)")
    ap.add_argument("--yes", action="store_true", help="출발 질문 없이 바로")
    ap.add_argument("--spin", type=int, default=1, help="시작할 때 제자리 한 바퀴 보기 (1 켬)")
    ap.add_argument("--lap", type=int, default=1, help="방을 한 바퀴 돌면(처음 벽에 붙은 곳으로 오면) 멈춤 (1 켬)")
    ap.add_argument("--camobst", type=int, default=1, help="카메라 장애물 같이 쓰기 (PC obst_feed.py 필요)")
    ap.add_argument("--max_w", type=float, default=40.0, help="테이프 미로에서 최대 회전 속도 (도/초)")
    ap.add_argument("--vwalls", default="", help="테이프 미로 가상 벽 파일 (vwalls.json)")
    ap.add_argument("--poles", type=int, default=0, help="가는 기둥 둘레 35cm 막기 (11/27 20:40 실험: 방 안 모서리/가구 끝도 기둥으로 봐서 길이 막힘 -> 기본 끔)")
    ap.add_argument("--cover", type=int, default=1, help="한 바퀴 뒤 안 본 벽/길 찾아가기 (1 켬)")
    ap.add_argument("--cover_secs", type=float, default=120, help="찾아가기 최대 시간 (초)")
    ap.add_argument("--cover_r", type=float, default=3.0, help="출발점에서 이 거리(m) 안만 찾아감 (열린 문 너머 안 가게)")
    ap.add_argument("--home", type=int, default=1, help="한 바퀴 뒤 출발점으로 돌아가기 (1 켬)")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out")
    ap.add_argument("--det", help="detect_log.py 기록 (카드 위치를 지도에 찍기)")
    a = ap.parse_args()
    if a.cmd == "run":
        run(load_config(), a)
    else:
        if not a.log:
            sys.exit("build 할 기록 파일을 주세요")
        build(a.log, a.out, a.det)


if __name__ == "__main__":
    main()
