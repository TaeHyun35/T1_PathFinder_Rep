"""카드 인식 시험용 자리 잡기 (파이). 라이다로 벽까지 거리/움직인 양을 재면서 천천히 움직인다.

  python3 cardpose.py dist                     지금 벽까지 거리 (라이다 중심, 카메라, 손끝)
  python3 cardpose.py goto --cam 0.45          카메라~벽 45cm 가 되게 앞/뒤로
  python3 cardpose.py side --m -0.30           옆으로 30cm (+ 왼쪽, - 오른쪽). 라이다로 잰 만큼만
  python3 cardpose.py turn --deg 30            제자리에서 왼쪽 30도 (+ 왼쪽)
  python3 cardpose.py pass --m -1.5 --v 0.08   벽을 보고 옆으로 천천히 지나가기 (흔들림 시험)
멈춤: ~/stop.py (비상 정지 파일) 또는 Ctrl+C. 손끝이 벽에 18cm 보다 가까워지면 안 감.
"""
import argparse
import math
import time

from common import load_config

CAM_X = 0.215          # 라이다 중심 -> 카메라 (앞으로)
FINGER = 0.315         # 라이다 중심 -> 손끝
MIN_FINGER_GAP = 0.12  # 손끝과 벽 사이 최소
SIDE_MIN = 0.28        # 옆으로 갈 때 그쪽 물체(침대, 문)까지 최소 (라이다 중심 기준)


def front(lid, n=5):
    ds = []
    for _ in range(n * 3):
        s = lid.latest()
        d = s.ray(0, 4)
        if not math.isnan(d):
            ds.append(d)
        if len(ds) >= n:
            break
        time.sleep(0.12)
    ds.sort()
    return ds[len(ds) // 2] if ds else math.nan


def show(lid):
    d = front(lid)
    print(f"벽까지: 라이다 중심 {d * 100:.1f}cm / 카메라 {(d - CAM_X) * 100:.1f}cm / 손끝 {(d - FINGER) * 100:.1f}cm", flush=True)
    return d


def run_pose(car, lid, goal, v_max, w_max_deg, timeout, tol_xy=0.01, tol_th=1.5, keep_wall=None):
    """출발 자세 기준 goal=(x 앞, y 왼쪽, th 도) 로 가기. 라이다 스캔 맞춤으로 실제 움직인 양을 잼"""
    from heading import ScanOdom
    last = lid.latest()
    odo = ScanOdom(last)
    cmd, t0, tp = (0.0, 0.0, 0.0), time.time(), time.time()
    gx, gy, gth = goal[0], goal[1], math.radians(goal[2])
    pose = (0.0, 0.0, 0.0)
    moved_at, ref = time.time(), (0.0, 0.0, 0.0)
    try:
        while time.time() - t0 < timeout:
            s = lid.latest()
            if s is not last:
                now = time.time()
                dt, tp, last = now - tp, now, s
                pose = odo.update(s, (cmd[0] * dt, cmd[1] * dt, cmd[2] * dt))
                fd = s.ray(0, 4)
                if not math.isnan(fd) and fd - FINGER < MIN_FINGER_GAP and cmd[0] > 0:
                    print("손끝이 벽에 너무 가까움 -> 멈춤", flush=True)
                    break
                # 헛돌기 감지: 명령은 주는데 라이다로 1.5초 동안 거의 안 움직였으면 멈춤
                if math.hypot(pose[0] - ref[0], pose[1] - ref[1]) > 0.02 or abs(pose[2] - ref[2]) > math.radians(3):
                    moved_at, ref = now, pose
                elif now - moved_at > 1.5 and (abs(cmd[0]) + abs(cmd[1]) > 0.03 or abs(cmd[2]) > 0.15):
                    print("명령만큼 안 움직임 (바퀴 헛돎/막힘) -> 멈춤", flush=True)
                    break
                side = s.sector_min(90 if cmd[1] > 0 else -90, 25) if abs(cmd[1]) > 0.01 else math.nan
                if not math.isnan(side) and side < SIDE_MIN:
                    print(f"옆에 물체가 {side * 100:.0f}cm -> 멈춤", flush=True)
                    break
                ex, ey, eth = gx - pose[0], gy - pose[1], gth - pose[2]
                if abs(ex) < tol_xy and abs(ey) < tol_xy and abs(math.degrees(eth)) < tol_th:
                    break
                c, sn = math.cos(pose[2]), math.sin(pose[2])
                rx, ry = c * ex + sn * ey, -sn * ex + c * ey          # 로봇 기준 오차
                vx = max(-v_max, min(v_max, 1.5 * rx))
                vy = max(-v_max, min(v_max, 1.5 * ry))
                w = max(-math.radians(w_max_deg), min(math.radians(w_max_deg), 2.5 * eth))
                if abs(math.degrees(eth)) >= tol_th and abs(w) < math.radians(12):   # 너무 느리면 바퀴가 안 돎 (손 무게)
                    w = math.copysign(math.radians(12), eth)
                if keep_wall is not None and not math.isnan(fd):     # 지나가기: 벽 거리 유지
                    vx = max(-0.05, min(0.05, 1.5 * (fd - keep_wall)))
                cmd = (vx, vy, w)
            car.drive(*cmd)
            time.sleep(0.03)
        car.soft_stop()
    finally:
        car.stop()
    time.sleep(0.5)
    print(f"움직인 양 (라이다): 앞 {pose[0] * 100:+.1f}cm, 왼쪽 {pose[1] * 100:+.1f}cm, 회전 {math.degrees(pose[2]):+.1f}도, 맞춤 실패 {odo.fails}번", flush=True)
    return pose


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["dist", "goto", "side", "turn", "pass"])
    ap.add_argument("--cam", type=float, default=0.45)
    ap.add_argument("--m", type=float, default=0.0)
    ap.add_argument("--deg", type=float, default=0.0)
    ap.add_argument("--v", type=float, default=0.08)
    a = ap.parse_args()
    cfg = load_config()
    from lidar import RPLidar
    lid = RPLidar(cfg["lidar"])
    lid.wait_ready()
    time.sleep(1.0)
    d = show(lid)
    if a.cmd == "dist":
        lid.close()
        return
    from drive import RaspbotCar
    # 부드럽게 출발/멈춤: 갑자기 힘이 걸리면 손 무게로 앞으로 쏠려 뒷바퀴가 들림
    car = RaspbotCar(dict(cfg["drive"], accel=0.5, accel_rot_deg=120))
    if a.cmd == "goto":
        target = a.cam + CAM_X
        if target - FINGER < MIN_FINGER_GAP:
            print("너무 가까운 목표 -> 안 감")
            return
        run_pose(car, lid, (d - target, 0, 0), 0.10, 20, 15)
    elif a.cmd == "side" or a.cmd == "pass":
        sd = lid.latest().sector_min(90 if a.m > 0 else -90, 25)
        print(f"가는 쪽 옆 물체까지 {sd * 100:.0f}cm", flush=True)
        if not math.isnan(sd) and sd - SIDE_MIN < abs(a.m):
            print("가는 쪽 공간이 모자람 -> 안 감", flush=True)
            show(lid); lid.close()
            return
    if a.cmd == "side":
        run_pose(car, lid, (0, a.m, 0), 0.08, 20, 15)
    elif a.cmd == "turn":
        run_pose(car, lid, (0, 0, a.deg), 0.05, 20, 10, tol_th=1.0)
    elif a.cmd == "pass":
        run_pose(car, lid, (0, a.m, 0), a.v, 20, abs(a.m) / a.v + 6, keep_wall=d)
    show(lid)
    lid.close()


if __name__ == "__main__":
    main()
