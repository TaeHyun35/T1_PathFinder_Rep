"""직진 D 만큼 -> 후진으로 돌아오기. 옆으로는 안 밀고 방향만 라이다로 잡음 (11/26 사용자 요청)"""
import math, sys, time
from common import load_config
from selfcal import Rig
D = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
V = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15


def straight(rig, target_x, v, timeout=20):
    sgn = 1 if target_x > rig.odo.pose[0] else -1
    t0 = time.time()
    while time.time() - t0 < timeout:
        _, p = rig.tick(rig.car.cmd)
        rem = (target_x - p[0]) * sgn
        if rem <= 0.01:
            break
        if sgn > 0 and rig.front() - 0.315 < 0.13:           # 손끝과 벽 13cm -> 멈춤
            print("  손끝이 벽에 가까움 -> 멈춤", flush=True)
            break
        vx = sgn * max(0.05, min(v, 1.5 * rem))          # 가까워지면 천천히
        w = max(-0.5, min(0.5, -2.5 * p[2]))              # 처음 방향(0도) 유지
        rig.car.drive(vx, 0.0, w)
        time.sleep(0.03)
    rig.car.soft_stop()
    return rig.settle(0.8), time.time() - t0


rig = Rig(load_config())
try:
    f0 = rig.front()
    print(f"시작: 앞 {f0 * 100:.0f}cm", flush=True)
    if D <= 0:                                           # 0 이면 반대편 벽 앞(손끝 15cm)까지
        D = f0 - 0.315 - 0.15
        print(f"벽 앞까지 갈 거리 {D * 100:.0f}cm", flush=True)
    p, dt = straight(rig, D, V)
    print(f"  멈춘 뒤 벽까지: 손끝 {(rig.front() - 0.315) * 100:.1f}cm", flush=True)
    print(f"직진 {D * 100:.0f}cm -> 앞 {p[0] * 100:.1f}cm, 옆 {p[1] * 100:+.1f}cm, 방향 {math.degrees(p[2]):+.1f}도 ({dt:.1f}초)", flush=True)
    time.sleep(0.5)
    p, dt = straight(rig, 0.03, V)
    print(f"후진 (출발점 3cm 앞까지) -> 앞 {p[0] * 100:.1f}cm, 옆 {p[1] * 100:+.1f}cm, 방향 {math.degrees(p[2]):+.1f}도 ({dt:.1f}초)", flush=True)
finally:
    rig.close()
