"""왼쪽/오른쪽 번갈아 돌기 (라이다로 실제 돈 각도를 재며 목표까지). 헛돎 확인용"""
import math, time, sys
from common import load_config
from lidar import RPLidar
from drive import RaspbotCar
from heading import ScanOdom
N = int(sys.argv[1]) if len(sys.argv) > 1 else 5
DEG = float(sys.argv[2]) if len(sys.argv) > 2 else 45
cfg = load_config()
lid = RPLidar(cfg["lidar"]); lid.wait_ready(); time.sleep(1)
car = RaspbotCar(dict(cfg["drive"], accel=0.5, accel_rot_deg=120))
try:
    for i in range(N):
        for sgn, word in ((1, "왼쪽"), (-1, "오른쪽")):
            last = lid.latest(); odo = ScanOdom(last); t0 = tp = time.time(); w = sgn * math.radians(30)
            stall = 0.0; prev_th = 0.0; t_prev_move = t0
            while time.time() - t0 < 6 and sgn * math.degrees(odo.pose[2]) < DEG - 2:
                car.drive(0, 0, w); s = lid.latest()
                if s is not last:
                    now = time.time(); dt = now - tp; tp = now; last = s
                    odo.update(s, (0, 0, w * dt))
                    if abs(odo.pose[2] - prev_th) > math.radians(1):
                        prev_th = odo.pose[2]; t_prev_move = now
                time.sleep(0.03)
            took = time.time() - t0
            car.soft_stop(); time.sleep(0.5)
            real = math.degrees(odo.pose[2])
            print(f"{i + 1}회 {word}: 실제 {real:+.0f}도, {took:.1f}초 (평균 {abs(real) / took:.0f}도/초)", flush=True)
finally:
    car.stop(); lid.close()
