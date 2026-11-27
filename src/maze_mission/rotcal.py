"""회전 속도 재기: 명령 각속도별로 2초 돌고 라이다 스캔 맞춤으로 실제 돈 각도를 잰다 (파이)"""
import math, time, json
from common import load_config
from lidar import RPLidar
from drive import RaspbotCar
from heading import ScanOdom
cfg = load_config()
lid = RPLidar(cfg["lidar"]); lid.wait_ready(); time.sleep(1)
car = RaspbotCar(dict(cfg["drive"], accel=0.5, accel_rot_deg=120))
res = []
try:
    for wdeg in (20, 30, 40, -30):
        last = lid.latest(); odo = ScanOdom(last); t0 = tp = time.time(); w = math.radians(wdeg)
        cmd_int = 0.0
        while time.time() - t0 < 2.0:
            car.drive(0, 0, w); s = lid.latest()
            if s is not last:
                now = time.time(); dt = now - tp; tp = now; last = s
                odo.update(s, (0, 0, car.out[2] * dt if hasattr(car, "out") else w * dt))
            time.sleep(0.03)
        car.soft_stop(); car.stop(); time.sleep(0.8)
        for _ in range(5):
            s = lid.latest()
            if s is not last:
                last = s; odo.update(s, (0, 0, 0))
            time.sleep(0.15)
        real = math.degrees(odo.pose[2])
        res.append((wdeg, round(real, 1), odo.fails)); print(f"명령 {wdeg:+d}도/초 x 2초 (+가속/감속) -> 실제 {real:+.1f}도, 맞춤 실패 {odo.fails}", flush=True)
finally:
    car.stop(); lid.close()
