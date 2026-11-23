"""라이다 막힘 점검: 방향(10도)마다 빔이 얼마나 돌아오는지 보여 준다.

RPLIDAR A1 은 15cm 보다 가까운 물체도 "응답 없음"(거리 0)으로 준다. 그래서 손이 라이다
바로 앞을 가리면, 그 방향은 가까운 점이 찍히는 게 아니라 "응답 없음"이 많아진다.
상자를 로봇 정면 50cm 에 두고 실행했을 때 앞쪽 응답이 낮으면 손(또는 몸)이 가리는 것.

실행 (파이, 라이다 쓰는 다른 프로그램은 끄고):
  python3 lidar_check.py            # 10번 스캔 평균
  python3 lidar_check.py --scans 30
"""
import argparse
import time

from common import load_config
from lidar import RPLidar


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scans", type=int, default=10)
    a = ap.parse_args()
    cfg = load_config()["lidar"]
    cfg = dict(cfg, min_range=0.0, ignore=[])         # 가까운 점도 그대로 본다
    lid = RPLidar(cfg)
    lid.wait_ready(15)
    time.sleep(1.0)
    tot, ok, near = [0] * 36, [0] * 36, [9.0] * 36
    seen, last = 0, None
    while seen < a.scans:
        s = lid.latest()
        if s is not None and s is not last:
            last, seen = s, seen + 1
            for ang, d in s.pts:
                b = int(((ang + 180) % 360) // 10)
                tot[b] += 1
                if d > 0:
                    ok[b] += 1
                    near[b] = min(near[b], d)
        time.sleep(0.05)
    lid.close()
    print(f"로봇 기준 각도 (앞 0, 왼쪽 +), {a.scans}번 스캔, 회전 {lid.hz:.1f}Hz")
    print("  각도         응답률  가장 가까운 점")
    for b in range(36):
        lo = b * 10 - 180
        r = ok[b] / tot[b] if tot[b] else 0
        bar = "#" * int(10 * r)
        flag = "  <- 거의 안 돌아옴" if r < 0.3 else ""
        print(f"  {lo:+4d}~{lo + 10:+4d}  {r:5.0%} {bar:<10} {near[b] if near[b] < 9 else 0:5.2f}m{flag}")
    front = [ok[b] / tot[b] for b in range(36) if tot[b] and -30 <= b * 10 - 180 < 30]
    if front and sum(front) / len(front) < 0.4:
        print("\n앞쪽(-30~+30도) 응답이 40% 미만: 정면에 물체를 두고도 이러면 손/몸이 빔을 가리는 것."
              " 라이다를 손보다 높이 올리거나 config 의 lidar.ignore 로 그 각도를 빼기.")


if __name__ == "__main__":
    main()
