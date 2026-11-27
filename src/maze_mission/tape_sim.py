"""11/27 밤: 테이프 네모 시뮬. 벽 없는 방(책상 2개) 안에 가상 벽만 있는 상태로 roommap 을 돌린다.
로봇이 실제(정답) 위치로 테이프 선에 몇 번 닿았는지 센다.

  python tape_sim.py --vwalls vwalls.json --secs 150
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vwalls", default=os.path.join(HERE, "vwalls.json"))
    ap.add_argument("--secs", type=float, default=150)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    import sim
    from vwalls import VirtualWalls
    d = json.load(open(a.vwalls, encoding="utf-8"))
    truth = VirtualWalls(d["walls"], (0, 0, 0), door=d.get("door"))      # 미로 좌표 그대로
    sx, sy, sth = d.get("start", (0, 0, 0))
    xs = [p for w in d["walls"] for p in (w[0], w[2])]
    ys = [p for w in d["walls"] for p in (w[1], w[3])]
    x0, x1, y0, y1 = min(xs) - 1.5, max(xs) + 1.8, min(ys) - 1.2, max(ys) + 1.1
    cross = {"n": 0, "min": 9.0}
    Orig = sim.World

    class TapeWorld(Orig):
        def __init__(self, cfg, seed=1):
            super().__init__(cfg, seed)
            self.walls, self.door_open = [], True
            self.room = [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0)),
                         ((x1 - 1.1, y1 - 0.6), (x1, y1 - 0.6)), ((x1 - 1.1, y1 - 0.6), (x1 - 1.1, y1)),
                         ((x0, y0 + 0.7), (x0 + 0.6, y0 + 0.7)), ((x0 + 0.6, y0 + 0.7), (x0 + 0.6, y0))]
            self.cards, self.markers = [], []
            import math
            self.x, self.y, self.th = sx, sy, math.radians(sth)

        def step(self, dt):
            super().step(dt)
            dd = truth.inside_dist((self.x, self.y, 0))
            cross["min"] = min(cross["min"], dd)
            if dd < 0.08:
                cross["n"] += 1

    sim.World = TapeWorld
    import roommap
    sys.argv = ["roommap.py", "run", "--sim", "--yes", "--secs", str(a.secs), "--vwalls", a.vwalls, "--cover", "0",
                "--seed", str(a.seed)]
    try:
        roommap.main()
    finally:
        print(f"[정답 위치] 테이프 선 8cm 안으로 들어간 횟수 {cross['n']}, 선까지 가장 가까웠던 거리 {cross['min']:.3f}m", flush=True)


if __name__ == "__main__":
    main()
