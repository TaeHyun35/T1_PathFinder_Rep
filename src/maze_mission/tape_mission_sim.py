"""11/27 밤: 테이프 네모 시연 흐름 시뮬.
테이프 선 바로 바깥에 카드 3장 + 키보드 마커(ArUco 2) 를 세우고, mission.py --card-press --vwalls 로 돈다.
  탐색/지도 -> 출발점 -> 순서 말하기 -> 카드마다 앞에 서기 -> 키보드 마커 앞 -> 끝

  python tape_mission_sim.py [--seed 1] [--secs 400]
"""
import argparse
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vwalls", default=os.path.join(HERE, "vwalls.json"))
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--secs", type=float, default=400)
    a = ap.parse_args()
    import sim
    from vwalls import VirtualWalls
    d = json.load(open(a.vwalls, encoding="utf-8"))
    truth = VirtualWalls(d["walls"], (0, 0, 0))
    sx, sy, sd = d.get("start", (0, 0, 0))
    xs = [p for w in d["walls"] for p in (w[0], w[2])]
    ys = [p for w in d["walls"] for p in (w[1], w[3])]
    X0, X1, Y0, Y1 = min(xs), max(xs), min(ys), max(ys)
    cross = {"n": 0, "min": 9.0}
    Orig = sim.World
    rng_seed = a.seed

    class TapeWorld(Orig):
        def __init__(self, cfg, seed=1):
            super().__init__(cfg, seed)
            import random
            r = random.Random(rng_seed)
            self.walls, self.door_open = [], True
            x0, x1, y0, y1 = X0 - 1.5, X1 + 1.8, Y0 - 1.2, Y1 + 1.1
            self.room = [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0)),
                         ((x1 - 1.1, y1 - 0.6), (x1, y1 - 0.6)), ((x1 - 1.1, y1 - 0.6), (x1 - 1.1, y1)),
                         ((x0, y0 + 0.7), (x0 + 0.6, y0 + 0.7)), ((x0 + 0.6, y0 + 0.7), (x0 + 0.6, y0))]
            # 테이프 바로 바깥 (2cm), 앞면이 안쪽
            spots = [((X0 + X1) / 2 + r.uniform(-0.3, 0.3), Y1 + 0.02, 0.0, -1.0),
                     (X1 + 0.02, (Y0 + Y1) / 2 + r.uniform(-0.3, 0.3), -1.0, 0.0),
                     ((X0 + X1) / 2 + r.uniform(-0.3, 0.3), Y0 - 0.02, 0.0, 1.0),
                     (X0 - 0.02, (Y0 + Y1) / 2 + r.uniform(-0.3, 0.3), 1.0, 0.0)]
            r.shuffle(spots)
            names = list(cfg["animals"].keys())
            r.shuffle(names)
            self.cards = [{"label": names[i], "digit": str(r.randint(1, 4)), "x": spots[i][0], "y": spots[i][1],
                           "nx": spots[i][2], "ny": spots[i][3]} for i in range(3)]
            kx, ky, knx, kny = spots[3]
            self.keypad = {"x": kx, "y": ky, "nx": knx, "ny": kny}
            self.markers = [{"id": cfg["mission"]["keypad_marker"], "x": kx, "y": ky, "nx": knx, "ny": kny, "w": 0.10}]
            cfg["mission"]["fixed_order"] = [c["label"] for c in r.sample(self.cards, 3)]
            self.x, self.y, self.th = sx, sy, math.radians(sd)

        def outside(self):
            return False

        def step(self, dt):
            super().step(dt)
            dd = truth.inside_dist((self.x, self.y, 0))
            cross["min"] = min(cross["min"], dd)
            if dd < 0.08:
                cross["n"] += 1
            out_ = not (X0 < self.x < X1 and Y0 < self.y < Y1)
            if out_ and not cross.get("out"):
                print(f"[밖으로 나감] t={self.t:.1f} 상태표시={getattr(self, 'tag', '')} 위치 ({self.x:.2f}, {self.y:.2f})", flush=True)
            cross["out"] = out_

    sim.World = TapeWorld
    import mission
    sys.argv = ["mission.py", "run", "--sim", "--seed", str(a.seed), "--secs", str(a.secs), "--card-press", "--vwalls", a.vwalls]
    try:
        mission.main()
    finally:
        print(f"[정답 위치] 테이프 선 8cm 안 횟수 {cross['n']}, 선까지 가장 가까움 {cross['min']:.3f}m", flush=True)


if __name__ == "__main__":
    main()
