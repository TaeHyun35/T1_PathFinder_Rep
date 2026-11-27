"""내 방(오병주 방) 시뮬레이터 — 사진 4장 + 잰 길이 2개로 만든 1차 버전.

미로 시뮬(sim.py)과 다른 점
  - 가구가 있다. 종류마다 라이다/몸통/손에 보이는지가 다르다.
      wall   : 라이다에 보이고 부딪힘 (벽, 침대, 옷장, 책장 ...)
      sparse : 라이다에 가끔만 보이고 부딪힘 (옷걸이 철망, 책상 철다리) -> 실제로 로봇이 "벽이 멀다"고 보고 비볐음
      floor  : 라이다에 안 보이고 바퀴가 걸림 (바닥 전선)
  - 로봇이 실제처럼 움직인다: 손이 길고(라이다 중심에서 31.5cm), 무거워서 설 때 더 밀리고,
    앞으로 갈 때 왼쪽으로 도는 버릇, 바닥 미끄럼(마루/고무)
  - 라이다는 손에 가린 앞쪽이 자주 빈다 (실측 앞 ±30도 응답 63%)
  - 손 위 카메라: 라이다가 못 보는 것까지 보이는 "앞 물체 거리" (가끔 놓침)

좌표 (m): 원점 = 방문 있는 벽과 침대 쪽 긴 벽이 만나는 구석
  x: 방문 벽(0) -> 창문 벽(L)      y: 침대 쪽 벽(0) -> 책상 쪽 벽(W)

실행:
  python room_sim.py plan                      방 배치도 그림 (치수 확인용)
  python room_sim.py run --secs 180 --seed 1   지금 벽따라가기 규칙으로 돌려 보기
  python room_sim.py batch --n 20              여러 번 돌려서 어디에 부딪히는지 통계
"""
import argparse
import math
import os
import random
import time

from common import HERE, load_config
from lidar import Scan
from sim import point_seg_dist, seg_hit

# ---------------------------------------------------------------- 방 배치 (사진으로 어림, 확인 필요한 값은 ?)
L, W = 4.30, 2.70          # 문 벽 -> 창문 벽 4.3, 침대 쪽 벽 -> 책상 쪽 벽 2.7 (실측)
LIDAR_H = 0.27             # 바닥 -> 라이다 높이 (실측). 이보다 낮은 물건은 라이다에 안 보임
FINGER_LOW = 0.08          # 바닥 -> 손가락 아래 (실측). 8cm 넘는 물건은 손에 걸림


def box(x0, y0, x1, y1):
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


def pole(x, y, r=0.015):
    return box(x - r, y - r, x + r, y + r)


FURNITURE = [
    # 이름, 종류, 선분들, (sparse 일 때 라이다에 맞을 확률)
    # 종류 low = 라이다(27cm)보다 낮아서 안 보이지만 손(8cm~)/몸에 걸림
    # 책상 쪽 벽 (방문 쪽부터): 옷걸이 0.5 | 책장 1.16 | 프린터 0.4 | 책상 1.20 | 흰 옷장 1.04  = 4.30
    ("침대", "wall", box(2.30, 0.00, L, 1.10), 1.0),                       # 머리 = 창문 벽. 폭 1.1/길이 2.0 은 어림
    ("협탁+공기청정기", "wall", box(3.90, 1.12, L, 1.55), 1.0),
    ("흰 옷장", "wall", box(3.26, W - 0.61, L, W), 1.0),                  # 104 x 61 (실측)
    ("책상 철다리", "sparse", pole(2.08, W - 0.48) + pole(3.24, W - 0.48), 0.5),   # 책상 120 x 50 (실측), 상판은 라이다보다 높음
    ("컴퓨터 본체", "wall", box(2.08, W - 0.55, 2.30, W - 0.05), 1.0),
    ("프린터+휴지통", "wall", box(1.66, W - 0.40, 2.06, W), 1.0),
    ("책장", "wall", box(0.50, W - 0.30, 1.66, W), 1.0),                  # 116 x 30 (실측)
    ("옷걸이 아래칸/상자", "low", box(0.00, 1.85, 0.50, 2.65), 0.0),      # 80 x 50 (실측), 붙박이장에 붙임. 27cm 높이엔 기둥뿐
    ("옷걸이 기둥", "sparse", pole(0.48, 1.87) + pole(0.48, 2.63), 0.6),
]
FLOOR = []                                                                 # 바닥 전선은 시험 때 치움 (넣으려면 ("바닥 전선", (x0, y0, x1, y1)))
ROOM = box(0, 0, L, W)
DOOR = ((0.0, 0.05), (0.0, 0.95))                                          # 방문 (닫혀 있음 = 벽)

# ---------------------------------------------------------------- 로봇 (실측 2026-11-23)
BODY_R = 0.12              # 몸통 반지름 (라즈봇 V2 약 24 x 18cm)
FINGER = 0.315             # 라이다 중심 -> 손가락 끝 (라이다 앞면에서 28cm)
HAND_W = 0.045             # 손 반폭
CAM_X = 0.215              # 카메라 위치 (손가락 끝보다 10cm 뒤)
CAM_FOV = 48.0

SLIP = 0.0   # 헛돎 비율 (--slip)
MEMORY = False   # 장애물 기억 (--memory)
FLOORS = {                 # 바닥별: 설 때 감속(무게로 밀림), 속도 흔들림, 회전 흔들림
    "wood":   {"decel": 1.0, "noise": 0.08, "wnoise": 0.10},
    "rubber": {"decel": 1.6, "noise": 0.04, "wnoise": 0.06},
}


class RoomWorld:
    def __init__(self, cfg, seed=1, floor="wood", start=None):
        self.cfg, self.rng = cfg, random.Random(seed)
        self.fl = FLOORS[floor]
        self.walls_vis, self.walls_sparse, self.walls_all = list(ROOM), [], list(ROOM)
        self.names = {}
        for name, kind, segs, p in FURNITURE:
            for s in segs:
                self.names[s] = name
                self.walls_all.append(s)
                if kind == "wall":
                    self.walls_vis.append(s)
                else:
                    self.walls_sparse.append((s, p))
        for s in ROOM:
            self.names[s] = "벽"
        self.x, self.y, self.th = start or self._random_start()
        self.cmd, self.vel = (0.0, 0.0, 0.0), [0.0, 0.0, 0.0]
        self.t, self.trace, self.tag = 0.0, [], ""
        self.bumps, self.bump_what, self.bump_tag, self.snag = 0, {}, {}, 0
        self.yaw_bias = math.radians(10.0 + self.rng.uniform(-4, 4))       # 0.76m/s 일 때 왼쪽으로 도는 정도
        self.v_scale = 1 + self.rng.uniform(-0.08, 0.08)
        # 11/26 실측 (D:/aihand_isaac/twin_params.json): 느린 회전 명령도 실제론 초당 약 33도 아래로 안 내려감
        self.rot_min = math.radians(33.0)
        self.slip = SLIP                      # 무게추 없으면 0.45 (명령 구간의 45% 거의 안 움직임), 왼쪽 뒤에 두면 0
        self.slip_left = 0.0                  # 지금 헛도는 중이면 남은 시간
        self.hand_hit = None

    def _random_start(self):
        for _ in range(500):
            x, y, th = self.rng.uniform(0.3, 2.2), self.rng.uniform(0.3, 2.1), self.rng.uniform(-math.pi, math.pi)
            if not self._collide(x, y, th) and not self._on_floor_hazard(x, y):
                return x, y, th
        return 1.2, 1.2, 0.0

    # ---------- 부딪힘 ----------
    def _hand_pts(self, x, y, th):
        c, s = math.cos(th), math.sin(th)
        out = []
        for fx, fy in ((FINGER, 0), (FINGER, HAND_W), (FINGER, -HAND_W), (0.22, HAND_W), (0.22, -HAND_W)):
            out.append((x + c * fx - s * fy, y + s * fx + c * fy))
        return out

    def _collide(self, x, y, th):
        # 가구 안쪽에 들어가 있는 것도 부딪힘 (선분 거리만 보면 상자 한가운데는 못 잡음)
        for name, kind, segs, p in FURNITURE:
            xs = [q[0] for s in segs for q in s]
            ys = [q[1] for s in segs for q in s]
            if len(segs) == 4:
                pts = [(x, y)] + self._hand_pts(x, y, th)
                if any(min(xs) < px < max(xs) and min(ys) < py < max(ys) for px, py in pts):
                    return name + "(안쪽)"
        if not (0 < x < L and 0 < y < W):
            return "벽(밖)"
        for s in self.walls_all:
            if point_seg_dist(x, y, s) < BODY_R:
                return self.names.get(s, "?") + "(몸)"
        for hx, hy in self._hand_pts(x, y, th):
            for s in self.walls_all:
                if point_seg_dist(hx, hy, s) < 0.008:
                    return self.names.get(s, "?") + "(손)"
        return None

    def _on_floor_hazard(self, x, y):
        for name, (x0, y0, x1, y1) in FLOOR:
            if x0 - BODY_R < x < x1 + BODY_R and y0 - BODY_R < y < y1 + BODY_R:
                return name
        return None

    # ---------- 움직임 ----------
    def step(self, dt):
        sub = max(1, int(dt / 0.02))
        h = dt / sub
        for _ in range(sub):
            # 무거운 손: 빨라질 때 1.5 m/s^2, 느려질 때는 바닥 따라 (마루 1.0 = 더 밀림)
            for i, (acc_up, acc_dn) in enumerate(((1.5, self.fl["decel"]), (1.5, self.fl["decel"]), (10.0, 10.0))):
                cur, tgt = self.vel[i], self.cmd[i]
                a = acc_up if abs(tgt) > abs(cur) else acc_dn
                self.vel[i] = cur + max(-a * h, min(a * h, tgt - cur))
            vx, vy, w = self.vel
            if 0.02 < abs(w) < self.rot_min and abs(self.cmd[2]) > 0.02:     # 약한 회전 힘은 모터 최소 속도로 (실측)
                w = math.copysign(self.rot_min * self.rng.uniform(0.8, 1.15), w)
            if self.slip > 0:                                              # 뒷바퀴가 떠서 헛돎: 0.5~2초씩 거의 안 움직임
                if self.slip_left <= 0 and self.rng.random() < self.slip * h / 1.0:
                    self.slip_left = self.rng.uniform(0.5, 2.0)
                if self.slip_left > 0:
                    self.slip_left -= h
                    vx, vy, w = vx * 0.1, vy * 0.1, w * 0.1
            n = self.rng.gauss(0, self.fl["noise"])
            vx *= self.v_scale * (1 + n)
            vy *= self.v_scale * (1 + n)
            w = w * (1 + self.rng.gauss(0, self.fl["wnoise"])) + self.yaw_bias * (vx / 0.76)
            haz = self._on_floor_hazard(self.x, self.y)
            if haz:                                          # 전선 위: 바퀴가 걸려서 덜 가고 틀어짐
                vx, vy = vx * 0.3, vy * 0.3
                w += self.rng.gauss(0, 0.8)
                self.snag += 1
            nth = self.th + w * h
            c, s = math.cos(self.th), math.sin(self.th)
            nx, ny = self.x + (vx * c - vy * s) * h, self.y + (vx * s + vy * c) * h
            hit = self._collide(nx, ny, nth)
            if hit:
                self.bumps += 1
                self.bump_what[hit] = self.bump_what.get(hit, 0) + 1
                self.bump_tag[self.tag] = self.bump_tag.get(self.tag, 0) + 1
                self.vel = [0.0, 0.0, 0.0]
                if not self._collide(self.x, self.y, nth):
                    self.th = nth
            else:
                self.x, self.y, self.th = nx, ny, nth
            self.t += h
        self.trace.append((self.t, self.x, self.y, self.th))

    # ---------- 센서 ----------
    def lidar_pts(self):
        pts = []
        for k in range(360):
            a = k - 180.0
            ang = self.th + math.radians(a)
            dx, dy = math.cos(ang), math.sin(ang)
            best = math.inf
            for s in self.walls_vis:
                t = seg_hit(self.x, self.y, dx, dy, s)
                if t is not None and t < best:
                    best = t
            for s, p in self.walls_sparse:
                t = seg_hit(self.x, self.y, dx, dy, s)
                if t is not None and t < best and self.rng.random() < p:
                    best = t
            drop = 0.40 if -60 <= a <= 30 else 0.12           # 손에 가린 앞쪽은 자주 빈다
            if best > 6 or self.rng.random() < drop:
                pts.append((a, 0.0))
            else:
                d = best + self.rng.gauss(0, 0.008)
                pts.append((a, d if d > 0.12 else 0.0))
        return Scan(pts, self.t)

    def cam_front(self, miss=0.1, rng_max=1.0):
        """손 위 카메라로 본 앞 물체까지 거리 (손가락 끝 기준). 바닥 전선이 보이면 hazard=True"""
        cx, cy = self.x + math.cos(self.th) * CAM_X, self.y + math.sin(self.th) * CAM_X
        best = math.inf
        for k in range(-int(CAM_FOV / 2), int(CAM_FOV / 2) + 1, 3):
            ang = self.th + math.radians(k)
            dx, dy = math.cos(ang), math.sin(ang)
            for s in self.walls_all:
                t = seg_hit(cx, cy, dx, dy, s)
                if t is not None and t < best:
                    best = t
        dist = best - (FINGER - CAM_X)
        if dist > rng_max or self.rng.random() < miss:
            dist = math.inf
        hazard = False
        for k in (0.15, 0.3, 0.45):                          # 화면 아래쪽 = 손 앞 바닥
            px, py = cx + math.cos(self.th) * k, cy + math.sin(self.th) * k
            if self._on_floor_hazard(px, py):
                hazard = True
        return dist, hazard


class RoomLidar:
    def __init__(self, w):
        self.w = w

    def wait_ready(self, timeout=0):
        return True

    def latest(self):
        return self.w.lidar_pts()

    def close(self):
        pass


class RoomCar:
    def __init__(self, w):
        self.w, self.cmd = w, (0.0, 0.0, 0.0)

    def drive(self, vx, vy, w):
        if not all(math.isfinite(v) for v in (vx, vy, w)):
            vx = vy = w = 0.0
        self.cmd = self.w.cmd = (vx, vy, w)

    def stop(self):
        self.drive(0.0, 0.0, 0.0)
        self.w.vel = [0.0, 0.0, 0.0]

    def soft_stop(self, max_s=1.0):
        self.drive(0.0, 0.0, 0.0)


# ---------------------------------------------------------------- 그림
def draw(world, path, title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    plt.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, ax = plt.subplots(figsize=(9, 6.2))
    col = {"wall": "#8a9aa8", "sparse": "#e0a040", "low": "#c070c0"}
    for name, kind, segs, p in FURNITURE:
        xs = [q[0] for s in segs for q in s]
        ys = [q[1] for s in segs for q in s]
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
        if len(segs) == 4:
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc=col[kind], ec="#333", alpha=0.55, hatch="//" if kind == "sparse" else None))
            ax.text((x0 + x1) / 2, (y0 + y1) / 2, name, ha="center", va="center", fontsize=8)
        else:
            for k in range(0, len(segs), 4):
                q = segs[k:k + 4]
                qx = [a[0] for s in q for a in s]
                qy = [a[1] for s in q for a in s]
                ax.add_patch(Rectangle((min(qx), min(qy)), max(qx) - min(qx), max(qy) - min(qy), fc=col[kind], ec="#333"))
    for name, (x0, y0, x1, y1) in FLOOR:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc="#d64545", alpha=0.25, ec="#d64545", ls="--"))
        ax.text((x0 + x1) / 2, (y0 + y1) / 2, name, ha="center", va="center", fontsize=8, color="#a02020")
    for (x1, y1), (x2, y2) in ROOM:
        ax.plot([x1, x2], [y1, y2], color="#33424f", lw=3)
    (x1, y1), (x2, y2) = DOOR
    ax.plot([x1, x2], [y1, y2], color="#2a6fdb", lw=6)
    ax.text(0.06, 0.5, "방문", rotation=90, va="center", fontsize=9, color="#2a6fdb")
    ax.text(L - 0.06, 1.85, "창문(커튼)", rotation=90, va="center", ha="right", fontsize=9, color="#555")
    ax.text(0.06, 1.8, "붙박이장", rotation=90, va="center", fontsize=8, color="#555")
    if world is not None and world.trace:
        xs = [p[1] for p in world.trace]
        ys = [p[2] for p in world.trace]
        ax.plot(xs, ys, color="#1f77b4", lw=1.1, alpha=0.85, label="로봇 경로")
        ax.plot(xs[:1], ys[:1], "o", color="#1a9e55", ms=9, label="출발")
        ax.plot(xs[-1:], ys[-1:], "s", color="#d64545", ms=7, label="끝")
        ax.legend(loc="lower left", fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlim(-0.15, L + 0.15)
    ax.set_ylim(-0.15, W + 0.15)
    ax.set_xlabel("m (방문 벽 -> 창문 벽)")
    ax.set_ylabel("m (침대 벽 -> 책상 벽)")
    ax.grid(True, color="#e5e5e5", lw=0.5)
    ax.set_title(title, fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------- 지금 규칙(벽 따라가기)으로 돌려 보기
def run_follow(cfg, seed, secs, floor="wood", quiet=False):
    from mission import follow
    from sim import SimClock
    from obstacles import ScanMemory
    mem = ScanMemory() if MEMORY else None
    cfg = dict(cfg)
    cfg["wall"] = dict(cfg["wall"], finger_m=FINGER, finger_margin=0.08, front_stop=0.50)
    w = RoomWorld(cfg, seed=seed, floor=floor)
    lid, car, clock = RoomLidar(w), RoomCar(w), SimClock(w)
    tags, stall = {}, 0
    while w.t < secs:
        s = lid.latest()
        if mem is not None:
            s = mem.update(s, (w.x, w.y, w.th))
        vx, vy, wz, tag = follow(s, cfg, cfg["wall"]["side"])
        w.tag = tag
        car.drive(vx, vy, wz)
        tags[tag] = tags.get(tag, 0) + 1
        clock.sleep(0.13)
    covered = len({(int(p[1] / 0.25), int(p[2] / 0.25)) for p in w.trace})
    return w, tags, covered


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["plan", "run", "batch"])
    ap.add_argument("--secs", type=float, default=180)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--floor", default="wood", choices=list(FLOORS))
    ap.add_argument("--d_ref", type=float, default=None)
    ap.add_argument("--memory", action="store_true", help="장애물 기억 (최근 본 점을 쌓아 둠)")
    ap.add_argument("--corridor", type=int, default=1, help="손 띠 검사 (1 켬, 0 끔)")
    ap.add_argument("--slip", type=float, default=0.0, help="뒷바퀴 들려 헛도는 정도 (무게추 없음 0.45, 11/26)")
    ap.add_argument("--hand_deg", type=float, default=None)
    a = ap.parse_args()
    cfg = load_config()
    global SLIP, MEMORY
    SLIP, MEMORY = a.slip, a.memory
    cfg["wall"]["hand_corridor"] = bool(a.corridor)
    if a.d_ref is not None:
        cfg["wall"]["d_ref"] = a.d_ref
    if a.hand_deg is not None:
        cfg["wall"]["hand_half_deg"] = a.hand_deg
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    if a.cmd == "plan":
        p = os.path.join(HERE, "logs", "room_plan.png")
        draw(None, p, f"내 방 배치 (어림) {L} x {W} m  — 주황 = 라이다에 가끔 보임, 보라 = 라이다(27cm)보다 낮아 안 보임")
        print("그림:", p)
    elif a.cmd == "run":
        w, tags, cov = run_follow(cfg, a.seed, a.secs, a.floor)
        p = os.path.join(HERE, "logs", f"room_run_seed{a.seed}_{a.floor}.png")
        draw(w, p, f"지금 규칙 (벽 따라가기) {a.secs:.0f}초, 바닥 {a.floor}: 부딪힘 {w.bumps}번 {w.bump_what}, 전선 걸림 {w.snag}")
        print("상태:", tags)
        print("부딪힘:", w.bumps, w.bump_what, "상태별:", w.bump_tag, "전선:", w.snag, "다닌 칸(25cm):", cov)
        print("그림:", p)
    else:
        tot, what, tagb, snags, ok = 0, {}, {}, 0, 0
        for sd in range(1, a.n + 1):
            w, tags, cov = run_follow(cfg, sd, a.secs, a.floor)
            tot += w.bumps
            snags += w.snag
            ok += w.bumps == 0 and w.snag == 0
            for k, v in w.bump_what.items():
                what[k] = what.get(k, 0) + v
            for k, v in w.bump_tag.items():
                tagb[k] = tagb.get(k, 0) + v
            print(f"  seed {sd:2d}: 부딪힘 {w.bumps:4d} 전선 {w.snag:4d} 다닌 칸 {cov:3d}  {w.bump_what}")
        print(f"\n깨끗하게 끝난 판 {ok}/{a.n}, 부딪힘 평균 {tot / a.n:.0f}, 전선 걸림 평균 {snags / a.n:.0f}")
        print("무엇에:", dict(sorted(what.items(), key=lambda kv: -kv[1])))
        print("그때 상태:", dict(sorted(tagb.items(), key=lambda kv: -kv[1])))


if __name__ == "__main__":
    main()
