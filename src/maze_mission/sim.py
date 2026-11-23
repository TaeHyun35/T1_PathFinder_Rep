"""2D 시뮬레이터: 실제 로봇 없이 mission.py 전체 흐름을 노트북에서 돌려 본다.

진짜 부품과 같은 모양의 가짜 부품을 준다:
  SimLidar.latest() / SimCar.drive() / SimVision.perceive() / SimHand.press() / SimDoor
바퀴는 일부러 오차(속도, 회전 치우침)를 넣고, 카메라는 가끔 못 보거나 잘못 본다.
"""
import math
import random

from common import wrap_deg, RAD
from lidar import Scan

CELL_W, CELL_H = 0.75, 0.90          # 실제 바닥 3.0 x 2.7 m 를 4 x 3 칸으로 나눈 크기
ROBOT_R = 0.13                       # 부딪힘 판정 반지름 (몸통)
HAND_REACH = 0.06                    # 손끝이 몸통 앞보다 더 나온 길이 (부딪힘 판정 보조)


def seg_hit(px, py, dx, dy, s):
    (x1, y1), (x2, y2) = s
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    t = ((x1 - px) * ey - (y1 - py) * ex) / den
    u = ((x1 - px) * dy - (y1 - py) * dx) / den
    return t if (t > 1e-6 and 0 <= u <= 1) else None


def point_seg_dist(px, py, s):
    (x1, y1), (x2, y2) = s
    ex, ey = x2 - x1, y2 - y1
    L = ex * ex + ey * ey
    t = 0 if L == 0 else max(0, min(1, ((px - x1) * ex + (py - y1) * ey) / L))
    return math.hypot(px - (x1 + t * ex), py - (y1 + t * ey))


class World:
    def __init__(self, cfg, seed=1, cols=4, rows=3):
        self.cfg, self.rng = cfg, random.Random(seed)
        self.cols, self.rows = cols, rows
        self.W, self.H = cols * CELL_W, rows * CELL_H
        self._maze()
        self._props()
        self.x, self.y, self.th = CELL_W / 2, CELL_H / 2, 0.0       # 출발: 왼쪽 아래 칸, 오른쪽(+x)을 봄
        self.cmd = (0.0, 0.0, 0.0)
        self.t, self.bumps, self.trace = 0.0, 0, []
        self.tag, self.bump_tag = "", {}
        self.v_scale = 1 + self.rng.uniform(-0.12, 0.12)            # 바퀴 속도 오차
        self.w_scale = 1 + self.rng.uniform(-0.15, 0.15)            # 회전 속도 오차
        self.w_bias = self.rng.uniform(-0.04, 0.04)                 # 직진할 때 한쪽으로 휘는 것 (rad/s)

    # ---------- 미로 ----------
    def _maze(self):
        C, R, rng = self.cols, self.rows, self.rng
        # 벽 있음 표시: v[c][r] = 칸 (c,r) 오른쪽 벽, h[c][r] = 칸 (c,r) 위쪽 벽
        v = [[True] * R for _ in range(C)]
        h = [[True] * R for _ in range(C)]
        seen, stack = {(0, 0)}, [(0, 0)]
        while stack:                                    # 고리 없는 미로 (벽 따라가기로 전부 돌 수 있음)
            c, r = stack[-1]
            nb = [(c + dc, r + dr) for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1))
                  if 0 <= c + dc < C and 0 <= r + dr < R and (c + dc, r + dr) not in seen]
            if not nb:
                stack.pop()
                continue
            n = rng.choice(nb)
            if n[0] != c:
                v[min(c, n[0])][r] = False
            else:
                h[c][min(r, n[1])] = False
            seen.add(n)
            stack.append(n)
        self.walls = []
        for c in range(C):
            for r in range(R):
                if c < C - 1 and v[c][r]:
                    x = (c + 1) * CELL_W
                    self.walls.append(((x, r * CELL_H), (x, (r + 1) * CELL_H)))
                if r < R - 1 and h[c][r]:
                    y = (r + 1) * CELL_H
                    self.walls.append(((c * CELL_W, y), ((c + 1) * CELL_W, y)))
        W, H = self.W, self.H
        # 바깥 벽. 출구 문 = 오른쪽 위 칸의 위쪽 벽
        dc = C - 1
        self.door = ((dc * CELL_W, H), ((dc + 1) * CELL_W, H))
        self.walls += [((0, 0), (W, 0)), ((W, 0), (W, H)), ((0, H), (dc * CELL_W, H)), ((0, 0), (0, H))]
        # 미로 바깥 방 벽 (실제로도 라이다는 방 벽을 본다. 없으면 바깥이 전부 "응답 없음")
        m = 1.5
        self.room = [((-m, -m), (W + m, -m)), ((W + m, -m), (W + m, H + m)), ((W + m, H + m), (-m, H + m)), ((-m, H + m), (-m, -m))]
        self.door_open = False

    def all_walls(self):
        return self.walls + self.room + ([] if self.door_open else [self.door])

    def _props(self):
        rng, cfg = self.rng, self.cfg
        W, H = self.W, self.H
        # 키패드: 오른쪽 위 칸의 오른쪽 바깥 벽 안쪽, 문 옆
        self.keypad = {"x": W - 0.005, "y": H - CELL_H / 2, "nx": -1.0, "ny": 0.0}
        self.markers = [
            {"id": cfg["mission"]["keypad_marker"], "x": W - 0.005, "y": H - CELL_H / 2, "nx": -1.0, "ny": 0.0, "w": 0.10},
            {"id": cfg["mission"]["door_marker"], "x": W - CELL_W / 2, "y": H - 0.005, "nx": 0.0, "ny": -1.0, "w": 0.10},
        ]
        # 동물 카드: 키패드/문이 없는 벽면에 하나씩
        faces = []
        for s in self.walls:
            (x1, y1), (x2, y2) = s
            mx, my = (x1 + x2) / 2, (y1 + y2) / 2
            L = math.hypot(x2 - x1, y2 - y1)
            nx, ny = -(y2 - y1) / L, (x2 - x1) / L
            for sgn in (1, -1):
                fx, fy = mx + sgn * nx * 0.005, my + sgn * ny * 0.005
                inside = 0.05 < fx < W - 0.05 and 0.05 < fy < H - 0.05
                near_kp = math.hypot(fx - self.keypad["x"], fy - self.keypad["y"]) < 0.3
                if inside and not near_kp and math.hypot(fx - CELL_W / 2, fy - CELL_H / 2) > 0.45:
                    faces.append((fx, fy, sgn * nx, sgn * ny))
        rng.shuffle(faces)
        names = list(cfg["animals"].keys())
        rng.shuffle(names)
        n = cfg["mission"]["expect"]
        self.cards = [{"label": names[i], "x": f[0], "y": f[1], "nx": f[2], "ny": f[3]} for i, f in enumerate(faces[:n])]

    # ---------- 물리 ----------
    def step(self, dt):
        sub = max(1, int(dt / 0.02))
        h = dt / sub
        for _ in range(sub):
            vx, vy, w = self.cmd
            noise = self.rng.gauss(0, 0.02)
            vx *= self.v_scale * (1 + noise)
            vy *= self.v_scale * (1 + noise)
            w = w * self.w_scale + (self.w_bias if abs(vx) > 0.01 else 0.0)
            nth = self.th + w * h
            c, s = math.cos(self.th), math.sin(self.th)
            nx = self.x + (vx * c - vy * s) * h
            ny = self.y + (vx * s + vy * c) * h
            if self._collide(nx, ny, nth):
                self.bumps += 1
                self.bump_tag[self.tag] = self.bump_tag.get(self.tag, 0) + 1
                self.th = nth if not self._collide(self.x, self.y, nth) else self.th
            else:
                self.x, self.y, self.th = nx, ny, nth
            self.t += h
        self.trace.append((self.t, self.x, self.y, self.th))

    def _collide(self, x, y, th):
        hx, hy = x + math.cos(th) * (ROBOT_R + HAND_REACH), y + math.sin(th) * (ROBOT_R + HAND_REACH)
        for s in self.all_walls():
            if point_seg_dist(x, y, s) < ROBOT_R or point_seg_dist(hx, hy, s) < 0.005:
                return True
        return False

    def outside(self):
        return self.y > self.H + 0.25

    def los(self, x, y, tx, ty):
        dx, dy = tx - x, ty - y
        L = math.hypot(dx, dy)
        for s in self.all_walls():
            t = seg_hit(x, y, dx / L, dy / L, s)
            if t is not None and t < L - 0.02:
                return False
        return True


class SimClock:
    def __init__(self, world):
        self.w = world

    def now(self):
        return self.w.t

    def sleep(self, s):
        if s > 0:
            self.w.step(s)


class SimLidar:
    def __init__(self, world):
        self.w = world

    def wait_ready(self, timeout=0):
        return True

    def latest(self):
        w, pts = self.w, []
        walls = w.all_walls()
        for k in range(0, 360, 1):
            a = k - 180.0
            ang = w.th + a * RAD
            dx, dy = math.cos(ang), math.sin(ang)
            best = math.inf
            for s in walls:
                t = seg_hit(w.x, w.y, dx, dy, s)
                if t is not None and t < best:
                    best = t
            d = best + w.rng.gauss(0, 0.006) if best < 12 else 0.0
            pts.append((a, d if d > 0.12 else 0.0))
        return Scan(pts, w.t)

    def close(self):
        pass


class SimCar:
    def __init__(self, world):
        self.w = world
        self.cmd = (0.0, 0.0, 0.0)

    def drive(self, vx, vy, w):
        self.cmd = self.w.cmd = (vx, vy, w)

    def stop(self):
        self.drive(0.0, 0.0, 0.0)


class SimVision:
    """앞 카메라: 시야각 안, 1.6 m 안, 가려지지 않고, 카드를 너무 비스듬히 보지 않으면 보임"""

    def __init__(self, world, cam_cfg, miss=0.2, wrong=0.02):
        self.w, self.fov = world, cam_cfg["fov_deg"]
        self.miss, self.wrong = miss, wrong

    def _view(self, o):
        w = self.w
        dx, dy = o["x"] - w.x, o["y"] - w.y
        d = math.hypot(dx, dy)
        if d > 1.6 or d < 0.08:
            return None
        bearing = wrap_deg(math.degrees(math.atan2(dy, dx) - w.th))
        if abs(bearing) > self.fov / 2:
            return None
        inc = math.degrees(math.acos(max(-1, min(1, (-dx * o["nx"] - dy * o["ny"]) / d))))
        if inc > 65:
            return None
        if not w.los(w.x, w.y, o["x"] + o["nx"] * 0.02, o["y"] + o["ny"] * 0.02):
            return None
        return d, 0.5 - bearing / self.fov

    def perceive(self):
        w, out = self.w, {"animals": [], "markers": []}
        for c in w.cards:
            v = self._view(c)
            if v and w.rng.random() > self.miss:
                lab = c["label"]
                if w.rng.random() < self.wrong:
                    lab = w.rng.choice(list(w.cfg["animals"].keys()))
                out["animals"].append({"label": lab, "conf": 0.8, "cx": v[1], "w": 0.15 / v[0]})
        for m in w.markers:
            v = self._view(m)
            if v and w.rng.random() > 0.1:
                out["markers"].append({"id": m["id"], "cx": v[1], "size": m["w"] / (2 * v[0] * math.tan(self.fov / 2 * RAD))})
        return out


class SimDoor:
    """문 스테이션: 정답 비밀번호를 알고, 눌린 숫자를 모아 맞으면 문을 연다"""

    def __init__(self, world, code):
        self.w, self.code, self.buf, self.state = world, code, "", "closed"

    def key(self, digit):
        if self.state == "open":
            return
        self.buf += digit
        if len(self.buf) >= len(self.code):
            if self.buf == self.code:
                self.state, self.w.door_open = "open", True
            else:
                self.state = "wrong"
            self.buf = ""

    def send_code(self, code):
        for d in code:
            self.key(d)

    def status(self):
        s = self.state
        if s == "wrong":
            self.state = "closed"
        return s


class SimHand:
    """손끝이 키패드 위에 있을 때만 그 손가락 아래 키가 눌린다.
    키 4개(1~4)가 손가락 간격(22 mm)과 같게 한 줄로 놓였다고 가정."""

    PITCH = 0.022

    def __init__(self, world, door, cfg):
        self.w, self.door, self.cfg = world, door, cfg
        self.pressed = []

    def open(self):
        pass

    def press(self, finger, angle=None, sleep=None):
        w, kp = self.w, self.w.keypad
        # 키패드 기준 좌표: 벽까지 거리, 옆 오차
        dx, dy = w.x - kp["x"], w.y - kp["y"]
        dist = dx * kp["nx"] + dy * kp["ny"]
        lat = dx * (-kp["ny"]) + dy * kp["nx"]          # 키패드를 보고 섰을 때 왼쪽(+)
        facing = abs(wrap_deg(math.degrees(w.th) - math.degrees(math.atan2(-kp["ny"], -kp["nx"])))) < 12
        pd = self.cfg["mission"]["press_dist"]
        ok_dist = abs(dist - pd) < 0.035 and facing
        # 손가락 f 의 옆 위치 = lat + (3.5-f)*간격 (검지가 왼쪽). 키도 같은 간격으로 놓임.
        # 손가락 f 아래에 있는 키 = |lat + (키손가락 - f)*간격| < 간격/2 인 키
        key = None
        if ok_dist:
            for digit, kf in self.cfg["hand"]["key_finger"].items():
                if abs(lat + (kf - finger) * self.PITCH) < self.PITCH / 2:
                    key = digit
                    break
        self.pressed.append((finger, key))
        if key:
            self.door.key(key)
        if sleep:
            sleep(0.8)
        return key

    def close(self):
        pass


def draw(world, path, title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["Malgun Gothic", "DejaVu Sans"]
    fig, ax = plt.subplots(figsize=(7, 6.6))
    for (x1, y1), (x2, y2) in world.walls:
        ax.plot([x1, x2], [y1, y2], color="#33424f", lw=3)
    (x1, y1), (x2, y2) = world.door
    ax.plot([x1, x2], [y1, y2], color="#2e9e5b" if world.door_open else "#c0392b", lw=4, ls="--")
    xs = [p[1] for p in world.trace]
    ys = [p[2] for p in world.trace]
    ax.plot(xs, ys, color="#1f77b4", lw=1.2, alpha=0.8, label="로봇 경로")
    ax.plot(xs[:1], ys[:1], "o", color="#1f77b4", ms=9, label="출발")
    for c in world.cards:
        ax.plot(c["x"], c["y"], "s", color="#e67e22", ms=11)
        ax.annotate(world.cfg["animals"][c["label"]]["ko"], (c["x"], c["y"]), xytext=(c["x"] + c["nx"] * 0.14, c["y"] + c["ny"] * 0.14),
                    ha="center", va="center", fontsize=10, color="#a04000")
    kp = world.keypad
    ax.plot(kp["x"], kp["y"], "D", color="#8e44ad", ms=11, label="키패드")
    ax.set_aspect("equal")
    ax.set_xlim(-0.2, world.W + 0.2)
    ax.set_ylim(-0.2, world.H + 0.6)
    ax.set_title(title, fontsize=11)
    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
