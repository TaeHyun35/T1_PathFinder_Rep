"""안 가 본 길 찾기 (11/27): 벽 따라 한 바퀴만 돌면 섬처럼 둘러싸인 곳(침대-책상 사잇길)은 안 들어감.
라이다로 '비어 있다고 확인한 바닥' 과 '아직 모르는 곳' 의 경계(프론티어)를 찾아서, 그쪽으로 길을 짜서 간다.

  g = Grid()
  g.add(scan, pose)                 # 스캔마다 (주행거리계 pose = x, y, th)
  goal, path = g.next_goal(pose)    # 가 볼 곳과 가는 길 [(x, y), ...] 또는 (None, None)
"""
import heapq
import math
import os

import numpy as np

CELL = 0.05


class Grid:
    def __init__(self, size_m=14.0, cell=CELL, robot_r=0.20, max_r=4.0):
        self.cell, self.n = cell, int(size_m / cell)
        self.o = self.n // 2                       # (0,0) 이 가운데
        self.L = np.zeros((self.n, self.n), np.float32)   # +: 물체, -: 빈 바닥, 0: 모름
        self.robot_r, self.max_r = robot_r, max_r
        self.visited = []                          # 가 봤던 목표 (같은 데 또 안 가게)
        self.cam_cnt = np.zeros((self.n, self.n), np.uint8)   # 카메라가 가까이서 본 벽 칸 (카드 읽을 수 있는 거리)

    def ij(self, x, y):
        return int(math.floor(x / self.cell)) + self.o, int(math.floor(y / self.cell)) + self.o

    def xy(self, i, j):
        return (i - self.o + 0.5) * self.cell, (j - self.o + 0.5) * self.cell

    def add(self, scan, pose, step=2, cam_half=20.0, cam_r=1.1):
        x, y, th = pose
        # 카메라(몸 정면, 화면 폭 48도)가 1.1m 안에서 본 벽 칸 세기. 3번 넘게 봐야 '본 것' (카드 확인은 3번 연속)
        mark = set()
        for a0, d0 in scan.pts:
            if abs(a0) < cam_half and 0.12 < d0 < cam_r:
                i, j = self.ij(x + d0 * math.cos(th + math.radians(a0)), y + d0 * math.sin(th + math.radians(a0)))
                if 1 <= i < self.n - 1 and 1 <= j < self.n - 1:
                    mark.update((i + di, j + dj) for di in (-1, 0, 1) for dj in (-1, 0, 1))
        for i, j in mark:
            if self.cam_cnt[i, j] < 250:
                self.cam_cnt[i, j] += 1
        pts = [(a, d) for a, d in scan.pts[::step] if 0.12 < d]
        if not pts:
            return
        a = th + np.radians([p[0] for p in pts])
        d = np.array([p[1] for p in pts])
        hit = d < self.max_r
        dd = np.minimum(d, self.max_r)
        # 빈 바닥: 라이다 줄 위 (끝 8cm 앞까지)
        fx, fy = [], []
        for ai, di in zip(a, dd):
            r = np.arange(0.10, max(0.10, di - 0.08), self.cell * 0.7)
            fx.append(x + r * np.cos(ai))
            fy.append(y + r * np.sin(ai))
        if fx:
            fx, fy = np.concatenate(fx), np.concatenate(fy)
            fi = np.floor(fx / self.cell).astype(int) + self.o
            fj = np.floor(fy / self.cell).astype(int) + self.o
            ok = (fi >= 0) & (fi < self.n) & (fj >= 0) & (fj < self.n)
            idx = np.unique(fi[ok] * self.n + fj[ok])
            self.L.flat[idx] = np.maximum(self.L.flat[idx] - 0.4, -4.0)
        hx, hy = x + d[hit] * np.cos(a[hit]), y + d[hit] * np.sin(a[hit])
        hi = np.floor(hx / self.cell).astype(int) + self.o
        hj = np.floor(hy / self.cell).astype(int) + self.o
        ok = (hi >= 0) & (hi < self.n) & (hj >= 0) & (hj < self.n)
        idx = np.unique(hi[ok] * self.n + hj[ok])
        self.L.flat[idx] = np.minimum(self.L.flat[idx] + 1.2, 4.0)

    def add_points(self, pts_xy):
        """카메라 장애물 같은 바닥 좌표 점을 물체로"""
        for px, py in pts_xy:
            i, j = self.ij(px, py)
            if 0 <= i < self.n and 0 <= j < self.n:
                self.L[i, j] = min(self.L[i, j] + 1.2, 4.0)

    # ------------------------------------------------------------ 길 짜기
    def _maps(self):
        occ = self.L > 0.6
        free = self.L < -0.6
        unk = ~occ & ~free
        # 물체를 로봇 반지름만큼 부풀리기 (원 모양)
        r = int(math.ceil(self.robot_r / self.cell))
        blocked = np.zeros_like(occ)
        oi, oj = np.nonzero(occ)
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                if di * di + dj * dj <= r * r:
                    ii, jj = oi + di, oj + dj
                    k = (ii >= 0) & (ii < self.n) & (jj >= 0) & (jj < self.n)
                    blocked[ii[k], jj[k]] = True
        # 손이 돌 수 있는 거리(35cm) 안 = '가까움' -> 길 짤 때 비싸게 (되도록 넓은 데로)
        rn = int(math.ceil(0.35 / self.cell))
        near = np.zeros_like(occ)
        for di in range(-rn, rn + 1):
            for dj in range(-rn, rn + 1):
                if di * di + dj * dj <= rn * rn:
                    ii, jj = oi + di, oj + dj
                    k = (ii >= 0) & (ii < self.n) & (jj >= 0) & (jj < self.n)
                    near[ii[k], jj[k]] = True
        self._near = near
        return occ, free, unk, blocked

    def frontiers(self, min_cells=6):
        """빈 바닥인데 바로 옆이 '모름' 인 칸 덩어리 -> [(가운데 x, y, 칸 수, [(i,j)...])]"""
        occ, free, unk, blocked = self._maps()
        u = np.zeros_like(unk)
        u[1:, :] |= unk[:-1, :]; u[:-1, :] |= unk[1:, :]
        u[:, 1:] |= unk[:, :-1]; u[:, :-1] |= unk[:, 1:]
        fr = free & u
        seen = np.zeros_like(fr)
        out = []
        for i0, j0 in zip(*np.nonzero(fr)):
            if seen[i0, j0]:
                continue
            stack, cells = [(i0, j0)], []
            seen[i0, j0] = True
            while stack:
                i, j = stack.pop()
                cells.append((i, j))
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        a, b = i + di, j + dj
                        if 0 <= a < self.n and 0 <= b < self.n and fr[a, b] and not seen[a, b]:
                            seen[a, b] = True
                            stack.append((a, b))
            if len(cells) >= min_cells:
                ci = sum(c[0] for c in cells) / len(cells)
                cj = sum(c[1] for c in cells) / len(cells)
                out.append((*self.xy(ci, cj), len(cells), cells))
        return out, blocked, free

    def view_targets(self, min_cells=8, view_d=0.70, dbg=False):
        """카메라가 아직 가까이서 안 본 벽 덩어리 -> [(볼 자리 x, y, 바라볼 방향, 덩어리 가운데 x, y, 칸 수)]"""
        occ, free, unk, blocked = self._maps()
        strong = self.L > 1.5
        nf = np.zeros_like(free)                   # 빈 바닥과 닿은 벽 칸 (로봇 쪽 얼굴)
        for di in (-2, -1, 0, 1, 2):
            for dj in (-2, -1, 0, 1, 2):
                nf |= np.roll(np.roll(free, di, 0), dj, 1)
        cand = strong & nf & (self.cam_cnt < 3)
        seen = np.zeros_like(cand)
        out = []
        for i0, j0 in zip(*np.nonzero(cand)):
            if seen[i0, j0]:
                continue
            stack, cells = [(i0, j0)], []
            seen[i0, j0] = True
            while stack:
                i, j = stack.pop()
                cells.append((i, j))
                for di in (-2, -1, 0, 1, 2):
                    for dj in (-2, -1, 0, 1, 2):
                        a, b = i + di, j + dj
                        if 0 <= a < self.n and 0 <= b < self.n and cand[a, b] and not seen[a, b]:
                            seen[a, b] = True
                            stack.append((a, b))
            if dbg:
                print("  덩어리", len(cells), self.xy(*cells[0]))
            # 긴 벽은 60cm 조각으로 나눠서 조각마다 볼 자리
            rest = list(cells)
            while rest:
                i0, j0 = rest[0]
                chunk = [c for c in rest if (c[0] - i0) ** 2 + (c[1] - j0) ** 2 <= 36]
                rest = [c for c in rest if (c[0] - i0) ** 2 + (c[1] - j0) ** 2 > 36]
                if len(chunk) < min_cells:
                    continue
                t = self._viewpoint(chunk, free, blocked, view_d)
                if dbg:
                    print("    조각", len(chunk), "볼 자리", t)
                if t:
                    out.append(t + (len(chunk),))
        return out

    def _viewpoint(self, cells, free, blocked, view_d):
        ci = sum(c[0] for c in cells) / len(cells)
        cj = sum(c[1] for c in cells) / len(cells)
        ni = nj = 0.0                              # 벽 바깥 방향(빈 바닥 쪽)
        for i, j in cells:
            for di in range(-4, 5):
                for dj in range(-4, 5):
                    a, b = i + di, j + dj
                    if 0 <= a < self.n and 0 <= b < self.n and free[a, b]:
                        ni += di; nj += dj
        nn = math.hypot(ni, nj)
        if nn < 1e-6:
            return None
        ni, nj = ni / nn, nj / nn
        cx, cy = self.xy(ci, cj)
        second = None
        for dv in (view_d, view_d + 0.12, view_d - 0.12, view_d + 0.25, view_d + 0.4):
            for sa in (0, 15, -15, 30, -30):
                r = math.radians(sa)
                ux, uy = ni * math.cos(r) - nj * math.sin(r), ni * math.sin(r) + nj * math.cos(r)
                vx, vy = cx + ux * dv, cy + uy * dv
                a, b = self.ij(vx, vy)
                if 0 <= a < self.n and 0 <= b < self.n and free[a, b] and not blocked[a, b]:
                    t = (vx, vy, math.atan2(cy - vy, cx - vx), cx, cy)
                    if not self._near[a, b]:
                        return t
                    second = second or t
        return second

    def plan(self, start, goal_cells, blocked, free):
        """빈 바닥 + 안 부풀린 칸만 지나서 goal_cells 중 하나로 (A*, 8방향). -> [(x, y)...] 또는 None"""
        si, sj = self.ij(*start)
        goals = set(goal_cells)
        ok = free & ~blocked
        near = getattr(self, "_near", np.zeros_like(free))
        gi = np.mean([g[0] for g in goal_cells]); gj = np.mean([g[1] for g in goal_cells])
        # 출발 칸이 부풀린 곳 안이면(벽 옆) 그냥 통과 허용
        h = lambda i, j: math.hypot(i - gi, j - gj) * 0.5
        dist = {(si, sj): 0.0}
        prev = {}
        q = [(h(si, sj), 0.0, si, sj)]
        steps = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        n_pop = 0
        while q:
            _, g, i, j = heapq.heappop(q)
            n_pop += 1
            if n_pop > 60000:
                return None
            if (i, j) in goals:
                p = [(i, j)]
                while p[-1] in prev:
                    p.append(prev[p[-1]])
                return [self.xy(*c) for c in reversed(p)]
            if g > dist.get((i, j), 1e9):
                continue
            for di, dj, c in steps:
                a, b = i + di, j + dj
                if not (0 <= a < self.n and 0 <= b < self.n):
                    continue
                near_start = abs(a - si) + abs(b - sj) <= 4
                if not (ok[a, b] or (near_start and free[a, b]) or (a, b) in goals and not blocked[a, b]):
                    continue
                ng = g + c * (3.0 if near[a, b] else 1.0)
                if ng < dist.get((a, b), 1e9):
                    dist[(a, b)] = ng
                    prev[(a, b)] = (i, j)
                    heapq.heappush(q, (ng + h(a, b), ng, a, b))
        return None

    def next_goal(self, pose, min_cells=6, min_dist=0.5):
        """가장 가까운(길 기준) 큰 프론티어. 가 본 목표 근처(0.5m)는 뺌"""
        frs, blocked, free = self.frontiers(min_cells)
        cands = []
        for fx, fy, n, cells in frs:
            if math.hypot(fx - pose[0], fy - pose[1]) < min_dist:
                continue
            if any(math.hypot(fx - vx, fy - vy) < 0.5 for vx, vy in self.visited):
                continue
            # 로봇이 들어갈 수 있는 칸만 목표로
            gc = [c for c in cells if not blocked[c]]
            if len(gc) < 2:
                continue
            cands.append((math.hypot(fx - pose[0], fy - pose[1]) - 0.05 * n, fx, fy, gc))
        cands.sort()
        for _, fx, fy, gc in cands[:6]:
            path = self.plan(pose[:2], gc, blocked, free)
            if path:
                return (fx, fy), path
            self.visited.append((fx, fy))          # 못 가는 곳은 다시 안 봄
        return None, None


def simplify(path, step=0.25):
    """칸 단위 길 -> 25cm 간격 점"""
    out, acc = [path[0]], 0.0
    for a, b in zip(path, path[1:]):
        acc += math.hypot(b[0] - a[0], b[1] - a[1])
        if acc >= step:
            out.append(b)
            acc = 0.0
    if out[-1] != path[-1]:
        out.append(path[-1])
    return out


def clear_move(scan, dx, dy, finger=0.315, body_r=0.13, hand_w=0.06):
    """몸(원)과 손(앞으로 뻗은 띠)을 (dx, dy) 만큼 옮겨도 라이다 점에 안 닿나"""
    for a, d in scan.pts:
        if d <= 0 or d > finger + 0.5:
            continue
        r = math.radians(a)
        px, py = d * math.cos(r) - dx, d * math.sin(r) - dy
        if math.hypot(px, py) < body_r:
            return False
        if 0.0 < px < finger + 0.04 and abs(py) < hand_w:
            return False
    return True


class Nav:
    """프론티어로 가기 / 지정한 곳으로 가기. 스캔마다 step() -> ((vx, vy, w), 상태글자, 알릴 일)
    길을 따라가다 손 앞이 막히면 뒤로 조금 빠지고 길을 다시 짠다. 3번 막히면 그 목표는 포기."""

    def __init__(self, grid, cfg, home=(0.0, 0.0), max_r=3.0, v=0.20):
        self.g, self.cfg, self.home, self.max_r, self.v = grid, cfg, home, max_r, v
        self.goal, self.cells, self.path = None, None, None
        self.face, self.look_until = None, 0.0      # 볼 자리에 도착하면 바라볼 방향, 멈춰서 보는 시간
        self.t_plan, self.blocks, self.back_until, self.spin_left = -9.0, 0, 0.0, 0.0
        self.mode = "explore"           # explore -> (목표 없음) done ; goto: 지정한 곳
        self.done = False
        self.reached = 0
        self.last_t = None
        self.hist = []                  # (t, x, y, th) 걸림 감지용
        self.rot_dir = 0
        self.t_goal = 0.0

    def go_to(self, xy):
        self.mode, self.done = "goto", False
        self.goal = xy
        i, j = self.g.ij(*xy)
        self.cells = [(i + a, j + b) for a in range(-2, 3) for b in range(-2, 3)]
        self.path, self.t_plan, self.blocks = None, -9.0, 0
        self.t_goal = self.last_t   # None = 첫 step 에서 지금 시각으로 (11/27: 새 Nav 로 go_to 하면 0 이라 바로 45초 초과)

    def _pick(self, pose):
        self.face = None
        frs, blocked, free = self.g.frontiers()
        cands = []
        for vx, vy, face, cx, cy, n in self.g.view_targets():
            if math.hypot(cx - self.home[0], cy - self.home[1]) > self.max_r:
                continue
            if any(math.hypot(vx - qx, vy - qy) < 0.35 for qx, qy in self.g.visited):
                continue
            i, j = self.g.ij(vx, vy)
            cands.append((math.hypot(vx - pose[0], vy - pose[1]) - 0.02 * n, vx, vy, [(i, j)], face))
        for fx, fy, n, cells in frs:
            if math.hypot(fx - self.home[0], fy - self.home[1]) > self.max_r:
                continue                           # 방 밖(열린 문 너머)으로는 안 감
            if math.hypot(fx - pose[0], fy - pose[1]) < 0.45:
                continue
            if any(math.hypot(fx - vx, fy - vy) < 0.5 for vx, vy in self.g.visited):
                continue
            gc = [c for c in cells if not blocked[c]]
            if len(gc) < 2:
                continue
            cands.append((math.hypot(fx - pose[0], fy - pose[1]) - 0.03 * n, fx, fy, gc, None))
        cands.sort(key=lambda c: c[0])
        for _, fx, fy, gc, face in cands[:8]:
            p = self.g.plan(pose[:2], gc, blocked, free)
            if p:
                self.face = face
                return (fx, fy), gc, p
            self.g.visited.append((fx, fy))
        return None, None, None

    def _replan(self, pose):
        occ, free, unk, blocked = self.g._maps()
        if self.mode == "goto":
            # 집 가는 길은 '모름' 칸도 지나가게 (처음 출발점 근처는 다 봤지만 혹시 몰라)
            return self.g.plan(pose[:2], self.cells, blocked, free | unk)
        return self.g.plan(pose[:2], self.cells, blocked, free)

    def step(self, scan, pose, now):
        from obstacles import hand_corridor
        c = self.cfg["wall"]
        finger, margin = c.get("finger_m", 0.315), c.get("finger_margin", 0.08)
        x, y, th = pose
        self.g_pose = pose
        ev = None
        if now < self.back_until:
            back = scan.sector_min(180, 35)
            if math.isnan(back) or back > 0.25:
                return (-0.15, 0.0, 0.0), "길찾기-뒤로", None
            # 뒤도 막힘 -> 옆으로 (메카넘) 넓은 쪽
            lft, rgt = scan.sector_min(90, 30), scan.sector_min(-90, 30)
            lft = 9.0 if math.isnan(lft) else lft
            rgt = 9.0 if math.isnan(rgt) else rgt
            if max(lft, rgt) > 0.25:
                return (0.0, 0.12 if lft > rgt else -0.12, 0.0), "길찾기-옆으로", None
            self.back_until = 0.0
        if self.face is not None and self.goal is None and self.look_until >= 0:
            e = math.atan2(math.sin(self.face - pose[2]), math.cos(self.face - pose[2]))
            if abs(e) > math.radians(8) and self.look_until == 0.0:
                sd = 1 if e > 0 else -1
                sweep = scan.sector_min(sd * 50, 50)
                if math.isnan(sweep) or sweep >= finger + 0.04:
                    return (0.0, 0.0, sd * math.radians(45)), "길찾기-벽 보기 돌기", None
            if self.look_until == 0.0:
                self.look_until = now + 1.5
            if now < self.look_until:
                return (0.0, 0.0, 0.0), "길찾기-벽 보는 중", None
            self.face, self.look_until = None, 0.0
        if self.spin_left > 0:                     # 프론티어 도착 -> 한 바퀴 보기 (카드 찾기)
            dt = 0.1 if self.last_t is None else min(0.3, now - self.last_t)
            self.last_t = now
            sweep = scan.sector_min(50, 50)
            if not math.isnan(sweep) and sweep < finger + 0.04:
                self.spin_left = 0.0
                return (0.0, 0.0, 0.0), "길찾기-돌 공간 없음", "돌 공간이 없어 한 바퀴 보기 생략"
            self.spin_left -= math.radians(45) * dt
            return (0.0, 0.0, math.radians(45)), "길찾기-한바퀴보기", None
        self.last_t = now
        # 목표 고르기
        if self.goal is None:
            if self.mode == "goto":
                self.done = True
                return (0.0, 0.0, 0.0), "길찾기-끝", None
            self.goal, self.cells, self.path = self._pick(pose)
            self.t_plan, self.blocks, self.t_goal, self.rot_dir = now, 0, now, 0
            if self.goal is None:
                self.done = True
                return (0.0, 0.0, 0.0), "길찾기-끝", "더 가 볼 곳 없음"
            ev = ("벽 보러 감 " if self.face is not None else "안 가 본 곳으로 감 ") + f" ({self.goal[0]:+.2f}, {self.goal[1]:+.2f}), 길 {len(self.path) * self.g.cell:.1f}m"
        if self.t_goal is None:
            self.t_goal = now
        if self.mode == "goto" and now - self.t_goal > 45.0:
            self.done = True
            return (0.0, 0.0, 0.0), "길찾기-포기", "45초 넘게 출발점에 못 감 -> 똑바로 돌아가기로"
        if self.mode == "explore" and now - self.t_goal > 25.0:
            self.g.visited.append(self.goal)
            self.goal, self.face, self.path = None, None, None
            return (0.0, 0.0, 0.0), "길찾기-포기", "25초 넘게 못 감 -> 이 목표 포기"
        # 도착?
        dgoal = math.hypot(self.goal[0] - x, self.goal[1] - y)
        if (self.mode == "goto" and dgoal < 0.12) or (self.mode == "explore" and dgoal < (0.15 if self.face is not None else 0.30)):
            self.g.visited.append(self.goal)
            self.goal, self.path = None, None
            self.reached += 1
            if self.mode == "explore":
                if self.face is not None:
                    self.look_until = 0.0
                    return (0.0, 0.0, 0.0), "길찾기-도착", "볼 자리 도착 -> 벽 바라보기"
                self.spin_left = 2 * math.pi
                return (0.0, 0.0, 0.0), "길찾기-도착", "도착 -> 한 바퀴 둘러보기"
            self.done = True
            return (0.0, 0.0, 0.0), "길찾기-도착", "도착"
        # 1.5초마다 길 다시 짜기 (지도가 계속 늘어남)
        if self.path is None or now - self.t_plan > 1.5:
            p = self._replan(pose)
            self.t_plan = now
            if p is None and os.environ.get("NAVDBG"):
                occ_, free_, unk_, bl_ = self.g._maps()
                si, sj = self.g.ij(x, y)
                gi, gj = self.g.ij(*self.goal)
                okm = (free_ | unk_) & ~bl_
                from collections import deque
                seen = {(si, sj)}; dq = deque([(si, sj)])
                while dq and len(seen) < 80000:
                    i_, j_ = dq.popleft()
                    for a_, b_ in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        q_ = (i_ + a_, j_ + b_)
                        if q_ not in seen and okm[q_]:
                            seen.add(q_); dq.append(q_)
                print(f"[NAVDBG] reach {len(seen)} goal_in {(gi, gj) in seen} start {(si, sj)} goal {(gi, gj)} cells0 {self.cells[0]}", flush=True)
                print(f"[NAVDBG] start blocked {bl_[si, sj]} free {free_[si, sj]} occ {occ_[si, sj]} | goal blocked {bl_[gi, gj]} free {free_[gi, gj]} unk {unk_[gi, gj]} occ {occ_[gi, gj]} | mode {self.mode}", flush=True)
            if p is None:
                self.g.visited.append(self.goal)
                self.face = None
                ev = f"({self.goal[0]:+.2f}, {self.goal[1]:+.2f}) 가는 길이 막힘 -> 다른 곳"
                self.goal, self.path = None, None
                return (0.0, 0.0, 0.0), "길찾기-다시", ev
            self.path = p
        # 30cm 앞 길 위의 점을 보고 가기
        tgt = self.path[-1]
        k0 = min(range(len(self.path)), key=lambda k: math.hypot(self.path[k][0] - x, self.path[k][1] - y))
        for px, py in self.path[k0:]:
            if math.hypot(px - x, py - y) > 0.30:
                tgt = (px, py)
                break
        e = math.atan2(tgt[1] - y, tgt[0] - x) - th
        e = math.atan2(math.sin(e), math.cos(e))
        body_front = scan.sector_min(0, 25)
        hc = hand_corridor(scan, finger, c.get("hand_half_w", 0.08), margin)
        if abs(e) > math.radians(30):
            # 한 번 정한 도는 방향은 30도 안으로 들어올 때까지 유지 (뒤쪽 목표에서 좌우로 왔다 갔다 하던 문제)
            if not self.rot_dir or abs(e) < math.radians(150) and (1 if e > 0 else -1) != self.rot_dir and abs(e) < math.radians(60):
                self.rot_dir = 1 if e > 0 else -1
            sd = self.rot_dir
            room = lambda k: (lambda d: math.isnan(d) or d >= finger + 0.04)(scan.sector_min(k * 50, 50))
            if not room(sd):
                if room(-sd) and abs(e) > math.radians(100):
                    sd = -sd                                 # 반대로 크게 돌기
                else:
                    # 메카넘: 몸은 그대로 두고 가야 할 쪽으로 미끄러지듯 (그 쪽이 30cm 넘게 비었을 때)
                    if abs(e) < math.radians(120) and clear_move(scan, math.cos(e) * 0.10, math.sin(e) * 0.10, finger):
                        if self._stuck(now, x, y):
                            return self._blocked(now, "옆으로 가다 걸림", ev)
                        return (0.12 * math.cos(e), 0.12 * math.sin(e), 0.0), "길찾기-옆으로 가기", ev
                    return self._blocked(now, "돌 공간 없음", ev)
            self.hist = []
            return (0.0, 0.0, sd * math.radians(45)), "길찾기-방향", ev
        self.rot_dir = 0
        if (not math.isnan(body_front) and body_front < finger + margin) or hc is not None:
            return self._blocked(now, "손 앞 막힘", ev)
        # 걸림: 가라고 했는데 2초 동안 3cm 도 안 움직임 (라이다보다 낮은 것, 몸 옆 모서리)
        if self._stuck(now, x, y):
            if self.goal is not None:
                # 막힌 자리를 지도에 물체로 (몸 앞)
                self.g.add_points([(x + math.cos(th) * f, y + math.sin(th) * f) for f in (0.14, 0.18)])
            return self._blocked(now, "걸림", ev)
        # 몸 옆이 너무 가까우면 반대로 살짝 틀기
        lft, rgt = scan.sector_min(70, 30), scan.sector_min(-70, 30)
        side_push = 0.0
        if not math.isnan(lft) and lft < 0.18:
            side_push -= 0.4
        if not math.isnan(rgt) and rgt < 0.18:
            side_push += 0.4
        dist_left = math.hypot(tgt[0] - x, tgt[1] - y)
        v = self.v if self.mode == "explore" or dgoal > 0.4 else max(0.10, min(self.v, dgoal))
        w = max(-0.6, min(0.6, 2.0 * e + side_push))
        return (v, 0.0, w), "길찾기-가기", ev

    def _stuck(self, now, x, y):
        self.hist = [h for h in self.hist if now - h[0] <= 2.0] + [(now, x, y, 0.0)]
        if now - self.hist[0][0] > 1.8 and math.hypot(x - self.hist[0][1], y - self.hist[0][2]) < 0.03:
            self.hist = []
            return True
        return False

    def _blocked(self, now, why, ev):
        if self.mode == "goto" and self.goal is not None and math.hypot(self.goal[0] - self.g_pose[0], self.goal[1] - self.g_pose[1]) < 0.35:
            self.done = True                       # 출발점 바로 앞인데 막힘 -> 여기서 끝 (마지막은 go_home 이 맞춤)
            return (0.0, 0.0, 0.0), "길찾기-도착", f"{why} -> 출발점 근처라 여기서 끝"
        self.blocks += 1
        self.back_until = now + 1.0
        self.path = None
        if self.blocks >= 3 and self.mode == "explore":
            self.g.visited.append(self.goal)
            self.goal, self.face = None, None
            return (0.0, 0.0, 0.0), "길찾기-포기", f"{why} 3번 -> 이 목표 포기"
        if self.blocks >= 8 and self.mode == "goto":
            self.done = True
            return (0.0, 0.0, 0.0), "길찾기-포기", f"{why} 8번 -> 길 따라 가기 그만"
        return (0.0, 0.0, 0.0), "길찾기-막힘", ev or f"{why} -> 뒤로 조금 빠졌다가 길 다시 짜기 ({self.blocks}번째)"
