"""11/27 밤: 테이프 미로 = 가상 벽. 회의실이 좁아 우드락 미로 대신 바닥에 테이프로 구역을 표시하고,
그 선을 라이다 점처럼 스캔에 더해 준다 -> 벽 따라 돌기 / 지도 / 길찾기 코드가 진짜 벽처럼 피한다.
라이다는 회의실 벽/가구로 '내 위치'를 맞추는 데 쓰인다 (ScanOdom).

vwalls.json (미로 좌표, m):
  {"start": [x, y, 방향deg], "walls": [[x1, y1, x2, y2], ...]}
출발 자리가 주행거리계 (0, 0, 0) 이므로, 미로 좌표 -> 주행거리계 좌표로 바꿔서 쓴다.

  vw = VirtualWalls.load("vwalls.json")
  scan2 = vw.add(scan, pose)          # pose = 주행거리계 (x, y, th)
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))


class VirtualWalls:
    def __init__(self, walls, start=(0.0, 0.0, 0.0), step=0.015, max_r=2.5, door=None):
        sx, sy, sth = start[0], start[1], math.radians(start[2])
        c, s = math.cos(-sth), math.sin(-sth)
        self.pts = []                              # 주행거리계 좌표의 벽 점들
        self.segs = []
        self.door_open = False                     # 출구(문)는 비밀번호 확인 뒤에 열림 (그 전엔 벽)
        self.door_pts, self.door_seg = [], None
        for k_, (x1, y1, x2, y2) in enumerate(list(walls) + ([door] if door else [])):
            is_door = door is not None and k_ == len(walls)
            a = ((x1 - sx) * c - (y1 - sy) * s, (x1 - sx) * s + (y1 - sy) * c)
            b = ((x2 - sx) * c - (y2 - sy) * s, (x2 - sx) * s + (y2 - sy) * c)
            n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / step))
            pts = [(a[0] + (b[0] - a[0]) * (k / n), a[1] + (b[1] - a[1]) * (k / n)) for k in range(n + 1)]
            if is_door:
                self.door_seg, self.door_pts = (a, b), pts
            else:
                self.segs.append((a, b))
                self.pts += pts
        self.max_r = max_r

    @classmethod
    def load(cls, path=None):
        path = path or os.path.join(HERE, "vwalls.json")
        d = json.load(open(path, encoding="utf-8"))
        return cls(d["walls"], d.get("start", (0, 0, 0)), door=d.get("door"))

    def rebase(self, x0, y0, th0):
        """주행거리계가 출발 자리가 아니라 (x0, y0, th0) 에서 0 으로 시작했을 때 (roommap: 방향 확인/한 바퀴 뒤에 시작)"""
        c, s = math.cos(-th0), math.sin(-th0)

        def f(p):
            return ((p[0] - x0) * c - (p[1] - y0) * s, (p[0] - x0) * s + (p[1] - y0) * c)
        self.pts = [f(p) for p in self.pts]
        self.door_pts = [f(p) for p in self.door_pts]
        self.segs = [(f(a), f(b)) for a, b in self.segs]
        if self.door_seg:
            self.door_seg = (f(self.door_seg[0]), f(self.door_seg[1]))

    def rel(self, pose):
        """지금 로봇 기준 (각도deg, 거리m) 가상 벽 점"""
        x, y, th = pose
        out = []
        for px, py in self.pts + ([] if self.door_open else self.door_pts):
            dx, dy = px - x, py - y
            d = math.hypot(dx, dy)
            if 0.05 < d < self.max_r:
                out.append(((math.degrees(math.atan2(dy, dx) - th) + 180) % 360 - 180, d))
        return out

    def add(self, scan, pose):
        """가상 벽 점을 더하고, 그 너머(같은 방향 더 먼) 진짜 점은 지움 -> 테이프가 '안 비치는 벽'.
        11/27 밤: 그냥 더하기만 하면 ray()(방향 중간값)가 너머의 방 벽 점들에 묻혀 테이프를 못 봄"""
        from obstacles import _Scan
        vr = self.rel(pose)
        near = {}
        for a, d in vr:
            k = int(math.floor(a)) % 360
            if d < near.get(k, 99.0):
                near[k] = d
        # 빈 칸 메우기 (가상 점 간격이 1도보다 넓은 가까운 곳)
        for k in list(near):
            for kk in ((k - 1) % 360, (k + 1) % 360):
                if kk not in near:
                    near[kk] = near[k]
        keep = [(a, d) for a, d in scan.pts if not (d > 0 and d > near.get(int(math.floor(a)) % 360, 99.0) + 0.02)]
        return _Scan(scan, keep + vr)

    def outside(self, pose, margin=0.20):
        """미로 테두리(모든 벽 점의 사각형) 밖으로 margin 넘게 나갔나 = 탈출"""
        xs = [p[0] for p in self.pts]
        ys = [p[1] for p in self.pts]
        x, y, _ = pose
        return x < min(xs) - margin or x > max(xs) + margin or y < min(ys) - margin or y > max(ys) + margin

    def inside_dist(self, pose):
        """로봇 중심에서 가장 가까운 가상 벽까지 거리 (m)"""
        x, y, _ = pose
        best = 9.0
        for (ax, ay), (bx, by) in self.segs:
            vx, vy = bx - ax, by - ay
            L = vx * vx + vy * vy or 1e-9
            t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / L))
            best = min(best, math.hypot(x - ax - t * vx, y - ay - t * vy))
        return best
