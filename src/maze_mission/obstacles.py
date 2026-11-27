"""장애물 기억 (11/27). 라이다에 가끔만 보이는 물체(의자 다리, 옷걸이 그물)를 놓치지 않게,
최근 몇 초 동안 본 점을 '바닥 좌표'로 쌓아 두었다가 지금 스캔에 다시 더해 준다.
카메라가 본 장애물(라이다보다 낮은 것)도 같은 방식으로 점으로 넣는다.
-> follow() 는 바뀐 것 없이 '더 꽉 찬 스캔' 을 받는다.

  mem = ScanMemory()
  merged = mem.update(scan, pose, extra=[(각도deg, 거리m), ...])   # pose = (x, y, th) 주행거리계
"""
import math
import time


class _Scan:
    """lidar.Scan 과 같은 모양 (pts, t + sector_min/ray/coverage)"""

    def __init__(self, base, pts):
        self.pts, self.t = pts, base.t
        self._cls = base._cls if isinstance(base, _Scan) else type(base)   # 겹쳐 싸도 원래 Scan 의 계산을 씀

    def __getattr__(self, k):
        if k == "_cls":
            raise AttributeError(k)
        f = getattr(self._cls, k)
        if not callable(f):
            return f                                   # NEAR, NEED 같은 값
        return lambda *a, **kw: f(self, *a, **kw)


class ScanMemory:
    def __init__(self, keep_s=6.0, max_r=1.6, cell=0.03, min_hits=1, only_extra=False):
        self.only_extra = only_extra
        self.keep_s, self.max_r, self.cell, self.min_hits = keep_s, max_r, cell, min_hits
        self.cells = {}            # (ix, iy) -> [마지막 본 시각, 본 횟수, x, y]

    def _world(self, pose, a_deg, d):
        x, y, th = pose
        a = th + math.radians(a_deg)
        return x + d * math.cos(a), y + d * math.sin(a)

    def update(self, scan, pose, extra=()):
        now = time.time()
        src = [] if self.only_extra else list(scan.pts)
        for a, d in src + [(a, d) for a, d in extra]:
            if not (0.12 < d < self.max_r):
                continue
            wx, wy = self._world(pose, a, d)
            k = (int(wx // self.cell), int(wy // self.cell))
            c = self.cells.get(k)
            if c:
                c[0], c[1] = now, c[1] + 1
            else:
                self.cells[k] = [now, 1, wx, wy]
        # 오래된 것 지우기
        self.cells = {k: c for k, c in self.cells.items() if now - c[0] < self.keep_s}
        # 지금 로봇 기준으로 다시 그려서 지금 스캔에 더하기
        x, y, th = pose
        add = []
        for c in self.cells.values():
            if c[1] < self.min_hits:
                continue
            dx, dy = c[2] - x, c[3] - y
            d = math.hypot(dx, dy)
            if 0.12 < d < self.max_r:
                add.append((math.degrees(math.atan2(dy, dx) - th + math.pi) % 360 - 180, d))
        return _Scan(scan, list(scan.pts) + add)


def hand_corridor(scan, finger=0.315, half_w=0.08, margin=0.08):
    """손이 지나갈 띠(앞으로 finger+margin, 좌우 half_w+2cm) 안에 점이 있나 -> 가장 가까운 앞 거리 또는 None"""
    near = None
    for a, d in scan.pts:
        if d <= 0:
            continue
        r = math.radians(a)
        px, py = d * math.cos(r), d * math.sin(r)
        if 0.05 < px < finger + margin and abs(py) < half_w + 0.02:
            near = px if near is None else min(near, px)
    return near


# ---------------------------------------------------------------- 카메라 장애물 -> 라이다 점
CAM_H, CAM_TILT_UP, HFOV, W, H = 0.14, 11.8, 48.0, 640, 480
OBST_CLASSES = {"blanket", "clothes", "drying rack", "shelf", "clothes rack", "trash can", "table leg", "computer case", "stool", "cart", "chair", "cable", "backpack", "suitcase", "handbag", "bench", "couch", "potted plant", "bottle", "person",
                "dining table", "bed", "toilet", "tv", "laptop", "box"}


def cam_obstacles(dets, cam_x=0.21, lidar_scan=None, tilt_up=None):
    """PC 가 보낸 일반 물체 상자 [[이름, 확신도, x1, y1, x2, y2], ...] -> 라이다 기준 점 [(각도deg, 거리m)]
    카메라가 위로 12도 들려 있어 바닥은 약 1.2m 부터 보인다. 상자 아래가 화면 아래 끝에 닿으면 '1.2m 안' 이라는 뜻.
    거리: 상자 아래 = 바닥 닿는 곳으로 계산, 화면 아래에 잘렸으면 그 방향 라이다 거리, 그것도 없으면 0.6m (조심)"""
    f = (W / 2) / math.tan(math.radians(HFOV / 2))
    tu = CAM_TILT_UP if tilt_up is None else tilt_up
    out = []
    for name, cf, x1, y1, x2, y2 in dets:
        if name not in OBST_CLASSES or cf < (0.35 if name in WHEELED else 0.4):
            continue
        cut = y2 > H - 6
        wide = (x2 - x1) > 0.6 * W                 # 화면 폭 60% 넘게 = 아주 가까움 (선반 위 옷처럼 바닥에 안 닿는 것)
        if not cut:
            down = math.degrees(math.atan((y2 - H / 2) / f)) - tu                # 바닥 닿는 곳까지 아래 각도
            if down < 1.0:
                continue                                       # 너무 먼 곳 (지평선 근처)
            d_cam = CAM_H / math.tan(math.radians(down))
            if wide:
                d_cam = min(d_cam, 0.45)
            if d_cam > 1.5:
                continue
        for u in (x1, (x1 + x2) / 2, x2):                     # 왼끝, 가운데, 오른끝 세 방향
            b = -math.degrees(math.atan((u - W / 2) / f))
            if cut:
                d_cam = None
                if lidar_scan is not None:
                    r = lidar_scan.ray(b, 3)
                    if math.isfinite(r) and r < 1.2:
                        d_cam = max(0.1, r - cam_x)
                d_cam = 0.6 if d_cam is None else d_cam
                if wide:
                    d_cam = min(d_cam, 0.45)
            # 카메라 기준 -> 라이다 기준
            px, py = cam_x + d_cam * math.cos(math.radians(b)), d_cam * math.sin(math.radians(b))
            out.append((math.degrees(math.atan2(py, px)), math.hypot(px, py)))
        if name in WHEELED:
            ring = _base_ring(lidar_scan, x1, x2, f, cam_x) if lidar_scan is not None else []
            if ring:
                out += ring
            elif not cut:
                # 라이다에 아예 안 보임(옷걸이 가는 봉, 의자 받침): 바퀴가 몸보다 앞으로 나와 있음 -> 20cm 더 앞에 한 줄 더, 폭도 넓게
                dn = max(0.25, d_cam - 0.20)
                for u in (x1 - 30, x1, (x1 + x2) / 2, x2, x2 + 30):
                    b = -math.degrees(math.atan((u - W / 2) / f))
                    px, py = cam_x + dn * math.cos(math.radians(b)), dn * math.sin(math.radians(b))
                    out.append((math.degrees(math.atan2(py, px)), math.hypot(px, py)))
    return out


# 11/27: 의자 다리 바퀴(별 모양 받침)는 라이다(27cm)보다 낮아 안 보이고, 라이다엔 가운데 기둥만 가늘게 보임
#  -> 로봇이 기둥 바로 옆까지 가서 바퀴가 받침에 걸림. 카메라가 의자류를 본 방향에 가는 기둥이 있으면 둘레 35cm 를 막음
WHEELED = {"chair", "stool", "cart", "clothes rack", "drying rack"}
BASE_R = 0.35


def _base_ring(scan, x1, x2, f, cam_x):
    a1 = -math.degrees(math.atan((x2 - W / 2) / f))
    a2 = -math.degrees(math.atan((x1 - W / 2) / f))
    pts = []
    for a, d in scan.pts:
        if not (0.12 < d < 2.0):
            continue
        r = math.radians(a)
        px, py = d * math.cos(r), d * math.sin(r)
        ca = math.degrees(math.atan2(py, px - cam_x))          # 카메라에서 본 방향
        if a1 - 3 <= ca <= a2 + 3:
            pts.append((px, py))
    if not pts:
        return []
    out = []
    # 가까운 점부터 작은 덩어리(폭 12cm 안) = 기둥
    pts.sort(key=lambda q: math.hypot(*q))
    used = set()
    for k, (px, py) in enumerate(pts):
        if k in used:
            continue
        blob = [j for j, q in enumerate(pts) if math.hypot(q[0] - px, q[1] - py) < 0.12]
        used.update(blob)
        bx = sum(pts[j][0] for j in blob) / len(blob)
        by = sum(pts[j][1] for j in blob) / len(blob)
        # 기둥 둘레 0.25m 안에 다른 점이 많으면(벽, 큰 가구) 기둥 아님
        crowd = sum(1 for q in pts if 0.12 <= math.hypot(q[0] - bx, q[1] - by) < 0.25)
        if crowd > 3 or math.hypot(bx, by) > 1.6:
            continue
        for i in range(16):
            t = 2 * math.pi * i / 16
            qx, qy = bx + BASE_R * math.cos(t), by + BASE_R * math.sin(t)
            if math.hypot(qx, qy) > 0.15:
                out.append((math.degrees(math.atan2(qy, qx)), math.hypot(qx, qy)))
        if len(out) >= 64:
            break
    return out


def pole_rings(scan, max_d=1.3, r=BASE_R):
    """11/27 20:25: 카메라 없이도, 라이다에 '가늘고 혼자 있는 기둥'(의자 가운데 기둥, 옷걸이 봉)이 보이면
    둘레 r 을 막힌 곳으로 (받침/바퀴는 라이다보다 낮아 안 보임). 각도 순으로 이어진 점 덩어리 중
    폭 12cm 안 + 양옆 덩어리와 25cm 넘게 떨어진 것만 기둥으로 봄 (벽, 큰 가구는 제외)"""
    pts = sorted((a, d) for a, d in scan.pts if 0.12 < d < 2.5)
    if len(pts) < 3:
        return []
    xy = [(d * math.cos(math.radians(a)), d * math.sin(math.radians(a)), d) for a, d in pts]
    segs, cur = [], [xy[0]]
    for p in xy[1:]:
        q = cur[-1]
        if math.hypot(p[0] - q[0], p[1] - q[1]) < 0.06:
            cur.append(p)
        else:
            segs.append(cur)
            cur = [p]
    segs.append(cur)
    if len(segs) > 1 and math.hypot(segs[0][0][0] - segs[-1][-1][0], segs[0][0][1] - segs[-1][-1][1]) < 0.06:
        segs[0] = segs.pop() + segs[0]                      # 뒤쪽 -180/180 이음새
    out, done = [], []
    n = len(segs)
    for k, sg in enumerate(segs):
        if n < 3:
            break
        w = math.hypot(sg[0][0] - sg[-1][0], sg[0][1] - sg[-1][1])
        dmin = min(p[2] for p in sg)
        if w > 0.12 or dmin > max_d:
            continue
        bx = sum(p[0] for p in sg) / len(sg)
        by = sum(p[1] for p in sg) / len(sg)
        # 둘레 0.12~0.25m 에 다른 점이 3개 넘게 있으면 (벽, 큰 가구 일부) 기둥 아님
        if sum(1 for p in xy if 0.12 <= math.hypot(p[0] - bx, p[1] - by) < 0.25) > 3:
            continue
        if any(math.hypot(c[0] - bx, c[1] - by) < 0.15 for c in done):
            continue
        done.append((bx, by))
        for i in range(16):
            t = 2 * math.pi * i / 16
            qx, qy = bx + r * math.cos(t), by + r * math.sin(t)
            if math.hypot(qx, qy) > 0.15:
                out.append((math.degrees(math.atan2(qy, qx)), math.hypot(qx, qy)))
        if len(out) >= 96:
            break
    return out
