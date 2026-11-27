"""라이다로 가는 동안 위치/방향을 계속 재기 (스캔끼리 모양 맞추기 = ICP).

바퀴는 매번 조금씩 다르게 틀어진다 (줄자 위치, 바닥, 롤러 상태). 고정 보정값으로는 못 잡으니
스캔이 올 때마다 "출발 기준으로 지금 어디, 몇 도" 를 재서 바로 고친다.
"""
import math

import numpy as np


def to_xy(scan, rmax=5.0, cell=0.04):
    a = np.radians([p[0] for p in scan.pts if 0.15 < p[1] < rmax])
    d = np.array([p[1] for p in scan.pts if 0.15 < p[1] < rmax])
    p = np.stack([d * np.cos(a), d * np.sin(a)], 1) if len(d) else np.zeros((0, 2))
    if len(p):
        _, idx = np.unique(np.round(p / cell), axis=0, return_index=True)
        p = p[np.sort(idx)]
    return p


def icp_xy(ref, src, guess=(0.0, 0.0, 0.0), iters=25):
    """src(나중 스캔) 를 ref(기준 스캔) 에 맞춘다 -> 기준에서 본 로봇 이동 (x 앞, y 왼쪽, th rad), 맞춤 비율"""
    tx, ty, th = guess
    if len(ref) < 20 or len(src) < 20:
        return tx, ty, th, 0.0
    good = 0
    for it in range(iters):
        c, s = math.cos(th), math.sin(th)
        q = src @ np.array([[c, s], [-s, c]]) + (tx, ty)
        d2 = ((q[:, None, :] - ref[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        dist = np.sqrt(d2[np.arange(len(q)), j])
        thr = max(0.05, 0.25 * 0.85 ** it)
        k = dist < thr
        good = int(k.sum())
        if good < 15:
            return tx, ty, th, 0.0
        A, B = q[k], ref[j[k]]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        dth = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        R = np.array([[math.cos(dth), -math.sin(dth)], [math.sin(dth), math.cos(dth)]])
        t = cb - R @ ca
        nt = R @ np.array([tx, ty]) + t
        tx, ty, th = float(nt[0]), float(nt[1]), th + dth
        if abs(dth) < 1e-5 and float(np.hypot(*t)) < 1e-5:
            break
    return tx, ty, th, good / len(src)


def compose(a, b):
    """자세 a 다음에 (a 기준) b 만큼 움직인 자세"""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


class ScanOdom:
    """출발 자세 기준 지금 자세 (x 앞, y 왼쪽, th 왼쪽 rad). 25cm 마다 기준 스캔을 바꾼다."""

    def __init__(self, scan, rekey=0.25):
        self.key_pts, self.key_pose = to_xy(scan), (0.0, 0.0, 0.0)
        self.rel = (0.0, 0.0, 0.0)
        self.pose, self.fit, self.fails, self.rekey = (0.0, 0.0, 0.0), 1.0, 0, rekey

    def update(self, scan, predict=(0.0, 0.0, 0.0)):
        """predict: 지난번 이후 명령대로라면 움직였을 양 (로봇 기준). 맞춤이 실패하면 이걸 그대로 쓴다"""
        pts = to_xy(scan)
        # 11/27: 실패가 이어져도 찾기는 '그동안 명령대로 움직였을 곳' 근처에서 (보고하는 자세에는 안 더함)
        self.pending = compose(getattr(self, "pending", (0.0, 0.0, 0.0)), predict)
        guess = compose(self.rel, self.pending)
        x, y, th, fit = icp_xy(self.key_pts, pts, guess)
        # 11/27: 배터리가 약하면 실제 회전이 명령보다 훨씬 느림 -> '명령대로' 짐작만으로는 자꾸 실패.
        #  '안 움직였다' 에서도 한 번 더 맞춰 보고 더 잘 맞는 쪽을 씀
        guess0 = self.rel
        if fit < 0.6:
            x0, y0, th0, fit0 = icp_xy(self.key_pts, pts, guess0)
            if fit0 > fit:
                x, y, th, fit, guess = x0, y0, th0, fit0, guess0
        self.fit = fit
        # 11/27: 어수선한 곳(열린 옷장 등)에서 엉뚱하게 맞춰 한 번에 27도 도는 일 -> 명령으로 짐작한 것과 너무 다르면 버림
        jump_th = abs(math.atan2(math.sin(th - guess[2]), math.cos(th - guess[2])))
        jump_xy = math.hypot(x - guess[0], y - guess[1])
        slack = 1.0 + getattr(self, "fail_run", 0)            # 실패가 이어졌으면 그만큼 더 너그럽게
        if fit >= 0.3 and (jump_th > math.radians(8) * slack or jump_xy > 0.08 * slack):
            fit = 0.0
            self.fit = 0.0
        if fit >= 0.3:
            self.rel = (x, y, th)
            self.fail_run = 0
            self.pending = (0.0, 0.0, 0.0)
        else:
            # 11/27: 맞추기 실패 때 명령대로 움직였다고 치면, 실제론 걸려서 안 움직여도 '움직이는 중' 으로 보여
            #  막힘 감지가 안 됨 (옷걸이 앞에서 90초). -> 실패하면 움직이지 않았다고 봄 (다음 맞추기가 따라잡음)
            self.fails += 1
            self.fail_run = getattr(self, "fail_run", 0) + 1
        if getattr(self, "fail_run", 0) >= 4:
            # 11/27: 한 번 놓치면 기준 스캔이 너무 멀어져 다시는 못 맞춤 -> 자세가 얼어붙음 (기록: 12초 동안 그대로)
            #  -> 4번 연속 실패하면 그동안은 명령으로 짐작하고, 지금 스캔을 새 기준으로 삼아 다시 시작
            half = (self.pending[0] * 0.5, self.pending[1] * 0.5, self.pending[2] * 0.5)   # 명령의 절반만 믿음
            self.key_pose = compose(self.key_pose, compose(self.rel, half))
            self.key_pts, self.rel, self.pending, self.fail_run = pts, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0
            self.relocs = getattr(self, "relocs", 0) + 1
        self.pose = compose(self.key_pose, self.rel)
        if math.hypot(self.rel[0], self.rel[1]) > self.rekey or abs(self.rel[2]) > math.radians(15):
            if fit >= 0.3:
                self.key_pts, self.key_pose, self.rel = pts, self.pose, (0.0, 0.0, 0.0)
        return self.pose


class LineHold:
    """계속 한쪽으로 도는 버릇이 있으면 비례 제어만으로는 몇 도가 남는다 -> 남는 만큼 쌓아서(적분) 없앤다"""

    def __init__(self, v, k_i=1.5, i_max=0.6):
        self.v, self.k_i, self.i_max, self.i = v, k_i, i_max, 0.0

    def __call__(self, pose, dt):
        self.i = max(-self.i_max, min(self.i_max, self.i - self.k_i * pose[2] * dt))
        return hold_line(pose, self.v, w_extra=self.i)


def hold_line(pose, v, k_th=3.5, k_y=2.0, w_max=1.2, vy_max=0.10, w_extra=0.0):
    """출발 때 방향의 직선(y=0, th=0) 위를 가도록 고치는 명령 (vx, vy, w) — 로봇 기준"""
    x, y, th = pose
    w = max(-w_max, min(w_max, -k_th * th + w_extra))
    # 옆으로 샌 만큼 옆으로 밀어 되돌린다 (메카넘). 출발 좌표의 y 를 로봇 기준으로 바꿈
    vy_world = max(-vy_max, min(vy_max, -k_y * y))
    c, s = math.cos(th), math.sin(th)
    return v * c + vy_world * s, -v * s + vy_world * c, w
