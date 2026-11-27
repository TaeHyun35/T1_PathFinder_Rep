"""벽 따라 탐색 한 걸음 (11/27): roommap.py 와 mission.py 가 같이 쓴다.

follow() 위에 실제 방에서 찾은 문제들 대책을 얹는다:
  - 열린 곳에서 2초 넘게 돌면 똑바로 가서 벽 찾기
  - 앞이 안 보이면(검은 물체) 한 방향으로만 돌고, 3초 넘으면 뒤로 빠져나오기
  - 막힘(가라는데 3초 동안 안 움직임/안 돎), 맴돌기(20초 동안 반지름 12cm) -> 뒤로 빠졌다가 방향 틀기
  - 처음 벽에 붙은 자리로 한 바퀴 돌아오면 lap_done
  ex = Explorer(cfg); cmd, tag, event = ex.step(scan, pose, now)
"""
import math


class Explorer:
    def __init__(self, cfg, side=None, lap_min_travel=6.0, far_min=1.5):
        from mission import follow
        self._follow = follow
        self.cfg = cfg
        self.side_name = side or cfg["wall"]["side"]
        self.side = 1 if self.side_name == "right" else -1
        self.hist = []                       # (t, x, y, 누적 회전)
        self.escape_until, self.escapes = 0.0, 0
        self.turn_acc, self.travel = 0.0, 0.0
        self.last_pose = None
        self.open_since = None
        self.blind_since, self.blind_dir = None, 0
        self.wall0 = None
        self.lap_done = False
        self.lap_min_travel, self.far_min = lap_min_travel, far_min      # 11/27 밤: 테이프 네모(1.5m)는 작아서 줄여 씀
        self.marks = []                      # 11/27: 라이다에 안 보이는데 막혔던 자리 (바닥 좌표 x, y, 시각) -> 가짜 점으로 피함
        self.pose = (0.0, 0.0, 0.0)
        self.seq = []                        # 빠져나오기 순서 [(끝 시각, (vx, vy, w), 글자)]
        self.esc_spot, self.esc_rep = None, 0
        self.scan = None

    def add_mark(self, pose, now):
        """막힌 자리(몸 앞 폭 전체)를 바닥 좌표로 기억"""
        x, y, th = pose
        c, s_ = math.cos(th), math.sin(th)
        for fx in (0.12, 0.17, 0.22, 0.27):
            for fy in (-0.15, -0.10, -0.05, 0.0, 0.05, 0.10, 0.15):
                self.marks.append((x + c * fx - s_ * fy, y + s_ * fx + c * fy, now))

    def with_marks(self, scan, pose, now):
        """기억한 막힌 자리를 라이다 점처럼 더함. 11/27 20:25: 90초 -> 10분 (같은 옷걸이/의자에 다시 가서 걸림)"""
        self.marks = [m for m in self.marks if now - m[2] < 600.0]
        if not self.marks:
            return scan
        from obstacles import _Scan
        x, y, th = pose
        add = []
        for mx, my, _ in self.marks:
            dx, dy = mx - x, my - y
            d = math.hypot(dx, dy)
            if 0.05 < d < 1.5:
                add.append(((math.degrees(math.atan2(dy, dx) - th) + 180) % 360 - 180, d))
        return _Scan(scan, list(scan.pts) + add) if add else scan

    def _escape(self, now, why):
        self.escapes += 1
        # 11/27: 책상 다리 아래 가로대처럼 라이다(27cm)와 손(8cm)보다 낮은 것에 바퀴가 걸림
        #  -> 막힌 자리(몸 앞 폭 전체)를 기억해서 다음엔 그쪽을 장애물로 보고 피함
        if why == "막힘 감지":
            self.add_mark(self.pose, now)
        # 11/27 18:10: 같은 자리(25cm 안)에서 또 막히면 방법을 바꿈 (의자 바퀴에 끼어 뒤로도 못 감)
        x, y, _ = self.pose
        if self.esc_spot and math.hypot(x - self.esc_spot[0], y - self.esc_spot[1]) < 0.25 and now - self.esc_spot[2] < 40:
            self.esc_rep += 1
        else:
            self.esc_rep = 0
        self.esc_spot = (x, y, now)
        sc = self.scan
        lft = sc.sector_min(90, 35) if sc is not None else float("nan")
        rgt = sc.sector_min(-90, 35) if sc is not None else float("nan")
        lft = 9.0 if math.isnan(lft) else lft
        rgt = 9.0 if math.isnan(rgt) else rgt
        side_dir = 1 if lft >= rgt else -1                       # 넓은 쪽 (+ = 왼쪽)
        turn = (self.side * math.radians(55),)
        t = now
        seq = []
        def add(d, cmd, tag):
            nonlocal t
            t += d
            seq.append((t, cmd, tag))
        k = self.esc_rep
        if k == 0:
            add(1.0, (-0.20, 0.0, 0.0), "빠져나오기-후진")
            add(1.2 + (self.escapes % 2) * 0.3, (0.0, 0.0, turn[0]), "빠져나오기-회전")
        elif self.cfg["wall"].get("no_strafe"):           # 옆으로 미끄러지기 없이: 더 뒤로 + 반대로 크게 돌기
            add(1.2, (-0.20, 0.0, 0.0), "빠져나오기-후진")
            add(1.8, (0.0, 0.0, -turn[0] if k % 2 else turn[0]), "빠져나오기-회전")
        elif k % 2 == 1:
            sd = side_dir if k == 1 else -side_dir
            add(1.2, (0.0, 0.18 * sd, 0.0), "빠져나오기-옆으로")
            add(0.8, (-0.25, 0.0, 0.0), "빠져나오기-후진")
            add(1.2, (0.0, 0.0, turn[0]), "빠져나오기-회전")
        else:
            for w in (0.9, -0.9, 0.9):
                add(0.45, (0.0, 0.0, w), "빠져나오기-비틀기")
                add(0.5, (-0.25, 0.0, 0.0), "빠져나오기-비틀며 후진")
            add(1.2, (0.0, -0.18 * side_dir, 0.0), "빠져나오기-옆으로")
            add(1.2, (0.0, 0.0, turn[0]), "빠져나오기-회전")
        self.seq = seq
        self.escape_until = t
        self.hist = self.hist[-1:]
        self.blind_since, self.blind_dir = None, 0
        how = ["뒤로 빠졌다가 방향 틀기", "옆으로 미끄러져 빠지기", "비틀며 빼기"][0 if self.esc_rep == 0 else (1 if self.esc_rep % 2 else 2)]
        if self.esc_rep > 0 and self.cfg["wall"].get("no_strafe"):
            how = "더 뒤로 빠졌다가 크게 돌기"
        return (0.0, 0.0, 0.0), why, f"{why} -> {how} ({self.escapes}번째)"

    def step(self, scan, pose, now):
        """-> ((vx, vy, w), 상태글자, 알릴 일 또는 None)"""
        c = self.cfg["wall"]
        event = None
        if self.last_pose is not None:
            self.travel += math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
            self.turn_acc += abs(math.atan2(math.sin(pose[2] - self.last_pose[2]), math.cos(pose[2] - self.last_pose[2])))
        self.last_pose = pose
        self.pose = pose
        self.hist.append((now, pose[0], pose[1], self.turn_acc))
        self.hist = [h for h in self.hist if now - h[0] <= 20.0]

        w0 = self.wall0
        if w0 is not None:
            self.far = max(getattr(self, "far", 0.0), math.hypot(pose[0] - w0[0], pose[1] - w0[1]))
        # 11/27: 장애물 하나를 작게 한 바퀴(5m) 돌고 '지도 완성' 이라 함 -> 처음 벽 자리에서 1.5m 넘게 멀어졌다 와야 한 바퀴
        if w0 is not None and not self.lap_done and self.travel - w0[2] > self.lap_min_travel and getattr(self, "far", 0) > self.far_min \
                and self.turn_acc - w0[3] > math.radians(300) and math.hypot(pose[0] - w0[0], pose[1] - w0[1]) < 0.40:
            self.lap_done = True
            event = f"한 바퀴 돌아 처음 벽 자리로 옴 (간 거리 {self.travel:.1f}m)"

        self.scan = scan
        if now < self.escape_until:
            while self.seq and now >= self.seq[0][0]:
                self.seq.pop(0)
            if self.seq:
                _, cmd, tg = self.seq[0]
                if cmd[0] < 0:
                    back = scan.sector_min(180, 35)
                    if not (math.isnan(back) or back > 0.22):  # 뒤에 물체 -> 뒤로 대신 넓은 쪽 옆으로
                        lft, rgt = scan.sector_min(90, 35), scan.sector_min(-90, 35)
                        sd = 1 if (9 if math.isnan(lft) else lft) >= (9 if math.isnan(rgt) else rgt) else -1
                        if self.cfg["wall"].get("no_strafe"):
                            return (0.0, 0.0, sd * math.radians(40)), "빠져나오기-회전(뒤 막힘)", event
                        return (0.0, 0.15 * sd, 0.0), "빠져나오기-옆으로(뒤 막힘)", event
                if cmd[1] != 0:
                    sd_ = 1 if cmd[1] > 0 else -1
                    sm = scan.sector_min(90 * sd_, 30)
                    if not math.isnan(sm) and sm < 0.16:       # 그 옆이 붙어 있으면 반대로
                        cmd = (cmd[0], -cmd[1], cmd[2])
                return cmd, tg, event

        scan = self.with_marks(scan, pose, now)
        vx, vy, wz, tag = self._follow(scan, self.cfg, self.side_name)

        if tag.startswith("앞 안 보임"):
            self.blind_since = self.blind_since or now
            if tag.endswith("돌아서 찾기"):
                self.blind_dir = self.blind_dir or (1 if wz > 0 else -1)
                wz = self.blind_dir * abs(wz)
            if now - self.blind_since > 3.0:
                cmd, tag, ev = self._escape(now, "3초 넘게 앞이 안 보임")
                return cmd, tag, ev
        else:
            self.blind_since, self.blind_dir = None, 0

        if self.wall0 is None and ((tag == "벽따라" and (self.far_min >= 1.0 or self.travel > 0.3)) or (self.far_min < 1.0 and tag == "옆길-돌기" and self.travel > 0.6)):   # 작은 테이프 네모는 '옆길-돌기' 로 돎
            self.wall0 = (pose[0], pose[1], self.travel, self.turn_acc)
            event = event or f"벽에 붙음 -> 여기서부터 한 바퀴 셈 ({pose[0]:+.2f}, {pose[1]:+.2f})"

        if tag in ("옆길-돌기", "옆 비었음-벽 찾기"):
            self.open_since = self.open_since or now
            fr = scan.sector_min(0, 25)
            from obstacles import hand_corridor
            clear = (math.isnan(fr) or fr > c["front_stop"] + 0.2) and                 hand_corridor(scan, c.get("finger_m", 0.315), c.get("hand_half_w", 0.08), c.get("finger_margin", 0.08) + 0.1) is None
            if now - self.open_since > 2.0 and clear:        # 앞이 넉넉히 비었을 때만 똑바로
                vx, vy, wz, tag = c["v"], 0.0, 0.0, "직진-벽 찾기"
        else:
            self.open_since = None

        h3 = [h for h in self.hist if now - h[0] <= 3.0]
        moved3 = math.hypot(pose[0] - h3[0][1], pose[1] - h3[0][2]) if h3 else 1.0
        rot3 = self.turn_acc - h3[0][3] if h3 else 1.0
        stuck = (abs(vx) + abs(vy) > 0.03 or abs(wz) > 0.3) and now - self.hist[0][0] >= 3.0 and moved3 < 0.02 and rot3 < math.radians(5)
        # 같은 '후진/회전 공간 없음' 이 6초 넘게 이어지면 그것도 막힘 (좁은 곳에서 갔다 왔다만 함)
        self.tag_hist = [(t, g) for t, g in getattr(self, "tag_hist", []) if now - t <= 6.0] + [(now, tag)]
        if len(self.tag_hist) > 20 and all("후진" in g or "회전 공간" in g for _, g in self.tag_hist) and now - self.tag_hist[0][0] > 5.5:
            stuck = True
            self.tag_hist = []
        xs, ys = [h[1] for h in self.hist], [h[2] for h in self.hist]
        spread = max(math.hypot(x - sum(xs) / len(xs), y - sum(ys) / len(ys)) for x, y in zip(xs, ys))
        dither = now - self.hist[0][0] >= 19.5 and spread < 0.12 and self.turn_acc - self.hist[0][3] < math.radians(120)
        if stuck or dither:
            return self._escape(now, "막힘 감지" if stuck else "맴돌기 감지")
        return (vx, vy, wz), tag, event
