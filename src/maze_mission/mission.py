"""길잡이 미로 탈출 미션 (라즈베리파이에서 실행).

흐름: 라이다로 벽 따라 돌아다니기 -> 카메라로 동물 그림 찾기(고양이, 강아지...)
   -> 다 찾고 키패드 마커가 보이면 다가가기 -> 동물에 맞는 숫자를 손가락으로 누르기
   -> 문이 열리면 출구로 나가기

단계별 시험 (위에서부터 하나씩):
  python3 mission.py lidar                 라이다 방향 확인 (앞/왼/오른/뒤 거리 계속 표시)
  python3 mission.py calib-rot             제자리 회전 속도 재기 (돈 각도를 입력하면 저장)
  python3 mission.py wall --secs 60        벽 따라가기만 (카메라 X, 손 X)
  python3 mission.py see                   카메라 + 동물/마커 찾기만 (바퀴 X)
  python3 mission.py keys --sweep 3        손가락 3(중지)을 조금씩 굽혀 키 눌리는 각도 찾기
  python3 mission.py keys --code 312       숫자 누르기만
  python3 mission.py run                   전체 미션
  python3 mission.py run --virtual-press   손 대신 문 스테이션에 숫자를 바로 보냄 (손 없이 전체 시험)

노트북에서 시뮬레이션 (로봇 없이 전체 흐름 확인):
  python mission.py run --sim --seed 3     -> logs/ 에 경로 그림(png)과 기록(jsonl)
  python mission.py simbatch --n 20        여러 미로로 성공률 보기

옵션: --no-motor (바퀴 안 돌림) --no-hand (손 안 씀) --config 파일
급할 때: Ctrl+C (바퀴 멈추고 손 폄)
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.request

from common import HERE, RAD, Log, RealClock, clamp, load_config, save_config_patch, wrap_deg


# =================================================================== 벽 따라가기
def blocked(d, th):
    """거리 d 가 th 보다 가깝거나, 라이다가 그 방향을 못 보면(nan) True"""
    return math.isnan(d) or d < th


def follow(scan, cfg, side):
    """라이다 한 바퀴 -> (vx, vy, w, 상태글자). side: 'right' / 'left'"""
    c = cfg["wall"]
    s = 1 if side == "right" else -1
    # skew: 몸(카메라)을 벽 쪽으로 돌린 각도. 메카넘이라 몸은 비스듬히, 이동은 벽과 나란히.
    # 계산은 "가는 방향" 기준으로 하고, 마지막에 몸 기준 속도로 바꾼다.
    k = s * c.get("skew_deg", 0.0)
    front = scan.sector_min(0 + k, 22)
    r1 = scan.ray(-90 * s + k)
    r2 = scan.ray(-60 * s + k)
    body_front = scan.sector_min(0, 25)                  # 손이 향한 쪽 (몸 정면)
    if math.isnan(front) or math.isnan(body_front):
        return 0.0, 0.0, 0.0, "앞 안 보임-정지"          # 손이 라이다를 가리는 중일 수 있음 -> 움직이지 않음
    if body_front < c.get("hand_clear", 0.12) + 0.18:    # 손끝(라이다에서 약 0.18m 앞) 바로 앞에 뭔가 있음
        return -0.05, 0.0, 0.0, "손 앞 막힘-후진"
    if front < c["front_stop"]:
        return 0.0, 0.0, s * c["w_turn_deg"] * RAD, "막힘-회전"
    if math.isnan(r1):
        return 0.0, 0.0, 0.0, "옆 안 보임-정지"
    if r1 > c["open"] and r2 > c["open"] * 1.1:
        vx, vy, w, tag = c["v_arc"], 0.0, -s * c["w_arc_deg"] * RAD, "옆길-돌기"
    else:
        if math.isnan(r2) or math.isinf(r2):
            r2 = r1 / math.cos(30 * RAD)
        phi = math.atan2(s * (r1 - math.cos(30 * RAD) * r2), 0.5 * r2)     # 벽 기울기 (다가오면 +)
        dperp = r1 * math.cos(phi)
        e = dperp - c["d_ref"]
        vy = clamp(-s * c["ky"] * e, -0.08, 0.08)
        w = clamp(c["kw"] * phi, -0.6, 0.6)
        slow = clamp((front - c["front_stop"]) / 0.30, 0.35, 1.0)
        vx, tag = c["v"] * slow, "벽따라"
    kr = k * RAD
    return vx * math.cos(kr) - vy * math.sin(kr), vx * math.sin(kr) + vy * math.cos(kr), w, tag


# =================================================================== 문 스테이션 (노트북 door_station.py)
class DoorClient:
    def __init__(self, url):
        self.url = url.rstrip("/")

    def status(self):
        try:
            with urllib.request.urlopen(self.url + "/status", timeout=1.5) as r:
                return json.loads(r.read()).get("state", "unknown")
        except Exception:
            return "unknown"

    def send_code(self, code):
        try:
            req = urllib.request.Request(self.url + "/code", data=code.encode(), method="POST")
            urllib.request.urlopen(req, timeout=2).read()
            return True
        except Exception as e:
            print("  문 스테이션 연결 실패:", e)
            return False


# =================================================================== 미션
class Mission:
    def __init__(self, cfg, lidar, car, vision, hand, door, clock, log, world=None):
        self.cfg, self.lidar, self.car, self.vision, self.hand = cfg, lidar, car, vision, hand
        self.door, self.clock, self.log, self.world = door, clock, log, world
        self.state, self.t0 = "EXPLORE", clock.now()
        self.found, self.sight = [], {}          # 확정된 동물(순서대로), 동물별 본 시각들
        self.marks = {}                          # 마커 id -> (시각, cx, size)
        self.first_seen = {}                     # 마커 id -> 처음 본 시각
        self.side = cfg["wall"]["side"]
        self.next_look = clock.now() + cfg["look"]["every_s"]
        self.last_detect = 0.0
        self.sub, self.sub_t = None, 0.0         # 상태 안의 작은 단계
        self.press_tries, self.aligned_since = 0, None
        self.code = ""

    def t(self):
        return self.clock.now() - self.t0

    def say(self, msg):
        self.log.say(self.t(), msg)

    def go(self, state, why=""):
        self.say(f"{self.state} -> {state}  {why}")
        self.state, self.sub, self.sub_t = state, None, self.clock.now()

    # ---------- 보기 ----------
    def perceive(self, force=False):
        now = self.clock.now()
        if not force and now - self.last_detect < self.cfg["detect"]["every_s"]:
            return
        self.last_detect = now
        res = self.vision.perceive()
        if res.get("err"):
            self.log.write(t=round(self.t(), 2), err=res["err"])
        for m in res.get("markers", []):
            self.marks[m["id"]] = (now, m["cx"], m["size"])
            if m["id"] not in self.first_seen:
                self.first_seen[m["id"]] = now
                name = "키패드" if m["id"] == self.cfg["mission"]["keypad_marker"] else "문" if m["id"] == self.cfg["mission"]["door_marker"] else f"마커{m['id']}"
                self.say(f"{name} 마커 발견")
        for a in res.get("animals", []):
            lab = a["label"]
            if lab in self.found:
                continue
            ts = [x for x in self.sight.get(lab, []) if now - x < 3.0] + [now]
            self.sight[lab] = ts
            if len(ts) >= self.cfg["detect"]["confirm"]:
                self.found.append(lab)
                ko = self.cfg["animals"][lab]["ko"]
                self.say(f"동물 확정: {ko} ({len(self.found)}/{self.cfg['mission']['expect']})  찾은 순서 {[self.cfg['animals'][x]['ko'] for x in self.found]}")

    def marker(self, mid, within=1.5):
        m = self.marks.get(mid)
        return m if m and self.clock.now() - m[0] <= within else None

    def make_code(self):
        cfg, an = self.cfg["mission"], self.cfg["animals"]
        if cfg["code_rule"] == "sorted":
            return "".join(sorted(an[x]["digit"] for x in self.found))
        if cfg["code_rule"] == "fixed" and cfg["fixed_order"]:
            return "".join(an[x]["digit"] for x in cfg["fixed_order"])
        return "".join(an[x]["digit"] for x in self.found)

    # ---------- 한 번 돌기 (0.1초마다) ----------
    def tick(self, scan):
        st, now, cfg = self.state, self.clock.now(), self.cfg
        mc = cfg["mission"]
        kp = mc["keypad_marker"]
        done_animals = len(self.found) >= mc["expect"]

        if st in ("EXPLORE", "LOOK"):
            self.perceive()
            if done_animals and self.marker(kp):
                self.car.stop()
                return self.go("APPROACH", "동물 다 찾음 + 키패드 보임")

        if st == "EXPLORE":
            vx, vy, w, tag = follow(scan, cfg, self.side)
            side_d = scan.ray(-90 if self.side == "right" else 90)
            side_d = 9.0 if math.isnan(side_d) else side_d
            want_look = (not done_animals) or self.marker(kp, 3.0) is None
            if cfg["look"]["every_s"] > 0 and now >= self.next_look and tag == "벽따라" and side_d < 0.6 and want_look:
                self.car.stop()
                return self.go("LOOK", "옆 벽 보기")
            self.car.drive(vx, vy, w)

        elif st == "LOOK":                       # 옆 벽 쪽으로 돌아서 보고, 다시 돌아오기
            lc = cfg["look"]
            s = -1 if self.side == "right" else 1
            turn_t = lc["angle_deg"] / lc["w_deg"]
            el = now - self.sub_t
            if el < turn_t:
                self.car.drive(0, 0, s * lc["w_deg"] * RAD)
            elif el < turn_t + lc["hold_s"]:
                self.car.stop()
                self.perceive(force=True)
            elif el < 2 * turn_t + lc["hold_s"]:
                self.car.drive(0, 0, -s * lc["w_deg"] * RAD)
            else:
                self.car.stop()
                self.next_look = now + lc["every_s"]
                self.go("EXPLORE")

        elif st == "APPROACH":
            self.perceive(force=True)
            m = self.marker(kp, within=2.0)
            if m is None or now - self.sub_t > 30:
                self.car.stop()
                return self.go("BACKOFF", "키패드 놓침/시간 초과 -> 뒤로 물러나 다시")
            front = scan.ray(0, 6)
            if math.isnan(front):
                self.car.stop()
                return
            e = (m[1] - 0.5) - mc["cam_offset"]
            pd = mc["press_dist"]
            if front > pd + 0.15:                # 멀 때: 마커를 가운데로 돌리며 다가가기
                w = clamp(-2.0 * e, -0.5, 0.5)
                vx = 0.10 if abs(e) < 0.12 else 0.0
                # 옆 벽 모서리에 몸/손이 걸리지 않게: 비스듬히 앞쪽이 가까우면 반대로 비켜서기
                cl, cr = scan.sector_min(55, 35), scan.sector_min(-55, 35)
                vy = 0.0
                if blocked(cl, 0.24):
                    vy -= 0.06
                if blocked(cr, 0.24):
                    vy += 0.06
                if blocked(cl, 0.20) or blocked(cr, 0.20):
                    vx = 0.0
                self.car.drive(vx, vy, w)
                self.aligned_since = None
            else:                                # 가까울 때: 벽과 직각 + 옆으로 맞추기 + 거리 맞추기
                dl, dr = scan.ray(25, 3), scan.ray(-25, 3)
                sq = (dl - dr) if math.isfinite(dl) and math.isfinite(dr) else 0.0
                lat = e * 2 * front * math.tan(cfg["camera"]["fov_deg"] / 2 * RAD)
                w = clamp(-3.0 * sq, -0.3, 0.3)
                vy = clamp(-1.5 * lat, -0.06, 0.06)
                vx = clamp(0.9 * (front - pd), -0.05, 0.06)
                self.car.drive(vx, vy, w)
                ok = abs(lat) < 0.008 and abs(front - pd) < 0.012 and abs(sq) < 0.006
                if ok:
                    self.aligned_since = self.aligned_since or now
                    if now - self.aligned_since > 0.6:
                        self.car.stop()
                        self.code = self.make_code()
                        return self.go("PRESS", f"정렬 끝 (옆 {lat * 100:.1f}cm, 거리 {front:.3f}m) -> 비밀번호 {self.code}")
                else:
                    self.aligned_since = None

        elif st == "PRESS":
            self.car.stop()
            if mc["press_mode"] == "virtual":
                self.door.send_code(self.code)
                self.say(f"(가상 누르기) 문 스테이션에 {self.code} 보냄")
            else:
                for d in self.code:
                    f = cfg["hand"]["key_finger"][d]
                    self.say(f"숫자 {d} 누르기 (손가락 {f})")
                    self.hand.press(f, sleep=self.clock.sleep)
            self.press_tries += 1
            self.clock.sleep(1.0)
            ds = self.door.status()
            self.say(f"문 상태: {ds}")
            if ds == "wrong" and self.press_tries < 3:
                return self.go("BACKOFF", "틀렸다고 함 -> 다시 정렬해서 누르기")
            self.pressed = True
            return self.go("BACKOFF", "누르기 끝 -> 뒤로 물러나 문 찾기")

        elif st == "BACKOFF":
            if now - self.sub_t < 1.5:
                self.car.drive(-0.08, 0, 0)
            else:
                self.car.stop()
                self.go("FIND_DOOR" if getattr(self, "pressed", False) else "EXPLORE")

        elif st == "FIND_DOOR":                  # 제자리에서 돌며 문 마커 찾기
            self.perceive(force=True)
            if self.marker(mc["door_marker"], 0.5):
                self.car.stop()
                return self.go("GO_DOOR", "문 마커 보임")
            if now - self.sub_t > 420 / 30:
                self.car.stop()
                return self.go("TO_EXIT", "문 마커를 못 찾음 -> 벽 따라 출구 찾기")
            s = 1 if self.side == "right" else -1
            self.car.drive(0, 0, s * 30 * RAD)

        elif st == "GO_DOOR":                    # 문 마커 쪽으로 가서 통과
            self.perceive(force=True)
            if self.is_outside(scan, now):
                return self.finish()
            m = self.marker(mc["door_marker"], 2.0)
            front = scan.sector_min(0, 15)
            dl, dr = scan.ray(70, 15), scan.ray(-70, 15)
            vy = clamp(0.6 * (dl - dr), -0.05, 0.05) if (dl < 0.5 and dr < 0.5) else 0.0
            if blocked(front, 0.30):                     # 문이 아직 닫혀 있거나 벽 앞
                self.car.stop()
                if self.sub is None:
                    self.sub = now
                    self.say(f"앞이 막힘, 문 상태 {self.door.status()} -> 기다림")
                if now - self.sub > 15:
                    return self.go("TO_EXIT", "15초 기다려도 막힘 -> 벽 따라 출구 찾기")
                return
            self.sub = None
            if m:
                e = m[1] - 0.5
                self.car.drive(0.12 if abs(e) < 0.12 else 0.02, vy, clamp(-2.0 * e, -0.5, 0.5))
                self.door_last = now
            elif now - getattr(self, "door_last", now) < 4.0:
                self.car.drive(0.12, vy, 0.0)    # 가까워서 마커가 화면 밖 -> 곧장 통과
            else:
                return self.go("TO_EXIT", "문 마커 놓침 -> 벽 따라 출구 찾기")

        elif st == "TO_EXIT":
            if now - self.sub_t > 90 and not getattr(self, "flipped", False):
                self.side, self.flipped = ("left" if self.side == "right" else "right"), True
                self.say(f"90초 동안 못 나감 -> 반대 손({self.side})으로 바꿔 따라가기")
            vx, vy, w, tag = follow(scan, cfg, self.side)
            self.car.drive(vx, vy, w)
            if self.marker(mc["door_marker"], 0.3) and now - self.sub_t > 3:
                return self.go("GO_DOOR", "문 마커 다시 보임")
            if self.is_outside(scan, now):
                return self.finish()

    def is_outside(self, scan, now):
        if self.world is not None:
            return self.world.outside()
        open_front = scan.sector_min(0, 50) > self.cfg["mission"]["exit_open_m"]
        if not open_front:
            self.open_since = None
            return False
        self.open_since = getattr(self, "open_since", None) or now
        return now - self.open_since > 1.5

    def finish(self):
        self.car.drive(0.15, 0, 0)
        self.clock.sleep(1.5)
        self.car.stop()
        self.go("DONE", "미로 밖으로 나옴")

    def run(self):
        dt = 0.1
        self.say(f"미션 시작: 동물 {self.cfg['mission']['expect']}개 찾기, 규칙 {self.cfg['mission']['code_rule']}, 벽 {self.side}손")
        try:
            while self.state not in ("DONE", "FAIL"):
                if self.t() > self.cfg["mission"]["timeout_s"]:
                    self.go("FAIL", "시간 초과")
                    break
                scan = self.lidar.latest()
                if scan is None:
                    self.car.stop()
                    self.clock.sleep(dt)
                    continue
                self.tick(scan)
                if self.world is not None:
                    self.world.tag = self.state
                rec = {"t": round(self.t(), 2), "st": self.state, "cmd": [round(v, 3) for v in self.car.cmd], "found": self.found}
                if self.world is not None:
                    rec["pose"] = [round(self.world.x, 3), round(self.world.y, 3), round(math.degrees(self.world.th), 1)]
                self.log.write(**rec)
                self.clock.sleep(dt)
        finally:
            self.car.stop()
            self.hand.open()
        return self.state


# =================================================================== 실행 모드
def build_real(cfg, a, need_vision=True, need_hand=True):
    from drive import NoCar, RaspbotCar
    from hand import open_hand, NoHand
    from lidar import RPLidar
    lidar = RPLidar(cfg["lidar"])
    lidar.wait_ready()
    car = NoCar() if a.no_motor else RaspbotCar(cfg["drive"])
    vision = None
    if need_vision:
        from vision import Vision
        vision = Vision(cfg["camera"], cfg["detect"], list(cfg["animals"].keys()))
    hand = NoHand() if (a.no_hand or not need_hand or cfg["mission"]["press_mode"] == "virtual") else open_hand(cfg["hand"])
    return lidar, car, vision, hand


def build_sim(cfg, seed):
    import sim
    world = sim.World(cfg, seed=seed)
    an = cfg["animals"]
    placed = [c["label"] for c in world.cards]
    # 정답 비밀번호 (게임 진행자가 정함): 규칙에 맞게. found_order 는 순서를 미리 알 수 없으니 sorted 로 확인
    if cfg["mission"]["code_rule"] == "fixed" and cfg["mission"]["fixed_order"]:
        code = "".join(an[x]["digit"] for x in cfg["mission"]["fixed_order"])
    else:
        code = "".join(sorted(an[x]["digit"] for x in placed))
    door = sim.SimDoor(world, code)
    return world, sim.SimLidar(world), sim.SimCar(world), sim.SimVision(world, cfg["camera"]), sim.SimHand(world, door, cfg), door, sim.SimClock(world)


def cmd_run(cfg, a):
    if a.virtual_press:
        cfg["mission"]["press_mode"] = "virtual"
    if a.sim:
        if cfg["mission"]["code_rule"] == "found_order":
            cfg["mission"]["code_rule"] = "sorted"       # 시뮬 정답과 맞추기 위해
        world, lidar, car, vision, hand, door, clock = build_sim(cfg, a.seed)
        log = Log(f"sim_seed{a.seed}")
        m = Mission(cfg, lidar, car, vision, hand, door, clock, log, world)
        res = m.run()
        import sim
        png = log.path.replace(".jsonl", ".png")
        sim.draw(world, png, f"seed {a.seed}: {res}, {m.t():.0f}초, 찾은 동물 {[cfg['animals'][x]['ko'] for x in m.found]}, 부딪힘 {world.bumps}")
        print(f"결과 {res}  시간 {m.t():.0f}초  부딪힘 {world.bumps} {world.bump_tag}  그림 {png}")
        return res, m.t(), world.bumps
    lidar, car, vision, hand = build_real(cfg, a)
    door = DoorClient(cfg["door"]["station_url"])
    log = Log("run")
    print("3초 뒤 출발합니다. 멈추려면 Ctrl+C")
    time.sleep(3)
    try:
        res = Mission(cfg, lidar, car, vision, hand, door, RealClock(), log).run()
    except KeyboardInterrupt:
        res = "중지"
    finally:
        car.stop()
        hand.close()
        lidar.close()
    print("결과:", res, " 기록:", log.path)


def cmd_simbatch(cfg, a):
    import copy
    rows = []
    for seed in range(a.start, a.start + a.n):
        c = copy.deepcopy(cfg)
        a.seed = seed
        rows.append((seed,) + tuple(cmd_run(c, a)))
    ok = [r for r in rows if r[1] == "DONE"]
    print(f"\n성공 {len(ok)}/{len(rows)}  평균 시간 {sum(r[2] for r in ok) / max(1, len(ok)):.0f}초  평균 부딪힘 {sum(r[3] for r in rows) / len(rows):.1f}")
    for r in rows:
        if r[1] != "DONE":
            print("  실패:", r)


def cmd_lidar(cfg, a):
    from lidar import RPLidar
    lid = RPLidar(cfg["lidar"])
    lid.wait_ready()
    print("앞/왼/오른/뒤 가장 가까운 거리 (m). 상자를 앞에 두고 '앞' 값이 바뀌는지 보세요. Ctrl+C 끝")
    try:
        while True:
            s = lid.latest()
            f = lambda c: s.sector_min(c, 15)
            print(f"\r앞 {f(0):5.2f}  왼 {f(90):5.2f}  오른 {f(-90):5.2f}  뒤 {f(180):5.2f}   회전 {lid.hz:4.1f}/초  점 {sum(1 for _, d in s.pts if d > 0)}  ", end="", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print()


def cmd_calib_rot(cfg, a):
    from drive import RaspbotCar
    car = RaspbotCar(cfg["drive"])
    sec = 3.0
    input(f"세기 {cfg['drive']['power_ref']} 로 {sec}초 왼쪽으로 제자리 회전합니다. 바닥에 테이프로 방향을 표시하고 Enter ")
    p = cfg["drive"]["power_ref"]
    try:
        for n, s in (("FL", -p), ("RL", -p), ("FR", p), ("RR", p)):
            car._wheel(n, s)
        time.sleep(sec)
    finally:
        car.stop()
    deg = float(input("돈 각도(도)를 입력 (한 바퀴 넘으면 360 더해서): "))
    v = round(deg / sec, 1)
    save_config_patch({"drive": {"speed_rot_deg": v}})
    print(f"speed_rot_deg = {v} 저장")


def cmd_wall(cfg, a):
    if a.sim:
        world, lidar, car, vision, hand, door, clock = build_sim(cfg, a.seed)
    else:
        lidar, car, _, _ = build_real(cfg, a, need_vision=False, need_hand=False)
        clock = RealClock()
    print(f"{a.secs}초 동안 {cfg['wall']['side']}손 벽 따라가기. Ctrl+C 멈춤")
    t0 = clock.now()
    last = ""
    try:
        while clock.now() - t0 < a.secs:
            s = lidar.latest()
            vx, vy, w, tag = follow(s, cfg, cfg["wall"]["side"])
            car.drive(vx, vy, w)
            if tag != last:
                print(f"[{clock.now() - t0:5.1f}s] {tag}  앞 {s.sector_min(0, 22):.2f}  옆 {s.ray(-90):.2f}")
                last = tag
            clock.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        car.stop()
    if a.sim:
        import sim
        os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
        p = os.path.join(HERE, "logs", f"wall_sim_seed{a.seed}.png")
        sim.draw(world, p, f"벽 따라가기 {a.secs}초, 부딪힘 {world.bumps}")
        print("그림:", p)


def cmd_see(cfg, a):
    from vision import Vision
    v = Vision(cfg["camera"], cfg["detect"], list(cfg["animals"].keys()))
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    print("카메라 앞에 동물 그림과 마커를 대 보세요. 2초마다 logs/see_*.jpg 저장. Ctrl+C 끝")
    last_save = 0
    try:
        while True:
            t = time.time()
            r = v.perceive()
            an = ", ".join(f"{cfg['animals'][x['label']]['ko']}({x['conf']:.2f}, 위치 {x['cx']:.2f})" for x in r["animals"]) or "-"
            mk = ", ".join(f"#{m['id']}(위치 {m['cx']:.2f}, 크기 {m['size']:.2f})" for m in r["markers"]) or "-"
            print(f"동물: {an}   마커: {mk}   {r.get('err', '')}  ({(time.time() - t) * 1000:.0f}ms)")
            if time.time() - last_save > 2:
                v.save(os.path.join(HERE, "logs", time.strftime("see_%H%M%S.jpg")), r)
                last_save = time.time()
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass


def cmd_keys(cfg, a):
    from hand import open_hand
    h = open_hand(cfg["hand"])
    try:
        if a.sweep:
            f = a.sweep
            ang = cfg["hand"]["open"][f - 1]
            print(f"손가락 {f} 을 5도씩 굽힙니다. 키가 '딸깍' 눌리면 y, 아니면 Enter")
            while ang > 20:
                ang -= 5
                pose = list(cfg["hand"]["open"])
                pose[f - 1] = ang
                h.send(pose)
                if input(f"  {ang}도 눌렸나요? ").strip().lower() == "y":
                    target = ang - 5
                    save_config_patch({"hand": {"press_angle": {str(f): target}}})
                    print(f"  손가락 {f}: {ang}도에서 눌림 -> 누를 때 {target}도로 저장")
                    break
            h.open()
        if a.code:
            for d in a.code:
                f = cfg["hand"]["key_finger"].get(d)
                if f is None:
                    print(f"숫자 {d} 는 key_finger 에 없음")
                    continue
                print(f"숫자 {d} -> 손가락 {f}")
                h.press(f)
    finally:
        h.close()


def main():
    ap = argparse.ArgumentParser(description="길잡이 미로 탈출 미션")
    ap.add_argument("mode", choices=["run", "simbatch", "lidar", "calib-rot", "wall", "see", "keys"])
    ap.add_argument("--config")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--start", type=int, default=1)
    ap.add_argument("--secs", type=float, default=60)
    ap.add_argument("--no-motor", action="store_true")
    ap.add_argument("--no-hand", action="store_true")
    ap.add_argument("--virtual-press", action="store_true")
    ap.add_argument("--sweep", type=int)
    ap.add_argument("--code")
    a = ap.parse_args()
    cfg = load_config(a.config)
    if a.mode == "simbatch":
        a.sim = True
    {"run": cmd_run, "simbatch": cmd_simbatch, "lidar": cmd_lidar, "calib-rot": cmd_calib_rot,
     "wall": cmd_wall, "see": cmd_see, "keys": cmd_keys}[a.mode](cfg, a)


if __name__ == "__main__":
    main()
