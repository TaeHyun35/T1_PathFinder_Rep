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
    k = 0.0 if c.get("no_strafe") else s * c.get("skew_deg", 0.0)
    front = scan.sector_min(0 + k, 22)
    r1 = scan.ray(-90 * s + k)
    r2 = scan.ray(-60 * s + k)
    body_front = scan.sector_min(0, c.get("hand_half_deg", 25))   # 손이 향한 쪽 (몸 정면). 손 폭 4.5cm 라 실제 로봇은 15도면 충분
    finger = c.get("finger_m", 0.22)                     # 라이다 중심 -> 손가락 끝 (실제 로봇 0.315)
    margin = c.get("finger_margin", 0.08)
    if math.isnan(front) and not math.isnan(body_front) and body_front > 0.8:
        front = body_front                               # 11/27: 비스듬한 쪽만 안 보임(열린 옷장 등 어두운 곳) -> 정면 거리로
    if math.isnan(body_front) and not math.isnan(front) and front > 0.8:
        body_front = front
    if math.isnan(front) or math.isnan(body_front):
        # 앞이 안 보임: 그냥 서 있지 말고, 손이 쓸고 지나갈 곳이 비어 있는 쪽으로 천천히 돌아서 보이는 방향 찾기
        free = [(scan.sector_min(sd * 60, 25), sd) for sd in (1, -1)]
        free = [(d, sd) for d, sd in free if not math.isnan(d) and d > finger + 0.15]
        if free:
            sd = max(free)[1]
            return 0.0, 0.0, sd * c["w_turn_deg"] * RAD, "앞 안 보임-돌아서 찾기"
        return 0.0, 0.0, 0.0, "앞 안 보임-정지"          # 손이 라이다를 가리는 중일 수 있음 -> 움직이지 않음
    finger = c.get("finger_m", 0.22)                     # 라이다 중심 -> 손가락 끝 (실제 로봇 0.315)
    margin = c.get("finger_margin", 0.08)
    if body_front < finger + margin:                     # 손끝 바로 앞에 뭔가 있음
        return -0.15, 0.0, 0.0, "손 앞 막힘-후진"
    if c.get("hand_corridor", True):                     # 11/27: 손이 지나갈 띠(폭 16cm) 안 물체 (의자 다리처럼 각도로는 빠지는 것)
        from obstacles import hand_corridor
        hc = hand_corridor(scan, finger, c.get("hand_half_w", 0.08), margin)
        if hc is not None:
            sweep = scan.sector_min(s * 50, 50)
            if not math.isnan(sweep) and sweep < finger + 0.04:
                return -0.15, 0.0, 0.0, "손 띠 막힘-후진"
            return 0.0, 0.0, s * c["w_turn_deg"] * RAD, "손 띠 막힘-회전"
    if front < max(c["front_stop"], finger + margin + 0.05):
        # 제자리 회전하면 손끝이 원을 그린다. 도는 쪽 앞에 손끝 반경 안 물체가 있으면 먼저 물러난다
        sweep = scan.sector_min(s * 50, 50)
        if not math.isnan(sweep) and sweep < finger + 0.04:
            return -0.05, 0.0, 0.0, "회전 공간 없음-후진"
        return 0.0, 0.0, s * c["w_turn_deg"] * RAD, "막힘-회전"
    if math.isnan(r1):                                   # 한 방향만 응답 없음 -> 옆쪽을 넓게 다시 봄
        r1 = scan.sector_min(-90 * s + k, 25)
    if math.isnan(r1):                                   # 옆이 넓게 다 비었음 = 열린 공간 -> 벽 쪽으로 돌아 벽 찾기
        return c["v_arc"], 0.0, -s * c["w_arc_deg"] * RAD, "옆 비었음-벽 찾기"
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
        if c.get("no_strafe"):                       # 11/27 밤: 왼쪽 뒷바퀴 헛돎 -> 옆으로 밀지 않고 방향을 틀어 거리 맞춤
            vy = 0.0
            w = clamp(c["kw"] * phi - s * 1.8 * e, -0.6, 0.6)
        slow = clamp((front - c["front_stop"]) / 0.30, 0.35, 1.0)
        vx, tag = c["v"] * slow, "벽따라"
    kr = k * RAD
    return vx * math.cos(kr) - vy * math.sin(kr), vx * math.sin(kr) + vy * math.cos(kr), w, tag


class TurnFlip:
    """11/28 0시: 제자리 회전은 최소 45도/초 (느리면 바퀴가 헛돎). 반대로 돌기는 뺌 -
    라이다 방향 계산이 빠른 회전에서 실패하면 '안 돈다' 로 잘못 판단해 왼쪽/오른쪽을 왔다갔다 했음"""

    def __init__(self, car, pose_fn, clock, say=None):
        self.car = car

    def __getattr__(self, k):
        return getattr(self.car, k)

    def drive(self, vx, vy, w):
        if abs(vx) < 0.02 and abs(vy) < 0.02 and abs(w) > 0.15:
            w = math.copysign(max(abs(w), math.radians(45)), w)
        return self.car.drive(vx, vy, w)


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
        self.order = list(cfg["mission"].get("fixed_order") or [])     # 말한 순서
        self.digits, self.digit_seen = {}, {}    # 확정된 카드 숫자, 동물별 최근 (시각, 숫자)
        self.card_pos = {}                       # 동물 -> 지도 좌표 (x, y)
        self.outbox = []                         # PC 가 가져가서 스피커로 말할 대사
        self.announced_all = False
        self.link, self.odo, self.scanlog = None, None, None    # 실제 로봇: PC 우체통, 라이다 주행거리계, 지도용 스캔 기록
        self.pose_fn = None                      # 지금 로봇 자세 (x, y, th) 를 주는 함수 (실제: 라이다 주행거리계, 시뮬: 정답)

    def t(self):
        return self.clock.now() - self.t0

    def say(self, msg):
        self.log.say(self.t(), msg)

    def speak(self, text):
        """대사 (TTS): 기록에 남기고 PC 가 가져가 말하게 둠"""
        self.say(f"[대사] {text}")
        self.outbox.append(text)

    def ko(self, lab):
        return self.cfg["animals"].get(lab, {}).get("ko", lab)

    @staticmethod
    def josa(word, a="을", b="를"):
        c = ord(word[-1]) - 0xAC00
        return a if 0 <= c < 11172 and c % 28 else b

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
            if m["id"] == self.cfg["mission"]["keypad_marker"] and self.pose_fn and 0.06 < m["cx"] < 0.94                     and self.state not in ("CARD_GO", "APPROACH"):
                # 11/27 밤: 2번 마커 위치를 지도에 기억 (마지막에 찾아가기). 거리 = 그 방향 라이다(+테이프) 거리
                try:
                    x, y, th = self.pose_fn()
                    b = (0.5 - m["cx"]) * math.radians(self.cfg["camera"]["fov_deg"])
                    sc = self.lidar.latest()
                    if getattr(self, "vw", None) is not None:
                        sc = self.vw.add(sc, (x, y, th))
                    dd = sc.ray(math.degrees(b), 3)
                    if math.isfinite(dd) and 0.2 < dd < 2.5:
                        self.kp_obs = getattr(self, "kp_obs", []) + [(x + dd * math.cos(th + b), y + dd * math.sin(th + b))]
                        xs_ = sorted(q[0] for q in self.kp_obs)
                        ys_ = sorted(q[1] for q in self.kp_obs)
                        self.card_pos["__kp"] = (xs_[len(xs_) // 2], ys_[len(ys_) // 2])
                        self.card_from = getattr(self, "card_from", {})
                        self.card_from["__kp"] = (x, y)
                except Exception as e:
                    self.say(f"마커 위치 기억 실패: {e}")
            if m["id"] not in self.first_seen:
                self.first_seen[m["id"]] = now
                name = "키패드" if m["id"] == self.cfg["mission"]["keypad_marker"] else "문" if m["id"] == self.cfg["mission"]["door_marker"] else f"마커{m['id']}"
                self.say(f"{name} 마커 발견")
                if m["id"] == self.cfg["mission"]["keypad_marker"] and self.cfg["mission"].get("card_press"):
                    self.speak("비밀번호 입력 위치를 발견했습니다")
        need = self.cfg["detect"]["confirm"]
        for a in res.get("animals", []):
            lab, dg = a["label"], str(a.get("digit", "") or "")
            if lab in self.found and self.pose_fn and a.get("d") and 0.2 < a["cx"] < 0.8 \
                    and self.state in ("SPIN", "EXPLORE", "LOOK"):
                # 11/27 밤: 볼 때마다 위치를 다시 재서 중간값으로 기억 (처음 한 번만 재면 10~20cm 틀림)
                x, y, th = self.pose_fn()
                b = (0.5 - a["cx"]) * math.radians(self.cfg["camera"]["fov_deg"])
                d_ = a["d"] + self.cfg.get("cam_x", 0.21)
                if getattr(self, "vw", None) is not None:          # 테이프 선 거리로 (카드는 선 바로 바깥)
                    rr = self.vw.add(self.lidar.latest(), (x, y, th)).ray(math.degrees(b), 3)
                    if math.isfinite(rr) and abs(rr - d_) < 0.35:
                        d_ = rr + 0.02
                self.card_obs = getattr(self, "card_obs", {})
                self.card_obs.setdefault(lab, []).append((x + d_ * math.cos(th + b), y + d_ * math.sin(th + b), x, y))
                ob = self.card_obs[lab][-15:]
                xs_, ys_ = sorted(q[0] for q in ob), sorted(q[1] for q in ob)
                self.card_pos[lab] = (xs_[len(ob) // 2], ys_[len(ob) // 2])
                self.card_from = getattr(self, "card_from", {})
                self.card_from[lab] = (sorted(q[2] for q in ob)[len(ob) // 2], sorted(q[3] for q in ob)[len(ob) // 2])
            if lab in self.found or lab not in self.cfg["animals"]:
                continue
            if not dg and "digit" in self.cfg["animals"][lab] and self.cfg["mission"]["code_rule"] != "spoken":
                dg = self.cfg["animals"][lab]["digit"]
            seen = [x for x in self.digit_seen.get(lab, []) if now - x[0] < 3.0] + [(now, dg)]
            self.digit_seen[lab] = seen
            last = [x[1] for x in seen[-need:]]
            if len(last) >= need and all(x == last[0] and x for x in last):     # 3번 연속 같은 숫자
                self.found.append(lab)
                self.digits[lab] = last[0]
                if self.pose_fn and a.get("d"):
                    x, y, th = self.pose_fn()
                    b = (0.5 - a["cx"]) * math.radians(self.cfg["camera"]["fov_deg"])
                    self.card_pos[lab] = (x + a["d"] * math.cos(th + b), y + a["d"] * math.sin(th + b))
                    self.card_from = getattr(self, "card_from", {})
                    self.card_from[lab] = (x, y)                 # 카드를 본 자리 (카드 앞쪽 방향을 알려고)
                k = self.ko(lab)
                want = "" if not self.order else (" (찾을 동물)" if lab in self.order else " (찾을 동물 아님)")
                self.say(f"카드 확정: {k} {last[0]}{want}  지금까지 {[(self.ko(x), self.digits[x]) for x in self.found]}")
                self.speak(f"{k}{self.josa(k)} 발견했습니다, 숫자는 {last[0]}입니다")

    def _fuse(self, scan, pose):
        """PC obst_feed.py 가 보낸 장애물 상자 -> 라이다 점 (25초 기억). 스테이션 없으면 그대로"""
        from obstacles import ScanMemory, cam_obstacles, pole_rings, _Scan
        if getattr(self, "vw", None) is not None:
            scan = self.vw.add(scan, pose)              # 11/27 밤: 테이프 미로 = 가상 벽
        pr = pole_rings(scan) if self.cfg.get("mission", {}).get("poles", False) else []   # 11/27 20:40 실제 방에서 너무 자주 켜져 기본 끔
        if pr:
            scan = _Scan(scan, list(scan.pts) + pr)
        lk = self.link
        if lk is None:
            return scan
        if getattr(self, "cam_mem", None) is None:
            self.cam_mem = ScanMemory(keep_s=25.0, max_r=1.8, only_extra=True)
        extra = []
        if lk.obst and time.time() - lk.obst_rx < 0.8:
            extra = cam_obstacles(lk.obst["dets"], 0.21, scan, self.cfg["camera"].get("tilt_up_deg", 0.0))
        return self.cam_mem.update(scan, pose, extra)

    def marker(self, mid, within=1.5):
        m = self.marks.get(mid)
        return m if m and self.clock.now() - m[0] <= within else None

    def have_all(self):
        if self.order:
            return all(x in self.digits for x in self.order)
        return len(self.found) >= self.cfg["mission"]["expect"]

    def make_code(self):
        cfg, an = self.cfg["mission"], self.cfg["animals"]
        if cfg["code_rule"] == "spoken" and self.order:
            return "".join(self.digits[x] for x in self.order)
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
        done_animals = self.have_all()
        if done_animals and not self.announced_all:
            self.announced_all = True
            if not mc.get("card_press"):
                self.speak("단서를 전부 확인했습니다, 비밀번호를 입력하겠습니다")

        if st == "FIND_KEYPAD" and getattr(self, "vw", None) is not None and mc.get("card_press"):
            # 11/27 23:45: 계속 돌면 화면이 흔들려 마커를 못 봄 (빙글빙글) -> 30도씩 돌고 멈춰서 1초 보기, 한 바퀴
            r = self.step_search(now, lambda: (self.perceive(force=True) or True) and self.marker(kp, 0.6) is not None,
                                 [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330])
            if r == "found":
                self.card_target, self.card_n = "__kp", (0.0, 0.0)
                return self.go("CARD_GO", "2번 마커 보임 -> 가운데 맞추고 곧장 다가가기")
            if r == "fail":
                if "__kp" in self.card_pos and getattr(self, "kp_retry", 0) < 2:
                    self.kp_retry = getattr(self, "kp_retry", 0) + 1
                    self.card_target = "__kp"
                    return self.go("GOTO_KP", "한 바퀴 봐도 없음 -> 기억한 2번 마커 자리로")
                return self.go("FAIL", "한 바퀴 멈춰 보며 찾았지만 2번 마커 없음")
            return

        if st == "FIND_KEYPAD":                  # 키보드 시험: 제자리에서 천천히 돌며 마커 찾기
            self.perceive(force=True)
            if self.marker(kp, 0.5):
                self.car.stop()
                if getattr(self, "vw", None) is not None and mc.get("card_press"):
                    self.card_target, self.card_n = "__kp", (0.0, 0.0)     # 카드와 같은 방법으로 (직각은 테이프 기울기로)
                    return self.go("CARD_GO", "2번 마커 보임 -> 가운데 맞추고 곧장 다가가기")
                return self.go("APPROACH", "키보드 마커 보임")
            if now - self.sub_t > 30:
                self.car.stop()
                if mc.get("card_press") and "__kp" in self.card_pos and getattr(self, "kp_retry", 0) < 2:
                    self.kp_retry = getattr(self, "kp_retry", 0) + 1
                    self.card_target = "__kp"
                    return self.go("GOTO_KP", "마커가 안 보임 -> 기억한 2번 마커 자리로 다시")
                return self.go("FAIL", "30초 동안 키보드 마커를 못 찾음")
            last = getattr(self, "kp_last_cx", 0.5)          # 마지막으로 본 쪽으로 돌기 (오른쪽에서 놓쳤으면 오른쪽)
            self.car.drive(0, 0, (-40 if last > 0.5 else 40) * RAD)
            return

        if st == "WAIT_ORDER":                   # PC 가 음성으로 받은 순서를 보내 줄 때까지
            self.car.stop()
            od = self.link.order if self.link else None
            if od:
                self.order = [x for x in od if x in self.cfg["animals"]]
                self.cfg["mission"]["expect"] = len(self.order)
                self.speak("네, " + ", ".join(self.ko(x) for x in self.order) + " 순서대로 찾겠습니다")
                if self.cfg["mission"].get("stage") == "visit":
                    from frontier import Grid
                    self.grid = Grid()
                    return self.go("ANNOUNCE", f"4단계: 순서 받음 {self.order}")
                self.go("SPIN" if self.pose_fn else "EXPLORE", f"순서 받음 {self.order}")
            return

        if st == "SPIN":                         # 11/27: 시작 자리 둘레를 한 바퀴 보기 (주변 42cm 가 비었을 때만)
            self.perceive()
            x, y, th = self.pose_fn()
            if self.sub is None:
                around = min([d for d in (scan.sector_min(a_, 30) for a_ in range(-150, 181, 60)) if d == d] or [9])
                if around <= 0.42:
                    self.say(f"시작 자리가 좁아서({around * 100:.0f}cm) 한 바퀴 보기는 건너뜀")
                    return self.go("EXPLORE")
                self.sub, self._spin_acc, self._spin_last, self._spin_t0 = "TURN", 0.0, th, now
            self._spin_acc += abs(math.atan2(math.sin(th - self._spin_last), math.cos(th - self._spin_last)))
            self._spin_last = th
            if self._spin_acc >= 2 * math.pi or now - self.sub_t > 14:
                self.car.stop()
                el_ = now - getattr(self, "_spin_t0", now)
                if el_ > 1 and math.radians(30) < self._spin_acc / el_ < math.radians(120):
                    self.turn_rate = self._spin_acc / el_          # 45도/초 명령의 실제 속도 (11/28: 약 60~70도/초)
                    self.say(f"실제 회전 속도 {math.degrees(self.turn_rate):.0f}도/초 (45도/초 명령)")
                self.reanchor()
                if self.cfg["mission"].get("stage") == "calib":
                    self.speak(f"보정을 마쳤습니다, 회전 속도 초당 {math.degrees(getattr(self, 'turn_rate', 0)):.0f}도")
                    return self.go("DONE", "1단계 보정 끝")
                return self.go("EXPLORE", f"한 바퀴 보기 끝 ({math.degrees(self._spin_acc):.0f}도)")
            self.car.drive(0, 0, 45 * RAD)
            return

        if st in ("EXPLORE", "LOOK"):
            self.perceive()
            if done_animals and self.marker(kp) and not mc.get("card_press"):
                self.car.stop()
                return self.go("APPROACH", "동물 다 찾음 + 키패드 보임")

        if st == "EXPLORE" and self.pose_fn is not None and cfg["mission"].get("explorer", True):
            # 11/27: 실제 방에서 고친 탐색 (막힘/맴돌기/안 보임 빠져나오기, 한 바퀴 = 지도 완성)
            if getattr(self, "ex", None) is None:
                from explore import Explorer
                from frontier import Grid
                self.ex = Explorer(cfg, self.side, **self.small())
                self.grid, self.nav, self.t_cover = Grid(), None, 0.0
            pose = self.pose_fn()
            scan = self._fuse(scan, pose)                    # 카메라가 본 장애물(의자, 옷걸이...)을 라이다 점처럼
            self.grid.add(scan, pose)
            if self.nav is None:
                cmd, tag, ev = self.ex.step(scan, pose, now)
            else:
                cmd, tag, ev = self.nav.step(scan, pose, now)
            if ev:
                self.say(ev)
            # 11/27 밤: 찾을 카드 + 키보드 마커를 다 봤으면 한 바퀴를 안 끝냈어도 바로 출발점으로 (시간 아끼기)
            if mc.get("stage") == "find" and kp in self.first_seen and "__kp" in self.card_pos and (all(k in self.card_pos for k in mc.get("find_cards") or self.cfg["animals"]) or (self.t() > mc.get("find_secs", 480) and len(self.found) >= 3)):   # 놓아 둔 카드(정답 목록)를 다 찾을 때까지
                self.car.stop()
                self.speak("동물 카드와 탈출 위치를 모두 기억했습니다, 출발점으로 돌아갑니다")
                self.after_home = "SAVE_END"
                return self.go("HOME", "3단계: 카드 + 탈출 위치 기억 -> 출발점으로")
            if mc.get("stage") != "find" and mc.get("card_press") and done_animals and all(k in self.card_pos for k in self.order)                     and (self.ex.lap_done or kp in self.first_seen):
                self.car.stop()
                self.speak("카드를 모두 찾았습니다, 출발점으로 돌아갑니다")
                return self.go("HOME", "찾을 카드 + 키보드 마커 다 봄 -> 출발점으로")
            if self.nav is None and self.ex.lap_done:
                if not getattr(self, "map_done", False):
                    self.map_done = True
                    self.speak("지도를 완성했습니다, 다음 단계를 진행합니다")
                # 11/27: 한 바퀴 뒤 아직 못 찾은 카드/키보드가 있으면 '카메라로 안 본 벽' 찾아가기
                from frontier import Nav
                self.nav, self.t_cover = Nav(self.grid, cfg, max_r=mc.get("cover_r", 3.0), v=0.20), now
                self.say("아직 안 본 곳 찾아가기 시작")
            elif self.nav is not None and (self.nav.done or now - self.t_cover > mc.get("cover_secs", 120)):
                # 다 봤는데도 못 찾음 -> 다시 벽 따라 한 바퀴 (그 사이 본 곳 기록은 지도에 남음)
                from explore import Explorer
                self.say(f"찾아가기 끝 ({self.nav.reached}군데) -> 다시 벽 따라가기")
                self.nav, self.ex = None, Explorer(cfg, self.side, **self.small())
            if self.world is None and not cfg["wall"].get("no_strafe"):   # 명령했는데 안 움직이면 바퀴 힘 올리기 (테이프 모드는 끔: 헛도는 바퀴만 더 헛돎)
                from drive import Boost
                self.bst = getattr(self, "bst", None) or Boost()
                self.car.boost = self.bst.update(cmd, pose, now)
            self.car.drive(*cmd)
            if self.world is not None:
                self.world.tag = tag
            return

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

        elif st == "APPROACH" and self.pose_fn is not None and getattr(self, "decisive", True) and (self.world is None or os.environ.get("DECISIVE_SIM") or mc.get("card_press")):
            return self.approach_decisive(scan, now)

        elif st == "APPROACH":
            self.perceive(force=True)
            m = self.marker(kp, within=3.0)
            if m is not None:
                self.kp_last_cx = m[1]
            if m is None or now - self.sub_t > 60:
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
                far = front > pd + 0.5
                vx = (0.20 if far else 0.10) if abs(e) < 0.2 else 0.0   # 멀면 빠르게, 조금 벗어나도 돌면서 같이 다가감
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
                tol = mc.get("align_tol_m", 0.015)
                ok = abs(lat) < tol and abs(front - pd) < tol and abs(sq) < 0.012
                if ok:
                    self.aligned_since = self.aligned_since or now
                    if now - self.aligned_since > 0.6:
                        self.car.stop()
                        if not getattr(self, "keytest", None):
                            self.code = self.make_code()
                        if getattr(self, "no_press", False):
                            self.speak("비밀번호 입력 위치에 도착했습니다" if mc.get("card_press") else "키보드 앞에 섰습니다")
                            return self.go("DONE", f"정렬 끝 (옆 {lat * 100:.1f}cm, 거리 {front:.3f}m) -> 누르지 않고 끝 (손가락 위치 확인)")
                        return self.go("PRESS", f"정렬 끝 (옆 {lat * 100:.1f}cm, 거리 {front:.3f}m) -> 비밀번호 {self.code}")
                else:
                    self.aligned_since = None

        elif st in ("HOME", "GOTO_CARD", "GOTO_KP"):
            return self.card_nav(scan, now)

        elif st == "BREAKOUT":                   # 5단계: 탈출구로 밀고 나가기 (테이프 무시, 2.5초 직진)
            if now - self.sub_t < 2.5:
                self.car.drive(0.15, 0, 0)
                return
            self.car.stop()
            self.speak("탈출했습니다")
            return self.go("DONE", "5단계 탈출")

        elif st == "SAVE_END":
            self.car.stop()
            self.save_state()
            self.speak("3단계를 마쳤습니다")
            return self.go("DONE", "3단계 끝 (기억 저장)")

        elif st == "ANNOUNCE":
            self.car.stop()
            if self.sub is None:
                self.sub = "SAID"
                self.code = self.make_code()
                self.press_i, self.pressed_seq, self.card_tries = 0, [], 0
                self.speak(", ".join(self.ko(x) for x in self.order) + (" 순서로 누르겠습니다" if mc.get("card_touch") else " 순서로 찾아가겠습니다"))
            if now - self.sub_t > 3.0:
                return self.go("GOTO_CARD", f"첫 카드: {self.ko(self.order[0])}")
            return

        elif st == "AIM_CARD" and getattr(self, "vw", None) is not None:
            # 11/28: 멈춘 상태에서 각도를 정하고 시간으로 돌기 -> 0.6초 멈춰서(위치 바로잡기) 한 번 더 -> 카메라로
            x, y, th = self.pose_fn()
            nx, ny = getattr(self, "card_n", (0.0, 0.0))
            want = math.atan2(-ny, -nx) if (nx or ny) else math.atan2(self.card_pos[self.card_target][1] - y, self.card_pos[self.card_target][0] - x)
            if self.sub is None:
                self.sub, self._aim_n, self._aim_t = "WAIT", 0, now
            if self.sub == "WAIT":
                self.car.stop()
                if now - self._aim_t < 0.6:
                    return
                e = math.atan2(math.sin(want - th), math.cos(want - th))
                if abs(e) < math.radians(6) or self._aim_n >= 2:
                    return self.go("CARD_GO", f"{self.ko(self.card_target)} 정면 ({math.degrees(e):+.0f}도) -> 카메라로 맞추고 다가가기")
                self._aim_n += 1
                self.tturn(now, e)
                self.sub = "TURN"
                return
            if self.tturn(now):
                self.sub, self._aim_t = "WAIT", now
            return

        elif st == "AIM_CARD":                   # 지도에 적힌 카드 쪽으로 돌기 -> 카메라로 맞추기 (키보드 맞추기와 같은 방법)
            x, y, th = self.pose_fn()
            nx, ny = getattr(self, "card_n", (0.0, 0.0))
            if nx or ny:
                e = math.atan2(-ny, -nx) - th               # 벽을 똑바로 봄
            else:
                cx_, cy_ = self.card_pos[self.card_target]
                e = math.atan2(cy_ - y, cx_ - x) - th
            e = math.atan2(math.sin(e), math.cos(e))
            if abs(e) < math.radians(4) or now - self.sub_t > 8:
                self.car.stop()
                if getattr(self, "vw", None) is not None:
                    return self.go("CARD_GO", f"{self.ko(self.card_target)} 정면 -> 돌아서 가운데 맞추고 곧장 다가가기")
                if self.card_target == "__kp":
                    self.card_target = None
                    return self.go("FIND_KEYPAD", "2번 마커 자리에서 벽 봄 -> 마커 찾기")
                return self.go("APPROACH", f"{self.ko(self.card_target)} 카드 쪽 봄 -> 카메라로 맞추기")
            self.car.drive(0, 0, math.copysign(45 * RAD, e))
            return

        elif st == "CARD_GO":
            return self.card_go(scan, now)

        elif st == "SWEEP_CARD":                 # 11/27 밤: 카드 앞인데 안 보임 -> 왼쪽 40도, 오른쪽 80도 천천히 돌며 찾기
            el = now - self.sub_t
            res = self.vision.perceive()
            for a_ in res.get("animals", []):
                if a_["label"] == self.card_target and 0.2 < a_["cx"] < 0.8:
                    self.car.stop()
                    return self.go("APPROACH", f"둘러보다 {self.ko(self.card_target)} 카드 찾음")
            w = 45 * RAD
            if el < 40 / 45:
                self.car.drive(0, 0, w)
            elif el < 120 / 45:
                self.car.drive(0, 0, -w)
            elif el < 160 / 45:
                self.car.drive(0, 0, w)
            else:
                self.car.stop()
                return self.go("GOTO_CARD", "둘러봐도 없음 -> 카드 다시 찾아가기")
            return

        elif st == "PUSH":                       # 벽의 키를 손끝으로 앞으로 밀어 누르기
            front = scan.ray(0, 6)
            el = now - self.sub_t
            push_to = mc.get("card_push_dist", 0.30)
            if self.sub is None and not mc.get("card_touch", False):
                self.sub = "HOLD"                            # 11/27 밤 시연: 누르지 않고 카드 앞에 서서 말하기만
                self.speak(f"{self.ko(self.card_target)} 앞에 도착했습니다")
            if self.sub is None:
                self.sub = "IN"
                self.speak(f"{self.ko(self.card_target)} 누릅니다")
            if self.sub == "IN":
                if (math.isfinite(front) and front <= push_to) or el > 2.5:
                    self.car.stop()
                    self.sub, self.sub_t = "HOLD", now
                    if self.world is not None and hasattr(self.world, "card_at_finger"):
                        self.world.card_pushed = getattr(self.world, "card_pushed", []) + [self.world.card_at_finger()]
                    return
                self.car.drive(0.04, 0, 0)
                return
            if self.sub == "HOLD":
                self.car.stop()
                if el > (0.5 if mc.get("card_touch", False) else 2.0):
                    self.sub, self.sub_t = "OUT", now
                return
            if self.sub == "OUT":
                if el < 1.5:
                    self.car.drive(-0.10, 0, 0)
                    return
                self.car.stop()
                self.pressed_seq.append(self.card_target)
                self.say(f"누른 순서 {[self.ko(x) for x in self.pressed_seq]}")
                self.press_i += 1
                self.card_tries = 0
                if self.press_i < len(self.order):
                    return self.go("GOTO_CARD", f"다음 카드: {self.ko(self.order[self.press_i])}")
                if mc.get("stage") == "visit":
                    self.speak("말씀하신 동물 앞에 모두 다녀왔습니다")
                    self.after_home = "DONE"
                    return self.go("HOME", "4단계 끝 -> 출발점으로")
                if mc.get("end_keypad", True):
                    self.no_press = True
                    self.speak("비밀번호를 입력하러 가겠습니다")
                    self.cg_fails = 0
                    if "__kp" in self.card_pos:
                        self.card_target = "__kp"
                        return self.go("GOTO_KP", "카드 다 돎 -> 기억한 2번 마커 자리로")
                    self.card_target = None
                    return self.go("FIND_KEYPAD", "카드 다 돎 -> 키보드 마커(2번) 찾기")
                return self.go("VERIFY", "다 누름")

        elif st == "VERIFY":
            self.car.stop()
            if self.sub is None:
                self.sub = "WAIT"
                if mc["press_mode"] == "virtual":
                    self.door.send_code(self.code)
                    self.virtual_code = self.code
                    self.say(f"(가상 누르기) 스테이션에 {self.code} 보냄")
            ds = self.door.status()
            if ds == "open":
                self.speak("문이 열렸습니다, 탈출 성공입니다")
                return self.go("DONE", "키 확인: 맞음")
            if ds == "wrong":
                self.speak("틀렸다고 합니다")
                return self.go("DONE", "키 확인: 틀림")
            if now - self.sub_t > 10:
                self.speak("입력을 마쳤습니다")
                return self.go("DONE", "키 확인 결과 없음 (문 없이 시연)")
            return

        elif st == "PRESS":
            self.car.stop()
            self.speak("비밀번호는 " + ", ".join(self.code) + "입니다")
            if mc["press_mode"] == "virtual":
                self.door.send_code(self.code)
                self.virtual_code = self.code
                self.say(f"(가상 누르기) 스테이션에 {self.code} 보냄")
            else:
                for d in self.code:
                    f = cfg["hand"]["key_finger"][d]
                    self.say(f"숫자 {d} 누르기 (손가락 {f})")
                    self.hand.press(f, sleep=self.clock.sleep)
            self.press_tries += 1
            ds, t_wait = "closed", self.clock.now()
            while ds == "closed" and self.clock.now() - t_wait < 10.0:   # PC(키보드) 결과 기다림
                self.clock.sleep(0.5)
                ds = self.door.status()
            self.say(f"확인 결과: {ds}")
            if ds == "wrong" and self.press_tries < 3:
                self.speak("틀렸으니 재시도하겠습니다")
                return self.go("BACKOFF", "틀렸다고 함 -> 다시 정렬해서 누르기")
            if getattr(self, "keytest", None):
                self.speak("눌렀습니다" if ds != "closed" else "누르기를 끝냈습니다")
                return self.go("DONE", f"키보드 시험 끝 (확인 결과 {ds})")
            if ds == "open":
                self.speak("정답입니다, 탈출구로 이동합니다" if mc.get("end") == "exit" else "문이 열립니다")
            self.pressed = True
            return self.go("BACKOFF", "누르기 끝 -> 뒤로 물러나 문 찾기")

        elif st == "BACKOFF":
            if now - self.sub_t < 1.5:
                self.car.drive(-0.08, 0, 0)
            else:
                self.car.stop()
                if getattr(self, "keytest", None):
                    return self.go("FIND_KEYPAD", "키보드 시험: 다시 마커 찾기")
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
            if mc.get("end") == "exit" and m and blocked(front, 0.50):        # 탈출구 마커 바로 앞에 도착
                self.car.stop()
                self.speak("탈출했습니다")
                return self.go("DONE", "탈출구(마커 1번) 앞 도착")
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

    # ---------- 키보드로 다가가기 (11/26 v3): 마커가 화면에 다 보이는 거리에서 조준을 다 끝내고, 마지막은 라이다만 보고 직진 ----------
    #  LOOK(멀리서 1초 보기) -> TURN(한 번에 돌기) -> GO(약 80cm 지점까지 쭉) -> SQUARE(라이다로 벽과 직각)
    #  -> LOOK2(1초 보기: 옆으로 몇 cm 어긋났나) -> SIDE(옆으로 그만큼) -> FINAL(라이다 거리 보며 직진, press_dist 에서 멈춤)
    def reanchor(self):
        """한 바퀴 보기 끝: 넓게(+-20도) 출발 사진과 맞추기"""
        self.relocalize(force=True)

    def relocalize(self, force=False):
        """11/27 23:45: 로봇은 늘 출발점 1.5m 안 -> 1초마다 지금 라이다를 '출발할 때 사진' 에 직접 맞춰 위치/방향을 바로잡음.
        (주행거리계는 도는 동안 조금씩 틀어져 테이프 네모/기억한 카드 위치가 같이 돌아가던 문제). 좌표는 출발 좌표 그대로."""
        if self.world is not None or getattr(self, "vw", None) is None or getattr(self, "scan_first", None) is None:
            return
        now = self.clock.now()
        if not force and (now - getattr(self, "_reloc_t", 0.0) < 0.7 or abs(self.car.cmd[2]) > 0.8):
            return
        self._reloc_t = now
        from heading import icp_xy, to_xy
        cur = self.lidar.latest()
        if cur is None or len(cur.pts) < 120:                  # 덜 찍힌 사진으로는 안 맞춤
            return
        g = self.odo.pose
        ref = getattr(self, "_ref_pts", None)
        if ref is None:
            ref = self._ref_pts = to_xy(self.scan_first)
        cur_pts = to_xy(cur)
        best = None
        guesses = list(range(0, 360, 15)) if force else (0.0, -10.0, 10.0, -20.0, 20.0, -35.0, 35.0)   # 시작: 한 바퀴 전체에서 찾기
        for dth in guesses:
            gx = (g[0], g[1], g[2] + math.radians(dth))
            r_ = icp_xy(ref, cur_pts, gx)
            if best is None or r_[3] > best[3]:
                best = r_
        dth_ = math.degrees(math.atan2(math.sin(best[2] - g[2]), math.cos(best[2] - g[2])))
        dxy = math.hypot(best[0] - g[0], best[1] - g[1])
        # 11/28 1시: 주행 중 20도 넘게 바꾸는 건 아주 잘 맞을 때만 (5단계에서 +35, -50, +28도로 잘못 휙휙 바뀜)
        if best[3] < (0.55 if force else 0.6) or (not force and (abs(dth_) > 60 or dxy > 0.5 or (abs(dth_) > 20 and best[3] < 0.8))):
            if force:
                self.say(f"출발 사진과 다시 맞추기 실패 (맞춤 {best[3]:.2f}) -> 그대로")
            return
        if force or abs(dth_) > 2 or dxy > 0.03:
            self.say(f"출발 사진으로 위치 바로잡음: 방향 {dth_:+.1f}도, 위치 {dxy * 100:.0f}cm (맞춤 {best[3]:.2f})")
        o = self.odo
        o.key_pts, o.key_pose, o.rel = cur_pts, (best[0], best[1], best[2]), (0.0, 0.0, 0.0)
        o.pending, o.fail_run = (0.0, 0.0, 0.0), 0
        o.pose = o.key_pose
        self._last_scan = cur
        self.relocs = getattr(self, "relocs", 0) + 1

    def small(self):
        """테이프 네모처럼 작은 구역이면 '한 바퀴' 기준을 줄임"""
        return dict(lap_min_travel=2.5, far_min=0.5) if getattr(self, "vw", None) is not None else {}

    def card_face(self, lab):
        """11/27: 카드가 붙은 벽의 바깥쪽 방향(지도 칸으로 벽 직선 맞추기)과 설 자리 거리.
        -> (벽 위 카드 x, y, 바깥 방향 nx, ny, 설 거리). 벽 앞이 좁으면 가운데쯤에 섬"""
        import numpy as np
        mc = self.cfg["mission"]
        cx_, cy_ = self.card_pos[lab]
        fx, fy = getattr(self, "card_from", {}).get(lab, (0.0, 0.0))
        g = self.grid
        i0, j0 = g.ij(cx_, cy_)
        r = int(0.30 / g.cell)
        pts = []
        for i in range(max(0, i0 - r), min(g.n, i0 + r + 1)):
            for j in range(max(0, j0 - r), min(g.n, j0 + r + 1)):
                if g.L[i, j] > 0.6:
                    pts.append(g.xy(i, j))
        dx, dy = fx - cx_, fy - cy_
        n0 = math.hypot(dx, dy) or 1.0
        nx, ny = dx / n0, dy / n0
        vw = getattr(self, "vw", None)
        if vw is not None and vw.segs:
            # 11/27 자정: 테이프 네모는 변 위치를 아니까 가장 가까운 변에 직각으로 (모서리 근처 카드가 대각선 -135도로 나오던 문제)
            best = None
            for (ax_, ay_), (bx_, by_) in vw.segs:
                vx_, vy_ = bx_ - ax_, by_ - ay_
                L2 = vx_ * vx_ + vy_ * vy_ or 1e-9
                t_ = max(0.08, min(0.92, ((cx_ - ax_) * vx_ + (cy_ - ay_) * vy_) / L2))   # 모서리에서 조금 안쪽
                px_, py_ = ax_ + t_ * vx_, ay_ + t_ * vy_
                dd = math.hypot(cx_ - px_, cy_ - py_)
                if best is None or dd < best[0]:
                    L = math.sqrt(L2)
                    ux, uy = -vy_ / L, vx_ / L
                    hx, hy = getattr(self, "home", (0.0, 0.0))
                    if ux * (hx - px_) + uy * (hy - py_) < 0:
                        ux, uy = -ux, -uy                 # 네모 안쪽으로
                    best = (dd, px_, py_, ux, uy)
            cx_, cy_, nx, ny = best[1], best[2], best[3], best[4]
            pts = []
        if len(pts) >= 5:
            P = np.array(pts)
            m = P.mean(0)
            w, v = np.linalg.eigh(np.cov((P - m).T))
            tx, ty = v[:, 1]                          # 벽 방향
            ux, uy = -ty, tx                          # 벽에 수직
            if ux * dx + uy * dy < 0:
                ux, uy = -ux, -uy                     # 로봇이 본 쪽으로
            # 카드 위치를 벽 직선 위로 옮김
            t = (cx_ - m[0]) * tx + (cy_ - m[1]) * ty
            cx_, cy_ = m[0] + t * tx, m[1] + t * ty
            nx, ny = ux, uy
        # 벽 앞 빈 거리 재기 (맞은편 벽까지)
        free = 1.2
        k = 3
        while k * g.cell < 1.2:
            i, j = g.ij(cx_ + nx * k * g.cell, cy_ + ny * k * g.cell)
            if not (0 <= i < g.n and 0 <= j < g.n) or g.L[i, j] > 0.6:
                free = k * g.cell
                break
            k += 1
        sd = max(0.32, min(mc.get("card_standoff", 0.60), free - 0.30))
        self.say(f"{self.ko(lab)} 카드 벽: 바깥 방향 {math.degrees(math.atan2(ny, nx)):+.0f}도, 앞 빈 거리 {free:.2f}m -> {sd:.2f}m 앞에 섬")
        return cx_, cy_, nx, ny, sd

    def tturn(self, now, ang=None):
        """11/28: 시간으로 돌기 (라이다 방향 계산이 빠른 회전에서 자주 실패). 1단계 한 바퀴에서 잰 실제 속도 사용.
        ang 를 주면 시작. 매번 불러서 True 면 끝"""
        if ang is not None:
            rate = getattr(self, "turn_rate", math.radians(self.cfg["mission"].get("turn_rate_deg", 62.0)))
            ang = math.atan2(math.sin(ang), math.cos(ang))
            self._tt = (0, now) if abs(ang) < math.radians(2) else ((1 if ang > 0 else -1), now + abs(ang) / rate + 0.12)
        sg, tend = getattr(self, "_tt", (0, now))
        if sg == 0 or now >= tend:
            self.car.stop()
            self._tt = (0, now)
            return True
        self.car.drive(0, 0, sg * 45 * RAD)
        return False

    STATE = os.path.join(HERE, "demo_state.json")

    def save_state(self):
        """3단계 끝: 출발 사진 + 기억한 카드/탈출 위치 저장 -> 4, 5단계가 불러씀"""
        sf = self.scan_first
        d = {"scan_first": {"a": [round(p[0], 2) for p in sf.pts], "d": [round(p[1], 3) for p in sf.pts]},
             "card_pos": {k: list(v) for k, v in self.card_pos.items()},
             "card_from": {k: list(v) for k, v in getattr(self, "card_from", {}).items()},
             "digits": self.digits, "found": self.found,
             "turn_rate_deg": math.degrees(getattr(self, "turn_rate", math.radians(62))),
             "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        json.dump(d, open(self.STATE, "w", encoding="utf-8"), ensure_ascii=False)
        self.say(f"기억 저장: 카드 {[(self.ko(k), self.digits.get(k)) for k in self.card_pos if k != '__kp']}, 탈출 위치 {'있음' if '__kp' in self.card_pos else '없음'}")

    def load_state(self):
        from lidar import Scan
        d = json.load(open(self.STATE, encoding="utf-8"))
        self.scan_first = Scan(list(zip(d["scan_first"]["a"], d["scan_first"]["d"])), time.time())
        self.card_pos = {k: tuple(v) for k, v in d["card_pos"].items()}
        self.card_from = {k: tuple(v) for k, v in d["card_from"].items()}
        self.digits, self.found = d["digits"], d["found"]
        self.turn_rate = math.radians(d.get("turn_rate_deg", 62.0))
        self.say(f"기억 불러옴 ({d['time']}): 카드 {[(self.ko(k), self.digits.get(k)) for k in self.card_pos if k != '__kp']}, "
                 f"탈출 위치 {'있음' if '__kp' in self.card_pos else '없음'}, 회전 {d.get('turn_rate_deg', 62):.0f}도/초")

    def step_search(self, now, seen, angles):
        """제자리에서 정해진 각도들로 돌며 '멈춰서 보기' -> 'found' / 'look' / 'fail'.
        seen(): 지금 보이면 True. angles: 처음 방향 기준 도 단위 목록"""
        x, y, th = self.pose_fn()
        ss = getattr(self, "_ss", None)
        if ss is None:
            ss = self._ss = {"h0": th, "i": 0, "t": now, "phase": "turn"}
        if ss["i"] >= len(angles):
            self.car.stop()
            self._ss = None
            return "fail"
        if ss["phase"] == "turn":
            if ss.get("started") != ss["i"]:
                ss["started"] = ss["i"]
                prev = angles[ss["i"] - 1] if ss["i"] > 0 else 0
                self.tturn(now, math.radians(angles[ss["i"]] - prev))      # 시간으로 (라이다 방향이 놓쳐도 됨)
            if self.tturn(now):
                ss["phase"], ss["t"] = "look", now
            return "look"
        self.car.stop()
        if seen():
            self._ss = None
            return "found"
        if now - ss["t"] > 1.0:
            ss["i"], ss["phase"], ss["t"] = ss["i"] + 1, "turn", now
        return "look"

    def card_go(self, scan, now):
        """11/27 밤 (테이프 시연): 카드/2번 마커 앞에 '확실히' 서기. 옆으로 미끄러지지 않음 (왼쪽 뒷바퀴 헛돎).
        CENTER: 제자리에서 돌며 카메라 가운데에 -> DRIVE: 보면서 곧장, 손끝이 7cm 앞까지 -> SQUARE: 테이프와 직각
        -> NOD: 콕 찍었다 빠지기 (보는 사람이 어느 것 앞에 섰는지 알게) -> 말하기"""
        mc = self.cfg["mission"]
        x, y, th = self.pose_fn()
        sc = self.vw.add(scan, (x, y, th)) if getattr(self, "vw", None) is not None else scan   # 시험(테이프 없음): 카드 뒤 박스/벽을 라이다로
        front = sc.ray(0, 4)
        kp_mode = self.card_target == "__kp"
        stop_d = mc.get("kp_stop_dist2", 0.40) if kp_mode else mc.get("card_stop_dist2", 0.38)
        sub = self.sub or "CENTER"
        el = now - self.sub_t

        def see():
            res = self.vision.perceive()
            if kp_mode:
                for m in res.get("markers", []):
                    if m["id"] == mc["keypad_marker"]:
                        return m["cx"]
            else:
                for a_ in res.get("animals", []):
                    if a_["label"] == self.card_target:
                        return a_["cx"]
            return None

        def nxt(sub_):
            self.sub, self.sub_t = sub_, now

        if sub == "SEARCH":                          # 안 보임 -> 15도씩 돌고 멈춰서 보기 (좌우 60도까지)
            self._last_cx = None

            def seen_():
                self._last_cx = see()
                return self._last_cx is not None
            r = self.step_search(now, seen_, [0, 15, 30, 45, 60, -15, -30, -45, -60, 0])
            if r == "found":
                self._cg_seen = now
                return nxt("CENTER")
            if r == "fail":
                self.cg_fails = getattr(self, "cg_fails", 0) + 1
                if self.cg_fails <= 1:
                    self.say(f"{self.ko(self.card_target)} 이(가) 안 보임 -> 기억한 자리로 다시 가서 찾기")
                    return self.go("GOTO_KP" if kp_mode else "GOTO_CARD", "다시 찾기")
                self.say(f"{self.ko(self.card_target)} 을(를) 끝내 못 봄 -> 기억한 위치 정면으로 다가감")
                return nxt("DRIVE")
            return

        if sub == "CENTER":                          # 11/28 0시: 돌면서 보면 사진이 0.3~0.5초 늦어 가운데를 지나침 (왔다갔다)
            # -> 멈춰서 0.6초 뒤 보기 -> 몇 도인지 계산 -> 라이다로 재며 그만큼만 돌기 -> 다시 멈춰 확인 (최대 4번)
            ph = getattr(self, "_c_ph", None)
            if ph is None or getattr(self, "_c_for", None) != self.sub_t:
                self._c_for, self._c_ph, self._c_t, self._c_n = self.sub_t, "look", now, 0
                ph = "look"
            if ph == "turn":
                if self.tturn(now):
                    self._c_ph, self._c_t = "look", now
                return
            self.car.stop()
            if now - self._c_t < 0.6:                # 멈춘 뒤 새 사진이 올 때까지
                return
            cx = see()
            if cx is not None:
                self._cg_seen = now
                e = cx - 0.5 - mc.get("cam_offset", 0.0)
                if abs(e) < 0.07 or self._c_n >= 4:
                    return nxt("DRIVE")
                self._c_n += 1
                b = -e * math.radians(self.cfg["camera"]["fov_deg"])       # 오른쪽(cx>0.5)이면 오른쪽(-)으로
                self.tturn(now, b)
                self._c_ph, self._c_t = "turn", now
                return
            if now - self._c_t > 1.8:                 # 멈춰서 1초 넘게 봐도 없음 -> 찾기
                self._ss = None
                return nxt("SEARCH")
            return

        if sub == "DRIVE":                           # 보면서 곧장 (옆으로 안 감, 방향만 고침)
            if (math.isfinite(front) and front <= stop_d) or el > 10:
                self.car.stop()
                self.say(f"다가가기 끝: 벽까지 {front:.2f}m ({'도착' if front <= stop_d else '10초 지남'}, {el:.1f}초)")
                return nxt("SQUARE")
            cx = see()
            w = 0.0
            if cx is not None:
                self._cg_seen = now
                w = clamp(-2.0 * (cx - 0.5 - mc.get("cam_offset", 0.0)), -0.4, 0.4)
            elif now - getattr(self, "_cg_seen", now) > 1.2 and getattr(self, "cg_fails", 0) < 2 \
                    and math.isfinite(front) and front > stop_d + 0.12:
                self.car.stop()
                self._ss = None
                return nxt("SEARCH")
            v = 0.06 if (math.isfinite(front) and front < stop_d + 0.15) else 0.14
            self.car.drive(v, 0, w)
            return

        if sub == "SQUARE":                          # 테이프 선과 직각: 테이프 방향을 아니까 그 방향으로 정확히 돌기
            nx_, ny_ = getattr(self, "card_n", (0.0, 0.0))
            if nx_ or ny_:
                t = math.atan2(-ny_, -nx_) - th
            else:                                    # 방향을 모름: 처음 한 번 잰 벽 기울기만큼만 (매번 재면 47도까지 틀어짐)
                if getattr(self, "_sq_for", None) != self.sub_t:
                    t0_ = self.wall_tilt(sc)
                    self._sq_for = self.sub_t
                    self._sq_to = th + t0_ if abs(t0_) < math.radians(19) else th     # 20도에 걸리면 못 믿음 -> 안 돎
                t = self._sq_to - th
            t = math.atan2(math.sin(t), math.cos(t))
            if getattr(self, "_sqt_for", None) != self.sub_t:            # 멈춰서 각도 정하고 시간으로 돌기 -> 0.6초 멈춤 -> 한 번 더 (최대 2번)
                self._sqt_for, self._sq_n, self._sq_ph, self._sq_t = self.sub_t, 0, "wait", now - 1.0
            if self._sq_ph == "turn":
                if self.tturn(now):
                    self._sq_ph, self._sq_t = "wait", now
                return
            self.car.stop()
            if now - self._sq_t < 0.6:
                return
            if abs(t) <= math.radians(5) or self._sq_n >= 2 or el > 8:
                return nxt("NOD")
            self._sq_n += 1
            self.tturn(now, t)
            self._sq_ph = "turn"
            return

        if sub == "NOD":                             # 콕: 4cm 앞으로 -> 0.3초 -> 뒤로
            if self.world is not None and getattr(self, "_nod_logged", None) != (self.card_target, self.sub_t):
                self._nod_logged = (self.card_target, self.sub_t)
                w_ = self.world
                tgt = w_.keypad if kp_mode else next((c for c in w_.cards if c["label"] == self.card_target), None)
                if tgt:
                    dx, dy = tgt["x"] - w_.x, tgt["y"] - w_.y
                    lat = -dx * math.sin(w_.th) + dy * math.cos(w_.th)
                    self.say(f"[시뮬 정답] {self.ko(self.card_target)} 과 좌우 어긋남 {lat * 100:+.1f}cm, 각도 {math.degrees(math.atan2(math.sin(w_.th - math.atan2(-tgt['ny'], -tgt['nx'])), math.cos(w_.th - math.atan2(-tgt['ny'], -tgt['nx'])))):+.1f}도")
            if el < 0.5:
                self.car.drive(0.08, 0, 0)
                return
            if el < 0.8:
                self.car.stop()
                return
            if el < 1.3:
                self.car.drive(-0.08, 0, 0)
                return
            self.car.stop()
            if kp_mode:
                self.card_target = None
                if mc.get("stage") == "exit":
                    self.speak("탈출구에 도착했습니다, 나가겠습니다")
                    return self.go("BREAKOUT", f"2번 마커 앞 (벽까지 {front:.2f}m) -> 밀고 나가기")
                self.speak("비밀번호 입력 위치에 도착했습니다")
                return self.go("DONE", f"2번 마커 앞 (벽까지 {front:.2f}m)")
            return self.go("PUSH", f"{self.ko(self.card_target)} 앞 (벽까지 {front:.2f}m)")

    def card_nav(self, scan, now):
        """HOME: 출발점으로 / GOTO_CARD: 지도에 적힌 카드 앞 65cm 자리로 (frontier.Nav 길찾기)"""
        st, mc = self.state, self.cfg["mission"]
        pose = self.pose_fn()
        scan = self._fuse(scan, pose)
        self.grid.add(scan, pose)
        if self.sub is None:
            self._ss = None
            from frontier import Nav
            self.cnav = Nav(self.grid, self.cfg, max_r=mc.get("cover_r", 3.0), v=0.20)
            if st == "HOME":
                goal = getattr(self, "home", (0.0, 0.0))
            else:
                if st == "GOTO_CARD" and self.order[self.press_i] not in self.card_pos:   # 기억에 없는 동물 -> 건너뛰기
                    miss = self.order[self.press_i]
                    self.speak(f"{self.ko(miss)}{self.josa(self.ko(miss), '은', '는')} 위치를 모릅니다, 건너뜁니다")
                    self.press_i += 1
                    self.sub = None
                    if self.press_i >= len(self.order):
                        self.after_home = "DONE"
                        return self.go("HOME", "기억에 없는 동물 건너뜀 -> 출발점으로")
                    return
                if st == "GOTO_CARD":
                    if getattr(self, "_cg_for", None) != self.press_i:
                        self._cg_for, self.cg_fails = self.press_i, 0
                    self.card_target = self.order[self.press_i]
                cx_, cy_, nx, ny, sd = self.card_face(self.card_target)
                self.card_n = (nx, ny)
                goal = (cx_ + nx * sd, cy_ + ny * sd)
                if st == "GOTO_CARD":
                    self.speak(f"{self.ko(self.card_target)} 카드로 갑니다")
            self.cnav.go_to(goal)
            self.say(f"{st}: ({goal[0]:+.2f}, {goal[1]:+.2f}) 로 길찾기")
            self.sub = "NAV"
        cmd, tag, ev = self.cnav.step(scan, pose, now)
        if ev:
            self.say(ev)
        if self.cnav.done or now - self.sub_t > 60:
            self.car.stop()
            if st == "HOME":
                nxt_ = getattr(self, "after_home", "ANNOUNCE")
                return self.go(nxt_, "출발점 도착")
            return self.go("AIM_CARD", "카드 앞 도착")
        if self.world is None and not self.cfg["wall"].get("no_strafe"):
            from drive import Boost
            self.bst = getattr(self, "bst", None) or Boost()
            self.car.boost = self.bst.update(cmd, pose, now)
        self.car.drive(*cmd)
        if self.world is not None:
            self.world.tag = tag

    def wall_tilt(self, scan):
        """정면 ±20도 라이다 점들로 벽 직선을 맞춰 기울기 (왼쪽이 멀면 +, 그만큼 왼쪽으로 돌면 직각)"""
        f0 = scan.ray(0, 4)
        if not math.isfinite(f0):
            return 0.0
        pts = [(d * math.cos(math.radians(a)), d * math.sin(math.radians(a))) for a, d in scan.pts
               if d > 0 and abs(a) <= 20 and abs(d * math.cos(math.radians(a)) - f0) < 0.08]
        if len(pts) < 6:
            return 0.0
        mx = sum(q[0] for q in pts) / len(pts)
        my = sum(q[1] for q in pts) / len(pts)
        sxy = sum((q[0] - mx) * (q[1] - my) for q in pts)
        syy = sum((q[1] - my) ** 2 for q in pts) or 1e-9
        return max(-math.radians(20), min(math.radians(20), math.atan(sxy / syy)))

    def approach_decisive(self, scan, now):
        mc = self.cfg["mission"]
        kp, pd = mc["keypad_marker"], mc["press_dist"]
        fov = self.cfg["camera"]["fov_deg"]
        cam_x = self.cfg.get("cam_x", 0.21)
        slide = mc.get("brake_slide", 0.03)          # 초속 20cm 에서 멈출 때 더 가는 거리 (11/26 실측 약 3cm)
        mid = pd + 0.35                              # 조준을 끝낼 거리 (라이다~벽). 마커가 화면에 다 들어옴
        x, y, th = self.pose_fn()
        sub = self.sub or "LOOK"
        front = scan.ray(0, 6)

        card = getattr(self, "card_target", None) if mc.get("card_press") else None
        if getattr(self, "vw", None) is not None:
            scan = self.vw.add(scan, (x, y, th))             # 테이프 선 = 카드/마커가 서 있는 '벽'
            front = scan.ray(0, 6)
            if not card:
                pd = mc.get("kp_stop_dist", 0.45)
                mid = pd + 0.35
        if card:
            pd = mc.get("card_press_dist", 0.34) if mc.get("card_touch") else mc.get("card_stop_dist", 0.45)   # 시연: 손끝이 카드 13cm 앞
            mid = pd + 0.35

        def look(n_need=3, secs=1.0):
            self.car.stop()
            if card:                                   # 카드: PC 가 찾은 동물 상자 가운데
                res = self.vision.perceive()
                for a_ in res.get("animals", []):
                    if a_["label"] == card:
                        self._seen.append(a_["cx"])
                        break
            else:
                self.perceive(force=True)
                m = self.marks.get(kp)
                if m and now - m[0] < 0.3:
                    self._seen.append(m[1])
            if now - self.sub_t < secs:
                return None
            if len(self._seen) < n_need:
                return "lost"
            v = sorted(self._seen)
            return v[len(v) // 2]

        def lost():
            if card:
                self.card_tries = getattr(self, "card_tries", 0) + 1
                if self.card_tries >= 3:
                    self.say(f"{self.ko(card)} 카드를 카메라로 못 맞춤 -> 지도 위치대로 누르기")
                    return self.go("PUSH", "카메라 맞추기 실패")
                if self.card_tries == 1:
                    return self.go("SWEEP_CARD", f"{self.ko(card)} 카드가 안 보임 -> 좌우로 둘러보기")
                self.say(f"{self.ko(card)} 카드를 확실히 못 봄 ({len(self._seen)}번) -> 다시 찾아가기")
                return self.go("GOTO_CARD", "카드 다시 찾아가기")
            self.say(f"키보드 마커를 확실히 못 봄 ({len(self._seen)}번) -> 다시 찾기")
            try:
                f = getattr(self.vision, "last_frame", None)
                if f is not None:
                    import cv2
                    cv2.imwrite(os.path.join(HERE, "logs", time.strftime("lost_marker_%H%M%S.jpg")), f)
            except Exception:
                pass
            return self.go("FIND_KEYPAD" if (getattr(self, "keytest", None) or mc.get("card_press")) else "BACKOFF", "마커 확인 실패")

        if self.sub is None:
            self._seen, self.sub, self.sub_t = [], "LOOK", now
            sub = "LOOK"

        if sub == "LOOK":
            r = look()
            if r is None:
                return
            if r == "lost":
                return lost()
            b = math.radians((0.5 - r) * fov)
            self._turn_to = th + b
            far = math.isfinite(front) and front > mid + 0.15
            self._next = "GO" if far else "SQUARE"
            self.say(f"마커 확인 ({len(self._seen)}번, 방향 {math.degrees(b):+.1f}도, 벽까지 {front:.2f}m) -> {'80cm 지점까지 가기' if far else '직각 맞추기'}")
            self.sub, self.sub_t = "TURN", now
            return

        if sub == "TURN":                              # 라이다로 재며 한 번에 돌기 (초당 30도)
            e = math.atan2(math.sin(self._turn_to - th), math.cos(self._turn_to - th))
            if abs(e) < math.radians(2.0) or now - self.sub_t > 6:
                self.car.stop()
                self._seen, self.sub, self.sub_t = [], self._next, self.clock.now()
                return
            self.car.drive(0, 0, math.copysign((40 if abs(e) > math.radians(8) else 30) * RAD, e))
            return

        if not math.isfinite(front):
            self.car.stop()
            return

        if sub == "GO":                                # 한 번에 쭉 (초속 20cm), 조준 거리에서 멈춤
            if front - slide <= mid or now - self.sub_t > 15:
                self.car.soft_stop()
                self.sub, self.sub_t = "SQUARE", self.clock.now()
                return
            e = math.atan2(math.sin(th - self._turn_to), math.cos(th - self._turn_to))
            self.car.drive(0.20, 0, clamp(-2.5 * e, -0.5, 0.5))
            return

        if sub == "SQUARE":                            # 벽과 직각
            self.car.stop()
            if now - self.sub_t < 0.4:
                return
            t = self.wall_tilt(scan)
            self.say(f"벽 기울기 {math.degrees(t):+.1f}도 -> 직각 맞추기")
            self._turn_to, self._next = th + t, "LOOK2"
            self.sub, self.sub_t = "TURN", now
            return

        if sub == "LOOK2":                             # 옆으로 몇 cm 어긋났나
            r = look()
            if r is None:
                return
            if r == "lost":
                return lost()
            d_cam = max(0.1, front - cam_x)
            lat = (0.5 - r) * 2 * d_cam * math.tan(math.radians(fov / 2)) - mc.get("cam_offset_m", 0.0)   # 왼쪽 +
            self.say(f"마커가 가운데에서 {lat * 100:+.1f}cm (왼쪽 +) -> {'옆으로 맞추기' if abs(lat) > 0.01 else '바로 앞으로'}")
            self._side_to = (x - math.sin(th) * lat, y + math.cos(th) * lat)
            self._side_dir = 1 if lat > 0 else -1
            self._lat0 = lat
            self.sub, self.sub_t = ("SIDE" if abs(lat) > 0.01 else "FINAL"), now
            self._final_th = th
            return

        if sub == "SIDE":                              # 옆으로 한 번에 (초속 20cm 게걸음), 주행거리계로 그만큼만
            ex, ey = self._side_to[0] - x, self._side_to[1] - y
            rem = (-math.sin(th) * ex + math.cos(th) * ey) * self._side_dir     # 남은 옆 거리 (가는 쪽 +)
            if rem <= 0.01 + slide * 0.5 or now - self.sub_t > 5:
                self.car.soft_stop()
                self.sub, self.sub_t = "SQUARE2", self.clock.now()          # 게걸음하면 몸이 돌아감 (11/26: 14도) -> 직각 다시
                return
            e = math.atan2(math.sin(th - self._final_th), math.cos(th - self._final_th))
            self.car.drive(0, self._side_dir * 0.10, clamp(-2.5 * e, -0.5, 0.5))
            return

        if sub == "SQUARE2":
            self.car.stop()
            if now - self.sub_t < 0.4:
                return
            t = self.wall_tilt(scan)
            if abs(t) < math.radians(2):
                self._final_th = th
                self.sub, self.sub_t = "FINAL", now
                return
            self.say(f"게걸음 뒤 벽 기울기 {math.degrees(t):+.1f}도 -> 다시 직각")
            self._turn_to, self._next = th + t, "FINAL"
            self._final_th = th + t
            self.sub, self.sub_t = "TURN", now
            return

        if sub == "FINAL":                             # 라이다 거리만 보고 곧장 (마커는 안 봄: 가까우면 화면 밖)
            # 11/26: 초속 20cm 로 끝까지 가면 7cm 더 들어감 -> 목표 12cm 앞까지만 빠르게, 그 뒤는 초속 6cm (끊어서 가기)
            near = front - pd < 0.12
            if front - (0.012 if near else slide) <= pd or now - self.sub_t > 12:
                self.car.soft_stop()
                self.clock.sleep(0.6)
                f2 = self.lidar.latest().ray(0, 6)
                t2 = math.degrees(self.wall_tilt(self.lidar.latest()))
                msg = f"정렬 끝 (벽까지 {f2:.3f}m, 목표 {pd:.3f}m, 벽 기울기 {t2:+.1f}도, 옆 보정 {self._lat0 * 100:+.1f}cm)"
                if card:
                    return self.go("PUSH", msg + f" -> {self.ko(card)} 키 누르기")
                if mc.get("fine_align", True) and self.world is None and getattr(self, "vw", None) is None:
                    self.say(msg + " -> 모듈 3 으로 mm 맞추기")
                    self._fine_n = 0
                    self.sub, self.sub_t = "FINE", self.clock.now()
                    return
                if not getattr(self, "keytest", None):
                    self.code = self.make_code()
                if getattr(self, "no_press", False):
                    self.speak("비밀번호 입력 위치에 도착했습니다" if mc.get("card_press") else "키보드 앞에 섰습니다")
                    return self.go("DONE", msg + " -> 누르지 않고 끝 (손가락 위치 확인)")
                return self.go("PRESS", msg + f" -> 비밀번호 {self.code}")
            e = math.atan2(math.sin(th - self._final_th), math.cos(th - self._final_th))
            self.car.drive(0.06 if near else 0.20, 0, clamp(-2.5 * e, -0.5, 0.5))
            return

        if sub == "FINE":                              # 손목 아래 모듈 3: 키보드 뒷면 USB 포트로 mm 맞추기 (최대 4번)
            self.car.stop()
            import finealign
            img = finealign.capture()
            fwd, side, ok = finealign.offset(img) if img is not None else (0, 0, False)
            self._fine_n += 1
            if not ok:
                self.say("모듈 3 에서 키보드 포트를 못 찾음 -> 라이다 위치로 누르기")
                return self.fine_done("포트 못 찾음")
            self.say(f"모듈 3: 앞으로 {fwd:+.1f}mm, 옆 {side:+.1f}mm ({self._fine_n}번째)")
            if (abs(fwd) < 4 and abs(side) < 4) or self._fine_n > 4:
                return self.fine_done(f"앞뒤 {fwd:+.1f}mm, 옆 {side:+.1f}mm")
            # 포트가 사진 y+ 로 밀려 보이면(side > 0) 키보드가 로봇 왼쪽에 있음 -> 왼쪽(+)으로
            self._nudge = (fwd / 1000.0, side / 1000.0)
            self._nudge_from = (x, y, th)
            self.sub, self.sub_t = "NUDGE", self.clock.now()
            return

        if sub == "NUDGE":                             # 주행거리계로 딱 그만큼 (앞뒤 먼저, 그다음 옆)
            x0, y0, th0 = self._nudge_from
            c, s_ = math.cos(th0), math.sin(th0)
            dfx = c * (x - x0) + s_ * (y - y0)
            dfy = -s_ * (x - x0) + c * (y - y0)
            ex, ey = self._nudge[0] - dfx, self._nudge[1] - dfy
            if (abs(ex) < 0.003 and abs(ey) < 0.003) or now - self.sub_t > 4:
                self.car.stop()
                self.clock.sleep(0.4)
                self.sub, self.sub_t = "FINE", self.clock.now()
                return
            vx = math.copysign(0.05, ex) if abs(ex) >= 0.003 else 0.0
            vy = math.copysign(0.06, ey) if abs(ex) < 0.003 and abs(ey) >= 0.003 else 0.0
            self.car.drive(vx, vy, 0.0)
            return

    def fine_done(self, why):
        if not getattr(self, "keytest", None):
            self.code = self.make_code()
        if getattr(self, "no_press", False):
            self.speak("비밀번호 입력 위치에 도착했습니다" if self.cfg["mission"].get("card_press") else "키보드 앞에 섰습니다")
            return self.go("DONE", f"맞추기 끝 ({why}) -> 누르지 않고 끝 (손가락 위치 확인)")
        return self.go("PRESS", f"맞추기 끝 ({why}) -> 비밀번호 {self.code}")

    def is_outside(self, scan, now):
        if self.world is not None:
            return self.world.outside()
        if getattr(self, "vw", None) is not None and self.pose_fn is not None:
            return self.vw.outside(self.pose_fn())
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
        self.say(f"미션 시작: 말한 순서 {[self.ko(x) for x in self.order]}, 규칙 {self.cfg['mission']['code_rule']}, 벽 {self.side}손")
        if self.order and not getattr(self, "keytest", None) and not getattr(self, "approach", False):
            self.speak("네, " + ", ".join(self.ko(x) for x in self.order) + " 순서대로 찾겠습니다")
        elif self.link is not None and not self.order:
            self.state = "WAIT_ORDER"
            self.say("PC 스테이션에서 찾을 동물 순서를 기다립니다")
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
                if self.odo is not None and scan is not getattr(self, "_last_scan", None):
                    now = self.clock.now()
                    dt_ = now - getattr(self, "_odo_t", now)
                    self._odo_t, self._last_scan = now, scan
                    o = getattr(self.car, "out", self.car.cmd)
                    # 11/28 1시: 실제 회전이 명령보다 약 1.3배 빠름 (기록: 4.8초에 실제 389도, 계산 308도) -> 예측을 맞춤
                    kr = self.cfg["mission"].get("rot_scale", 1.3)
                    pose = self.odo.update(scan, (o[0] * dt_, o[1] * dt_, o[2] * dt_ * kr))
                    if self.state not in ("WAIT_ORDER", "SPIN"):
                        self.relocalize()
                        pose = self.odo.pose
                    if self.scanlog:
                        self.scanlog.write(json.dumps({"t": round(self.t(), 3), "ts": round(time.time(), 3), "cmd": [round(v, 3) for v in self.car.cmd],
                                                       "odo": [round(pose[0], 3), round(pose[1], 3), round(pose[2], 4)],
                                                       "a": [round(p[0], 2) for p in scan.pts if p[1] > 0],
                                                       "d": [round(p[1], 3) for p in scan.pts if p[1] > 0]}) + "\n")
                if self.link is not None and self.link.polled and self.state not in ("WAIT_ORDER", "DONE", "FAIL") and \
                        time.time() - self.link.last_poll > self.cfg["mission"].get("link_timeout_s", 4.0):
                    if not getattr(self, "_paused", False):
                        self.say("PC 스테이션 연결이 끊김 (와이파이?) -> 멈추고 기다림")
                        self._paused = True
                    self.car.stop()
                    self.clock.sleep(0.2)
                    continue
                if getattr(self, "_paused", False):
                    self.say("PC 스테이션 다시 연결됨 -> 계속")
                    self._paused = False
                    self.sub_t = self.clock.now()
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
    if cfg["mission"]["code_rule"] == "spoken":
        dmap = {c["label"]: c["digit"] for c in world.cards}
        code = "".join(dmap[x] for x in cfg["mission"]["fixed_order"])
        door = sim.SimDoor(world, code)
        return world, sim.SimLidar(world), sim.SimCar(world), sim.SimVision(world, cfg["camera"]), sim.SimHand(world, door, cfg), door, sim.SimClock(world)
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
        if cfg["mission"].get("card_press") and cfg["mission"].get("card_touch"):   # 시뮬 손끝은 몸 중심에서 19cm (실제 라이다~손끝 31.5cm)
            import sim as _s
            tip = _s.ROBOT_R + _s.HAND_REACH
            cfg["mission"]["card_press_dist"], cfg["mission"]["card_push_dist"] = tip + 0.03, tip + 0.004
        world, lidar, car, vision, hand, door, clock = build_sim(cfg, a.seed)
        log = Log(f"sim_seed{a.seed}")
        m = Mission(cfg, lidar, car, vision, hand, door, clock, log, world)
        m.pose_fn = lambda: (world.x, world.y, world.th)
        if getattr(a, "vwalls", ""):                     # 시뮬은 정답 좌표(미로 좌표)를 씀 -> 출발점 = start
            from vwalls import VirtualWalls
            d_ = json.load(open(a.vwalls, encoding="utf-8"))
            m.vw = VirtualWalls(d_["walls"], (0, 0, 0), door=d_.get("door"))
            cfg["wall"]["no_strafe"] = True
            m.home = tuple(d_.get("start", (0, 0, 0))[:2])
        res = m.run()
        print("정답 비밀번호", door.code, "/ 말한 순서", [m.ko(x) for x in m.order], "/ 카드 숫자", m.digits)
        import sim
        png = log.path.replace(".jsonl", ".png")
        sim.draw(world, png, f"seed {a.seed}: {res}, {m.t():.0f}초, 찾은 동물 {[cfg['animals'][x]['ko'] for x in m.found]}, 부딪힘 {world.bumps}")
        if cfg["mission"].get("card_press") and cfg["mission"].get("card_touch"):
            got = getattr(world, "card_pushed", [])
            ok_ = [g[0] for g in got if g[1] < 0.06] == m.order
            print(f"카드 키: 말한 순서 {m.order} / 손끝이 닿은 카드 {got} -> {'맞음' if ok_ else '틀림'}")
            if res == "DONE" and not ok_:
                res = "WRONG_KEYS"
        print(f"결과 {res}  시간 {m.t():.0f}초  부딪힘 {world.bumps} {world.bump_tag}  그림 {png}")
        return res, m.t(), world.bumps
    if a.station:
        return cmd_run_station(cfg, a)
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


def cmd_run_station(cfg, a):
    """실제 로봇 + PC 스테이션 (토요일 시연): 카드 찾기/대사/키 확인은 PC, 주행/지도/손은 파이"""
    from drive import NoCar, RaspbotCar
    from hand import NoHand, open_hand
    from heading import ScanOdom
    from lidar import RPLidar
    from link import KeyCheck, Link, RemoteVision
    cfg["mission"]["code_rule"] = "spoken"
    if a.order:
        cfg["mission"]["fixed_order"] = a.order.split(",")
    link = Link(a.link_port)
    lidar = RPLidar(cfg["lidar"])
    lidar.wait_ready()
    car = NoCar() if a.no_motor else RaspbotCar(dict(cfg["drive"], accel=0.5, accel_rot_deg=120))
    vision = RemoteVision(cfg["camera"], link)
    hand = NoHand() if (a.no_hand or cfg["mission"]["press_mode"] == "virtual") else open_hand(cfg["hand"])
    log = Log("station")
    try:
        hand.open()                              # 손가락 펴기 (누를 준비)
    except Exception as e:
        print("손 펴기 실패:", e)
    m = Mission(cfg, lidar, car, vision, hand, KeyCheck(link), RealClock(), log)
    m.link, link.mission = link, m
    if a.vwalls:
        from vwalls import VirtualWalls
        m.vw = VirtualWalls.load(a.vwalls)
        cfg["wall"]["no_strafe"] = True
        print(f"테이프 미로(가상 벽) {a.vwalls}: 벽 {len(m.vw.segs)}개 (옆으로 미끄러지기 끔)")
    if a.stage:                                  # 11/28: 단계별 시연
        cfg["mission"]["stage"] = a.stage
        if a.find_cards:
            cfg["mission"]["find_cards"] = [x for x in a.find_cards.split(",") if x]
        cfg["mission"]["card_press"] = True
        cfg["wall"]["no_strafe"] = True
        if a.stage in ("calib", "find"):
            m.order = ["_"]                          # 순서 안 받음
            m.state, m.sub_t = "SPIN", m.clock.now()
            m.approach = True
            m.speak("1단계 보정을 시작합니다" if a.stage == "calib" else "3단계, 동물 카드와 탈출 위치를 찾겠습니다")
        else:
            m._stage_load = True                     # 주행거리계를 만든 뒤에 불러오기 (아래)
            if a.stage == "exit":
                m.order, m.approach = ["_"], True
                m.no_press, m.card_target = True, "__kp"
                m.state, m.sub_t = "GOTO_KP", m.clock.now()
                m.speak("5단계, 탈출구로 가겠습니다")
    if a.approach_test:                          # 11/28 0시: '카드 앞에 서기' 만 시험 (로봇을 카드 60cm 앞에, 대충 카드 쪽 보게)
        lab_ = a.approach_test.strip()
        m.order, m.press_i, m.no_press = ([lab_] if lab_ != "marker" else ["_"]), 0, True
        cfg["mission"]["card_press"], cfg["mission"]["end_keypad"] = True, False
        cfg["wall"]["no_strafe"] = True
        m.card_target = "__kp" if lab_ == "marker" else lab_
        m.card_n = (0.0, 0.0)
        m.state, m.sub_t = "CARD_GO", m.clock.now()
        m.approach, m.pressed_seq, m.card_tries = True, [], 0
        m.speak(("2번 마커" if lab_ == "marker" else m.ko(lab_)) + " 앞에 서기 시험을 시작합니다")
    if a.keytest:                                # 키보드 시험: 탐색 없이 바로 키보드 찾기
        m.keytest, m.code, m.no_press = a.keytest, a.keytest, a.no_press
        m.order = ["_"]
        m.state, m.sub_t = "FIND_KEYPAD", m.clock.now()
    # 11/28 1시: 라이다가 막 켜지면 첫 사진이 점 2개뿐 -> 기준/주행거리계가 망가짐 (맞춤 0.00). 꽉 찬 사진을 기다림
    t_w = time.time()
    while time.time() - t_w < 5 and (lidar.latest() is None or len(lidar.latest().pts) < 150):
        time.sleep(0.1)
    m.odo = ScanOdom(lidar.latest())
    m.scan_first = lidar.latest()                    # 출발 자리 라이다 사진 (한 바퀴 보기 뒤 방향 다시 맞추기)
    m.pose_fn = lambda: m.odo.pose
    m.car = TurnFlip(car, m.pose_fn, m.clock, m.say)      # 제자리 회전 최소 45도/초
    if getattr(m, "_stage_load", False):                  # 4, 5단계: 3단계 기억 불러와서 '지금 어디' 맞추기
        m.load_state()
        m.relocalize(force=True)
        from frontier import Grid
        m.grid = Grid()
    m.scanlog = open(log.path.replace("station_", "station_scans_"), "w", encoding="utf-8")
    if getattr(m, "vw", None) is not None:                # 테이프 네모: 지도 그림에 겹쳐 그리게 기록
        m.vw_json = json.load(open(a.vwalls, encoding="utf-8"))
        m.scanlog.write(json.dumps({"vw": {"rebase": [0, 0, 0], "walls": m.vw_json}}, ensure_ascii=False) + chr(10))
    import signal

    def _bail(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGHUP, _bail)          # ssh 가 끊겨도 바퀴 멈춤 (finally 에서 stop)
    signal.signal(signal.SIGTERM, _bail)
    if a.virtual_press:
        cfg["mission"]["press_mode"] = "virtual"
    try:
        res = m.run()
    except KeyboardInterrupt:
        res = "중지"
    finally:
        car.stop()
        hand.close()
        lidar.close()
        m.scanlog.close()
    print("결과:", res, " 기록:", log.path, " 지도용:", m.scanlog.name)
    print("카드 위치(주행거리계 기준):", {m.ko(k): (round(v[0], 2), round(v[1], 2), m.digits.get(k)) for k, v in m.card_pos.items()})


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
    ap.add_argument("--station", action="store_true", help="PC 스테이션과 같이 (pc_station.py)")
    ap.add_argument("--order", default="", help="말한 순서 (음성 대신): giraffe,dog,elephant")
    ap.add_argument("--link-port", type=int, default=8093)
    ap.add_argument("--keytest", default="", help="키보드 시험: 이 번호를 누름 (예 1234). 탐색 없이 마커 찾기부터")
    ap.add_argument("--no-press", action="store_true", help="키보드 앞에 서기만 하고 누르지 않음")
    ap.add_argument("--vwalls", default="", help="테이프 미로 가상 벽 파일 (vwalls.json)")
    ap.add_argument("--find-cards", default="", help="3단계: 놓아 둔 카드 (정답 목록) dog,rabbit,... 다 찾으면 끝")
    ap.add_argument("--stage", default="", help="calib(1단계 보정) / find(3단계 카드+탈출 위치 기억) / visit(4단계 말한 동물 찾아가기) / exit(5단계 탈출)")
    ap.add_argument("--approach-test", default="", help="카드 앞에 서기만 시험: dog/rabbit/elephant... 또는 marker (2번 마커)")
    ap.add_argument("--card-press", action="store_true", help="11/27: 카드마다 벽에 붙은 키를 순서대로 누르기 (출발점 복귀 -> 순서 말하기 -> 카드로)")
    a = ap.parse_args()
    cfg = load_config(a.config)
    if a.mode == "simbatch":
        a.sim = True
    if a.card_press:
        cfg["mission"]["card_press"] = True
    {"run": cmd_run, "simbatch": cmd_simbatch, "lidar": cmd_lidar, "calib-rot": cmd_calib_rot,
     "wall": cmd_wall, "see": cmd_see, "keys": cmd_keys}[a.mode](cfg, a)


if __name__ == "__main__":
    main()
