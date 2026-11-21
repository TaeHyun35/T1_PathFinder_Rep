"""공통: 설정 불러오기, 시간(실제/시뮬), 기록."""
import copy
import json
import math
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))

# 모든 기본값. 현장에서는 config.json 에 바꿀 값만 적으면 덮어쓴다.
DEFAULTS = {
    "lidar": {
        "port": "",                 # 비우면 자동 (/dev/rplidar, /dev/ttyUSB*)
        "front_offset_deg": 0.0,    # 라이다 0도가 로봇 앞에서 몇 도 돌아가 있는지 (lidar_view 로 확인)
        "clockwise": True,          # RPLIDAR 원본은 시계 방향
        "min_range": 0.12,          # 이보다 가까운 점은 로봇 몸/손으로 보고 버림
        "ignore": [],               # 손에 가리는 각도 [[시작도, 끝도], ...] 로봇 기준 (앞 0, 왼쪽 +)
        "motor_invert": False,
    },
    "drive": {
        "motor": {"FL": 0, "RL": 1, "FR": 2, "RR": 3},
        "invert": {"FL": False, "RL": False, "FR": False, "RR": False},
        "flip_side": False,
        "power_ref": 120,           # 아래 속도를 잰 바퀴 세기 (car.py calib 와 같음)
        "speed_fwd": 0.30,          # power_ref 일 때 앞으로 m/s
        "speed_side": 0.22,         # power_ref 일 때 옆으로 m/s
        "speed_rot_deg": 120.0,     # power_ref 일 때 제자리 회전 도/초 (mission.py calib-rot 로 잼)
        "power_max": 160,
    },
    "wall": {                       # 벽 따라가기 (라이다)
        "side": "right",            # right = 오른손 법칙
        "skew_deg": 30.0,           # 몸(카메라)을 따라가는 벽 쪽으로 돌린 각도. 메카넘이라 비스듬히 간다 (시뮬: 0도 18/20, 30도 20/20)
        "d_ref": 0.32,              # 옆 벽과 유지할 거리 (m). 복도 폭의 절반 정도
        "front_stop": 0.40,         # 앞 벽이 이보다 가까우면 제자리 회전
        "open": 0.70,               # 옆 거리가 이보다 멀면 "옆 길이 열림" -> 그쪽으로 돌기
        "v": 0.15,                  # 직진 속도 m/s
        "v_arc": 0.10,              # 모퉁이 돌 때 속도
        "w_arc_deg": 18.0,          # 모퉁이 돌 때 회전 도/초 (반지름 = v_arc / w)
        "w_turn_deg": 35.0,         # 막혔을 때 제자리 회전 도/초
        "ky": 1.2,                  # 벽 거리 오차 -> 옆으로 미는 세기
        "kw": 1.6,                  # 벽 기울기 오차 -> 회전 세기
    },
    "look": {"every_s": 7.0, "angle_deg": 75.0, "hold_s": 1.0, "w_deg": 40.0},  # 주기적으로 옆 벽 보기
    "camera": {"source": "usb", "index": 0, "width": 640, "height": 480, "fov_deg": 48.0},
    "detect": {
        "mode": "local",            # local = 파이에서 YOLO / remote = 노트북 vision_server.py
        "model": "yolo11n.pt",       # 처음 실행 때 자동으로 받음 (약 6MB)
        "url": "http://192.168.0.10:8090/detect",
        "conf": 0.45,
        "confirm": 3,               # 같은 동물을 이만큼 보면 확정
        "every_s": 0.4,
    },
    "animals": {                    # YOLO(COCO) 이름 -> 한국어, 누를 숫자
        "cat": {"ko": "고양이", "digit": "1"},
        "dog": {"ko": "강아지", "digit": "2"},
        "bird": {"ko": "새", "digit": "3"},
        "horse": {"ko": "말", "digit": "4"},
    },
    "mission": {
        "expect": 3,                # 찾아야 할 동물 수
        "code_rule": "sorted",      # sorted = 숫자 작은 순 (문 스테이션이 정답을 미리 알 수 있음) / found_order = 찾은 순서 / fixed = fixed_order 순
        "fixed_order": [],          # 예: ["dog", "cat", "bird"] (음성으로 받은 순서)
        "keypad_marker": 0,         # 키패드 위에 붙인 ArUco 번호
        "door_marker": 1,           # 출구 문에 붙인 ArUco 번호
        "press_dist": 0.22,         # 키패드 벽까지 이 거리(라이다 중심 기준)에서 누름 -> 현장에서 잼
        "align_tol": 0.04,          # 마커 가운데 맞춤 허용 (화면 폭 비율)
        "cam_offset": 0.0,          # 손가락이 카메라보다 오른쪽이면 + (화면 폭 비율)
        "exit_open_m": 1.2,         # 앞쪽이 이만큼 비면 미로 밖으로 나간 것
        "timeout_s": 900,
        "press_mode": "hand",       # hand = 손으로 누름 / virtual = 문 스테이션에 숫자를 바로 보냄 (손 없이 시험)
    },
    "hand": {
        "port": "",
        "open": [50, 130, 130, 130, 130],
        "wrist": 90,
        "key_finger": {"1": 2, "2": 3, "3": 4, "4": 5},   # 숫자 -> 손가락 (2검지 3중지 4약지 5새끼)
        "press_angle": {"2": 70, "3": 70, "4": 70, "5": 70},  # 이 각도까지 굽히면 키가 눌림 (mission.py keys --find 로 찾음)
        "hold_s": 0.35,
        "move_ms": 300,
    },
    "door": {"station_url": "http://192.168.0.10:8095"},
}


def deep_merge(base, over):
    out = copy.deepcopy(base)
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_config(path=None):
    path = path or os.path.join(HERE, "config.json")
    cfg = DEFAULTS
    if os.path.exists(path):
        cfg = deep_merge(DEFAULTS, json.load(open(path, encoding="utf-8")))
    return cfg


def save_config_patch(patch, path=None):
    """config.json 에 일부 값만 덮어써서 저장"""
    path = path or os.path.join(HERE, "config.json")
    cur = json.load(open(path, encoding="utf-8")) if os.path.exists(path) else {}
    json.dump(deep_merge(cur, patch), open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)


class RealClock:
    def now(self):
        return time.time()

    def sleep(self, s):
        time.sleep(max(0.0, s))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Log:
    def __init__(self, name):
        os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
        self.path = os.path.join(HERE, "logs", time.strftime(f"{name}_%Y%m%d_%H%M%S.jsonl"))
        self.f = open(self.path, "w", encoding="utf-8")

    def write(self, **kw):
        self.f.write(json.dumps(kw, ensure_ascii=False) + "\n")
        self.f.flush()

    def say(self, t, msg):
        print(f"[{t:7.1f}s] {msg}", flush=True)
        self.write(t=round(t, 2), msg=msg)


RAD = math.pi / 180.0
