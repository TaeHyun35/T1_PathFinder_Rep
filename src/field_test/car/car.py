"""라즈봇 V2 바퀴 조종 (라즈베리파이에서 실행).

쓰는 법 (파이 터미널, SSH로 접속해서):
  python3 car.py wheels                 바퀴 4개를 하나씩 1초씩 앞으로 돌림 (번호/방향 확인)
  python3 car.py teleop                 키보드로 조종 (넘어짐 테스트용)
  python3 car.py run fwd 120 1.0        정해진 세기로 정해진 시간만큼 가다가 바로 멈춤 (반복 시험용)
                                        방향: fwd back left right rotl rotr
  python3 car.py calib fwd              속도 재기: 2초 가고, 잰 거리(cm)를 입력하면 저장
  python3 car.py calib left             옆으로 가는 속도 재기
  python3 car.py server                 노트북(VR)에서 "여기로 가" 명령을 받는 서버 (포트 8800)

모든 움직임은 정해진 시간이 지나면 저절로 멈춘다. 급할 때는 Ctrl+C.
"""
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
CFG_FILE = os.path.join(HERE, "car_config.json")

# ---------------------------------------------------------------- 설정 (현장에서 바꿀 수 있음)
CFG = {
    # 바퀴 번호: Raspbot_Lib의 motor_id. wheels 테스트로 확인해서 고친다.
    "motor": {"FL": 0, "RL": 1, "FR": 2, "RR": 3},   # 앞왼쪽, 뒤왼쪽, 앞오른쪽, 뒤오른쪽
    "invert": {"FL": False, "RL": False, "FR": False, "RR": False},  # 앞으로 줬는데 뒤로 돌면 True
    "power": 120,          # 자동 이동(server, calib) 때 바퀴 세기 (0~255)
    "speed_fwd": 0.30,     # power일 때 앞으로 가는 속도 (m/s). calib fwd로 잰다
    "speed_side": 0.22,    # power일 때 옆으로 가는 속도 (m/s). calib left로 잰다
    "max_move": 2.0,       # 한 번에 이만큼(m)보다 멀면 거부
    "flip_side": False,    # teleop에서 a(왼쪽옆)를 눌렀는데 오른쪽으로 가면 True
}
if os.path.exists(CFG_FILE):
    CFG.update(json.load(open(CFG_FILE, encoding="utf-8")))


def save_cfg():
    json.dump(CFG, open(CFG_FILE, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("저장:", CFG_FILE)


# ---------------------------------------------------------------- 바퀴
class Car:
    def __init__(self):
        try:
            import Raspbot_Lib
            self.bot = Raspbot_Lib.Raspbot()
        except Exception as e:
            sys.exit(f"Raspbot_Lib를 못 불러왔습니다 ({e}). Yahboom 드라이버 라이브러리가 설치된 파이에서 실행하세요.")
        self.lock = threading.Lock()

    def wheel(self, name, speed):
        speed = max(-255, min(255, int(speed)))
        if CFG["invert"][name]:
            speed = -speed
        mid = CFG["motor"][name]
        if hasattr(self.bot, "Ctrl_Muto"):
            self.bot.Ctrl_Muto(mid, speed)
        else:  # 옛 버전: Ctrl_Car(id, 방향 0앞/1뒤, 세기)
            self.bot.Ctrl_Car(mid, 0 if speed >= 0 else 1, abs(speed))

    def drive(self, vx, vy, w=0):
        """vx 앞(+), vy 왼쪽(+), w 왼쪽으로 돌기(+). 값은 바퀴 세기 단위."""
        if CFG.get("flip_side"):
            vy = -vy
        fl, rl = vx - vy - w, vx + vy - w
        fr, rr = vx + vy + w, vx - vy + w
        m = max(255, abs(fl), abs(rl), abs(fr), abs(rr))
        k = 255 / m
        for n, s in (("FL", fl), ("RL", rl), ("FR", fr), ("RR", rr)):
            self.wheel(n, s * k)

    def stop(self):
        for n in ("FL", "RL", "FR", "RR"):
            self.wheel(n, 0)

    def timed(self, vx, vy, w, sec):
        with self.lock:
            try:
                self.drive(vx, vy, w)
                time.sleep(max(0.0, sec))
            finally:
                self.stop()


DIRS = {"fwd": (1, 0, 0), "back": (-1, 0, 0), "left": (0, 1, 0), "right": (0, -1, 0),
        "rotl": (0, 0, 1), "rotr": (0, 0, -1)}


# ---------------------------------------------------------------- 모드
def mode_wheels(car):
    for n in ("FL", "RL", "FR", "RR"):
        print(f"{n} (motor_id {CFG['motor'][n]}) 앞으로 1초 ... 어느 바퀴가 어느 쪽으로 도는지 보세요")
        car.wheel(n, 100)
        time.sleep(1.0)
        car.wheel(n, 0)
        time.sleep(0.8)
    print("다르면 car.py 위쪽 CFG의 motor / invert를 고치세요.")


def mode_run(car, d, power, sec):
    vx, vy, w = (c * power for c in DIRS[d])
    print(f"{d} 세기 {power} {sec}초 -> 바로 멈춤")
    t0 = time.time()
    car.timed(vx, vy, w, sec)
    print(f"끝 ({time.time() - t0:.2f}초)")


def mode_calib(car, d):
    sec = 2.0
    p = CFG["power"]
    input(f"{d} 방향으로 세기 {p}, {sec}초 갑니다. 출발선에 맞추고 Enter ")
    vx, vy, _ = (c * p for c in DIRS[d])
    car.timed(vx, vy, 0, sec)
    cm = float(input("간 거리(cm)를 줄자로 재서 입력: "))
    v = cm / 100 / sec
    key = "speed_fwd" if d in ("fwd", "back") else "speed_side"
    CFG[key] = round(v, 4)
    print(f"{key} = {v:.3f} m/s")
    save_cfg()


def mode_teleop(car):
    import termios
    import tty
    print(__doc__.split("\n")[0])
    print("w 앞  s 뒤  a 왼쪽옆  d 오른쪽옆  q 왼쪽돌기  e 오른쪽돌기  스페이스 멈춤")
    print("1/2/3 세기 (80/150/255)   x 종료.  키를 누르고 있는 동안만 움직이고, 떼면 0.3초 뒤 멈춤")
    levels = {"1": 80, "2": 150, "3": 255}
    power = 80
    keymap = {"w": "fwd", "s": "back", "a": "left", "d": "right", "q": "rotl", "e": "rotr"}
    state = {"cmd": None, "until": 0.0, "run": True}

    def loop():
        cur = None
        while state["run"]:
            now = time.time()
            want = state["cmd"] if now < state["until"] else None
            if want != cur:
                if want is None:
                    car.stop()
                else:
                    vx, vy, w = (c * power for c in DIRS[want])
                    car.drive(vx, vy, w)
                cur = want
            time.sleep(0.02)
        car.stop()

    th = threading.Thread(target=loop, daemon=True)
    th.start()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            k = sys.stdin.read(1).lower()
            if k == "x":
                break
            if k in levels:
                power = levels[k]
                print(f"\r세기 {power}      ", end="", flush=True)
            elif k == " ":
                state["until"] = 0
            elif k in keymap:
                state["cmd"], state["until"] = keymap[k], time.time() + 0.3
    finally:
        state["run"] = False
        th.join(1)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        car.stop()
        print("\n종료")


def mode_server(car, port=8800):
    busy = threading.Lock()

    class H(BaseHTTPRequestHandler):
        def reply(self, code, obj):
            b = json.dumps(obj, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path.startswith("/stop"):
                car.stop()
                return self.reply(200, {"ok": True})
            self.reply(200, {"ok": True, "cfg": CFG})

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            d = json.loads(self.rfile.read(n) or b"{}")
            if self.path.startswith("/goto"):
                fwd, left = float(d.get("fwd", 0)), float(d.get("left", 0))
                if abs(fwd) > CFG["max_move"] or abs(left) > CFG["max_move"]:
                    return self.reply(400, {"ok": False, "err": f"너무 멀어요 ({fwd:.2f}, {left:.2f} m)"})
                if not busy.acquire(blocking=False):
                    return self.reply(409, {"ok": False, "err": "이미 움직이는 중"})
                try:
                    p = CFG["power"]
                    t1 = abs(fwd) / CFG["speed_fwd"]
                    t2 = abs(left) / CFG["speed_side"]
                    print(f"goto 앞 {fwd * 100:+.0f}cm ({t1:.2f}s), 왼쪽 {left * 100:+.0f}cm ({t2:.2f}s)", flush=True)
                    if t1 > 0.05:
                        car.timed(p if fwd > 0 else -p, 0, 0, t1)
                        time.sleep(0.3)
                    if t2 > 0.05:
                        car.timed(0, p if left > 0 else -p, 0, t2)
                    return self.reply(200, {"ok": True, "t_fwd": round(t1, 2), "t_side": round(t2, 2)})
                finally:
                    busy.release()
            self.reply(404, {"ok": False})

    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    print(f"차 서버 시작: 포트 {port}. 노트북에서 이 파이의 IP를 vr\\server.py --car <IP> 로 넣으세요. Ctrl+C로 종료")
    print("현재 설정:", json.dumps(CFG, ensure_ascii=False))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        car.stop()


if __name__ == "__main__":
    a = sys.argv[1:]
    if not a:
        sys.exit(__doc__)
    car = Car()
    try:
        if a[0] == "wheels":
            mode_wheels(car)
        elif a[0] == "run":
            mode_run(car, a[1], int(a[2]), float(a[3]))
        elif a[0] == "calib":
            mode_calib(car, a[1] if len(a) > 1 else "fwd")
        elif a[0] == "teleop":
            mode_teleop(car)
        elif a[0] == "server":
            mode_server(car)
        else:
            sys.exit(__doc__)
    finally:
        car.stop()
