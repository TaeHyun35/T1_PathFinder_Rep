"""문 스테이션 (노트북에서 실행). 무선키보드 수신기를 이 노트북에 꽂는다.

하는 일:
  - 이 창에 들어오는 숫자 키를 모아서 정답 비밀번호와 비교
  - 맞으면 피코 WH 에 "열어" 신호 -> 서보가 문을 연다
  - 로봇이 상태를 물어볼 수 있게 작은 서버를 연다 (GET /status, POST /code = 손 없이 시험용)

실행:
  python door_station.py --code 124 --pico http://192.168.0.50
  python door_station.py --code 124 --no-pico        (피코 없이 화면으로만)
키: 숫자 = 입력, Backspace = 지우기, r = 다시 닫기(초기화), q = 끝
입력할 때 이 창이 앞에 있어야 한다 (다른 창을 누르면 키가 그쪽으로 감).
"""
import argparse
import json
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class Door:
    def __init__(self, code, pico):
        self.code, self.pico, self.buf, self.state = code, pico, "", "closed"
        self.lock = threading.Lock()
        self.log = []

    def pico_cmd(self, cmd):
        if not self.pico:
            return "피코 없음"
        try:
            with urllib.request.urlopen(f"{self.pico.rstrip('/')}/{cmd}", timeout=3) as r:
                return r.read().decode(errors="replace")
        except Exception as e:
            return f"피코 연결 실패: {e}"

    def key(self, ch, src="키보드"):
        with self.lock:
            if self.state == "open":
                return
            self.buf += ch
            self.log.append((time.strftime("%H:%M:%S"), src, ch))
            print(f"\r입력: {self.buf:<8}", end="", flush=True)
            if len(self.buf) >= len(self.code):
                if self.buf == self.code:
                    self.state = "open"
                    print(f"\n정답 {self.buf} -> 문 열기: {self.pico_cmd('open')}")
                else:
                    self.state = "wrong"
                    print(f"\n틀림 ({self.buf}, 정답 {len(self.code)}자리) -> 다시 입력")
                self.buf = ""

    def status(self):
        with self.lock:
            s = self.state
            if s == "wrong":
                self.state = "closed"
            return s

    def reset(self):
        with self.lock:
            self.state, self.buf = "closed", ""
        print(f"\n초기화 -> 문 닫기: {self.pico_cmd('close')}")


def serve(door, port):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, obj):
            b = json.dumps(obj, ensure_ascii=False).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path.startswith("/status"):
                self._send({"state": door.status()})
            else:
                self._send({"ok": True})

        def do_POST(self):
            if self.path.startswith("/code"):
                n = int(self.headers.get("Content-Length", 0))
                code = self.rfile.read(n).decode().strip()
                for ch in code:
                    door.key(ch, "로봇(가상)")
                self._send({"state": door.state})
    ThreadingHTTPServer(("0.0.0.0", port), H).serve_forever()


def read_keys():
    """키 하나씩 읽기 (윈도우 / 리눅스)"""
    try:
        import msvcrt
        while True:
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):          # 방향키 등
                msvcrt.getwch()
                continue
            yield ch
    except ImportError:
        import termios
        import tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while True:
                yield sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--code", required=True, help="정답 비밀번호 (예: 124)")
    ap.add_argument("--pico", default="http://192.168.0.50", help="피코 WH 주소")
    ap.add_argument("--no-pico", action="store_true")
    ap.add_argument("--port", type=int, default=8095)
    a = ap.parse_args()
    door = Door(a.code, None if a.no_pico else a.pico)
    threading.Thread(target=serve, args=(door, a.port), daemon=True).start()
    print(f"문 스테이션 시작. 정답 {len(a.code)}자리. 로봇 설정 door.station_url = http://<이 노트북 IP>:{a.port}")
    if not a.no_pico:
        print("피코 확인:", door.pico_cmd("status"))
    print("숫자를 누르세요 (r 초기화, q 끝)")
    for ch in read_keys():
        if ch in ("q", "Q"):
            break
        if ch in ("r", "R"):
            door.reset()
        elif ch in ("\b", "\x7f"):
            door.buf = door.buf[:-1]
            print(f"\r입력: {door.buf:<8}", end="", flush=True)
        elif ch.isdigit():
            door.key(ch)


if __name__ == "__main__":
    main()
