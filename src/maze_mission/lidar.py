"""라이다 (RPLIDAR A1M8) 직접 읽기 + 방향별 거리 계산.

로봇 기준 각도: 앞 0도, 왼쪽 +90, 오른쪽 -90 (반시계 +).
pyserial 이 없으면 리눅스 기본 termios 로 직접 연결한다.
"""
import glob
import math
import os
import struct
import threading
import time

from common import wrap_deg


class Scan:
    """한 바퀴 점들. pts = [(로봇기준 각도deg, 거리m)]"""

    def __init__(self, pts, t):
        self.pts, self.t = pts, t

    # 응답 없음(거리 0)은 "아주 멂"이 아니라 "모름"이다. 손이 라이다 앞을 가리면 응답이 없어진다.
    # 모를 때는 nan 을 돌려주고, 부르는 쪽은 nan 을 "막힘"으로 다룬다 (mission.py 의 blocked()).
    NEAR = 2.0          # 이보다 가까운 점이 하나라도 있으면 응답률과 상관없이 그 거리를 믿는다
    NEED = 0.30         # 부채꼴 안 점 중 응답이 이 비율보다 적으면 "모름"

    def coverage(self, center, half):
        tot = ok = 0
        for a, d in self.pts:
            if abs(wrap_deg(a - center)) <= half:
                tot += 1
                ok += d > 0
        return ok / tot if tot else 0.0

    def sector_min(self, center, half):
        """부채꼴 안 가장 가까운 거리. 모르면 nan"""
        tot, ds = 0, []
        for a, d in self.pts:
            if abs(wrap_deg(a - center)) <= half:
                tot += 1
                if d > 0:
                    ds.append(d)
        if ds and min(ds) < self.NEAR:
            return min(ds)
        if not tot or len(ds) / tot < self.NEED:
            return math.nan
        return min(ds)

    def ray(self, center, half=3.0):
        """한 방향의 거리 (작은 부채꼴 안 중간값). 응답이 하나도 없으면 nan"""
        ds = sorted(d for a, d in self.pts if d > 0 and abs(wrap_deg(a - center)) <= half)
        return ds[len(ds) // 2] if ds else math.nan


def to_robot_frame(raw_deg, cfg):
    a = raw_deg - cfg["front_offset_deg"]
    return wrap_deg(-a if cfg["clockwise"] else a)


def keep_point(a, d, cfg):
    if d < cfg["min_range"]:
        return False
    for lo, hi in cfg["ignore"]:
        if lo <= a <= hi:
            return False
    return True


# ---------------- 시리얼 ----------------
class TermiosSerial:
    def __init__(self, port):
        import fcntl
        import termios
        self.fcntl, self.termios = fcntl, termios
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200
        a[6][termios.VMIN] = 0
        a[6][termios.VTIME] = 5
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def read(self, n):
        try:
            return os.read(self.fd, n)
        except BlockingIOError:
            return b""

    def write(self, b):
        os.write(self.fd, b)

    def reset_input_buffer(self):
        self.termios.tcflush(self.fd, self.termios.TCIFLUSH)

    def setDTR(self, on):
        req = self.termios.TIOCMBIS if on else self.termios.TIOCMBIC
        self.fcntl.ioctl(self.fd, req, struct.pack("I", self.termios.TIOCM_DTR))

    def close(self):
        os.close(self.fd)


def open_serial(port):
    try:
        import serial
        return serial.Serial(port, 115200, timeout=0.5)
    except ImportError:
        return TermiosSerial(port)


def find_port():
    for p in ["/dev/rplidar"] + sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*")):
        if os.path.exists(p):
            return p
    return None


def read_exact(ser, n, timeout=2.0):
    buf, t = b"", time.time()
    while len(buf) < n and time.time() - t < timeout:
        buf += ser.read(n - len(buf))
    return buf


def parse_node(b):
    s, ns = b[0] & 1, (b[0] >> 1) & 1
    if s == ns or not (b[1] & 1):
        return None
    ang = ((b[1] >> 1) | (b[2] << 7)) / 64.0
    dist = (b[3] | (b[4] << 8)) / 4000.0
    return bool(s), ang % 360.0, dist


class RPLidar:
    """백그라운드에서 계속 읽고, latest() 로 마지막 한 바퀴를 준다"""

    def __init__(self, cfg):
        self.cfg = cfg
        self.port = cfg["port"] or find_port()
        if not self.port:
            raise RuntimeError("라이다 포트를 못 찾음 (ls /dev/ttyUSB* 로 확인 후 config 의 lidar.port 에 적기)")
        self._scan, self.err, self.hz = None, "", 0.0
        self._stop = False
        self._lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def latest(self):
        with self._lock:
            return self._scan

    def wait_ready(self, timeout=8.0):
        t = time.time()
        while time.time() - t < timeout:
            if self.latest() is not None:
                return True
            time.sleep(0.1)
        raise RuntimeError(f"라이다 데이터가 안 들어옴 ({self.err or '응답 없음'}). 다른 프로그램이 라이다를 쓰는지 확인")

    def close(self):
        self._stop = True

    def _run(self):
        inv = self.cfg["motor_invert"]
        while not self._stop:
            ser = None
            try:
                ser = open_serial(self.port)
                ser.setDTR(inv)
                time.sleep(1.2)
                ser.reset_input_buffer()
                ser.write(b"\xA5\x20")
                if len(read_exact(ser, 7)) != 7:
                    raise RuntimeError("측정 시작 응답 없음")
                cur, revs = [], []
                while not self._stop:
                    b = read_exact(ser, 5, 1.0)
                    if len(b) < 5:
                        continue
                    n = parse_node(b)
                    if n is None:
                        ser.read(1)
                        continue
                    new, raw, d = n
                    if new and cur:
                        now = time.time()
                        revs = (revs + [now])[-10:]
                        self.hz = (len(revs) - 1) / (revs[-1] - revs[0]) if len(revs) > 2 else 0.0
                        with self._lock:
                            self._scan = Scan(cur, now)
                        cur = []
                    a = to_robot_frame(raw, self.cfg)
                    cur.append((a, d if (d > 0 and keep_point(a, d, self.cfg)) else 0.0))
            except Exception as e:
                self.err = f"{type(e).__name__}: {e}"
                time.sleep(1.5)
            finally:
                if ser:
                    try:
                        ser.write(b"\xA5\x25")
                        ser.setDTR(not inv)
                        ser.close()
                    except Exception:
                        pass
