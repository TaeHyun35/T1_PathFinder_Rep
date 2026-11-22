"""AiHand (micro:bit USB 시리얼). 팀 hand.py 와 같은 줄 형식:
"엄지,검지,중지,약지,새끼,손목,움직임ms\n"  -> micro:bit 가 한 줄 응답.
micro:bit 에는 microbit-serial-range.hex 가 올라가 있어야 한다.
"""
import sys
import time


def find_microbit():
    from serial.tools import list_ports
    for p in list_ports.comports():
        t = f"{p.description} {p.manufacturer or ''} {p.hwid}".lower()
        if "mbed" in t or "0d28" in t or "micro:bit" in t or "microbit" in t:
            return p.device
    return None


class MicrobitHand:
    def __init__(self, cfg):
        import serial
        self.cfg = cfg
        port = cfg["port"] or find_microbit()
        if not port:
            sys.exit("micro:bit 포트를 못 찾음. config 의 hand.port 에 적기 (예: /dev/ttyACM0)")
        self.ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(0.5)
        self.fingers, self.wrist = list(cfg["open"]), cfg["wrist"]
        self.send(self.fingers, self.wrist)

    def send(self, fingers=None, wrist=None, move=None):
        if fingers is not None:
            self.fingers = list(fingers)
        if wrist is not None:
            self.wrist = int(wrist)
        move = move or self.cfg["move_ms"]
        line = ",".join(str(int(a)) for a in self.fingers) + f",{self.wrist},{int(move)}"
        self.ser.reset_input_buffer()
        self.ser.write((line + "\n").encode())
        return self.ser.readline().decode(errors="replace").strip()

    def open(self):
        self.send(self.cfg["open"])

    def press(self, finger, angle=None, sleep=time.sleep):
        """손가락 하나를 눌림 각도까지 굽혔다가 편다. finger: 2~5"""
        angle = angle if angle is not None else self.cfg["press_angle"][str(finger)]
        pose = list(self.cfg["open"])
        pose[finger - 1] = angle
        self.send(pose)
        sleep(self.cfg["move_ms"] / 1000 + self.cfg["hold_s"])
        self.send(self.cfg["open"])
        sleep(self.cfg["move_ms"] / 1000 + 0.15)

    def close(self):
        try:
            self.open()
            self.ser.close()
        except Exception:
            pass


class NoHand:
    def __init__(self, cfg=None):
        self.pressed = []

    def open(self):
        pass

    def press(self, finger, angle=None, sleep=time.sleep):
        self.pressed.append(finger)
        print(f"  (손 없음) 손가락 {finger} 누름")

    def close(self):
        pass


def open_hand(cfg):
    """config hand.ble = true -> Bluetooth micro:bit (ble_hand.py), otherwise USB."""
    if cfg.get("ble"):
        from ble_hand import MicrobitBleHand
        return MicrobitBleHand(cfg)
    return MicrobitHand(cfg)
