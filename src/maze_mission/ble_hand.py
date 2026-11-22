"""AiHand over Bluetooth (micro:bit running robot_hand_ble/main.ts).

Same methods as hand.MicrobitHand (send / open / press / close), so the mission code
can use either. Needs `bleak` (on the Pi: ~/handenv/bin/python).

BLE UART carries 20 bytes per packet, so a pose is sent as short commands:
  s<i>,<deg>  per servo that changed (1 thumb .. 5 little, 6 wrist), then  m<ms>  -> "OK"

Quick test on the Pi (hand open between every press):
  ~/handenv/bin/python ble_hand.py open
  ~/handenv/bin/python ble_hand.py press 2          # index finger once (press_angle from config)
  ~/handenv/bin/python ble_hand.py code 421          # F4 F2 F1 with key_finger from config
  ~/handenv/bin/python ble_hand.py sweep 2          # find the angle where the key clicks
"""
import asyncio
import sys
import threading
import time

TX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # micro:bit -> Pi (notify/indicate)
RX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # Pi -> micro:bit (write)


class MicrobitBleHand:
    def __init__(self, cfg, addr=None, name="BBC micro:bit", timeout=15):
        from bleak import BleakClient, BleakScanner
        self.cfg = cfg
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()
        self.lines, self.buf = [], ""
        self.cond = threading.Condition()

        async def connect():
            addr_ = addr or cfg.get("ble_addr")
            if addr_:
                dev = await BleakScanner.find_device_by_address(addr_, timeout=timeout)
            else:
                dev = await BleakScanner.find_device_by_filter(
                    lambda d, ad: (d.name or "").startswith(name), timeout=timeout)
            if dev is None:
                raise RuntimeError("micro:bit 블루투스를 못 찾음 (전원, 거리, 다른 기기 연결 확인)")
            client = BleakClient(dev)
            await client.connect()
            await client.start_notify(TX, self._on_data)
            return client, dev

        self.client, dev = self._run(connect(), timeout + 10)
        print(f"micro:bit 연결: {dev.name} {dev.address}")
        self.fingers, self.wrist = list(cfg["open"]), cfg["wrist"]
        self.sent = [None] * 6                       # last angles the micro:bit has
        self.open()

    # ---- plumbing ----
    def _run(self, coro, timeout=10):
        return asyncio.run_coroutine_threadsafe(coro, self.loop).result(timeout)

    def _on_data(self, _h, data):
        with self.cond:
            self.buf += data.decode(errors="replace")
            while "\n" in self.buf:
                line, self.buf = self.buf.split("\n", 1)
                self.lines.append(line.strip())
                self.cond.notify_all()

    def _write_line(self, line):
        data = (line + "\n").encode()
        async def w():
            for i in range(0, len(data), 20):
                await self.client.write_gatt_char(RX, data[i:i + 20], response=True)
        self._run(w())

    def _wait_reply(self, timeout=2.0):
        end = time.time() + timeout
        with self.cond:
            while not self.lines and time.time() < end:
                self.cond.wait(end - time.time())
            return self.lines.pop(0) if self.lines else ""

    # ---- same interface as hand.MicrobitHand ----
    def send(self, fingers=None, wrist=None, move=None):
        if fingers is not None:
            self.fingers = list(fingers)
        if wrist is not None:
            self.wrist = int(wrist)
        move = int(move or self.cfg["move_ms"])
        pose = [int(a) for a in self.fingers] + [int(self.wrist)]
        with self.cond:
            self.lines.clear()
        for i, a in enumerate(pose):
            if self.sent[i] != a:
                self._write_line(f"s{i + 1},{a}")
                self.sent[i] = a
        self._write_line(f"m{move}")
        reply = self._wait_reply()
        if reply != "OK":
            self.sent = [None] * 6                   # resend everything next time
        return reply

    def open(self):
        return self.send(self.cfg["open"], self.cfg["wrist"])

    def press(self, finger, angle=None, sleep=time.sleep):
        """open hand -> bend one finger to its press angle -> hold -> open hand again"""
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
            self._run(self.client.disconnect())
        except Exception:
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)


def main():
    from common import load_config, save_config_patch
    cfg = load_config()["hand"]
    names = {1: "엄지", 2: "검지", 3: "중지", 4: "약지", 5: "새끼"}
    cmd = sys.argv[1] if len(sys.argv) > 1 else "open"
    h = MicrobitBleHand(cfg)
    try:
        if cmd == "open":
            print("손 펴기:", h.open())
        elif cmd == "press":
            f = int(sys.argv[2])
            print(f"{names[f]}({f}) 한 번 누르기 ({cfg['press_angle'][str(f)]}도)")
            h.press(f)
        elif cmd == "code":
            for d in sys.argv[2]:
                f = cfg["key_finger"].get(d)
                if f is None:
                    print(f"숫자 {d}: key_finger 에 없음")
                    continue
                print(f"숫자 {d} (F{d}) -> {names[f]}")
                h.press(f)
        elif cmd == "sweep":
            # bend one finger 5 deg more each step, opening the hand fully between steps
            f = int(sys.argv[2])
            ang = cfg["open"][f - 1]
            print(f"{names[f]}을 5도씩 더 굽혀 봅니다 (매번 손을 폈다가). 키가 눌리면 y, 아니면 Enter")
            while ang > 20:
                ang -= 5
                pose = list(cfg["open"])
                pose[f - 1] = ang
                h.send(pose)
                time.sleep(cfg["move_ms"] / 1000 + cfg["hold_s"])
                h.open()
                if input(f"  {ang}도: 눌렸나요? ").strip().lower() == "y":
                    target = ang - 5
                    save_config_patch({"hand": {"press_angle": {str(f): target}}})
                    print(f"  {names[f]}: {ang}도에서 눌림 -> 누를 때 {target}도로 저장")
                    break
        else:
            print(__doc__)
    finally:
        h.close()


if __name__ == "__main__":
    main()
