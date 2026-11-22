#!/usr/bin/env python3
"""Flash a MakeCode program to a micro:bit V2 over Bluetooth (partial flashing, no USB).

Only the MakeCode program region is replaced, so the micro:bit's CODAL (firmware) must be
the same build as the one in the hex - checked by hash before anything is written.
Protocol: codal-microbit-v2 MicroBitPartialFlashingService.

    python3 ble_flash.py <mbcodal-binary.hex> [BLE address]
"""
import asyncio
import sys
import time

from bleak import BleakClient, BleakScanner

PF = "e97d3b10-251d-470a-a062-fa1922dfa9a8"
MAGIC = bytes.fromhex("708E3B92C615A841C49866C975EE5197")
REGION_INFO, FLASH_DATA, END_OF_TRANSMISSION, STATUS = 0x00, 0x01, 0x02, 0xEE


def load_hex(path):
    mem, base = {}, 0
    for line in open(path).read().splitlines():
        if not line.startswith(":"):
            continue
        b = bytes.fromhex(line[1:])
        n, addr, typ = b[0], (b[1] << 8) | b[2], b[3]
        if typ == 0x04:
            base = int.from_bytes(b[4:6], "big") << 16
        elif typ in (0x00, 0x0D):
            for k in range(n):
                mem[base + addr + k] = b[4 + k]
    return mem


async def main(hex_path, addr):
    mem = load_hex(hex_path)
    dev = await BleakScanner.find_device_by_address(addr, timeout=15)
    if dev is None:
        sys.exit("micro:bit 을 못 찾음")
    async with BleakClient(dev) as c:
        q = asyncio.Queue()
        await c.start_notify(PF, lambda h, d: q.put_nowait(bytes(d)))

        async def ask(cmd, want):
            await c.write_gatt_char(PF, bytes(cmd), response=False)
            while True:
                r = await asyncio.wait_for(q.get(), 3)
                if r[0] == want:
                    return r

        st = await ask([STATUS], STATUS)
        print(f"partial flashing v{st[1]}, mode {'application' if st[2] == 1 else 'pairing'}")
        r1 = await ask([REGION_INFO, 1], REGION_INFO)
        r2 = await ask([REGION_INFO, 2], REGION_INFO)       # also resets packet counters
        dal_hash = r1[10:18]
        start, end = int.from_bytes(r2[2:6], "big"), int.from_bytes(r2[6:10], "big")

        prog = bytearray(b"\xff" * (end - start))
        for a, v in mem.items():
            if start <= a < end:
                prog[a - start] = v
        if prog[:16] != MAGIC:
            sys.exit(f"hex 의 프로그램이 {hex(start)} 에서 시작하지 않음 (V2 용 hex 인지 확인)")
        if prog[16:24] != dal_hash:
            sys.exit(f"펌웨어 hash 다름: hex {prog[16:24].hex()} / micro:bit {dal_hash.hex()} -> USB 로 올려야 함")
        last = max(i for i in range(len(prog)) if prog[i] != 0xFF)
        nblocks = last // 64 + 1
        print(f"펌웨어 hash 일치 ({dal_hash.hex()}). 프로그램 {last + 1} 바이트 = {nblocks} 블록 전송")

        pnum, t0, retries = 0, time.time(), 0
        b = 0
        while b < nblocks:
            a = start + b * 64
            for k in range(4):
                off = (a & 0xFFFF) if k == 0 else ((a >> 16) & 0xFFFF) if k == 1 else 0
                pkt = bytes([FLASH_DATA, off >> 8, off & 0xFF, (pnum + k) & 0xFF]) + bytes(prog[b * 64 + k * 16: b * 64 + k * 16 + 16])
                await c.write_gatt_char(PF, pkt, response=False)
            r = await asyncio.wait_for(q.get(), 5)
            while r[0] != FLASH_DATA:
                r = await asyncio.wait_for(q.get(), 5)
            if r[1] == 0xFF:
                pnum = (pnum + 4) & 0xFF
                b += 1
                if b % 50 == 0 or b == nblocks:
                    print(f"  {b}/{nblocks} 블록 ({time.time() - t0:.0f}s)", flush=True)
            else:                                   # 0xAA: device expects this block again, numbered +4
                retries += 1
                pnum = (pnum + 4) & 0xFF
                if retries > 50:
                    sys.exit("재전송이 너무 많음 - 중단 (micro:bit 은 블루투스 대기 상태로 켜짐, 다시 실행)")
        await c.write_gatt_char(PF, bytes([END_OF_TRANSMISSION]), response=False)
        print(f"전송 끝 ({time.time() - t0:.0f}s, 재전송 {retries}회). micro:bit 이 새 프로그램으로 다시 켜집니다.")
        await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "E1:EF:DD:AE:52:EF"))
