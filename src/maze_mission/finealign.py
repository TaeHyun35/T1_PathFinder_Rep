"""손목 아래 카메라 모듈 3 으로 키보드 앞 마지막 맞추기 (11/26 사용자 아이디어).

모듈 3 은 키보드 뒷면 테두리를 정면으로 본다 (사진은 90도 돌아감: 사진 y = 로봇 옆쪽).
테두리의 USB 포트 두 개(위 USB-A, 아래 USB-C)를 찾아서
  - 앞뒤: 두 포트 사이 간격(px)이 정답보다 작으면 멀다 (크기로 거리)
  - 좌우: 두 포트 가운데가 화면 가운데에서 얼마나 떨어졌나 (간격으로 나눠서 거리 영향을 뺌)
정답 자리(손가락이 F1~F4 위)에서 'ref' 로 기준을 저장해 둔다.

  python3 finealign.py ref       지금 자리를 정답으로 저장
  python3 finealign.py check     지금 어긋난 정도 (mm)
"""
import json
import os
import subprocess
import sys

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
META = os.path.join(HERE, "logs", "m3_ref.json")
CY = 360.0                 # 사진 세로 가운데 (720 의 반)
D0_MM = 33.0               # 정답 자리에서 카메라~포트 거리 (11/26: 1cm 뒤로 당기니 간격 178 -> 137px 로 역산)
PORT_MM = 23.4             # 두 포트 사이 실제 거리 (178px / 7.6px/mm)


def capture(path="/tmp/m3_now.jpg"):
    subprocess.run(["rpicam-still", "-n", "-t", "600", "--width", "1280", "--height", "720", "-o", path],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    return cv2.imread(path)


def ports(img, x_lo=600, x_hi=700):
    """키보드 테두리의 어두운 포트 구멍 두 개 -> [(x, y), (x, y)] (y 작은 것 먼저) 또는 None"""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    roi = g[:, x_lo:x_hi]
    _, th = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    n, lab, st, cen = cv2.connectedComponentsWithStats(th)
    cand = []
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if 400 < a < 12000 and w < 80 and h < 160 and x > 2 and x + w < (x_hi - x_lo) - 2:   # ROI 가장자리에 붙은 것(손 부품) 빼기
            cand.append((a, cen[i][0] + x_lo, cen[i][1]))
    if len(cand) < 2:
        return None
    cand.sort(key=lambda c: -c[0])
    two = sorted(cand[:2], key=lambda c: c[2])
    return [(two[0][1], two[0][2]), (two[1][1], two[1][2])]


def offset(img, ref=None):
    """-> (앞으로 더 갈 mm, 옆 mm (사진 y+ 쪽이 +), 찾음 여부)"""
    ref = ref or json.load(open(META))
    p = ports(img)
    if p is None:
        return 0.0, 0.0, False
    s = p[1][1] - p[0][1]
    mid = (p[0][1] + p[1][1]) / 2
    s0, mid0 = ref["s"], ref["mid"]
    if s < 30:
        return 0.0, 0.0, False
    fwd = D0_MM * (s0 / s - 1.0)                                       # 멀어졌으면 + (앞으로 더)
    side = ((mid - CY) / s - (mid0 - CY) / s0) * PORT_MM              # 포트가 사진 y+ 로 옮겨 감 -> +
    return fwd, side, True


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    src = sys.argv[2] if len(sys.argv) > 2 else None
    img = cv2.imread(src) if src else capture()
    if cmd == "ref":
        p = ports(img)
        if p is None:
            sys.exit("포트를 못 찾음")
        json.dump({"s": p[1][1] - p[0][1], "mid": (p[0][1] + p[1][1]) / 2, "ports": p}, open(META, "w"))
        cv2.imwrite(os.path.join(HERE, "logs", "m3_ref_aligned.jpg"), img)
        print("기준 저장:", p)
    else:
        print("포트:", ports(img))
        f, s, ok = offset(img)
        print(f"앞으로 {f:+.1f}mm, 옆(사진 y+) {s:+.1f}mm, 찾음 {ok}")
