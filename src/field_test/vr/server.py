"""VR 가리키기 실험 서버 (노트북에서 실행). 피코 브라우저에서 https://<노트북IP>:8443 접속.

  python server.py                          정확도 테스트만 (차 없이)
  python server.py --car 192.168.0.50       가리킨 곳으로 차 보내기 (파이에서 car.py server 실행 중이어야 함)
  python server.py --car <IP> --hand        차가 도착하면 손이 펴졌다가 쥠 (micro:bit를 노트북에 연결)
  python server.py --hand-offset 0,-0.20    손이 차 중심에서 앞 0m, 오른쪽 20cm에 있으면 (왼쪽이 +)

터미널 명령 (서버 실행 중 입력 후 Enter):
  u  마지막 점 지우기     n  처음부터 다시 (새 라운드)     s  지금까지 결과 요약     q  끝
기록: logs 폴더에 jsonl(모든 점)과 csv(정확도 결과)가 저장된다.
"""
import argparse
import datetime
import ipaddress
import json
import math
import os
import socket
import ssl
import threading
import time
import urllib.request
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
LOGDIR = os.path.join(HERE, "logs")
os.makedirs(LOGDIR, exist_ok=True)
STAMP = time.strftime("%Y%m%d_%H%M%S")
LOG = os.path.join(LOGDIR, f"vr_{STAMP}.jsonl")
CSV = os.path.join(LOGDIR, f"accuracy_{STAMP}.csv")

# 테이프 격자 (단위 m). 사람은 A2에서 50cm 뒤에 서서 C쪽(앞)을 본다.
#   C1 C2 C3   <- 100cm 앞
#   B1 B2 B3   <- 50cm 앞
#   A1 A2 A3   <- A2가 원점, 1이 왼쪽 3이 오른쪽 (간격 50cm)
#        사람
GRID = {f"{r}{c}": ((c - 2) * 0.5, i * 0.5) for i, r in enumerate("ABC") for c in (1, 2, 3)}
ACC_SEQ = ["A1", "A3", "A2", "B1", "B2", "B3", "C1", "C2", "C3"]   # 앞 2개는 보정용

args = None
lock = threading.Lock()
S = {}   # 상태


def reset():
    S.clear()
    S.update(acc_i=0, acc_pts={}, acc_rows=[], car_c=None, car_f=None, picks=[], round=S.get("round", 0) + 1)


S["round"] = 0
reset()


def log(obj):
    obj["time"] = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    with open(LOG, "a", encoding="utf-8") as f:
        f.write(json.dumps(obj, ensure_ascii=False) + "\n")


def frame_from(p_origin, p_axis, axis_is_x):
    """두 점으로 바닥 좌표계를 만든다. 반환: (원점, x단위, y단위). y는 '앞', x는 '오른쪽'."""
    ox, oz = p_origin
    dx, dz = p_axis[0] - ox, p_axis[1] - oz
    n = math.hypot(dx, dz) or 1e-9
    ux, uz = dx / n, dz / n
    if axis_is_x:           # 주어진 축이 오른쪽(x): 앞(y) = x를 위에서 보아 반시계 90도
        X = (ux, uz)
        Y = (uz, -ux)
    else:                   # 주어진 축이 앞(y): 오른쪽(x) = 앞을 시계 90도
        Y = (ux, uz)
        X = (-uz, ux)
    return (ox, oz), X, Y


def to_frame(fr, p):
    (ox, oz), X, Y = fr
    dx, dz = p[0] - ox, p[1] - oz
    return dx * X[0] + dz * X[1], dx * Y[0] + dz * Y[1]


# ---------------------------------------------------------------- 정확도 테스트
def acc_pick(p, kind):
    i = S["acc_i"]
    if i >= len(ACC_SEQ):
        return "이번 라운드 끝. 터미널에서 n을 누르면 새 라운드"
    label = ACC_SEQ[i]
    S["acc_pts"][label] = p
    S["acc_i"] += 1
    msg = f"{label} 찍음"
    if label == "A3":
        a1, a3 = S["acc_pts"]["A1"], S["acc_pts"]["A3"]
        mid = ((a1[0] + a3[0]) / 2, (a1[1] + a3[1]) / 2)
        S["acc_frame"] = frame_from(mid, a3, axis_is_x=True)
        span = math.hypot(a3[0] - a1[0], a3[1] - a1[1])
        msg += f" (보정 완료, A1~A3 거리 {span * 100:.0f}cm / 실제 100cm)"
        print("  보정: A1-A3 거리", round(span * 100, 1), "cm", flush=True)
    elif label not in ("A1",):
        gx, gy = to_frame(S["acc_frame"], p)
        tx, ty = GRID[label]
        err = math.hypot(gx - tx, gy - ty) * 100
        row = dict(round=S["round"], label=label, kind=kind, x_cm=round(gx * 100, 1), y_cm=round(gy * 100, 1),
                   err_cm=round(err, 1), err_x_cm=round((gx - tx) * 100, 1), err_y_cm=round((gy - ty) * 100, 1))
        S["acc_rows"].append(row)
        new = not os.path.exists(CSV)
        with open(CSV, "a", encoding="utf-8-sig") as f:
            if new:
                f.write(",".join(row.keys()) + "\n")
            f.write(",".join(str(v) for v in row.values()) + "\n")
        print(f"  {label}: 오차 {err:.1f}cm (오른쪽 {row['err_x_cm']:+.1f}, 앞 {row['err_y_cm']:+.1f})", flush=True)
        msg += f": 오차 {err:.0f}cm"
    nxt = ACC_SEQ[S["acc_i"]] if S["acc_i"] < len(ACC_SEQ) else None
    return msg + (f"\n다음: {nxt}" + (" (보정)" if nxt in ("A1", "A3") else "") if nxt else "\n라운드 끝")


def acc_hint():
    i = S["acc_i"]
    if i >= len(ACC_SEQ):
        return "라운드 끝"
    return f"다음: {ACC_SEQ[i]}" + (" (보정)" if ACC_SEQ[i] in ("A1", "A3") else "")


# ---------------------------------------------------------------- 차 보내기
def http_json(url, obj=None, timeout=15):
    data = json.dumps(obj).encode() if obj is not None else None
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read() or b"{}")


hand = None


def hand_grab():
    if hand is None:
        return
    O, T = [50, 130, 130, 130, 130], [140, 30, 20, 20, 20]

    def go():
        def send(f, ms=500):
            hand.reset_input_buffer()
            hand.write((",".join(map(str, f)) + f",90,{ms}\n").encode())
            hand.readline()
        send(O)
        print("  손: 펴짐. 5초 안에 공을 손바닥에 대 주세요", flush=True)
        time.sleep(5)
        send(T, 600)
        print("  손: 꽉 쥠 (3초)", flush=True)
        time.sleep(3)
        send(O)
        print("  손: 내려놓기", flush=True)
    threading.Thread(target=go, daemon=True).start()


def car_pick(p, kind):
    if S["car_c"] is None:
        S["car_c"] = p
        return "차 중심 찍음\n다음: 차 앞 1m 표시 (보정)"
    if S["car_f"] is None:
        S["car_f"] = p
        S["car_frame"] = frame_from(S["car_c"], p, axis_is_x=False)
        d = math.hypot(p[0] - S["car_c"][0], p[1] - S["car_c"][1])
        return f"보정 완료 (두 점 거리 {d * 100:.0f}cm / 실제 100cm)\n이제 차를 보낼 곳을 가리키고 집기"
    x, y = to_frame(S["car_frame"], p)        # x 오른쪽, y 앞 (차 기준)
    fwd, left = y - args.hand_offset[0], -x - args.hand_offset[1]
    log(dict(type="car_target", fwd=fwd, left=left, x=x, y=y))
    print(f"  목표: 차 기준 앞 {y * 100:+.0f}cm, 오른쪽 {x * 100:+.0f}cm -> 차 이동 앞 {fwd * 100:+.0f}, 왼쪽 {left * 100:+.0f}", flush=True)
    if not args.car:
        return f"목표 앞 {y * 100:.0f}cm 오른쪽 {x * 100:.0f}cm\n(--car 없이 실행: 차는 안 움직임)"
    try:
        t0 = time.time()
        r = http_json(f"http://{args.car}:8800/goto", {"fwd": fwd, "left": left}, timeout=30)
    except Exception as e:
        return f"차 연결 실패: {e}"
    if not r.get("ok"):
        return "차가 거부: " + str(r.get("err"))
    # 차가 이만큼 움직였으니 차 좌표계도 옮긴다 (다음 목표를 이어서 보낼 수 있게)
    (ox, oz), X, Y = S["car_frame"]
    S["car_frame"] = ((ox + Y[0] * fwd - X[0] * left, oz + Y[1] * fwd - X[1] * left), X, Y)
    log(dict(type="car_done", sec=round(time.time() - t0, 2), resp=r))
    hand_grab()
    return f"도착 ({time.time() - t0:.1f}초). 실제 오차를 줄자로 재세요" + ("\n손: 5초 안에 공 대 주기" if hand else "")


# ---------------------------------------------------------------- HTTP
class H(SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=HERE, **k)

    def log_message(self, fmt, *a):
        if self.command == "GET" and self.path in ("/", "/index.html"):
            print("피코 접속:", self.client_address[0], flush=True)

    def reply(self, obj):
        b = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        d = json.loads(self.rfile.read(n) or b"{}")
        t = d.get("type")
        with lock:
            if t == "pick" and d.get("hit"):
                p = (d["hit"][0], d["hit"][2])
                mode = d.get("mode")
                S["picks"].append((mode, p))
                log(dict(d, round=S["round"]))
                print(f"[{mode}] {d.get('kind')} {d.get('side')} 바닥점 ({p[0]:+.2f}, {p[1]:+.2f})", flush=True)
                msg = acc_pick(p, d.get("kind")) if mode == "acc" else car_pick(p, d.get("kind"))
                return self.reply({"hud": msg})
            if t == "hint":
                return self.reply({"hud": acc_hint() if d.get("mode") == "acc" else
                                   ("다음: 차 중심 (보정)" if S["car_c"] is None else
                                    "다음: 차 앞 1m 표시 (보정)" if S["car_f"] is None else "보낼 곳을 가리키고 집기")})
            log(d)
            if t not in ("frame",):
                print(t, {k: v for k, v in d.items() if k not in ("type", "t")}, flush=True)
        self.reply({})


def summary():
    rows = S["acc_rows"]
    if not rows:
        print("  정확도 결과 없음")
        return
    errs = [r["err_cm"] for r in rows]
    print(f"  라운드 {S['round']}: {len(errs)}점, 평균 오차 {sum(errs) / len(errs):.1f}cm, 최대 {max(errs):.1f}cm")
    for r in rows:
        print(f"   {r['label']} {r['kind']}: {r['err_cm']}cm")


def console():
    while True:
        try:
            c = input().strip().lower()
        except EOFError:
            return
        with lock:
            if c == "u":
                if S["acc_i"] > 0:
                    S["acc_i"] -= 1
                    lab = ACC_SEQ[S["acc_i"]]
                    S["acc_pts"].pop(lab, None)
                    S["acc_rows"] = [r for r in S["acc_rows"] if r["label"] != lab]
                    print("  지움:", lab, "-> 다시 찍으세요")
                elif S["car_f"] is not None and len(S["picks"]) <= 2:
                    S["car_f"] = None
                    print("  차 보정점 지움")
            elif c == "n":
                summary()
                reset()
                print(f"  새 라운드 {S['round']} 시작 (A1부터, 차 보정도 처음부터)")
            elif c == "s":
                summary()
            elif c == "q":
                summary()
                os._exit(0)


# ---------------------------------------------------------------- 인증서 (https가 있어야 피코에서 WebXR이 켜짐)
def local_ips():
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    ips.discard("127.0.0.1")
    return sorted(ips)


def make_cert(ips):
    cert, key = os.path.join(HERE, "cert.pem"), os.path.join(HERE, "key.pem")
    tag = os.path.join(HERE, "cert_ips.txt")
    if os.path.exists(cert) and os.path.exists(tag) and open(tag).read() == ",".join(ips):
        return cert, key
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    from cryptography.x509.oid import NameOID
    k = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "aihand-vr")])
    san = [x509.IPAddress(ipaddress.ip_address(i)) for i in ips + ["127.0.0.1"]] + [x509.DNSName("localhost")]
    now = datetime.datetime.now(datetime.timezone.utc)
    c = (x509.CertificateBuilder().subject_name(name).issuer_name(name).public_key(k.public_key())
         .serial_number(x509.random_serial_number()).not_valid_before(now - datetime.timedelta(days=1))
         .not_valid_after(now + datetime.timedelta(days=60))
         .add_extension(x509.SubjectAlternativeName(san), critical=False).sign(k, hashes.SHA256()))
    open(key, "wb").write(k.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.TraditionalOpenSSL,
                                          serialization.NoEncryption()))
    open(cert, "wb").write(c.public_bytes(serialization.Encoding.PEM))
    open(tag, "w").write(",".join(ips))
    print("새 인증서 만듦:", ips)
    return cert, key


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--car", help="라즈봇(파이) IP")
    ap.add_argument("--hand", nargs="?", const="auto", help="도착하면 손 쥐기 (micro:bit 포트, 생략하면 자동)")
    ap.add_argument("--hand-offset", default="0,0", help="차 중심에서 손 잡는 점까지 '앞,왼쪽' (m)")
    ap.add_argument("--port", type=int, default=8443)
    args = ap.parse_args()
    args.hand_offset = [float(v) for v in args.hand_offset.split(",")]
    if args.hand:
        import serial
        from serial.tools import list_ports
        port = None if args.hand == "auto" else args.hand
        if port is None:
            for p in list_ports.comports():
                t = f"{p.description} {p.manufacturer or ''} {p.hwid}".lower()
                if "mbed" in t or "0d28" in t or "microbit" in t or "micro:bit" in t:
                    port = p.device
        if port is None:
            raise SystemExit("micro:bit 포트를 못 찾았습니다. --hand COM5 처럼 적어 주세요.")
        hand = serial.Serial(port, 115200, timeout=1)
        time.sleep(0.5)
        print("손 연결:", port)
    if args.car:
        try:
            print("차 연결 확인:", http_json(f"http://{args.car}:8800/", timeout=4).get("ok"))
        except Exception as e:
            print("경고: 차 서버에 연결 안 됨 ->", e, "(파이에서 python3 car.py server 실행했는지, 같은 와이파이인지 확인)")
    ips = local_ips()
    cert, key = make_cert(ips)
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(cert, key)
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), H)
    srv.socket = ctx.wrap_socket(srv.socket, server_side=True)
    print("피코 브라우저 주소:", "  ".join(f"https://{i}:{args.port}" for i in ips))
    print("기록:", LOG)
    print("터미널 명령: u 지우기 / n 새 라운드 / s 요약 / q 끝")
    threading.Thread(target=console, daemon=True).start()
    srv.serve_forever()
