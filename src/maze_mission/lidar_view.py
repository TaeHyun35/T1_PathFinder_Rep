"""라이다 실시간 화면 (RPLIDAR A1M8) - 라즈봇/ROS 설정을 거치지 않고 라이다에 직접 붙어서 보여 준다.

목적: "가까이만 잡힌다"가 라이다 기계 문제인지, 테스트 프로그램/설정 문제인지 확실히 가르기.
이 프로그램은 필터, 거리 제한, 반응 거리 같은 것이 전혀 없다. 라이다가 보내는 점을 그대로 그린다.

라즈베리파이에서 실행:
  1) 라이다를 쓰는 다른 프로그램(ROS 라이다 노드, 라즈봇 데모, 도커)을 먼저 끈다
  2) python3 lidar_view.py
  3) 같은 와이파이의 폰/노트북 브라우저에서  http://<파이IP>:8080  열기  (파이 IP: hostname -I)

옵션:
  --port /dev/ttyUSB0    라이다 포트 (기본: 자동 찾기)
  --web 8080             웹 포트
  --ros                  비교용: 라이다 대신 ROS2 /scan 토픽을 그린다 (라이다 노드 켜 둔 상태)
  --fake                 라이다 없이 가짜 방으로 화면 점검
  --motor-invert         모터가 안 돌면 이걸 붙여서 다시 실행

필요한 것: 파이썬3 만 있으면 됨 (pyserial 있으면 쓰고, 없으면 리눅스 기본 기능으로 직접 연결)
화면의 [저장] 버튼을 누르면 logs_lidar/ 폴더에 그 순간의 점 전체가 json 으로 저장된다.
"""
import argparse
import glob
import json
import math
import os
import random
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

STATE = {"pts": [], "hz": 0.0, "n": 0, "health": "-", "info": "-", "src": "-", "err": "", "bad": 0, "t": 0}
LOCK = threading.Lock()
SAVE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs_lidar")


# ---------------- 시리얼 (pyserial 없어도 동작) ----------------
class TermiosSerial:
    """리눅스 기본 termios 로 115200 raw 시리얼 열기"""
    def __init__(self, port):
        import fcntl
        import termios
        self.fcntl, self.termios = fcntl, termios
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        a = termios.tcgetattr(self.fd)
        a[0] = 0                                       # iflag
        a[1] = 0                                       # oflag
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[3] = 0                                       # lflag
        a[4] = a[5] = termios.B115200
        a[6][termios.VMIN] = 0
        a[6][termios.VTIME] = 5                        # 0.5초 기다림
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
        return serial.Serial(port, 115200, timeout=0.5), "pyserial"
    except ImportError:
        return TermiosSerial(port), "termios"


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


def request(ser, cmd, n):
    ser.reset_input_buffer()
    ser.write(bytes([0xA5, cmd]))
    d = read_exact(ser, 7)
    if len(d) != 7 or d[:2] != b"\xA5\x5A":
        return None
    return read_exact(ser, n)


def parse_node(b):
    """표준 측정 5바이트 -> (새 바퀴 시작?, 각도deg, 거리m, 품질0~63) / 깨지면 None"""
    s, ns = b[0] & 1, (b[0] >> 1) & 1
    if s == ns or not (b[1] & 1):
        return None
    ang = ((b[1] >> 1) | (b[2] << 7)) / 64.0
    dist = (b[3] | (b[4] << 8)) / 4000.0
    return bool(s), ang % 360.0, dist, b[0] >> 2


def publish(cur, t_last_revs):
    t_last_revs.append(time.time())
    while len(t_last_revs) > 11:
        t_last_revs.pop(0)
    hz = (len(t_last_revs) - 1) / (t_last_revs[-1] - t_last_revs[0]) if len(t_last_revs) > 2 else 0
    with LOCK:
        STATE["pts"] = cur
        STATE["hz"] = hz
        STATE["n"] += 1
        STATE["t"] = time.time()


def lidar_thread(port, motor_invert):
    while True:
        ser = None
        try:
            ser, how = open_serial(port)
            ser.setDTR(motor_invert)                   # Slamtec USB 어댑터: DTR 꺼짐 = 모터 켜짐
            time.sleep(1.5)
            info = request(ser, 0x50, 20)
            h = request(ser, 0x52, 3)
            with LOCK:
                STATE["src"] = f"라이다 직접 연결 {port} ({how})"
                STATE["info"] = (f"모델 0x{info[0]:02X}, 펌웨어 {info[2]}.{info[1]:02d}, 하드웨어 {info[3]}, "
                                 f"시리얼 {info[4:].hex().upper()}") if info and len(info) == 20 else "장치 정보 응답 없음"
                STATE["health"] = ({0: "정상", 1: "경고", 2: "에러"}.get(h[0], "?") + f" (코드 {struct.unpack('<H', h[1:3])[0]})") if h and len(h) == 3 else "응답 없음"
                STATE["err"] = ""
            ser.reset_input_buffer()
            ser.write(bytes([0xA5, 0x20]))
            if len(read_exact(ser, 7)) != 7:
                raise RuntimeError("측정 시작 응답 없음 (다른 프로그램이 포트를 쓰는 중이거나 전원/케이블 문제)")
            cur, revs, last_data = [], [], time.time()
            while True:
                b = read_exact(ser, 5, 1.0)
                if len(b) < 5:
                    if time.time() - last_data > 3:
                        raise RuntimeError("3초 동안 데이터 없음 (모터가 안 돌면 --motor-invert 로 다시 실행)")
                    continue
                last_data = time.time()
                n = parse_node(b)
                if n is None:
                    with LOCK:
                        STATE["bad"] += 1
                    ser.read(1)                        # 한 바이트 밀어서 다시 맞춤
                    continue
                new, a, d, q = n
                if new and cur:
                    publish(cur, revs)
                    cur = []
                cur.append((round(a, 2), round(d, 3), q))
        except Exception as e:
            with LOCK:
                STATE["err"] = f"{type(e).__name__}: {e}"
            time.sleep(2)
        finally:
            if ser:
                try:
                    ser.write(bytes([0xA5, 0x25]))
                    ser.setDTR(not motor_invert)
                    ser.close()
                except Exception:
                    pass


def ros_thread(topic):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    rclpy.init()
    node = rclpy.create_node("lidar_view")
    revs = []

    def cb(m):
        pts = []
        for i, r in enumerate(m.ranges):
            ok = math.isfinite(r) and m.range_min <= r <= m.range_max
            # ROS 는 반시계(+), 라이다 원본은 시계 방향 -> 화면을 맞추려고 뒤집음
            a = (-math.degrees(m.angle_min + i * m.angle_increment)) % 360
            q = int(m.intensities[i]) if i < len(m.intensities) else 0
            pts.append((round(a, 2), round(r, 3) if ok else 0.0, q))
        with LOCK:
            STATE["info"] = f"range_min {m.range_min:.2f} m, range_max {m.range_max:.2f} m (ROS 설정값)"
        publish(pts, revs)
    node.create_subscription(LaserScan, topic, cb, qos_profile_sensor_data)
    with LOCK:
        STATE["src"], STATE["health"] = f"ROS2 {topic} (드라이버+설정 거친 값)", "-"
    rclpy.spin(node)


def fake_thread():
    """3.0 x 2.7 m 방 + 상자 하나, 로봇은 (0.8, 0.9)"""
    walls = [((0, 0), (3, 0)), ((3, 0), (3, 2.7)), ((3, 2.7), (0, 2.7)), ((0, 2.7), (0, 0)),
             ((1.5, 1.2), (1.8, 1.2)), ((1.8, 1.2), (1.8, 1.5)), ((1.8, 1.5), (1.5, 1.5)), ((1.5, 1.5), (1.5, 1.2))]
    rx, ry, revs = 0.8, 0.9, []
    with LOCK:
        STATE["src"], STATE["info"], STATE["health"] = "가짜 라이다 (화면 점검용)", "가짜 방 3.0 x 2.7 m", "정상 (가짜)"
    while True:
        pts = []
        for k in range(360):
            a = k + random.random() * 0.5
            th = math.radians(90 - a)                  # 0도 = 화면 위(앞), 시계 방향
            dx, dy, best = math.cos(th), math.sin(th), 0.0
            for (x1, y1), (x2, y2) in walls:
                ex, ey = x2 - x1, y2 - y1
                den = dx * ey - dy * ex
                if abs(den) < 1e-9:
                    continue
                t = ((x1 - rx) * ey - (y1 - ry) * ex) / den
                u = ((x1 - rx) * dy - (y1 - ry) * dx) / den
                if t > 0 and 0 <= u <= 1 and (best == 0 or t < best):
                    best = t
            if 150 < a < 210:
                best = 0.0                              # 뒤쪽은 몸체에 가려졌다고 가정
            pts.append((round(a, 2), round(best + random.gauss(0, 0.005), 3) if best else 0.0, 45))
        publish(pts, revs)
        time.sleep(1 / 7)


# ---------------- 웹 화면 ----------------
PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>라이다 실시간</title>
<style>
:root{--bg:#0f1418;--panel:#182028;--fg:#e4ebf0;--mut:#8fa0ad;--ring:#33424f;--acc:#4fc3e8;--bad:#ff7a6b;--ok:#6ad49b}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px system-ui,sans-serif;padding:12px}
.wrap{max-width:1100px;margin:0 auto;display:grid;grid-template-columns:minmax(0,1fr) 340px;gap:12px}
@media(max-width:820px){.wrap{grid-template-columns:1fr}}
canvas{width:100%;aspect-ratio:1;background:var(--panel);border-radius:10px;display:block;touch-action:none}
.panel{background:var(--panel);border-radius:10px;padding:12px;display:flex;flex-direction:column;gap:10px;min-width:0}
h1{font-size:17px;margin:0}.mut{color:var(--mut);font-size:13px}
.row{display:flex;flex-wrap:wrap;gap:6px}
button{font:inherit;background:#26333e;color:var(--fg);border:1px solid #34444f;border-radius:8px;padding:7px 11px;cursor:pointer}
button.on{background:var(--acc);color:#06222c;border-color:var(--acc);font-weight:700}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums;font-size:14px}
td{padding:3px 4px;border-bottom:1px solid #26323c}td:last-child{text-align:right}
.verdict{border-left:4px solid var(--acc);padding:8px 10px;background:#1f2a33;border-radius:6px;line-height:1.5}
.err{color:var(--bad);font-weight:700}
label{font-size:13px;color:var(--mut)}input[type=range]{width:100%}
</style></head><body>
<div class="wrap">
 <div><canvas id="c" width="900" height="900"></canvas>
  <div class="mut" style="margin-top:6px">화면 위쪽 = 라이다 0도. 원은 거리 표시. 점 색: 가까움(빨강) → 멂(파랑). 화면을 누르면 그 점까지 거리 표시.</div></div>
 <div class="panel">
  <h1>라이다 실시간 (필터 없음)</h1>
  <div id="src" class="mut"></div><div id="err" class="err"></div>
  <div class="row" id="zoom"></div>
  <div><label for="rot">앞 방향 돌리기: <span id="rotv">0</span>도 (상자를 로봇 앞에 두고 화면 위로 오게 맞추기)</label>
   <input id="rot" type="range" min="-180" max="180" step="5" value="0"></div>
  <table id="tb"></table>
  <div class="verdict" id="vd">데이터 기다리는 중...</div>
  <div class="row"><button id="save" type="button">지금 화면 저장</button><button id="hold" type="button">멈춰 보기</button></div>
  <div id="saved" class="mut"></div>
 </div>
</div>
<script>
const C=document.getElementById('c'),X=C.getContext('2d');
const ZOOMS=[["자동",0],["1m",1],["2m",2],["4m",4],["8m",8],["12m",12]];let zoom=0,rot=0,hold=false,last=null,pick=null;
const zr=document.getElementById('zoom');
ZOOMS.forEach(([t,v])=>{const b=document.createElement('button');b.type='button';b.textContent=t;b.onclick=()=>{zoom=v;[...zr.children].forEach(x=>x.classList.toggle('on',x===b));draw()};zr.appendChild(b)});zr.children[0].classList.add('on');
const rotI=document.getElementById('rot');rotI.oninput=()=>{rot=+rotI.value;document.getElementById('rotv').textContent=rot;draw()};
document.getElementById('hold').onclick=e=>{hold=!hold;e.target.classList.toggle('on',hold);e.target.textContent=hold?'다시 보기':'멈춰 보기'};
document.getElementById('save').onclick=async()=>{const r=await fetch('/save',{method:'POST'});document.getElementById('saved').textContent=await r.text()};
const SEC=["앞","앞오른쪽","오른쪽","뒤오른쪽","뒤","뒤왼쪽","왼쪽","앞왼쪽"];
function col(d){const t=Math.min(1,d/4);return `hsl(${t*210},85%,60%)`}
function scale(){let R=zoom;if(!R){const ds=(last?last.pts:[]).map(p=>p[1]).filter(d=>d>0).sort((a,b)=>a-b);R=ds.length?Math.max(1,ds[Math.floor(ds.length*0.98)]*1.1):4}return R}
function xy(a,d,R){const th=(a+rot)*Math.PI/180,s=C.width/2*0.94/R;return [C.width/2+Math.sin(th)*d*s,C.height/2-Math.cos(th)*d*s]}
function draw(){
 const W=C.width,R=scale();X.clearRect(0,0,W,W);X.font='22px system-ui';X.textAlign='left';
 let step=R<=1.5?0.25:R<=3?0.5:R<=6?1:2;
 for(let r=step;r<=R+1e-6;r+=step){X.strokeStyle='#33424f';X.lineWidth=2;X.beginPath();X.arc(W/2,W/2,W/2*0.94*r/R,0,7);X.stroke();X.fillStyle='#8fa0ad';X.fillText(r.toFixed(r<1?2:1)+'m',W/2+4,W/2-W/2*0.94*r/R-4)}
 X.strokeStyle='#26323c';X.beginPath();X.moveTo(W/2,0);X.lineTo(W/2,W);X.moveTo(0,W/2);X.lineTo(W,W/2);X.stroke();
 X.fillStyle='#4fc3e8';X.beginPath();X.moveTo(W/2,W/2-22);X.lineTo(W/2-12,W/2+12);X.lineTo(W/2+12,W/2+12);X.fill();
 if(!last)return;
 for(const [a,d] of last.pts){if(d<=0)continue;const [x,y]=xy(a,d,R);X.fillStyle=col(d);X.fillRect(x-3,y-3,6,6)}
 if(pick){const [x,y]=xy(pick[0],pick[1],R);X.strokeStyle='#fff';X.lineWidth=2;X.beginPath();X.moveTo(W/2,W/2);X.lineTo(x,y);X.stroke();X.fillStyle='#fff';X.font='26px system-ui';X.fillText(pick[1].toFixed(2)+' m',x+8,y-8)}
}
C.addEventListener('pointerdown',e=>{if(!last)return;const b=C.getBoundingClientRect(),px=(e.clientX-b.left)*C.width/b.width,py=(e.clientY-b.top)*C.height/b.height,R=scale();let best=null,bd=1e9;for(const p of last.pts){if(p[1]<=0)continue;const [x,y]=xy(p[0],p[1],R),dd=(x-px)**2+(y-py)**2;if(dd<bd){bd=dd;best=p}}pick=best;draw()});
function stats(s){
 const g=s.pts.filter(p=>p[1]>0),ds=g.map(p=>p[1]),tb=[];
 const far=ds.length?Math.max(...ds):0,near=ds.length?Math.min(...ds):0;
 tb.push(["회전 속도",s.hz.toFixed(1)+" 바퀴/초"],["한 바퀴 점 수",s.pts.length+" (잰 점 "+g.length+")"],["가장 먼 점",far.toFixed(2)+" m"],["가장 가까운 점",near.toFixed(2)+" m"],["1m 넘는 점",(ds.length?ds.filter(d=>d>1).length/ds.length*100:0).toFixed(0)+" %"],["2m 넘는 점",(ds.length?ds.filter(d=>d>2).length/ds.length*100:0).toFixed(0)+" %"],["자가진단",s.health],["장치",s.info],["깨진 데이터",s.bad+" 개"]);
 for(let i=0;i<8;i++){const sec=g.filter(p=>{const a=((p[0]+rot)%360+360)%360;return ((a-i*45+22.5+360)%360)<45}).map(p=>p[1]);tb.push([SEC[i]+" 가장 가까운",sec.length?Math.min(...sec).toFixed(2)+" m":"못 잼"])}
 document.getElementById('tb').innerHTML=tb.map(([k,v])=>`<tr><td>${k}</td><td>${v}</td></tr>`).join('');
 let v=[];
 if(!g.length)v.push("잰 점이 없음 → 모터가 도는지, 케이블, 전원 확인");
 else{
  if(far>2.5)v.push("<b>라이다는 멀리("+far.toFixed(1)+" m)까지 잡고 있음.</b> 다른 프로그램에서 가까이만 반응했다면 그 프로그램의 설정(반응 거리, 필터) 문제.");
  else v.push("가장 먼 점이 "+far.toFixed(1)+" m. 주변에 그보다 먼 벽이 있는데도 안 잡히면: 라이다 높이가 벽보다 높은지, 기울었는지 확인. 그래도 안 되면 기계/전원 의심.");
  if(s.hz>0&&s.hz<4)v.push("회전이 느림("+s.hz.toFixed(1)+") → 전원 부족 의심 (보조배터리나 전원 좋은 USB로)");
  if(String(s.health).startsWith("에러"))v.push("자가진단 에러 → 기계 문제 가능성");
 }
 document.getElementById('vd').innerHTML=v.join("<br><br>");
}
async function tick(){try{const r=await fetch('/scan.json',{cache:'no-store'}),s=await r.json();document.getElementById('src').textContent=s.src;document.getElementById('err').textContent=s.err?("문제: "+s.err):"";if(!hold&&s.pts.length){last=s;stats(s);draw()}}catch(e){document.getElementById('err').textContent="파이와 연결 끊김"}setTimeout(tick,200)}
draw();tick();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send(self, code, body, ctype):
        b = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path.startswith("/scan.json"):
            with LOCK:
                s = dict(STATE)
            self.send(200, json.dumps(s, ensure_ascii=False), "application/json; charset=utf-8")
        elif self.path in ("/", "/index.html"):
            self.send(200, PAGE, "text/html; charset=utf-8")
        else:
            self.send(404, "없음", "text/plain; charset=utf-8")

    def do_POST(self):
        if self.path == "/save":
            os.makedirs(SAVE_DIR, exist_ok=True)
            with LOCK:
                s = dict(STATE)
            fn = os.path.join(SAVE_DIR, time.strftime("scan_%Y%m%d_%H%M%S.json"))
            json.dump(s, open(fn, "w", encoding="utf-8"), ensure_ascii=False)
            self.send(200, f"저장됨: {fn} (점 {len(s['pts'])}개)", "text/plain; charset=utf-8")
        else:
            self.send(404, "없음", "text/plain; charset=utf-8")


def my_ips():
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ips.add(s.getsockname()[0])
        s.close()
    except Exception:
        pass
    return sorted(ips) or ["<파이IP>"]


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--web", type=int, default=8080)
    ap.add_argument("--ros", action="store_true")
    ap.add_argument("--topic", default="/scan")
    ap.add_argument("--fake", action="store_true")
    ap.add_argument("--motor-invert", action="store_true")
    a = ap.parse_args()
    if a.fake:
        threading.Thread(target=fake_thread, daemon=True).start()
    elif a.ros:
        threading.Thread(target=ros_thread, args=(a.topic,), daemon=True).start()
    else:
        port = a.port or find_port()
        if not port:
            sys.exit("라이다 포트를 못 찾음. USB 연결 확인 후  ls /dev/ttyUSB*  로 보고 --port 로 지정")
        threading.Thread(target=lidar_thread, args=(port, a.motor_invert), daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", a.web), H)
    for ip in my_ips():
        print(f"브라우저에서 열기: http://{ip}:{a.web}")
    print("끄기: Ctrl+C")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
