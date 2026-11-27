"""Lab 로봇 테스트 공통 (11/27 밤): 파이에 ssh 로 명령 보내고, 출력을 한 줄씩 Lab 화면으로, 멈춤 버튼 = 로봇 정지.
PC 는 항상 파이 쪽으로 연결한다 (PC 쪽에 로봇용 서버를 열지 않음).
"""
import os
import queue
import subprocess
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))      # aihand
MM = os.path.join(ROOT, "maze_mission")
PI = "raspberry@192.168.137.4"
SYS_PY = r"C:\Users\User\AppData\Local\Programs\Python\Python311\python.exe"   # scipy/matplotlib 있음 (지도 그리기, 시뮬)
ML_PY = os.path.join(ROOT, ".venv-ml", "Scripts", "python.exe")                  # YOLO (카드/장애물 찾기)
if not os.path.exists(SYS_PY):           # 다른 노트북: 지금 실행 중인 파이썬 하나로 다 (install.bat 이 필요한 것 설치)
    SYS_PY = __import__("sys").executable
if not os.path.exists(ML_PY):
    ML_PY = __import__("sys").executable
ENV = dict(os.environ, PYTHONIOENCODING="utf-8")
SSH = ["ssh", "-o", "ConnectTimeout=8", "-o", "BatchMode=yes", PI]


def ssh(cmd, timeout=30):
    r = subprocess.run(SSH + [cmd], capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=timeout)
    return r.returncode, (r.stdout or "") + (r.stderr or "")


def stop_robot(ctx=None):
    try:
        rc, out = ssh("python3 ~/stop.py; pkill -f 'roommap[.]py'; pkill -f 'mission[.]py'; echo done", 20)
        if ctx:
            ctx.log("로봇 정지: " + out.strip().replace("\n", " "))
    except Exception as e:                                  # noqa
        if ctx:
            ctx.log(f"로봇 정지 명령 실패 ({e}) -> 로봇 전원을 직접 끄세요")


def pi_ready(ctx, need_cam=True):
    """연결 확인 + 카메라 서버 켜기 (같은 ssh 안에서 pkill/실행을 섞으면 자기 연결이 끊겨서 따로 보냄)"""
    try:
        rc, out = ssh("echo ok", 15)
    except subprocess.TimeoutExpired:
        rc, out = 1, "시간 초과"
    if rc != 0 or "ok" not in out:
        ctx.log("로봇(파이)에 연결 안 됨: 로봇 전원과 노트북 핫스팟을 확인하세요")
        return False
    if need_cam:
        rc, out = ssh("pgrep -f 'cam_serve[.]py' >/dev/null && echo on || echo off", 15)
        if "off" in out:
            ssh("cd ~/maze_home && (nohup setsid ~/handenv/bin/python cam_serve.py --port 8092 > logs_cam.txt 2>&1 &) ; echo s", 15)
            ctx.log("카메라 서버 켬")
            time.sleep(2)
    ssh("pgrep -f 'powerl[o]g' >/dev/null || (nohup setsid /home/raspberry/powerlog.sh > /dev/null 2>&1 &); echo s", 15)
    rc, out = ssh("tail -n 1 ~/power.log", 10)
    ctx.log("파이 연결됨. 전압: " + out.strip())
    return True


def deploy(ctx, files):
    """PC 의 maze_mission 파일을 파이로 (항상 최신 코드로 돌게)"""
    paths = [os.path.join(MM, f) for f in files]
    r = subprocess.run(["scp", "-q", "-o", "ConnectTimeout=8"] + paths + [PI + ":maze_home/"], capture_output=True, text=True)
    if r.returncode != 0:
        ctx.log("코드 보내기 실패: " + (r.stderr or "").strip())
        return False
    ctx.log("최신 코드 보냄: " + ", ".join(files))
    return True


def fetch(remote, local_dir=None):
    local_dir = local_dir or os.path.join(MM, "logs")
    r = subprocess.run(["scp", "-q", PI + ":" + remote, local_dir + os.sep], capture_output=True, text=True)
    return os.path.join(local_dir, os.path.basename(remote)) if r.returncode == 0 else None


def stream(cmd, ctx, on_line=None, cwd=None, robot=True, new_console=False):
    """명령을 돌리며 출력 줄을 Lab 진행 기록에 보냄. 멈춤 버튼 -> (robot 이면) 로봇 정지 + 끝냄. -> 모든 줄"""
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, cwd=cwd or MM, env=ENV,
                         text=True, encoding="utf-8", errors="replace", bufsize=1)
    q = queue.Queue()

    def rd():
        for line in p.stdout:
            q.put(line.rstrip("\n"))
        q.put(None)
    threading.Thread(target=rd, daemon=True).start()
    lines = []
    while True:
        if ctx.stop.is_set():
            ctx.log("멈춤 버튼 -> 정지")
            if robot:
                stop_robot(ctx)
            try:
                p.terminate()
            except Exception:
                pass
            break
        try:
            line = q.get(timeout=0.2)
        except queue.Empty:
            continue
        if line is None:
            break
        lines.append(line)
        if "스캔 " in line and "지금:" in line:                   # 5초마다 나오는 줄은 상태로만
            ctx.status(line.strip())
        else:
            ctx.log(line)
        if on_line:
            on_line(line)
    try:
        p.wait(timeout=10)
    except Exception:
        p.kill()
    return lines


def start_pc(args, ctx, name, console=False):
    """PC 쪽 도우미 (카드 찾기 기록, 장애물 찾기, 스테이션) 켜기 -> Popen"""
    kw = {}
    if console:
        kw["creationflags"] = subprocess.CREATE_NEW_CONSOLE       # 키 입력/음성 안내가 보이는 창
        return subprocess.Popen([ML_PY, "-u"] + args, cwd=ROOT, env=ENV, **kw)
    log = open(os.path.join(MM, "logs", f"lab_{name}.txt"), "w", encoding="utf-8")
    p = subprocess.Popen([ML_PY, "-u"] + args, cwd=ROOT, env=ENV, stdout=log, stderr=subprocess.STDOUT)
    ctx.log(f"PC 도우미 켬: {name}")
    return p


def newest(folder, prefix, after):
    best = None
    for f in os.listdir(folder):
        if f.startswith(prefix):
            pth = os.path.join(folder, f)
            if os.path.getmtime(pth) >= after - 2 and (best is None or os.path.getmtime(pth) > os.path.getmtime(best)):
                best = pth
    return best
