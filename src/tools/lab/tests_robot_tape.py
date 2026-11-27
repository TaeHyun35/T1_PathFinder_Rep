"""Lab 테스트: 테이프 네모 (가상 벽) 안에서 돌며 지도 만들기 (11/27 밤). 실제 로봇 또는 시뮬."""
import json
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_util as R

FILES = ["roommap.py", "explore.py", "frontier.py", "obstacles.py", "drive.py", "heading.py", "vwalls.py", "common.py",
         "mission.py", "link.py", "lidar.py", "tapemap.py", "vwalls.json"]


def write_vwalls(p):
    w, h = float(p["width"]), float(p["height"])
    d = {"_note": f"Lab 에서 만든 테이프 네모 {w} x {h} m. 왼쪽 아래 = (0,0), x = {w}m 변. 출발 = 한가운데, +x({w}m 변과 나란히) 보고",
         "start": [w / 2, h / 2, 0], "walls": [[0, 0, w, 0], [w, 0, w, h], [w, h, 0, h], [0, h, 0, 0]]}
    path = os.path.join(R.MM, "vwalls.json")
    json.dump(d, open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    return path


def run(p, ctx):
    t0 = time.time()
    vwp = write_vwalls(p)
    ctx.log(f"테이프 네모: {p['width']} x {p['height']} m, 로봇은 한가운데에서 {p['width']}m 변과 나란히 출발")
    info = {"lap": None, "home": None, "stuck": 0, "log": None, "reanchor": None}

    def on_line(line):
        if "한 바퀴 완료" in line:
            m = re.search(r"([\d.]+)초", line)
            info["lap"] = m.group(1) if m else "예"
            ctx.status("한 바퀴 완료 -> 출발점으로")
        if "출발점 도착" in line:
            m = re.search(r"남은 거리 (\d+)cm", line)
            info["home"] = m.group(1) if m else "?"
        if "막힘 감지" in line or "맴돌기 감지" in line:
            info["stuck"] += 1
        if "출발 사진과 다시 맞" in line:
            info["reanchor"] = line.strip()
        m = re.search(r"기록: (\S+\.jsonl)", line)
        if m:
            info["log"] = m.group(1)

    helpers = []
    if p["mode"] == "sim":
        ctx.status("시뮬 실행 중 (벽 없는 방 + 가상 벽)")
        R.stream([R.SYS_PY, "-u", "tape_sim.py", "--vwalls", vwp, "--secs", str(p["secs"])], ctx, on_line, robot=False)
        local = info["log"]
    else:
        if not R.pi_ready(ctx) or not R.deploy(ctx, FILES):
            return
        helpers.append(R.start_pc(["maze_mission/detect_log.py", "--secs", str(p["secs"] + 60)], ctx, "detect"))
        ctx.status("로봇 주행 중 (멈춤 버튼 = 바로 정지)")
        try:
            R.stream(R.SSH + [f"cd ~/maze_home && timeout {int(p['secs']) + 60} ~/handenv/bin/python -u roommap.py run --yes "
                              f"--secs {p['secs']} --vwalls vwalls.json --cover 0 --camobst 0 --max_w {p['max_w']}"], ctx, on_line)
        finally:
            for h in helpers:
                h.terminate()
        local = R.fetch(info["log"]) if info["log"] else None
    if not local or not os.path.exists(local):
        ctx.log("주행 기록 파일을 못 가져옴")
        return
    sys.path.insert(0, R.MM)
    import importlib
    import tapemap
    importlib.reload(tapemap)
    det = R.newest(os.path.join(R.MM, "logs"), "detect_", t0) if p["mode"] != "sim" else None
    png, st = tapemap.draw(local, det=det, title=f"테이프 네모 {p['width']} x {p['height']} m")
    ctx.image(png, "검은 선 = 테이프(가상 벽), 회색 = 라이다가 본 방, 파랑 = 로봇이 간 길" + (", 별 = 카드" if det else ""))
    ctx.summary({"성공": "한 바퀴 완료" if info["lap"] else "한 바퀴 못 돎", "한 바퀴 시간": info["lap"],
                 "출발점까지 남은 거리(cm)": info["home"], "막힘 횟수": info["stuck"],
                 "테이프까지 가장 가까움(m, 로봇 계산)": st.get("테이프까지 가장 가까움(로봇 가운데)")})
    if info["reanchor"]:
        ctx.log(info["reanchor"])


TEST = {
    "id": "robot_tape",
    "title": "로봇: 테이프 네모 안에서 지도",
    "desc": "바닥 테이프 네모를 '보이지 않는 벽'으로 넣고, 로봇이 그 안을 돌며 지도를 만든 뒤 출발점으로 돌아옵니다.\n"
            "테이프는 라이다에 안 보이므로 출발 자리 기준으로 위치를 넣고, 라이다는 방 벽/가구로 '내 위치'를 맞춰요.\n"
            "로봇은 꼭 네모 한가운데에, 가로(첫 번째 크기) 변과 나란한 방향으로 놓으세요.",
    "steps": ["테이프 네모 크기 입력", "로봇을 네모 한가운데에, 가로 변과 나란히", "실행", "지도 그림에서 테이프 안에 있었는지 확인"],
    "params": [
        {"key": "mode", "label": "어디서", "type": "select", "default": "real", "options": [["real", "실제 로봇"], ["sim", "시뮬"]]},
        {"key": "width", "label": "가로 (로봇이 보는 방향)", "type": "float", "default": 1.5, "min": 1.0, "max": 4.0, "step": 0.05, "help": "m"},
        {"key": "height", "label": "세로", "type": "float", "default": 1.4, "min": 1.0, "max": 4.0, "step": 0.05, "help": "m"},
        {"key": "secs", "label": "최대 시간", "type": "int", "default": 150, "min": 60, "max": 600, "step": 30, "help": "초"},
        {"key": "max_w", "label": "최대 회전 속도", "type": "int", "default": 40, "min": 25, "max": 60, "step": 5, "help": "도/초 (빠르면 방향 계산을 놓침, 느리면 바퀴가 헛돎)"},
    ],
    "charts": [],
    "columns": [],
    "run": run,
}
