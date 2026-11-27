"""Lab 테스트: 방 돌아다니며 지도 만들기 (11/27 roommap). 실제 로봇 또는 시뮬."""
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_util as R

FILES = ["roommap.py", "explore.py", "frontier.py", "obstacles.py", "drive.py", "heading.py", "vwalls.py", "common.py",
         "mission.py", "link.py", "lidar.py", "tapemap.py"]


def run(p, ctx):
    t0 = time.time()
    info = {"lap": None, "home": None, "stuck": 0, "log": None}

    def on_line(line):
        if "한 바퀴 완료" in line:
            m = re.search(r"([\d.]+)초", line)
            info["lap"] = m.group(1) if m else "예"
            ctx.status("한 바퀴 완료 -> 지도 완성")
        if "출발점 도착" in line:
            m = re.search(r"남은 거리 (\d+)cm", line)
            info["home"] = m.group(1) if m else "?"
        if "막힘 감지" in line or "맴돌기 감지" in line:
            info["stuck"] += 1
        m = re.search(r"기록: (\S+\.jsonl)", line)
        if m:
            info["log"] = m.group(1)

    args = f"--secs {p['secs']} --cover {p['cover']} --camobst {p['camobst']} --spin {p['spin']}"
    helpers = []
    if p["mode"] == "sim":
        ctx.status("시뮬 실행 중")
        R.stream([R.SYS_PY, "-u", "roommap.py", "run", "--sim", "--yes", "--seed", str(p["seed"])] + args.split(), ctx, on_line, robot=False)
        local = info["log"]
    else:
        if not R.pi_ready(ctx) or not R.deploy(ctx, FILES):
            return
        helpers.append(R.start_pc(["maze_mission/detect_log.py", "--secs", str(p["secs"] + 60)], ctx, "detect"))
        if p["camobst"]:
            helpers.append(R.start_pc(["maze_mission/obst_feed.py", "--secs", str(p["secs"] + 60)], ctx, "obst"))
            time.sleep(8)                                              # YOLO 켜지는 시간
        ctx.status("로봇 주행 중 (멈춤 버튼 = 바로 정지)")
        try:
            R.stream(R.SSH + [f"cd ~/maze_home && timeout {int(p['secs']) + 60} ~/handenv/bin/python -u roommap.py run --yes {args}"], ctx, on_line)
        finally:
            for h in helpers:
                h.terminate()
        local = R.fetch(info["log"]) if info["log"] else None
    if not local or not os.path.exists(local):
        ctx.log("주행 기록 파일을 못 가져옴")
        return
    det = R.newest(os.path.join(R.MM, "logs"), "detect_", t0) if p["mode"] != "sim" else None
    cmd = [R.SYS_PY, "roommap.py", "build", local] + (["--det", det] if det else [])
    r = subprocess.run(cmd, cwd=R.MM, capture_output=True, text=True, encoding="utf-8", errors="replace", env=R.ENV)
    png = local.replace(".jsonl", ".png")
    if os.path.exists(png):
        ctx.image(png, "방 지도 (라이다) + 로봇이 간 길" + (" + 카드 위치" if det else ""))
    ctx.log(r.stdout.strip()[-300:])
    ctx.summary({"성공": "한 바퀴 완료" if info["lap"] else "한 바퀴 못 돎", "한 바퀴 시간": info["lap"],
                 "출발점까지 남은 거리(cm)": info["home"], "막힘 횟수": info["stuck"]})


TEST = {
    "id": "robot_room",
    "title": "로봇: 방 전체 지도 (테이프 무시)",
    "desc": "로봇이 벽을 따라 방을 한 바퀴 돌며 라이다 지도를 만들고, 안 본 곳을 찾아간 뒤 출발점으로 돌아옵니다.\n"
            "실제 로봇: 파이에 최신 코드를 보내고 바로 출발해요 (멈춤 버튼 = 즉시 정지). 시뮬: 미로 시뮬로 같은 코드를 돌려요.\n"
            "끝나면 지도 그림(카드 위치 포함)이 아래에 떠요.",
    "steps": ["로봇 전원, 핫스팟 확인", "로봇을 출발 자리에 두기", "실행", "지도 그림 확인"],
    "params": [
        {"key": "mode", "label": "어디서", "type": "select", "default": "real", "options": [["real", "실제 로봇"], ["sim", "시뮬"]]},
        {"key": "secs", "label": "최대 시간", "type": "int", "default": 300, "min": 60, "max": 900, "step": 30, "help": "초"},
        {"key": "cover", "label": "한 바퀴 뒤 안 본 곳 찾아가기", "type": "select", "default": "1", "options": [["1", "켬"], ["0", "끔"]]},
        {"key": "camobst", "label": "카메라로 장애물 보기", "type": "select", "default": "1", "options": [["1", "켬 (PC YOLO)"], ["0", "끔"]]},
        {"key": "spin", "label": "출발 때 한 바퀴 둘러보기", "type": "select", "default": "1", "options": [["1", "켬"], ["0", "끔"]]},
        {"key": "seed", "label": "시뮬 미로 번호", "type": "int", "default": 1, "min": 1, "max": 50, "step": 1},
    ],
    "charts": [],
    "columns": [],
    "run": run,
}
