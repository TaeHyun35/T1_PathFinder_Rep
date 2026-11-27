"""Lab 테스트: 토요일 시연 흐름 (11/27 밤, 테이프 네모)
순서 말하기(음성/글) -> 네모 안 탐색 + 지도 + 카드 찾기 -> 출발점 -> "OO, OO 순서로 찾아가겠습니다"
-> 카드마다 앞에 서서 "OO 앞에 도착했습니다" -> 키보드 마커(ArUco 2) 앞 -> 끝.
실제 로봇: 파이에서 mission.py, PC 에서 pc_station.py(카드 찾기/말하기/음성) 가 같이 돈다.
"""
import json
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import robot_util as R
import tests_robot_tape as TT

FILES = TT.FILES + ["vision.py", "finealign.py", "hand.py", "ble_hand.py", "sim.py"]
STEPS = [("순서 받음", "순서 받기"), ("한 바퀴 보기 끝", "제자리 둘러보기"), ("지도를 완성", "지도 완성"),
         ("출발점 도착", "출발점 복귀"), ("순서로 찾아가겠습니다", "순서 말하기"), ("앞에 도착했습니다", "카드 앞"),
         ("키보드 마커 보임", "키보드 마커 찾음"), ("입력 위치에 도착했습니다", "키보드 앞 도착")]


def run(p, ctx):
    t0 = time.time()
    vwp = TT.write_vwalls(p)
    info = {"log": None, "scans": None, "result": None, "cards": [], "done": set()}

    def on_line(line):
        for key, name in STEPS:
            if key in line:
                if name == "카드 앞":
                    m = re.search(r"\] (\S+) 앞에 도착", line)
                    info["cards"].append(m.group(1) if m else "?")
                    ctx.status(f"카드 앞 도착: {info['cards']}")
                elif name not in info["done"]:
                    ctx.status(name)
                info["done"].add(name)
        m = re.search(r"결과:? (\S+)", line)
        if m and ("결과:" in line or line.startswith("결과 ")):
            info["result"] = m.group(1)
        m = re.search(r"지도용: (\S+\.jsonl)", line)
        if m:
            info["scans"] = m.group(1)

    order = ",".join(x.strip() for x in p["order"].replace("，", ",").split(",") if x.strip())
    if p["mode"] == "sim":
        ctx.status("시뮬 (테이프 네모 + 카드 3장 + 키보드 마커)")
        R.stream([R.SYS_PY, "-u", "tape_mission_sim.py", "--vwalls", vwp, "--seed", str(p["seed"])], ctx, on_line, robot=False)
        img = R.newest(os.path.join(R.MM, "logs"), f"sim_seed{p['seed']}_", t0)
        if img and img.endswith(".jsonl"):
            img = img.replace(".jsonl", ".png")
        pngs = [f for f in os.listdir(os.path.join(R.MM, "logs")) if f.startswith(f"sim_seed{p['seed']}_") and f.endswith(".png")
                and os.path.getmtime(os.path.join(R.MM, "logs", f)) >= t0 - 2]
        if pngs:
            ctx.image(os.path.join(R.MM, "logs", sorted(pngs)[-1]), "시뮬 경로 (정답 위치)")
    else:
        if not R.pi_ready(ctx, need_cam=False) or not R.deploy(ctx, FILES):
            return
        R.ssh("pkill -f 'cam_serve[.]py'; echo s", 15)                     # 미션이 카메라를 직접 씀 (따로 보냄: 같은 줄이면 ssh 가 끊김)
        ctx.log("카메라 서버 끔 (미션이 카메라를 직접 씀)")
        hand = "" if p["hand"] == "1" else "--no-hand"
        if p.get("stage") in ("calib", "find", "visit", "exit"):           # 11/28 단계별
            t_ = {"calib": 60, "find": 300, "visit": 400, "exit": 150}[p["stage"]]
            fc = ""
            if p["stage"] == "find" and p.get("cards"):
                KO = {"강아지": "dog", "토끼": "rabbit", "코끼리": "elephant", "판다": "panda", "거북이": "turtle", "기린": "giraffe"}
                fc = "--find-cards " + ",".join(KO.get(x.strip(), x.strip()) for x in p["cards"].split(",") if x.strip())
            cmd = R.SSH + [f"cd ~/maze_home && timeout {t_ + 200} ~/handenv/bin/python -u mission.py run --station --vwalls vwalls.json --stage {p['stage']} {fc} {hand}"]
            ctx.log(f"단계 실행: {p['stage']}")
        elif p.get("stage", "full") != "full":                                # 카드 앞에 서기만 (테이프 없이, 카드 뒤에 박스/벽)
            cmd = R.SSH + [f"cd ~/maze_home && timeout 120 ~/handenv/bin/python -u mission.py run --station --approach-test {p['stage']} {hand}"]
            ctx.log(f"카드 앞에 서기만 시험: {p['stage']} (로봇을 60cm 앞에 두기)")
        else:
            cmd = R.SSH + [f"cd ~/maze_home && timeout 900 ~/handenv/bin/python -u mission.py run --station --card-press --vwalls vwalls.json {hand}"]
        st_args = ["maze_mission/pc_station.py"] + (["--voice"] if p["voice"] == "1" else ["--order", order])
        station = None
        import threading

        def later():
            time.sleep(6)                                                   # 로봇 쪽 우체통(:8093)이 켜질 때까지
            nonlocal station
            if not ctx.stop.is_set():
                station = R.start_pc(st_args, ctx, "station", console=True)
                ctx.log("PC 스테이션 창을 열었어요 (카드 찾기, 말하기" + (", 마이크로 순서 듣기)" if p["voice"] == "1" else ")"))
        threading.Thread(target=later, daemon=True).start()
        ctx.status("로봇 미션 중 (멈춤 버튼 = 바로 정지)")
        try:
            R.stream(cmd, ctx, on_line)
        finally:
            time.sleep(1)
            if station is not None:
                station.terminate()
        if not info["scans"]:                                               # 멈춤 버튼으로 끝나면 경로 줄이 안 나옴 -> 가장 최근 기록
            rc, out = R.ssh("ls -t ~/maze_home/logs/station_scans_*.jsonl | head -1", 15)
            info["scans"] = out.strip() or None
        local = R.fetch(info["scans"]) if info["scans"] else None
        if local and os.path.exists(local):
            sys.path.insert(0, R.MM)
            import importlib
            import tapemap
            importlib.reload(tapemap)
            dets = [os.path.join(R.MM, "logs", f) for f in os.listdir(os.path.join(R.MM, "logs"))
                    if f.startswith("station_") and f.endswith("_detect.jsonl") and os.path.getmtime(os.path.join(R.MM, "logs", f)) >= t0 - 2]
            det = max(dets, key=os.path.getmtime) if dets else None
            png, st = tapemap.draw(local, det=det, title="시연: 테이프 네모 지도 + 카드")
            ctx.image(png, "검은 선 = 테이프, 회색 = 라이다 방 지도, 파랑 = 로봇 길, 별 = 카드")
    ctx.summary({"성공": info["result"] or "-", "카드 앞 도착": ", ".join(info["cards"]) or "-",
                 "지난 단계": ", ".join(n for _, n in STEPS if n in info["done"])})


TEST = {
    "id": "robot_demo",
    "title": "로봇: 시연 전체 (테이프 네모)",
    "desc": "토요일 시연 흐름: 순서 말하기 -> 테이프 네모 안을 돌며 지도/카드 찾기 -> 출발점 -> 말한 순서대로 카드 앞에 서기 -> 키보드 마커(2번) 앞.\n"
            "카드는 테이프 선 바로 바깥에 세우고 앞면이 안쪽을 보게, 2번 마커도 테이프 선 바깥에 세워 주세요.\n"
            "실제 로봇이면 PC 스테이션 창이 따로 열려요 (말하기, 카드 찾기). 음성을 켜면 그 창이 마이크로 순서를 들어요.",
    "steps": ["테이프 네모 + 카드 + 2번 마커 세우기", "로봇을 네모 한가운데, 가로 변과 나란히", "순서 입력 (또는 음성)", "실행", "지도 그림 확인"],
    "params": [
        {"key": "mode", "label": "어디서", "type": "select", "default": "real", "options": [["real", "실제 로봇"], ["sim", "시뮬"]]},
        {"key": "stage", "label": "무엇을", "type": "select", "default": "full",
         "options": [["calib", "1단계 보정"], ["find", "3단계 동물+탈출 위치 찾기(기억)"], ["visit", "4단계 말한 동물 찾아가기"], ["exit", "5단계 탈출"], ["full", "시연 전체 (한 번에)"], ["dog", "강아지 앞에 서기만"], ["rabbit", "토끼 앞에 서기만"], ["elephant", "코끼리 앞에 서기만"],
                     ["panda", "판다 앞에 서기만"], ["turtle", "거북이 앞에 서기만"], ["giraffe", "기린 앞에 서기만"], ["marker", "2번 마커 앞에 서기만"]],
         "help": "서기만: 로봇을 카드 60cm 앞에 대충 카드 쪽 보게 (카드 뒤에 박스/벽)"},
        {"key": "voice", "label": "순서 받기", "type": "select", "default": "0", "options": [["0", "글로 (아래 순서)"], ["1", "음성 (마이크)"]]},
        {"key": "order", "label": "찾을 순서", "type": "text", "default": "강아지,거북이,판다", "help": "쉼표로 (강아지, 판다, 거북이, 토끼, 기린, 코끼리)"},
        {"key": "cards", "label": "놓아 둔 카드 (3단계)", "type": "text", "default": "강아지,토끼,코끼리,판다,거북이,기린", "help": "이 카드를 다 찾으면 3단계 끝"},
        {"key": "width", "label": "테이프 가로", "type": "float", "default": 1.5, "min": 1.0, "max": 4.0, "step": 0.05, "help": "m (로봇이 보는 방향)"},
        {"key": "height", "label": "테이프 세로", "type": "float", "default": 1.4, "min": 1.0, "max": 4.0, "step": 0.05, "help": "m"},
        {"key": "hand", "label": "손 (BLE)", "type": "select", "default": "0", "options": [["0", "안 씀 (누르지 않는 시연)"], ["1", "씀"]]},
        {"key": "seed", "label": "시뮬 배치 번호", "type": "int", "default": 1, "min": 1, "max": 50, "step": 1},
    ],
    "charts": [],
    "columns": [],
    "run": run,
}
