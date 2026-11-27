"""테스트 실험실 (Lab): 브라우저에서 설정을 바꾸고, 실행하고, 진행 과정을 실시간으로 본다.

실행 (aihand 폴더에서):
  .venv\\Scripts\\python tools\\lab\\lab.py
  -> 브라우저가 http://127.0.0.1:8765 로 열린다.

새 테스트 추가하는 법: tools/lab/tests_*.py 에 TEST = {...} 를 만들면 자동으로 목록에 뜬다.
  TEST = {
    "id": "...", "title": "...", "desc": "쉬운 설명 (줄바꿈 가능)",
    "steps": ["1단계 설명", ...],
    "params": [{"key", "label", "type": int|float|select|text, "default", "min", "max", "step", "options", "help", "group"}],
    "charts": [{"id", "type": "line"|"dots", "title", "y_label", ...}],
    "columns": ["표에 보일 열", ...],
    "run": run(params, ctx),
  }
  ctx.log(글), ctx.status(글), ctx.point(chart, x, y, mark=None), ctx.row(dict), ctx.summary(dict), ctx.stop(Event)
결과는 logs/lab/ 에 CSV + JSON 으로 저장된다.
"""
import csv
import importlib.util
import json
import os
import sys
import threading
import time
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))          # aihand
sys.path.insert(0, os.path.dirname(HERE))               # tools (key_press_test 등 import 용)
LOG_DIR = os.path.join(ROOT, "logs", "lab")
PORT = 8765


def load_tests():
    tests = {}
    for name in sorted(os.listdir(HERE)):
        if name.startswith("tests_") and name.endswith(".py"):
            spec = importlib.util.spec_from_file_location(name[:-3], os.path.join(HERE, name))
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            t = mod.TEST
            tests[t["id"]] = t
    return tests


class Run:
    """지금 돌고 있는(또는 마지막) 실행 하나"""

    def __init__(self, test, params):
        self.test, self.params = test, params
        self.events, self.rows, self.summary_data = [], [], {}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.running = True
        self.started = time.time()
        self.name = time.strftime(f"{test['id']}_%Y%m%d_%H%M%S")

    def _emit(self, kind, **kw):
        with self.lock:
            self.events.append({"seq": len(self.events), "kind": kind, "t": round(time.time() - self.started, 3), **kw})

    # ---- 테스트 코드가 쓰는 함수들
    def log(self, text):
        print(text)
        self._emit("log", text=str(text))

    def status(self, text):
        self._emit("status", text=str(text))

    def point(self, chart, x, y, mark=None, **extra):
        self._emit("point", chart=chart, x=x, y=y, mark=mark, **extra)

    def row(self, data):
        self.rows.append(data)
        self._emit("row", data=data)

    def image(self, path, caption=""):
        """그림 보여 주기 (지도 등). 허락된 폴더(aihand 안) 파일만"""
        self.images = getattr(self, "images", []) + [{"path": os.path.abspath(path), "caption": caption}]
        self._emit("image", path=os.path.abspath(path), caption=caption)

    def summary(self, data):
        self.summary_data.update(data)
        self._emit("summary", data=data)

    def since(self, n):
        with self.lock:
            return self.events[n:]

    def save(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        base = os.path.join(LOG_DIR, self.name)
        if self.rows:
            keys = list(dict.fromkeys(k for r in self.rows for k in r))
            with open(base + ".csv", "w", newline="", encoding="utf-8-sig") as f:
                w = csv.DictWriter(f, fieldnames=keys)
                w.writeheader()
                w.writerows(self.rows)
        with open(base + ".json", "w", encoding="utf-8") as f:
            json.dump({"test": self.test["id"], "title": self.test["title"], "name": self.name,
                       "time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(self.started)),
                       "params": self.params, "summary": self.summary_data, "rows": self.rows,
                       "images": getattr(self, "images", []),
                       "stopped": self.stop.is_set()}, f, ensure_ascii=False, indent=1)
        return base


TESTS = load_tests()
current = None


def start_run(test_id, params):
    global current
    if current and current.running:
        return False, "이미 실행 중입니다. 먼저 멈추세요."
    test = TESTS[test_id]
    clean = {}
    for p in test["params"]:
        v = params.get(p["key"], p.get("default"))
        if p["type"] == "int":
            v = int(float(v))
        elif p["type"] == "float":
            v = float(v)
        clean[p["key"]] = v
    run = Run(test, clean)
    current = run

    def worker():
        run.status("시작")
        run.log(f"[{test['title']}] 설정: " + ", ".join(f"{k}={v}" for k, v in clean.items()))
        try:
            test["run"](clean, run)
            run.status("멈춤" if run.stop.is_set() else "끝")
        except Exception as e:                     # 오류도 화면에 보여 준다
            run.log("오류: " + "".join(traceback.format_exception_only(type(e), e)).strip())
            run.status("오류로 멈춤")
        finally:
            path = run.save()
            run.log(f"저장: {os.path.relpath(path, ROOT)}.csv / .json")
            run.running = False
            run._emit("done")

    threading.Thread(target=worker, daemon=True).start()
    return True, run.name


def history():
    out = []
    if os.path.isdir(LOG_DIR):
        for name in sorted(os.listdir(LOG_DIR), reverse=True):
            if name.endswith(".json"):
                try:
                    with open(os.path.join(LOG_DIR, name), encoding="utf-8") as f:
                        d = json.load(f)
                    out.append({k: d.get(k) for k in ("test", "title", "name", "time", "params", "summary", "stopped")})
                except (OSError, ValueError):
                    pass
    return out[:50]


def history_one(name):
    path = os.path.join(LOG_DIR, os.path.basename(name) + ".json")
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def public_tests():
    return [{k: v for k, v in t.items() if k != "run"} for t in TESTS.values()]


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path == "/":
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                return self._send(200, f.read(), "text/html; charset=utf-8")
        if u.path == "/api/tests":
            return self._send(200, public_tests())
        if u.path == "/api/poll":
            n = int(q.get("since", ["0"])[0])
            if current is None:
                return self._send(200, {"name": None, "running": False, "events": []})
            if q.get("name", [""])[0] != current.name:
                n = 0                                  # 새 실행이면 처음부터
            return self._send(200, {"name": current.name, "test": current.test["id"], "params": current.params,
                                    "running": current.running, "events": current.since(n)})
        if u.path == "/api/img":
            p = os.path.abspath(q.get("path", [""])[0])
            if p.startswith(ROOT) and p.lower().endswith((".png", ".jpg", ".jpeg")) and os.path.isfile(p):
                with open(p, "rb") as f:
                    return self._send(200, f.read(), "image/png" if p.lower().endswith(".png") else "image/jpeg")
            return self._send(404, {"error": "없음"})
        if u.path == "/api/history":
            return self._send(200, history())
        if u.path == "/api/history_one":
            try:
                return self._send(200, history_one(q["name"][0]))
            except (OSError, KeyError, ValueError):
                return self._send(404, {"error": "없음"})
        self._send(404, {"error": "없음"})

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length) or b"{}")
        if self.path == "/api/run":
            ok, msg = start_run(body["test"], body.get("params", {}))
            return self._send(200, {"ok": ok, "msg": msg})
        if self.path == "/api/stop":
            if current:
                current.stop.set()
            return self._send(200, {"ok": True})
        self._send(404, {"error": "없음"})


def main():
    ThreadingHTTPServer.allow_reuse_address = False   # 윈도우에서 이미 켜진 Lab이 있으면 두 번 켜지지 않게
    try:
        srv = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    except OSError:
        print(f"Lab이 이미 켜져 있습니다: http://127.0.0.1:{PORT}")
        webbrowser.open(f"http://127.0.0.1:{PORT}")
        return
    url = f"http://127.0.0.1:{PORT}"
    print(f"테스트 실험실: {url}  (끝내려면 이 창에서 Ctrl+C)")
    print("테스트 목록:", ", ".join(t["title"] for t in TESTS.values()))
    if "--no-browser" not in sys.argv:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        if current:
            current.stop.set()
        print("끝")


if __name__ == "__main__":
    main()
