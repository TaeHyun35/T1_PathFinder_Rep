"""VLM 으로 두 번 확인 (노트북 GPU, Qwen3-VL). YOLO 가 '이 카드 같다' 하면 그 부분 사진을 보내 확정.

  서버:  D:\\aihand-ml\\venv-lerobot\\Scripts\\python maze_mission\\vlm_check.py serve --port 8091
  한 장:  ... vlm_check.py ask 사진.jpg
POST /check (body = jpg) -> {"animal": "dog"|..|"none", "digit": "1".."4"|"", "raw": 답, "ms": 걸린 시간}
"""
import argparse
import json
import os
import re
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

os.environ.setdefault("HF_HOME", "D:/aihand-ml/hf-cache")
os.environ.setdefault("HF_HUB_OFFLINE", "1")
import torch
from PIL import Image
from transformers import AutoModelForImageTextToText, AutoProcessor

KO = {"강아지": "dog", "개": "dog", "판다": "panda", "거북이": "turtle", "토끼": "rabbit", "기린": "giraffe", "코끼리": "elephant"}
EN = {"dog": "dog", "puppy": "dog", "panda": "panda", "turtle": "turtle", "tortoise": "turtle", "rabbit": "rabbit",
      "giraffe": "giraffe", "elephant": "elephant"}
Q = ("This is a printed card seen by a robot camera. Which animal is in the picture? "
     "Choose one of: dog, panda, turtle, rabbit, giraffe, elephant, none. "
     "Also read the big digit printed in the box under the picture (1, 2, 3 or 4, or none). "
     'Answer only as JSON like {"animal": "dog", "digit": "2"}.')


class Checker:
    def __init__(self, mid="Qwen/Qwen3-VL-4B-Instruct"):
        self.proc = AutoProcessor.from_pretrained(mid)
        self.model = AutoModelForImageTextToText.from_pretrained(mid, dtype=torch.bfloat16, device_map="cuda:0")

    def __call__(self, img):
        t = time.time()
        img = img.convert("RGB")
        if max(img.size) > 640:
            img.thumbnail((640, 640))
        msgs = [{"role": "user", "content": [{"type": "image", "image": img}, {"type": "text", "text": Q}]}]
        x = self.proc.apply_chat_template(msgs, add_generation_prompt=True, tokenize=True, return_dict=True,
                                          return_tensors="pt").to(self.model.device)
        with torch.no_grad():
            out = self.model.generate(**x, max_new_tokens=40, do_sample=False)
        raw = self.proc.decode(out[0][x["input_ids"].shape[1]:], skip_special_tokens=True).strip()
        return {**parse(raw), "raw": raw, "ms": round((time.time() - t) * 1000)}


def parse(raw):
    animal, digit = "none", ""
    m = re.search(r"\{.*?\}", raw, re.S)
    if m:
        try:
            j = json.loads(m.group(0))
            a = str(j.get("animal", "")).lower().strip()
            animal = EN.get(a, KO.get(a, "none"))
            d = str(j.get("digit", "")).strip()
            digit = d if d in "1234" and len(d) == 1 else ""
            return {"animal": animal, "digit": digit}
        except json.JSONDecodeError:
            pass
    low = raw.lower()
    for k, v in {**EN, **KO}.items():
        if k in low:
            animal = v
            break
    m = re.search(r"\b([1-4])\b", raw)
    return {"animal": animal, "digit": m.group(1) if m else ""}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["serve", "ask"])
    ap.add_argument("img", nargs="?")
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--model", default="Qwen/Qwen3-VL-4B-Instruct")
    a = ap.parse_args()
    chk = Checker(a.model)
    if a.cmd == "ask":
        print(json.dumps(chk(Image.open(a.img)), ensure_ascii=False))
        return
    import io

    class H(BaseHTTPRequestHandler):
        def log_message(self, *x):
            pass

        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0))
            try:
                r = chk(Image.open(io.BytesIO(self.rfile.read(n))))
            except Exception as e:
                r = {"animal": "none", "digit": "", "err": str(e)}
            body = json.dumps(r, ensure_ascii=False).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    print(f"VLM 확인 서버 :{a.port} ({a.model})", flush=True)
    ThreadingHTTPServer(("0.0.0.0", a.port), H).serve_forever()


if __name__ == "__main__":
    main()
