"""카메라 + 동물 찾기(YOLO) + ArUco 마커(키패드, 문).

perceive() -> {"animals": [{"label","conf","cx","w"}], "markers": [{"id","cx","size"}]}
cx, w, size 는 화면 폭에 대한 비율 (0~1). cx 0.5 = 화면 가운데.
"""
import json
import threading
import time
import urllib.request


class Camera:
    """마지막 프레임만 들고 있는 카메라 (밀린 프레임 때문에 늦어지는 것 방지)"""

    def __init__(self, cfg):
        import cv2
        self.cv2, self.frame, self.err = cv2, None, ""
        self.src = cfg["source"]
        if self.src == "picam":
            from picamera2 import Picamera2
            self.picam = Picamera2()
            self.picam.configure(self.picam.create_video_configuration(main={"size": (cfg["width"], cfg["height"]), "format": "RGB888"}))
            self.picam.start()
        else:
            self.cap = cv2.VideoCapture(cfg["index"])
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["width"])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["height"])
            if not self.cap.isOpened():
                raise RuntimeError(f"카메라 {cfg['index']} 번을 못 엶 (ls /dev/video* 로 확인)")
        self._lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                if self.src == "picam":
                    f = self.picam.capture_array()
                    ok = f is not None
                else:
                    ok, f = self.cap.read()
                if ok:
                    with self._lock:
                        self.frame = f
                else:
                    time.sleep(0.05)
            except Exception as e:
                self.err = str(e)
                time.sleep(0.2)

    def read(self):
        with self._lock:
            return None if self.frame is None else self.frame.copy()


class Aruco:
    def __init__(self):
        import cv2
        self.cv2 = cv2
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())

    def find(self, frame):
        h, w = frame.shape[:2]
        gray = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.det.detectMarkers(gray)
        out = []
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                xs, ys = c[0][:, 0], c[0][:, 1]
                out.append({"id": int(i), "cx": float(xs.mean() / w), "size": float((xs.max() - xs.min()) / w)})
        return out


class LocalYolo:
    def __init__(self, cfg, names):
        from ultralytics import YOLO
        self.m, self.conf, self.names = YOLO(cfg["model"]), cfg["conf"], set(names)

    def find(self, frame):
        h, w = frame.shape[:2]
        r = self.m.predict(frame, conf=self.conf, verbose=False, imgsz=480)[0]
        out = []
        for b in r.boxes:
            name = r.names[int(b.cls)]
            if name in self.names:
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                out.append({"label": name, "conf": float(b.conf), "cx": (x1 + x2) / 2 / w, "w": (x2 - x1) / w})
        return out


class RemoteYolo:
    """노트북 vision_server.py 에 JPEG 를 보내고 결과를 받는다"""

    def __init__(self, cfg, names):
        import cv2
        self.cv2, self.url, self.conf, self.names = cv2, cfg["url"], cfg["conf"], set(names)

    def find(self, frame):
        ok, jpg = self.cv2.imencode(".jpg", frame, [self.cv2.IMWRITE_JPEG_QUALITY, 80])
        req = urllib.request.Request(f"{self.url}?conf={self.conf}", data=jpg.tobytes(), headers={"Content-Type": "image/jpeg"})
        with urllib.request.urlopen(req, timeout=3) as r:
            res = json.loads(r.read())
        return [d for d in res if d["label"] in self.names]


class Vision:
    def __init__(self, cam_cfg, det_cfg, animal_names):
        self.cam = Camera(cam_cfg)
        self.aruco = Aruco()
        self.yolo = (RemoteYolo if det_cfg["mode"] == "remote" else LocalYolo)(det_cfg, animal_names)
        self.last_frame = None

    def perceive(self):
        f = self.cam.read()
        if f is None:
            return {"animals": [], "markers": [], "err": self.cam.err or "프레임 없음"}
        self.last_frame = f
        out = {"markers": self.aruco.find(f), "animals": []}
        try:
            out["animals"] = self.yolo.find(f)
        except Exception as e:
            out["err"] = f"동물 찾기 실패: {e}"
        return out

    def save(self, path, res):
        """확인용: 박스 대신 화면 위에 글자로 결과를 적어 저장"""
        if self.last_frame is None:
            return
        cv2 = self.aruco.cv2
        f = self.last_frame.copy()
        h, w = f.shape[:2]
        for a in res.get("animals", []):
            x = int(a["cx"] * w)
            cv2.line(f, (x, 0), (x, h), (0, 200, 255), 2)
            cv2.putText(f, f"{a['label']} {a['conf']:.2f}", (max(0, x - 60), 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        for m in res.get("markers", []):
            x = int(m["cx"] * w)
            cv2.line(f, (x, 0), (x, h), (255, 120, 0), 2)
            cv2.putText(f, f"aruco {m['id']}", (max(0, x - 50), h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 120, 0), 2)
        cv2.imwrite(path, f)
