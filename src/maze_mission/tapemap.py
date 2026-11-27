"""11/27 밤: 테이프 네모 주행 지도 그림 = 라이다 방 지도 + 테이프 가상 벽 + 로봇이 간 길 (+ 카드).

  python tapemap.py logs/roommap_XXXX.jsonl [--det logs/detect_XXXX.jsonl] [--out 그림.png]
roommap.py 가 --vwalls 로 돌면 기록 안에 {"vw": {...}} 줄이 있어서 테이프 위치를 그대로 겹쳐 그린다.
"""
import argparse
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def load(path):
    scans, vw, meta, rb = [], None, {}, None
    for line in open(path, encoding="utf-8"):
        r = json.loads(line)
        if "vw" in r:
            vw = vw or r["vw"]
            rb = r["vw"]["rebase"]                    # 미션: 한 바퀴 보기 뒤 다시 맞추면 새 줄이 생김 (그 뒤 스캔부터 적용)
        elif "meta" in r:
            meta = r["meta"]
        elif "a" in r and "odo" in r:
            r["_rb"] = rb
            scans.append(r)
    return scans, vw, meta


def cards_from_det(scans, det_path, fov=48.0):
    """detect_log 기록(PC 가 찾은 카드, 시각 ts) + 스캔 자세 -> 카드 지도 위치 (roommap.build 와 같은 방법을 간단히)"""
    import bisect
    sys_ts = [s["ts"] for s in scans]
    seen = {}
    for line in open(det_path, encoding="utf-8"):
        try:
            d = json.loads(line)
        except ValueError:
            continue
        ts, w = d.get("ts"), d.get("w", 640)
        if ts is None:
            continue
        k = bisect.bisect_left(sys_ts, ts)
        if k <= 0 or k >= len(scans) or abs(sys_ts[k] - ts) > 0.3:
            continue
        x, y, th = scans[k].get("_m") or scans[k]["odo"]
        fpx = (w / 2) / math.tan(math.radians(fov / 2))
        dets = d.get("dets", [])
        nums = {c: b for c, _, *b in dets if c.startswith("num")}
        for c, cf, x1, y1, x2, y2 in dets:
            if c.startswith("num") or cf < 0.5:
                continue
            dg = ""
            for nc, (a1, b1, a2, b2) in nums.items():
                if x1 <= (a1 + a2) / 2 <= x2 and b1 > y2 - 0.2 * (y2 - y1):
                    dg = nc[3:]
                    hgt = b2 - b1
                    break
            if not dg:
                continue
            dist = fpx * 0.054 / max(1.0, hgt)
            b = -math.atan(((x1 + x2) / 2 - w / 2) / fpx)
            seen.setdefault((c, dg), []).append((x + (dist + 0.21) * math.cos(th + b), y + (dist + 0.21) * math.sin(th + b)))
    out = []
    for (c, dg), pts in seen.items():
        if len(pts) >= 3:
            pts.sort()
            m = pts[len(pts) // 2]
            out.append((c, dg, m[0], m[1], len(pts)))
    return out


def draw(path, out=None, det=None, title=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = "Malgun Gothic"
    plt.rcParams["axes.unicode_minus"] = False
    scans, vw, meta = load(path)
    def P(r):                                                # 스캔의 주행거리계 자세 -> 그릴 좌표 자세 (테이프 미로 좌표, 네모가 반듯하게)
        x, y, th = r["odo"]
        if not vw or r.get("_rb") is None:
            return x, y, th
        x0, y0, t0 = r["_rb"]
        sx, sy, sd = vw["walls"].get("start", (0, 0, 0))
        st = math.radians(sd)
        X, Y = x0 + x * math.cos(t0) - y * math.sin(t0), y0 + x * math.sin(t0) + y * math.cos(t0)
        return sx + X * math.cos(st) - Y * math.sin(st), sy + X * math.sin(st) + Y * math.cos(st), th + t0 + st
    for r in scans:
        r["_m"] = P(r)
    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    xs, ys = [], []
    for r in scans[::2]:
        x, y, th = r["_m"]
        for a, d in zip(r["a"], r["d"]):
            if 0.12 < d < 4.0:
                xs.append(x + d * math.cos(th + math.radians(a)))
                ys.append(y + d * math.sin(th + math.radians(a)))
    ax.scatter(xs, ys, s=0.3, c="0.55", label="라이다가 본 방")
    stats = {}
    if vw:
        from vwalls import VirtualWalls
        w = vw["walls"]
        v = VirtualWalls(w["walls"], (0, 0, 0), door=w.get("door"))          # 미로 좌표 그대로
        for k, (x1, y1, x2, y2) in enumerate(w["walls"]):
            ax.plot([x1, x2], [y1, y2], color="#222", lw=5, solid_capstyle="round", label="테이프 (가상 벽)" if k == 0 else None)
        stats["테이프까지 가장 가까움(로봇 가운데)"] = round(min(v.inside_dist(r["_m"]) for r in scans), 3) if scans else None
    pp = [r["_m"][:2] for r in scans]
    px = [q[0] for q in pp]
    py = [q[1] for q in pp]
    ax.plot(px, py, color="#2a6fdb", lw=1.4, label="로봇이 간 길")
    if scans:
        ax.plot(px[0], py[0], "o", color="#1a9a50", ms=10, label="출발")
        ax.plot(px[-1], py[-1], "s", color="#d33", ms=8, label="끝")
    if det and os.path.exists(det):
        for c, dg, cx, cy, n in cards_from_det(scans, det):
            ax.plot(cx, cy, "*", color="#e8a400", ms=16)
            ax.annotate(f"{c} {dg}", (cx, cy), xytext=(cx + 0.06, cy + 0.06), fontsize=11)
        stats["카드"] = [(c, dg) for c, dg, *_ in cards_from_det(scans, det)]
    ax.set_aspect("equal")
    lim = 2.2
    if px:
        cx, cy = (max(px) + min(px)) / 2, (max(py) + min(py)) / 2
        ax.set_xlim(cx - lim, cx + lim)
        ax.set_ylim(cy - lim, cy + lim)
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)
    ax.set_title(title or f"테이프 네모 주행 ({len(scans)}개 스캔)")
    out = out or path.replace(".jsonl", "_tape.png")
    fig.savefig(out, dpi=90, bbox_inches="tight")
    plt.close(fig)
    return out, stats


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--det")
    ap.add_argument("--out")
    a = ap.parse_args()
    o, st = draw(a.log, a.out, a.det)
    print("그림:", o, st)
