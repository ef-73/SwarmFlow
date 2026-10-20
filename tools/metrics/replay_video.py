#!/usr/bin/env python3
"""Top-down replay video of a run from ``replay_data.py`` output (OpenCV, VP8 WebM; also writes a mid-run still).
OpenCV with FFmpeg is in the sim image, e.g.:
    docker compose run --rm --no-deps --entrypoint python3 viz /ws/tools/metrics/replay_video.py replay.json out.webm 10 24
Usage: replay_video.py <replay.json> <out.webm> [speed-up, default 10] [fps, default 24]"""
import json, math, sys
import cv2
import numpy as np
d = json.load(open(sys.argv[1]))
out = sys.argv[2]
speed = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
fps = int(sys.argv[4]) if len(sys.argv) > 4 else 24
B = d["bounds"]; S = 26                      # px per metre
PAD, PANEL = 30, 330
W = int((B["x_max"] - B["x_min"]) * S) + 2 * PAD + PANEL
H = int((B["y_max"] - B["y_min"]) * S) + 2 * PAD
W += W % 2; H += H % 2
def px(x, y):
    return int(PAD + (x - B["x_min"]) * S), int(H - PAD - (y - B["y_min"]) * S)
BG, FLOOR, RACK, TEXT, DIM = (24, 22, 20), (44, 41, 38), (92, 120, 150), (235, 235, 235), (150, 150, 150)
COL = [(80, 170, 255), (110, 220, 120), (240, 160, 80), (210, 120, 230), (90, 220, 230)]
MODECOL = {"STUCK": (60, 60, 230), "LIFTING": (0, 200, 255), "UNLOADING": (0, 200, 255)}
base = np.full((H, W, 3), BG, np.uint8)
cv2.rectangle(base, px(B["x_min"], B["y_max"]), px(B["x_max"], B["y_min"]), FLOOR, -1)
for a in d["aisles"]:
    cv2.rectangle(base, px(a["x_min"], a["y_max"]), px(a["x_max"], a["y_min"]), (52, 49, 46), -1)
for r in d["racks"]:
    cv2.rectangle(base, px(r["x_min"], r["y_max"]), px(r["x_max"], r["y_min"]), RACK, -1)
for s in d["stations"]:
    p = px(s["x"], s["y"])
    if s["type"] == "dock":
        cv2.rectangle(base, (p[0] - 22, p[1] - 14), (p[0] + 22, p[1] + 14), (60, 130, 200), 2)
        cv2.putText(base, s["name"], (p[0] - 16, p[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, DIM, 1, cv2.LINE_AA)
    elif s["type"] == "pick":
        cv2.circle(base, p, 2, (120, 120, 120), -1, cv2.LINE_AA)
    elif s["type"] == "home":
        cv2.circle(base, p, 5, (90, 90, 90), 1, cv2.LINE_AA)
    elif s["type"] == "pallet":
        cv2.rectangle(base, (p[0] - 8, p[1] - 8), (p[0] + 8, p[1] + 8), (70, 110, 90), 1)
cv2.rectangle(base, px(B["x_min"], B["y_max"]), px(B["x_max"], B["y_min"]), (200, 200, 200), 2)
x0 = W - PANEL + 10
cv2.putText(base, "SwarmFlow v2", (x0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, TEXT, 2, cv2.LINE_AA)
cv2.putText(base, "warehouse_v2, 20 x 32 m", (x0, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.5, DIM, 1, cv2.LINE_AA)
robots = d["robots"]; modes = d["modes"]
cv2.putText(base, f"10-min Gazebo run, {len(robots)} robots", (x0, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.5, DIM, 1,
            cv2.LINE_AA)
LABEL = {"WAITING_RESERVATION": "yielding"}   # v2: WAITING_RESERVATION only means yielding to another robot
idx = {r: 0 for r in robots}
trail = {r: [] for r in robots}
ev = d["events"]
vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"VP80"), fps, (W, H))
assert vw.isOpened(), "VideoWriter VP80 not available"
dur = d["duration_s"]; nfr = int(dur / speed * fps)
for f in range(nfr + 1):
    t = f * speed / fps
    img = base.copy()
    lines = []
    for i, (rid, smp) in enumerate(robots.items()):
        while idx[rid] + 1 < len(smp) and smp[idx[rid] + 1][0] <= t:
            idx[rid] += 1
        a = smp[idx[rid]]; b = smp[min(idx[rid] + 1, len(smp) - 1)]
        u = 0.0 if b[0] <= a[0] else min(max((t - a[0]) / (b[0] - a[0]), 0.0), 1.0)
        x = a[1] + u * (b[1] - a[1]); y = a[2] + u * (b[2] - a[2])
        dy = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])); yaw = a[3] + u * dy
        mode = modes[a[4]]
        trail[rid].append(px(x, y)); trail[rid] = trail[rid][-int(fps * 2.5):]
        col = COL[i % len(COL)]
        for k in range(1, len(trail[rid])):
            c = tuple(int(v * (0.25 + 0.6 * k / len(trail[rid]))) for v in col)
            cv2.line(img, trail[rid][k - 1], trail[rid][k], c, 2, cv2.LINE_AA)
        c, s_ = math.cos(yaw), math.sin(yaw)
        pts = np.array([px(x + c * dx - s_ * dy_, y + s_ * dx + c * dy_) for dx, dy_ in
                        ((0.35, 0.3), (0.35, -0.3), (-0.35, -0.3), (-0.35, 0.3))], np.int32)
        cv2.fillPoly(img, [pts], col, cv2.LINE_AA)
        cv2.polylines(img, [pts], True, MODECOL.get(mode, (20, 20, 20)), 3 if mode in MODECOL else 1, cv2.LINE_AA)
        cv2.line(img, px(x, y), px(x + 0.45 * c, y + 0.45 * s_), (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(img, rid[-1], (px(x, y)[0] - 4, px(x, y)[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (10, 10, 10), 2, cv2.LINE_AA)
        lines.append((rid, mode, a[5], col))
    done = [e for e in ev if e[0] <= t and e[1] == "DELIVERED"]
    y0 = 130
    cv2.putText(img, f"t = {int(t // 60):02d}:{int(t % 60):02d}  ({speed:g}x)", (x0, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.7, TEXT, 2, cv2.LINE_AA)
    cv2.putText(img, f"deliveries: {len(done)}", (x0, y0 + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (120, 230, 140), 2, cv2.LINE_AA)
    for j, (rid, mode, oid, col) in enumerate(lines):
        yy = y0 + 80 + j * 30
        cv2.rectangle(img, (x0, yy - 14), (x0 + 16, yy + 2), col, -1)
        cv2.putText(img, f"{rid}  {LABEL.get(mode, mode.lower()):<10} {oid}", (x0 + 24, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                    (90, 90, 240) if mode == "STUCK" else TEXT, 1, cv2.LINE_AA)
    yy = y0 + 100 + len(lines) * 30
    cv2.putText(img, "recent:", (x0, yy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, DIM, 1, cv2.LINE_AA)
    for j, e in enumerate([e for e in ev if e[0] <= t][-8:][::-1]):
        cv2.putText(img, f"{int(e[0] // 60):02d}:{int(e[0] % 60):02d} {e[2]} {e[1].lower()} {e[3]}", (x0, yy + 24 + j * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 230, 140) if e[1] == "DELIVERED" else DIM, 1, cv2.LINE_AA)
    vw.write(img)
    if f == nfr // 2:
        cv2.imwrite(out.rsplit(".", 1)[0] + "_frame.png", img)
vw.release()
print(out, nfr + 1, "frames", W, "x", H)
