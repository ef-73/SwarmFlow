#!/usr/bin/env python3
"""Coordination highlight reel (T072): real moments of a run where robots yielded to each other or broke a
deadlock, slowed down, with the robots involved ringed and labelled and every robot's planned path drawn.

    docker compose run --rm --no-deps -T --entrypoint python3 \
        -v "$PWD/runs:/ws/runs" -v "$PWD/tools/metrics:/ws/tools/metrics" -v "$PWD/layouts:/ws/layouts" viz \
        /ws/tools/metrics/make_highlights.py /ws/runs/<id> [out.mp4] [--extra /ws/runs/<other run> ...]

Moments the run's "central" camera saw are cut from its footage (record_demo.sh). Deadlock breaks from ``--extra``
runs (no camera) are shown as a top-down replay of their recorded robot states, labelled as such. Nothing is staged:
every frame is the run as Gazebo simulated it; only the playback speed differs, and it is shown on screen.
"""
import math
import os
import pathlib
import sys

import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from highlights import caption, coord_events, moments, pick  # noqa: E402
from make_demo_video import (FPS, F_CAP, F_HUD, H, ROBOT_RGB, W, card, draw_paths, encode, font,  # noqa: E402
                             pill, read_jsonl, robot_plans, robot_tracks, stamps_of)
from video_cams import CAMERAS, frames_for, planned, project, trail  # noqa: E402

SPEED = 2.0               #: playback speed of the clips (sim seconds per video second)
PAD = {"yield": (4.0, 4.0), "unjam": (8.0, 12.0)}   #: (lead-in, follow-through) per moment kind: a deadlock break
                                                     #: shows the jam before and both robots leaving after
F_LABEL = font(30, True)
LAYOUT = pathlib.Path("/ws/layouts/warehouse_v2/layout.yaml")


def index_states(states):
    by = {}
    for s in states:
        by.setdefault(s["robot_id"], []).append(s)
    return by


def pose(by, rid, t):
    rs = by.get(rid, [])
    lo, hi = 0, len(rs)
    while lo < hi:
        mid = (lo + hi) // 2
        if rs[mid]["t"] <= t:
            lo = mid + 1
        else:
            hi = mid
    s = rs[max(0, lo - 1)] if rs else None
    return (s["x"], s["y"], s["yaw"]) if s else None


def visible_in(cam, by, m, margin=120):
    for t in (m.t0, m.t1):
        for rid in (m.actor, *m.others):
            p = pose(by, rid, t)
            uv = project(cam, p[0], p[1], 0.3) if p else None
            if uv is None or not (margin <= uv[0] <= W - margin and margin <= uv[1] <= H - margin):
                return False
    return True


def labels(img, items):
    """items: [(u, v, text, rgb)] -> ring + pill label per robot, each label on the side away from the nearest other
    robot so labels of robots close together do not overlap."""
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    for k, (u, v, text, rgb) in enumerate(items):
        r = 46
        d.ellipse((u - r, v - r, u + r, v + r), outline=rgb + (255,), width=6)
        tw = d.textlength(text, font=F_LABEL)
        near = [w for j, (_, w, _, _) in enumerate(items) if j != k and abs(items[j][0] - u) < 400]
        below = bool(near) and min(near, key=lambda w: abs(w - v)) < v      # the closest other robot is above
        if not near:
            below = k > 0
        x, y = u - tw / 2, (v + r + 18) if below else (v - r - 62)
        d.rounded_rectangle((x - 16, y - 8, x + tw + 16, y + 44), radius=12, fill=(15, 18, 24, 200))
        d.text((x, y), text, font=F_LABEL, fill=rgb + (255,))
    return Image.alpha_composite(img, layer)


def frame_overlay(bgr, head, sub, hud, items):
    img = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)).convert("RGBA")
    img = labels(img, items)
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    pill(d, (70, 56), head, F_CAP)
    pill(d, (70, 140), sub, F_HUD, alpha=150)
    pill(d, (70, H - 100), hud, F_HUD, alpha=150)
    return cv2.cvtColor(np.array(Image.alpha_composite(img, layer).convert("RGB")), cv2.COLOR_RGB2BGR)


def roles(m, t):
    """[(robot, label, rgb)] for the robots in a moment at time t."""
    amber, blue = (242, 165, 65), (93, 169, 233)
    if m.kind == "yield":
        waiting = m.t0 <= t <= m.t1
        return [(m.actor, "waits" if waiting else ("gives way" if t < m.t0 else "goes on"), amber)] + \
               [(o, "reserved first", blue) for o in m.others]
    backing = m.t0 <= t <= m.t1
    return [(m.actor, "backs out" if backing else ("blocked" if t < m.t0 else "drives on"), amber)] + \
           [(o, "blocked" if t < m.t1 else "drives on", blue) for o in m.others]


def write_camera_clip(writer, run, cam_name, m, states_by, tracks, plans, t_scn):
    cam = CAMERAS[cam_name]
    stamps = stamps_of(run / "video", cam_name)
    head, sub = caption(m)
    pre, post = PAD[m.kind]
    idx = frames_for(stamps, m.t0 - pre, m.t1 + post, SPEED, FPS)
    cap = cv2.VideoCapture(str(run / "video" / f"{cam_name}.avi"))
    cap.set(cv2.CAP_PROP_POS_FRAMES, idx[0])
    pos, frame, n = idx[0] - 1, None, 0
    for i in idx:
        while pos < i:
            ok, f = cap.read()
            if not ok:
                break
            frame, pos = f, pos + 1
        if frame is None:
            continue
        t = stamps[i]
        shot = draw_paths(frame, cam, tracks, plans, t) if plans else frame
        items = []
        for rid, text, rgb in roles(m, t):
            p = pose(states_by, rid, t)
            uv = project(cam, p[0], p[1], 0.3) if p else None
            if uv:
                items.append((uv[0], uv[1], f"{rid} · {text}", rgb))
        mm, ss = divmod(max(0.0, t - t_scn), 60)
        writer.write(frame_overlay(shot, head, sub, f"real Gazebo footage · sim {int(mm):02d}:{int(ss):02d} · ×{SPEED:g}",
                                   items))
        n += 1
    cap.release()
    return n


def topdown_frame(lay, states_by, plans, tracks, m, t, center, scale):
    """A top-down view of the moment from the recorded robot states (for runs no camera filmed)."""
    img = np.full((H, W, 3), (28, 24, 22), np.uint8)
    cx, cy = center

    def px(x, y):
        return int(W / 2 + (x - cx) * scale), int(H / 2 - (y - cy) * scale)
    b = lay["bounds"]
    cv2.rectangle(img, px(b["x_min"], b["y_max"]), px(b["x_max"], b["y_min"]), (48, 44, 41), -1)
    for r in lay["racks"]:
        cv2.rectangle(img, px(r["x_min"], r["y_max"]), px(r["x_max"], r["y_min"]), (70, 105, 140), -1)
    for rid, recs in plans.items():
        ahead = planned(recs, t)
        here = trail(tracks.get(rid, []), t, 0.5)
        pts = [px(x, y) for x, y in ([here[-1]] if here else []) + ahead]
        rgb = ROBOT_RGB.get(rid, (128, 128, 128))
        for a, c in zip(pts, pts[1:]):
            cv2.line(img, a, c, rgb[::-1], 4, cv2.LINE_AA)
    for rid in states_by:
        p = pose(states_by, rid, t)
        if not p:
            continue
        x, y, yaw = p
        box = cv2.boxPoints(((W / 2 + (x - cx) * scale, H / 2 - (y - cy) * scale), (0.7 * scale, 0.6 * scale),
                             -math.degrees(yaw)))
        cv2.fillPoly(img, [box.astype(np.int32)], ROBOT_RGB.get(rid, (128, 128, 128))[::-1], cv2.LINE_AA)
        hx, hy = px(x + 0.4 * math.cos(yaw), y + 0.4 * math.sin(yaw))
        cv2.line(img, px(x, y), (hx, hy), (240, 240, 240), 3, cv2.LINE_AA)
    items = []
    for rid, text, rgb in roles(m, t):
        p = pose(states_by, rid, t)
        if p:
            u, v = px(p[0], p[1])
            items.append((u, v, f"{rid} · {text}", rgb))
    return img, items


def write_topdown_clip(writer, run, m, t_scn):
    lay = yaml.safe_load(LAYOUT.read_text())
    states = read_jsonl(run / "robot_states.jsonl")
    by = index_states(states)
    tracks = robot_tracks(states)
    traj = run / "trajectories.jsonl"
    plans = robot_plans(read_jsonl(traj)) if traj.exists() else {}
    ps = [pose(by, r, m.t0) for r in (m.actor, *m.others)]
    center = (sum(p[0] for p in ps) / len(ps), sum(p[1] for p in ps) / len(ps))
    head, sub = caption(m)
    pre, post = PAD[m.kind]
    n = int((m.t1 - m.t0 + pre + post) / SPEED * FPS)
    for k in range(n):
        t = m.t0 - pre + k * SPEED / FPS
        img, items = topdown_frame(lay, by, plans, tracks, m, t, center, 150.0)
        mm, ss = divmod(max(0.0, t - t_scn), 60)
        writer.write(frame_overlay(img, head, sub,
                                   f"top-down replay of recorded robot states (run {run.name}) · sim "
                                   f"{int(mm):02d}:{int(ss):02d} · ×{SPEED:g}", items))
    return n


def t_scenario(run):
    cfg = yaml.safe_load((run / "config.yaml").read_text())
    return cfg["start_time_sim_s"] + (cfg.get("start_delay_s") or 0)


def main():
    args = sys.argv[1:]
    extra = [pathlib.Path(args[i + 1]) for i, a in enumerate(args) if a == "--extra"]
    pos = [a for i, a in enumerate(args) if not a.startswith("--") and (i == 0 or args[i - 1] != "--extra")]
    run = pathlib.Path(pos[0])
    out = pathlib.Path(pos[1]) if len(pos) > 1 else run / "swarmflow_coordination.mp4"
    states = read_jsonl(run / "robot_states.jsonl")
    by = index_states(states)
    tracks = robot_tracks(states)
    traj = run / "trajectories.jsonl"
    plans = robot_plans(read_jsonl(traj)) if traj.exists() else {}
    ms = moments(coord_events((run / "compose.log").read_text(encoding="utf-8", errors="ignore")))
    chosen = pick(ms, lambda m: visible_in(CAMERAS["central"], by, m))
    print("moments in run:", len(ms), "chosen:", [(m.kind, m.actor, m.others, round(m.t0)) for m in chosen])
    fallback = []
    if not any(m.kind == "unjam" for m in chosen):
        for e in extra:
            em = [m for m in moments(coord_events((e / "compose.log").read_text(encoding="utf-8", errors="ignore")))
                  if m.kind == "unjam"]
            fallback += [(e, m) for m in em[:1]]
            if fallback:
                break
    print("top-down deadlock breaks:", [(e.name, m.actor, m.others, round(m.t0)) for e, m in fallback])

    master = out.with_suffix(".master.avi")
    writer = cv2.VideoWriter(str(master), cv2.VideoWriter_fourcc(*"MJPG"), FPS, (W, H))
    writer.set(cv2.VIDEOWRITER_PROP_QUALITY, 97)
    cap = cv2.VideoCapture(str(run / "video" / "central.avi"))
    ok, first = cap.read()
    cap.release()
    t_scn = t_scenario(run)
    for k in range(int(3.5 * FPS)):
        writer.write(card(first, "Robots that sort out their own traffic",
                          ["Real moments from a Gazebo run: five robots, no traffic controller.",
                           "Each robot publishes its planned path; the one that reserved later gives way."],
                          min(1.0, k / (0.5 * FPS))))
    n = 0
    for m in chosen:
        n += write_camera_clip(writer, run, "central", m, by, tracks, plans, t_scn)
    for e, m in fallback:
        n += write_topdown_clip(writer, e, m, t_scenario(e))
    last = first
    for k in range(int(3.5 * FPS)):
        writer.write(card(last, "SwarmFlow",
                          ["Highlights from real runs; the full, unedited results are in docs/results.",
                           "github.com/ef-73/SwarmFlow"], min(1.0, k / (0.5 * FPS))))
    writer.release()
    print(f"{master}: {n} clip frames")
    encode(master, out)


if __name__ == "__main__":
    main()
