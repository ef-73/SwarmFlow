#!/usr/bin/env python3
"""Edit the 3D demo video from a record_demo.sh run: 1080p H.264 MP4 plus a small GIF preview for the README.

    docker compose run --rm --no-deps -T --entrypoint python3 \
        -v "$PWD/runs:/ws/runs" -v "$PWD/tools/metrics:/ws/tools/metrics" viz \
        /ws/tools/metrics/make_demo_video.py /ws/runs/<id> [out.mp4] [preview.gif]

Cuts (video_cams.plan_cuts): wide shot of the fleet, the pallet pick-up close up, the carry, the set-down at the dock,
wide to the end. Every frame is taken by SIM time (video_cams.frames_for), so the speed-ups shown are exact.
Needs OpenCV and Pillow (both in the sim image). The edit is written as a near-lossless MJPEG master
(``<out>.master.avi``); OpenCV's own H.264 writer ignores quality settings, so the final MP4 is encoded with ffmpeg
(x264, CRF 17) when an ``ffmpeg`` binary is on PATH, else by OpenCV at its default quality. With Docker:
    docker run --rm -v "$PWD/runs:/r" linuxserver/ffmpeg:7.1.1 -y -i /r/<id>/swarmflow_3d_demo.master.avi         -c:v libx264 -crf 17 -preset slow -pix_fmt yuv420p -movflags +faststart /r/<id>/swarmflow_3d_demo.mp4
"""
import csv
import json
import os
import pathlib
import shutil
import subprocess
import sys

import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from video_cams import CAMERAS, deliveries_until, frames_for, pallet_moves, plan_cuts, planned, project, trail  # noqa: E402

FPS = 30
W, H = 1920, 1080
PATH_Z = 0.03            #: planned paths are drawn just above the floor (T071)
# robot colours as in src/swarmflow_gazebo/launch/sim.launch.py COLORS (RGB 0-255)
ROBOT_RGB = {"robot_1": (26, 115, 217), "robot_2": (242, 140, 26), "robot_3": (51, 179, 77), "robot_4": (191, 51, 179),
             "robot_5": (26, 179, 191)}
FONT_DIRS = ["/usr/share/fonts/truetype/open-sans", "/usr/share/fonts/truetype/dejavu"]


def font(size, bold=False):
    names = (["OpenSans-Bold.ttf", "OpenSans-SemiBold.ttf", "DejaVuSans-Bold.ttf"] if bold
             else ["OpenSans-Regular.ttf", "DejaVuSans.ttf"])
    for d in FONT_DIRS:
        for n in names:
            p = os.path.join(d, n)
            if os.path.exists(p):
                return ImageFont.truetype(p, size)
    return ImageFont.load_default()


F_TITLE, F_SUB, F_CAP, F_HUD = font(120, True), font(40), font(40, True), font(30)


def read_jsonl(p):
    return [json.loads(line) for line in open(p, encoding="utf-8") if line.strip()]


def stamps_of(video_dir, cam):
    with open(video_dir / f"{cam}.csv", newline="") as f:
        return [float(r["t"]) for r in csv.DictReader(f)]


def pill(draw, xy, text, fnt, alpha=165, pad=(26, 14)):
    x, y = xy
    l, t, r, b = draw.textbbox((x, y), text, font=fnt)
    draw.rounded_rectangle((l - pad[0], t - pad[1], r + pad[0], b + pad[1]), radius=16, fill=(15, 18, 24, alpha))
    draw.text((x, y), text, font=fnt, fill=(255, 255, 255, 255))


def overlay(frame, caption, cap_alpha, hud):
    img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).convert("RGBA")
    layer = Image.new("RGBA", img.size, (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    if caption and cap_alpha > 0:
        tmp = Image.new("RGBA", img.size, (0, 0, 0, 0))
        pill(ImageDraw.Draw(tmp), (70, 60), caption, F_CAP)
        tmp.putalpha(tmp.getchannel("A").point(lambda a: int(a * cap_alpha)))
        layer = Image.alpha_composite(layer, tmp)
        d = ImageDraw.Draw(layer)
    pill(d, (70, H - 100), hud, F_HUD, alpha=150)
    tag = "SwarmFlow  ·  ROS 2 Jazzy + Nav2  ·  Gazebo Harmonic"
    tw = d.textlength(tag, font=F_HUD)
    pill(d, (W - 70 - tw, H - 100), tag, F_HUD, alpha=110)
    return cv2.cvtColor(np.array(Image.alpha_composite(img, layer).convert("RGB")), cv2.COLOR_RGB2BGR)


def robot_tracks(states):
    """robot -> [(t, x, y)] sorted by time."""
    out = {}
    for s in states:
        out.setdefault(s["robot_id"], []).append((float(s["t"]), float(s["x"]), float(s["y"])))
    return {r: sorted(v) for r, v in out.items()}


def robot_plans(rows):
    """robot -> [(recorded time, points)] from trajectories.jsonl rows, sorted by time."""
    out = {}
    for r in rows:
        out.setdefault(r["robot_id"], []).append((float(r["t"]), r["points"]))
    return {k: sorted(v, key=lambda e: e[0]) for k, v in out.items()}


def draw_paths(frame, cam, tracks, plans, t):
    """Each robot's planned path ahead of it (its reserved trajectory, T071) as a line in its colour, strong at the
    robot and fading towards the end of the plan; robots standing or yielding have none. Drawn over the scene."""
    layer = frame.copy()
    for rid, recs in sorted(plans.items()):
        ahead = planned(recs, t)
        if not ahead:
            continue
        here = trail(tracks.get(rid, []), t, 0.5)
        pts = ([here[-1]] if here else []) + ahead
        pts = [project(cam, x, y, PATH_Z) for x, y in pts]
        pts = [(int(round(u * 8)), int(round(v * 8))) for u, v in (p for p in pts if p is not None)]
        if len(pts) < 2:
            continue
        r, g, b = ROBOT_RGB.get(rid, (128, 128, 128))
        n = len(pts)
        for k, (a, c) in enumerate(zip(pts, pts[1:])):
            w = 1.0 - k / n                                   # 1 at the robot .. 0 at the end of the plan
            col = tuple(int(255 + (v - 255) * (0.3 + 0.7 * w)) for v in (b, g, r))
            cv2.line(layer, a, c, col, 6, cv2.LINE_AA, shift=3)
    return cv2.addWeighted(layer, 0.85, frame, 0.15, 0)


def card(frame, title, sub, dim):
    img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).convert("RGBA")
    layer = Image.new("RGBA", img.size, (10, 12, 16, int(200 * dim)))
    d = ImageDraw.Draw(layer)
    a = int(255 * dim)
    tw = d.textlength(title, font=F_TITLE)
    d.text(((W - tw) / 2, H / 2 - 120), title, font=F_TITLE, fill=(255, 255, 255, a))
    for i, line in enumerate(sub):
        sw = d.textlength(line, font=F_SUB)
        d.text(((W - sw) / 2, H / 2 + 40 + i * 58), line, font=F_SUB, fill=(215, 222, 232, a))
    return cv2.cvtColor(np.array(Image.alpha_composite(img, layer).convert("RGB")), cv2.COLOR_RGB2BGR)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    run = pathlib.Path(args[0])
    out = pathlib.Path(args[1]) if len(args) > 1 else run / "swarmflow_3d_demo.mp4"
    gif = pathlib.Path(args[2]) if len(args) > 2 else run / "swarmflow_3d_preview.gif"
    vdir = run / "video"
    states, orders = read_jsonl(run / "robot_states.jsonl"), read_jsonl(run / "orders.jsonl")
    tracks = robot_tracks(states)
    traj = run / "trajectories.jsonl"
    plans = {} if "--no-paths" in sys.argv or not traj.exists() else robot_plans(read_jsonl(traj))
    cfg = yaml.safe_load((run / "config.yaml").read_text())
    t_scn = cfg["start_time_sim_s"] + (cfg.get("start_delay_s") or 0)
    stamps = {c: stamps_of(vdir, c) for c in ("wide", "pallets", "docks")}
    moves = pallet_moves(states, orders)
    move = next((m for m in moves if m.order_id == "video_pallet"), moves[0] if moves else None)
    t_start = max(stamps["wide"][0], t_scn - 3)
    t_end = stamps["wide"][-1]
    cuts = plan_cuts(move, t_start, t_end)
    print("pallet move:", move)
    for c in cuts:
        print(f"  {c.cam:8s} {c.t0:7.1f}-{c.t1:7.1f}  x{c.speed:g}  {(c.t1 - c.t0) / c.speed:5.1f} s  {c.caption}")

    master = out.with_suffix(".master.avi")
    writer = cv2.VideoWriter(str(master), cv2.VideoWriter_fourcc(*"MJPG"), FPS, (W, H))
    writer.set(cv2.VIDEOWRITER_PROP_QUALITY, 97)
    n_out, timeline = 0, []
    intro, fade, outro = int(3.0 * FPS), int(0.5 * FPS), int(3.5 * FPS)
    cap = cv2.VideoCapture(str(vdir / "wide.avi"))
    cap.set(cv2.CAP_PROP_POS_FRAMES, frames_for(stamps["wide"], t_start, t_start + 1, 1, 1)[0])
    ok, first = cap.read()
    cap.release()
    first = overlay(first, "", 0, "sim 00:00")
    for k in range(intro):   # title over the still opening shot, then the run starts
        writer.write(card(first, "SwarmFlow", [f"A real Gazebo run: {len(robot_tracks(states))} robots, Nav2, shelf orders and a pallet move",
                                               "recorded in simulation time"], 1.0 if k < intro - fade else (intro - k) / fade))
    n_out, last = intro, first
    for ci, c in enumerate(cuts):
        idx = frames_for(stamps[c.cam], c.t0, c.t1, c.speed, FPS)
        cap = cv2.VideoCapture(str(vdir / f"{c.cam}.avi"))
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx[0])
        pos, frame = idx[0] - 1, None
        timeline.append((c, n_out))
        for k, i in enumerate(idx):
            while pos < i:
                ok, f = cap.read()
                if not ok:
                    break
                frame, pos = f, pos + 1
            if frame is None:
                continue
            t = stamps[c.cam][i]
            shot = draw_paths(frame, CAMERAS[c.cam], tracks, plans, t) if plans else frame
            mins, secs = divmod(max(0.0, t - t_scn), 60)
            hud = f"sim {int(mins):02d}:{int(secs):02d}   ×{c.speed:g}   {deliveries_until(orders, t)} delivered"
            img = overlay(shot, c.caption, min(1.0, k / fade) if k < 4 * FPS else max(0.0, 1 - (k - 4 * FPS) / fade),
                          hud)
            if k < fade:
                img = cv2.addWeighted(last, 1 - k / fade, img, k / fade, 0)
            writer.write(img)
            n_out += 1
            if k == len(idx) - 1:
                last = img
        cap.release()
    total = deliveries_until(orders, t_end)
    for k in range(outro):
        writer.write(card(last, "SwarmFlow", [f"{total} orders delivered in {int((t_end - t_scn) // 60)} sim minutes",
                                              "github.com/ef-73/SwarmFlow"], min(1.0, k / fade)))
    writer.release()
    print(f"{master}: {n_out + outro} frames, {(n_out + outro) / FPS:.1f} s")
    encode(master, out)
    make_gif(master, gif, timeline)


def encode(master, out):
    if shutil.which("ffmpeg"):
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(master), "-c:v", "libx264", "-crf", "17",
                        "-preset", "slow", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(out)], check=True)
    else:
        cap, w = cv2.VideoCapture(str(master)), cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"avc1"), FPS, (W, H))
        while True:
            ok, f = cap.read()
            if not ok:
                break
            w.write(f)
        w.release()
        print("no ffmpeg on PATH: encoded with OpenCV at its default quality; see the docstring for the ffmpeg command")
    print(f"{out}: {out.stat().st_size / 1e6:.1f} MB")


def make_gif(video, gif, timeline, width=960, fps=15, max_s=16.0):
    """Preview from the pallet pick-up close up on, into the wide shot of the carry (README hero)."""
    pick = next(((c, s) for c, s in timeline if c.cam == "pallets"), None)
    if pick is None:
        start, stop = 3 * FPS, 3 * FPS + int(max_s * FPS)
    else:
        start = max(3 * FPS, pick[1] - 4 * FPS) if pick[1] > 3 * FPS + 4 * FPS else pick[1]
        stop = start + int(max_s * FPS)
    cap = cv2.VideoCapture(str(video))
    cap.set(cv2.CAP_PROP_POS_FRAMES, start)
    frames = []
    for k in range(stop - start):
        ok, f = cap.read()
        if not ok:
            break
        if k % (FPS // fps) == 0:
            f = cv2.resize(f, (width, width * H // W), interpolation=cv2.INTER_AREA)
            frames.append(Image.fromarray(cv2.cvtColor(f, cv2.COLOR_BGR2RGB)))
    picks = frames[:: max(1, len(frames) // 8)][:8]          # one palette for all frames: no flicker
    sample = Image.new("RGB", (width, width * H // W * len(picks)))
    for i, f in enumerate(picks):
        sample.paste(f, (0, i * width * H // W))
    pal = sample.quantize(colors=256, method=Image.Quantize.MAXCOVERAGE)
    q = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
    q[0].save(gif, save_all=True, append_images=q[1:], duration=int(1000 / fps), loop=0, optimize=True)
    print(f"{gif}: {len(q)} frames, {gif.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
