#!/usr/bin/env python3
"""Check a run's dock-line claims: every place at a dock (the dock or a queue spot) was claimed by a robot near its
dock's lane entry, from the recorded decisions and robot poses.

Usage: dock_line_check.py <run_dir> [layout.yaml] [radius_m]
Prints one line per claim (time, robot, order, place, distance from the robot's recorded pose to the lane entry) and a
summary; exits 1 if a claim was made farther than ``radius_m`` (default 3.0 plus 0.5 m for pose sampling) from the
entry or a delivery happened without a claim.
"""

from __future__ import annotations

import bisect
import json
import math
import pathlib
import sys

import yaml

REPO = pathlib.Path(__file__).resolve().parents[2]


def lane_entries(layout: dict) -> dict:
    """dock -> (x, y) of the lane entry: the neighbour of its last queue spot that is neither a queue spot nor a dock."""
    kinds = {s["name"]: s["type"] for s in layout["stations"]}
    pos = {s["name"]: (s["pose"]["x"], s["pose"]["y"]) for s in layout["stations"]}
    pos.update({i["name"]: (i["pose"]["x"], i["pose"]["y"]) for i in layout.get("intersections", [])})
    last = {}
    for s in layout["stations"]:
        if s["type"] == "queue":
            d, k = s["queue_of"], s["queue_index"]
            if k > last.get(d, (0, ""))[0]:
                last[d] = (k, s["name"])
    out = {}
    for dock, (_, q) in last.items():
        for lane in layout["lanes"]:
            other = lane["to"] if lane["from"] == q else lane["from"] if lane["to"] == q else None
            if other and kinds.get(other) not in ("queue", "dock") and other in pos:
                out[dock] = pos[other]
    return out


def read_jsonl(p: pathlib.Path) -> list:
    return [json.loads(line) for line in p.read_text(encoding="utf-8").splitlines() if line.strip()]


def check(run_dir: pathlib.Path, layout_path: pathlib.Path, radius: float) -> int:
    entries = lane_entries(yaml.safe_load(layout_path.read_text(encoding="utf-8")))
    poses: dict = {}
    for s in read_jsonl(run_dir / "robot_states.jsonl"):
        poses.setdefault(s["robot_id"], []).append((s["t"], s["x"], s["y"]))
    for v in poses.values():
        v.sort()
    claims = [d for d in read_jsonl(run_dir / "decisions.jsonl")
              if d.get("decision_type") == "RESERVATION_GRANT" and d.get("trigger") == "arrived_at_dock_line"]
    bad, claimed = 0, set()
    for d in claims:
        place = d["new_decision"].split("-> ")[-1]
        dock = place if place in entries else place.split("_")[1] if place.startswith("Q_") else ""
        seq = poses.get(d["robot_id"], [])
        i = max(0, bisect.bisect_right([p[0] for p in seq], d["t"]) - 1)
        dist = math.hypot(seq[i][1] - entries[dock][0], seq[i][2] - entries[dock][1]) if seq and dock in entries \
            else math.inf
        ok = dist <= radius + 0.5
        bad += not ok
        claimed.add(d["order_id"])
        print(f"{d['t']:8.1f}  {d['robot_id']}  {d['order_id']:<14} {place:<9} {dist:5.2f} m  {'ok' if ok else 'FAR'}")
    delivered = {o["order_id"] for o in read_jsonl(run_dir / "orders.jsonl") if o["state"] == "DELIVERED"}
    unclaimed = sorted(delivered - claimed)
    print(f"claims {len(claims)}, farther than {radius + 0.5:.1f} m: {bad}; delivered {len(delivered)}, "
          f"delivered without a line claim: {len(unclaimed)} {unclaimed if unclaimed else ''}")
    return 1 if bad or unclaimed else 0


if __name__ == "__main__":
    run = pathlib.Path(sys.argv[1])
    layout = pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else REPO / "layouts" / "warehouse_v2" / "layout.yaml"
    sys.exit(check(run, layout, float(sys.argv[3]) if len(sys.argv) > 3 else 3.0))
