#!/usr/bin/env python3
"""Compact replay JSON of one run (robot poses and modes in the scenario window, order events, layout geometry), the
input of ``replay_video.py``. Usage: replay_data.py <run_dir> <layout.yaml> <out.json> [samples per second]"""
import json
import sys

import yaml

MODES = ["IDLE", "NAVIGATING", "CARRYING", "LIFTING", "UNLOADING", "STUCK", "WAITING_RESERVATION", "FAULT"]


def build(run: str, lay: str, hz: float = 4.0) -> dict:
    L = yaml.safe_load(open(lay, encoding="utf-8"))
    cfg = yaml.safe_load(open(run + "/config.yaml", encoding="utf-8"))
    t0 = float(cfg.get("start_time_sim_s") or 0) + float(cfg.get("start_delay_s") or 0)
    dur = float(cfg["scenario"]["duration_s"])
    rows = [json.loads(l) for l in open(run + "/robot_states.jsonl") if l.strip()]
    rob = {}
    for r in rows:
        t = r["t"] - t0
        if t < 0 or t > dur:
            continue
        k = round(t * hz)
        d = rob.setdefault(r["robot_id"], {})
        d[k] = [round(t, 2), round(r["x"], 2), round(r["y"], 2), round(r["yaw"], 2),
                MODES.index(r["mode"]) if r["mode"] in MODES else 0, r.get("order_id", "")]
    robots = {rid: [v for _, v in sorted(d.items())] for rid, d in sorted(rob.items())}
    ev = []
    for o in (json.loads(l) for l in open(run + "/orders.jsonl") if l.strip()):
        if o["state"] in ("DELIVERED", "QUEUED", "FAILED") and 0 <= o["t"] - t0 <= dur:
            ev.append([round(o["t"] - t0, 1), o["state"], o["order_id"], o.get("robot_id", "")])
    st = [{"name": s["name"], "type": s["type"], "x": s["pose"]["x"], "y": s["pose"]["y"]} for s in L["stations"]]
    return {"run": run.rstrip("/").split("/")[-1], "duration_s": dur, "modes": MODES,
            "bounds": L["bounds"], "racks": L["racks"], "aisles": L["aisles"], "stations": st,
            "robots": robots, "events": sorted(ev)}


if __name__ == "__main__":
    run, lay, out = sys.argv[1:4]
    d = build(run, lay, float(sys.argv[4]) if len(sys.argv) > 4 else 4.0)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(d, f, separators=(",", ":"))
    print(out, sum(len(v) for v in d["robots"].values()), "samples", len(d["events"]), "events")
