#!/usr/bin/env python3
"""Compute metrics.csv for one run directory (design §13.1, §13.5). Usage: compute.py <run_dir>"""

from __future__ import annotations

import csv
import json
import pathlib
import re
import sys

try:
    import swarmflow_core  # noqa: F401
except ImportError:  # running from a checkout without an installed package
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "src" / "swarmflow_core"))

import yaml

from swarmflow_core.metrics.clearance import count_violations

COLUMNS = ["run_id", "scenario", "policy", "seed", "duration_s", "deliveries", "throughput_per_min",
           "mean_latency_s", "total_wait_s", "mean_wait_s", "stuck_events", "failed_orders", "clearance_violations"]
SPEED_EPS = 0.05


def read_jsonl(path: pathlib.Path) -> list:
    if not path.is_file():
        raise FileNotFoundError(str(path))
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def fmt(v) -> str:
    if isinstance(v, int):
        return str(v)
    return f"{v:.3f}"


def compute(run_dir: pathlib.Path) -> dict:
    cfg_path = run_dir / "config.yaml"
    if not cfg_path.is_file():
        raise FileNotFoundError(str(cfg_path))
    cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
    sc = cfg.get("scenario") or {}
    orders = read_jsonl(run_dir / "orders.jsonl")
    decisions = read_jsonl(run_dir / "decisions.jsonl")
    states = read_jsonl(run_dir / "robot_states.jsonl")

    m = re.match(r"^(?P<name>.+)-(?P<policy>[^-]+)-s(?P<seed>\d+)(?:-.*)?$", run_dir.name)
    scenario = sc.get("name") or (m.group("name") if m else run_dir.name)
    policy = next((d["policy"] for d in decisions if d.get("policy")), None) or sc.get("policy") \
        or (m.group("policy") if m else "")
    seed = sc.get("seed", int(m.group("seed")) if m else 0)
    duration = sc.get("duration_s")
    if duration is None:
        duration = max((s["t"] for s in states), default=0.0)
    duration = float(duration)

    # Scenario window (M9 review D3): runs last longer in wall time than the scenario, so count only what happened in
    # [scenario start, scenario start + duration_s] when the scenario engine recorded its start (sim seconds).
    if cfg.get("start_time_sim_s") is not None:
        t0 = float(cfg["start_time_sim_s"]) + float(cfg.get("start_delay_s") or 0.0)
        t1 = t0 + duration
        orders = [o for o in orders if o["t"] <= t1]
        decisions = [d for d in decisions if d.get("t", t0) <= t1]
        states = [s for s in states if t0 <= s["t"] <= t1]

    first_queued, last_state, delivered_t = {}, {}, {}
    for o in sorted(orders, key=lambda r: r["t"]):
        oid = o["order_id"]
        if o["state"] == "QUEUED":
            first_queued.setdefault(oid, o["t"])
        last_state[oid] = o["state"]
        if o["state"] == "DELIVERED":
            delivered_t[oid] = o["t"]
    delivered = [oid for oid, st in last_state.items() if st == "DELIVERED"]
    failed = sum(1 for st in last_state.values() if st == "FAILED")
    lats = [delivered_t[oid] - first_queued[oid] for oid in delivered if oid in first_queued]
    mean_lat = sum(lats) / len(lats) if lats else 0.0
    throughput = len(delivered) / (duration / 60.0) if duration > 0 else 0.0

    by_robot, by_t = {}, {}
    for s in states:
        by_robot.setdefault(s["robot_id"], []).append(s)
        by_t.setdefault(s["t"], {})[s["robot_id"]] = (s["x"], s["y"], s["yaw"])
    total_wait = 0.0
    for samples in by_robot.values():
        samples.sort(key=lambda r: r["t"])
        for a, b in zip(samples, samples[1:]):
            if a.get("mode") == "WAITING_RESERVATION" or (
                    a.get("mode") == "NAVIGATING" and a.get("linear_speed", 0.0) < SPEED_EPS):
                total_wait += b["t"] - a["t"]
    mean_wait = total_wait / len(by_robot) if by_robot else 0.0

    return {
        "run_id": run_dir.name, "scenario": scenario, "policy": policy, "seed": seed,
        "duration_s": duration, "deliveries": len(delivered), "throughput_per_min": throughput,
        "mean_latency_s": mean_lat, "total_wait_s": total_wait, "mean_wait_s": mean_wait,
        "stuck_events": sum(1 for d in decisions if d.get("decision_type") == "STUCK_FAIL"),
        "failed_orders": failed, "clearance_violations": count_violations(by_t),
    }


def main(argv: list) -> int:
    if len(argv) != 2:
        print("usage: compute.py <run_dir>", file=sys.stderr)
        return 2
    run_dir = pathlib.Path(argv[1])
    try:
        row = compute(run_dir)
    except FileNotFoundError as e:
        print(f"compute.py: missing file: {e}", file=sys.stderr)
        return 1
    with open(run_dir / "metrics.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(COLUMNS)
        w.writerow([fmt(row[c]) if c not in ("run_id", "scenario", "policy") else row[c] for c in COLUMNS])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
