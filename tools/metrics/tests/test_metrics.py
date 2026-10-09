"""Metrics tests (design §13.1, §13.2, §13.5). Lead test: do not edit.

CLIs: ``python3 tools/metrics/compute.py runs/<run_id>/`` → ``metrics.csv`` in that dir;
``python3 tools/metrics/compare.py runs/A runs/B ... [--out table.md]`` → mean ± 95 % CI per (scenario, policy).
Library: ``swarmflow_core.metrics.clearance``.
"""

from __future__ import annotations

import csv
import json
import pathlib
import subprocess
import sys

import pytest
import yaml

from swarmflow_core.metrics.clearance import count_violations, footprint_distance

ROOT = pathlib.Path(__file__).resolve().parents[3]
COMPUTE = ROOT / "tools" / "metrics" / "compute.py"
COMPARE = ROOT / "tools" / "metrics" / "compare.py"


def jl(path, rows):
    path.write_text("".join(json.dumps(r, sort_keys=True) + "\n" for r in rows), encoding="utf-8")


def make_run(d: pathlib.Path, scenario="v1_corridor", policy="fcfs", seed=1, delivered_at=(40.0, 90.0),
             stuck=0, overlap=False):
    d.mkdir(parents=True)
    (d / "config.yaml").write_text(yaml.safe_dump({"scenario": {"name": scenario, "policy": policy, "seed": seed,
                                                                "duration_s": 120.0}}), encoding="utf-8")
    orders = []
    for i, td in enumerate(delivered_at):
        oid = f"o{i + 1:04d}"
        orders += [{"t": 10.0 * i, "order_id": oid, "state": "QUEUED", "robot_id": "", "failure_reason": ""},
                   {"t": 10.0 * i + 1, "order_id": oid, "state": "ASSIGNED", "robot_id": "robot_1", "failure_reason": ""},
                   {"t": td, "order_id": oid, "state": "DELIVERED", "robot_id": "robot_1", "failure_reason": ""}]
    orders.append({"t": 50.0, "order_id": "o0099", "state": "QUEUED", "robot_id": "", "failure_reason": ""})
    if stuck:
        orders.append({"t": 70.0, "order_id": "o0099", "state": "FAILED", "robot_id": "robot_2",
                       "failure_reason": "STUCK_TIMEOUT"})
    jl(d / "orders.jsonl", orders)
    decisions = [{"t": 70.0, "decision_type": "STUCK_FAIL", "robot_id": "robot_2", "order_id": "o0099"}] * stuck
    jl(d / "decisions.jsonl", decisions)
    states = []
    for k in range(0, 241):                      # 2 Hz for 120 s
        t = k * 0.5
        # robot_1: waiting for a reservation from t=20 to t=30 (10 s), navigating otherwise
        m1 = "WAITING_RESERVATION" if 20.0 <= t < 30.0 else "NAVIGATING"
        states.append({"t": t, "robot_id": "robot_1", "x": 2.0, "y": 2.0, "yaw": 0.0, "mode": m1,
                       "linear_speed": 0.0 if m1 != "NAVIGATING" else 0.5})
        # robot_2: navigating but stopped (speed 0.0) from t=60 to t=65 (5 s) → counts as waiting
        sp = 0.0 if 60.0 <= t < 65.0 else 0.5
        x2 = 2.3 if (overlap and 80.0 <= t < 82.0) else 8.0      # 0.3 m apart → padded footprints overlap
        states.append({"t": t, "robot_id": "robot_2", "x": x2, "y": 2.0, "yaw": 0.0, "mode": "NAVIGATING",
                       "linear_speed": sp})
    jl(d / "robot_states.jsonl", states)
    return d


def read_metrics(d):
    with open(d / "metrics.csv", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 1
    return rows[0]


def test_clearance_geometry():
    a = (0.0, 0.0, 0.0)
    assert footprint_distance(a, (2.0, 0.0, 0.0)) == pytest.approx(2.0 - 0.70)
    assert footprint_distance(a, (0.0, 2.0, 0.0)) == pytest.approx(2.0 - 0.60)
    assert footprint_distance(a, (0.5, 0.0, 0.0)) < 0
    # rotated 90°: 0.35 + 0.30 half-extents along x
    assert footprint_distance(a, (1.0, 0.0, 1.5707963)) == pytest.approx(1.0 - 0.65, abs=1e-6)


def test_violation_episodes_counted_once():
    samples = {}
    for k in range(20):
        t = k * 0.1
        far = not (0.5 <= t < 0.9 or 1.3 <= t < 1.5)
        samples[t] = {"robot_1": (0.0, 0.0, 0.0), "robot_2": ((3.0 if far else 0.2), 0.0, 0.0)}
    assert count_violations(samples) == 2


def test_compute_metrics(tmp_path):
    d = make_run(tmp_path / "v1_corridor-fcfs-s1-x", stuck=1, overlap=True)
    r = subprocess.run([sys.executable, str(COMPUTE), str(d)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    m = read_metrics(d)
    assert m["scenario"] == "v1_corridor" and m["policy"] == "fcfs" and m["seed"] == "1"
    assert int(m["deliveries"]) == 2 and int(m["failed_orders"]) == 1 and int(m["stuck_events"]) == 1
    assert float(m["duration_s"]) == pytest.approx(120.0)
    assert float(m["throughput_per_min"]) == pytest.approx(2 / 2.0)
    assert float(m["mean_latency_s"]) == pytest.approx(((40.0 - 0.0) + (90.0 - 10.0)) / 2)
    # waiting: robot_1 10 s (WAITING_RESERVATION) + robot_2 5 s (navigating, speed < 0.05) → mean per robot 7.5 s
    assert float(m["total_wait_s"]) == pytest.approx(15.0, abs=0.6)
    assert float(m["mean_wait_s"]) == pytest.approx(7.5, abs=0.3)
    assert int(m["clearance_violations"]) == 1


def test_compare_ci_table(tmp_path):
    runs = []
    for i, td in enumerate([(40.0, 90.0), (40.0,), (40.0, 60.0, 100.0)]):
        runs.append(make_run(tmp_path / f"a{i}", policy="fcfs", seed=i + 1, delivered_at=td))
    runs.append(make_run(tmp_path / "b0", policy="independent", seed=1, delivered_at=(50.0,), stuck=1))
    for d in runs:
        assert subprocess.run([sys.executable, str(COMPUTE), str(d)]).returncode == 0
    out = tmp_path / "table.md"
    r = subprocess.run([sys.executable, str(COMPARE), *map(str, runs), "--out", str(out)],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    text = out.read_text(encoding="utf-8")
    assert "| v1_corridor | fcfs | 3 |" in text and "| v1_corridor | independent | 1 |" in text
    # deliveries fcfs: 2, 1, 3 → mean 2.00, sd 1, t(0.975, df=2) = 4.303 → half-width 4.303 / sqrt(3) = 2.48
    assert "2.00 ± 2.48" in text
    assert "n/a" in text            # n = 1 → no CI
