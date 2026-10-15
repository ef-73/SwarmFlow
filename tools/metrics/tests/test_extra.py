"""Extra T011 tests (rotated overlap, empty run, missing file)."""

import math
import pathlib
import subprocess
import sys

from swarmflow_core.metrics.clearance import count_violations, footprint_distance

COMPUTE = pathlib.Path(__file__).resolve().parents[1] / "compute.py"


def test_rotated_overlap_negative_and_corner_distance():
    assert footprint_distance((0, 0, 0), (0.5, 0.2, math.pi / 4)) < 0
    d = footprint_distance((0, 0, 0), (2.0, 2.0, 0))
    assert abs(d - math.hypot(2.0 - 0.70, 2.0 - 0.60)) < 1e-9


def test_empty_samples():
    assert count_violations({}) == 0


def test_missing_file_exit_1(tmp_path):
    (tmp_path / "config.yaml").write_text("scenario: {}\n")
    r = subprocess.run([sys.executable, str(COMPUTE), str(tmp_path)], capture_output=True, text=True)
    assert r.returncode == 1 and "missing" in r.stderr


def test_empty_run(tmp_path):
    d = tmp_path / "sc-fcfs-s3-x"
    d.mkdir()
    for n in ("orders", "decisions", "robot_states"):
        (d / f"{n}.jsonl").write_text("")
    (d / "config.yaml").write_text("{}\n")
    assert subprocess.run([sys.executable, str(COMPUTE), str(d)]).returncode == 0
    lines = (d / "metrics.csv").read_text().splitlines()
    assert lines[1].startswith("sc-fcfs-s3-x,sc,fcfs,3,0.000,0,")


def test_scenario_window_from_config(tmp_path):
    """Events after start + duration_s are not counted when config.yaml records the scenario start (M9 D3)."""
    import json
    import subprocess
    import sys
    import yaml
    d = tmp_path / "v1_corridor-fcfs-s1-w"
    d.mkdir()
    (d / "config.yaml").write_text(yaml.safe_dump({"scenario": {"name": "v1_corridor", "policy": "fcfs", "seed": 1,
                                                                 "duration_s": 100.0},
                                                    "start_time_sim_s": 10.0, "start_delay_s": 5.0}), encoding="utf-8")
    rows = []
    for oid, td in (("o1", 50.0), ("o2", 114.0), ("o3", 140.0)):          # window = [15, 115]
        rows += [{"t": 15.0, "order_id": oid, "state": "QUEUED", "robot_id": "", "failure_reason": ""},
                 {"t": td, "order_id": oid, "state": "DELIVERED", "robot_id": "robot_1", "failure_reason": ""}]
    (d / "orders.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    (d / "decisions.jsonl").write_text("", encoding="utf-8")
    st = [{"t": float(t), "robot_id": "robot_1", "x": 1.0, "y": 1.0, "yaw": 0.0, "mode": "IDLE", "linear_speed": 0.0}
          for t in range(0, 150)]
    (d / "robot_states.jsonl").write_text("".join(json.dumps(r) + "\n" for r in st), encoding="utf-8")
    script = pathlib.Path(__file__).resolve().parents[1] / "compute.py"
    assert subprocess.run([sys.executable, str(script), str(d)]).returncode == 0
    text = (d / "metrics.csv").read_text(encoding="utf-8").splitlines()
    row = dict(zip(text[0].split(","), text[1].split(",")))
    assert row["deliveries"] == "2"


def test_separation_min_distance_and_near_miss_episodes():
    import importlib.util
    import math
    import pathlib
    spec = importlib.util.spec_from_file_location("compute_sep", pathlib.Path(__file__).resolve().parents[1] / "compute.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    # footprints 0.70 m long along x: centres 1.0 m apart → 0.30 m gap (not a near miss), 0.9 m → 0.20 m gap
    gaps = [3.0, 0.9, 0.9, 3.0, 0.9, 1.0]
    samples = {float(i): {"robot_1": (0.0, 0.0, 0.0), "robot_2": (g, 0.0, 0.0)} for i, g in enumerate(gaps)}
    best, near = mod.separation(samples)
    assert abs(best - 0.20) < 1e-9 and near == 2
    best1, near1 = mod.separation({0.0: {"robot_1": (0.0, 0.0, 0.0)}})
    assert math.isnan(best1) and near1 == 0


def test_short_run_uses_covered_sim_span(tmp_path):
    """A run whose sim time stopped at 300 s of a 600 s window reports rates over 300 s (X-10 finding)."""
    import json
    d = tmp_path / "v1_demo-fcfs-s1-short"
    d.mkdir()
    (d / "config.yaml").write_text("scenario: {name: v1_demo, seed: 1, duration_s: 600, policy: fcfs}\n"
                                   "start_time_sim_s: 10.0\nstart_delay_s: 0.0\n", encoding="utf-8")
    (d / "orders.jsonl").write_text("".join(json.dumps(r) + "\n" for r in [
        {"t": 20.0, "order_id": "o1", "state": "QUEUED", "robot_id": "", "failure_reason": ""},
        {"t": 200.0, "order_id": "o1", "state": "DELIVERED", "robot_id": "robot_1", "failure_reason": ""}]),
        encoding="utf-8")
    (d / "decisions.jsonl").write_text("", encoding="utf-8")
    (d / "robot_states.jsonl").write_text("".join(
        json.dumps({"t": 10.0 + k, "robot_id": "robot_1", "x": 0.0, "y": 0.0, "yaw": 0.0, "mode": "IDLE",
                    "linear_speed": 0.0}) + "\n" for k in range(0, 301, 10)), encoding="utf-8")
    r = subprocess.run([sys.executable, str(COMPUTE), str(d)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    header, row = (d / "metrics.csv").read_text(encoding="utf-8").splitlines()[:2]
    m = dict(zip(header.split(","), row.split(",")))
    assert abs(float(m["duration_s"]) - 300.0) < 1e-6 and abs(float(m["throughput_per_min"]) - 0.2) < 1e-6


def test_stuck_episodes_by_area():
    import importlib.util
    spec = importlib.util.spec_from_file_location("compute_stuck", COMPUTE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    aisles = mod.load_aisles("warehouse_v2")
    assert mod.area_of(4.0, 10.0, aisles) == "storage" and mod.area_of(10.0, 15.0, aisles) == "central"
    assert mod.area_of(5.0, 23.0, aisles) == "cross" and mod.area_of(6.5, 29.0, aisles) == "docks"
    assert mod.load_aisles("no_such_layout") == [] and mod.area_of(1.0, 1.0, []) == "other"
    modes = ["NAVIGATING", "STUCK", "STUCK", "NAVIGATING", "STUCK"]
    by_robot = {"robot_1": [{"t": float(i), "mode": m, "x": 4.0, "y": 10.0} for i, m in enumerate(modes)],
                "robot_2": [{"t": 0.0, "mode": "STUCK", "x": 6.5, "y": 29.0}]}
    assert sorted(mod.stuck_episodes(by_robot, aisles)) == ["docks", "storage", "storage"]
