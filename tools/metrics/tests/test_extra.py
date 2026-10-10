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
