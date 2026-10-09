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
