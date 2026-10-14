"""ladder_table.py aggregates runs per rung (lead test)."""

import json
import pathlib
import subprocess
import sys

SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "ladder_table.py"
HEADER = ("run_id,scenario,policy,seed,duration_s,deliveries,throughput_per_min,mean_latency_s,total_wait_s,"
          "mean_wait_s,stuck_events,failed_orders,clearance_violations,min_separation_m,near_misses\n")


def run_dir(root, name, deliveries, loc=None):
    d = root / name
    d.mkdir()
    (d / "metrics.csv").write_text(HEADER + f"{name},v2_steady,assign,2,600,{deliveries},{deliveries / 10},90,1,1,0,0,0,0.3,1\n",
                                   encoding="utf-8")
    (d / "realism.txt").write_text("localization=amcl\nnetem=\n" if loc else "localization=ground_truth\n",
                                   encoding="utf-8")
    if loc:
        (d / "loc_error.json").write_text(json.dumps({"robot_1": {"mean_m": loc, "max_m": loc * 3}}), encoding="utf-8")


def test_table(tmp_path):
    run_dir(tmp_path, "lad_r1_1", 8)
    run_dir(tmp_path, "lad_r1_2", 10)
    run_dir(tmp_path, "lad_r2_1", 6, loc=0.2)
    run_dir(tmp_path, "other_r1_1", 99)
    out = subprocess.run([sys.executable, str(SCRIPT), str(tmp_path), "lad"], capture_output=True, text=True).stdout
    lines = [l for l in out.splitlines() if l.startswith("| 1 ") or l.startswith("| 2 ")]
    assert "9.00 ± 1.41" in lines[0] and "| 2 |" in lines[0] and "ground truth" in lines[0]
    assert "6.00" in lines[1] and "0.20 / 0.60" in lines[1]
    assert "99" not in out
