"""T025 light mode end to end: the unchanged orchestrator and robot agents drive kinematic robots, no Gazebo.
Runs in the dev image with the workspace built."""

from __future__ import annotations

import json
import os
import pathlib
import subprocess
import sys
import time

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
pytest.importorskip("rclpy")
pytest.importorskip("swarmflow_orchestrator")
pytest.importorskip("swarmflow_robot_agent")

LAYOUT = ROOT / "layouts" / "standard" / "generated"
SCENARIO = ROOT / "scenarios" / "v1_demo.yaml"


def _jsonl(p):
    return [json.loads(line) for line in p.read_text(encoding="utf-8").splitlines() if line.strip()]


def test_light_mode_delivers_orders_with_the_real_orchestrator(tmp_path):
    run_dir = tmp_path / "light-run"
    env = dict(os.environ, PYTHONPATH=f"{ROOT / 'sim2d'}{os.pathsep}{os.environ.get('PYTHONPATH', '')}",
               ROS_DOMAIN_ID="77")       # isolated from anything else running
    cmd = [sys.executable, "-m", "sim2d.light", "--layout-dir", str(LAYOUT),
           "--robots", "robot_1,robot_2,robot_3", "--scenario", str(SCENARIO), "--run-dir", str(run_dir),
           "--rtf", "4", "--duration-s", "180"]
    t0 = time.monotonic()
    r = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=240)
    wall = time.monotonic() - t0
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-3000:]
    assert wall < 150, f"180 sim-s at rtf 4 took {wall:.0f} s wall"

    orders = _jsonl(run_dir / "orders.jsonl")
    delivered = {o["order_id"] for o in orders if o["state"] == "DELIVERED"}
    assert len(delivered) >= 2, f"only {len(delivered)} deliveries"
    states = _jsonl(run_dir / "robot_states.jsonl")
    assert {s["robot_id"] for s in states} == {"robot_1", "robot_2", "robot_3"}
    assert max(s["t"] for s in states) >= 170.0            # sim time advanced for the whole run

    m = subprocess.run([sys.executable, str(ROOT / "tools" / "metrics" / "compute.py"), str(run_dir)],
                       capture_output=True, text=True)
    assert m.returncode == 0, m.stderr
    header, row = (run_dir / "metrics.csv").read_text(encoding="utf-8").splitlines()[:2]
    metrics = dict(zip(header.split(","), row.split(",")))
    assert int(metrics["clearance_violations"]) == 0
    assert int(metrics["stuck_events"]) == 0
