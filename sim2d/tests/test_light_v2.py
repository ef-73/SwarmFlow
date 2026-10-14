"""T036: light mode for v2 (goal tasks on warehouse_v2): graph-path expansion, deadlock breaker, end-to-end v2 demo.
Lead test: do not edit."""

from __future__ import annotations

import json
import math
import os
import pathlib
import random
import subprocess
import sys
import time

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[2]
for p in (ROOT / "sim2d", ROOT / "src" / "swarmflow_core"):
    sys.path.insert(0, str(p))

from sim2d.light_paths import expand  # noqa: E402
from sim2d.light_world import LightWorld  # noqa: E402
from swarmflow_core.graph import load_layout  # noqa: E402
from swarmflow_core.metrics.clearance import footprint_distance  # noqa: E402

V2 = ROOT / "layouts" / "warehouse_v2"
GRAPH = load_layout(V2 / "generated")
RACKS = yaml.safe_load((V2 / "layout.yaml").read_text(encoding="utf-8"))["racks"]


def _segment_hits_rack(a, b, margin=0.25):
    for r in RACKS:
        x0, y0, x1, y1 = r["x_min"] - margin, r["y_min"] - margin, r["x_max"] + margin, r["y_max"] + margin
        for k in range(51):
            x = a[0] + (b[0] - a[0]) * k / 50
            y = a[1] + (b[1] - a[1]) * k / 50
            if x0 < x < x1 and y0 < y < y1:
                return r["name"]
    return None


@pytest.mark.parametrize("start,goal", [("HM1", "PK_W3_N_04"), ("PK_W1_S_04", "PK_E4_N_04"), ("HM8", "DK1"),
                                        ("PL_W1", "DK4"), ("Q_DK2_3", "HM5")])
def test_expansion_follows_the_graph_and_avoids_racks(start, goal):
    s, g = GRAPH.vertices[start], GRAPH.vertices[goal]
    pts = expand(GRAPH, (s.x + 0.1, s.y - 0.1), (g.x, g.y))
    assert pts[-1] == pytest.approx((g.x, g.y))
    prev = (s.x + 0.1, s.y - 0.1)
    for p in pts:
        assert _segment_hits_rack(prev, p) is None, (prev, p)
        prev = p


def test_expansion_short_hop_is_direct():
    a = GRAPH.vertices["PK_W2_S_01"]
    pts = expand(GRAPH, (a.x - 0.5, a.y), (a.x, a.y))
    assert pts == [pytest.approx((a.x, a.y))]


def test_dense_free_space_has_no_permanent_deadlock():
    """8 robots with random overlapping goals in a 12 x 8 m box (the T025 stress case that always deadlocked)."""
    for seed in (1, 2, 3):
        rng = random.Random(seed)
        w = LightWorld(max_speed=0.5, max_turn=1.5)
        for i in range(8):
            w.add_robot(f"robot_{i + 1}", 1.5 + 3.0 * (i % 4), 2.0 + 4.0 * (i // 4), rng.uniform(-3, 3))
        for i in range(8):
            w.set_goals(f"robot_{i + 1}", [(rng.uniform(1, 11), rng.uniform(1, 7), None)])
        ids, t, min_d = w.robot_ids(), 0.0, math.inf
        while t < 400.0 and not all(w.state(r) in ("idle", "arrived") for r in ids):
            w.step(0.1)
            t += 0.1
            poses = {r: w.pose(r) for r in ids}
            for i, a in enumerate(ids):
                for b in ids[i + 1:]:
                    min_d = min(min_d, footprint_distance(poses[a], poses[b]))
        assert all(w.state(r) in ("idle", "arrived") for r in ids), (seed, t, {r: w.state(r) for r in ids})
        assert min_d >= 0.0, seed


def _jsonl(p):
    return [json.loads(x) for x in p.read_text(encoding="utf-8").splitlines() if x.strip()]


def test_v2_demo_in_light_mode(tmp_path):
    pytest.importorskip("rclpy")
    pytest.importorskip("swarmflow_orchestrator")
    run_dir = tmp_path / "light-v2"
    env = dict(os.environ, PYTHONPATH=f"{ROOT / 'sim2d'}{os.pathsep}{os.environ.get('PYTHONPATH', '')}",
               ROS_DOMAIN_ID="78")
    cmd = [sys.executable, "-m", "sim2d.light", "--layout-dir", str(V2 / "generated"),
           "--robots", "robot_1,robot_2,robot_3,robot_4,robot_5", "--scenario", str(ROOT / "scenarios" / "v2_demo.yaml"),
           "--run-dir", str(run_dir), "--rtf", "4", "--duration-s", "300"]
    t0 = time.monotonic()
    r = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stdout[-3000:] + r.stderr[-3000:]
    assert time.monotonic() - t0 < 240
    orders = _jsonl(run_dir / "orders.jsonl")
    last = {}
    for o in orders:
        last[o["order_id"]] = o["state"]
    delivered = [k for k, v in last.items() if v == "DELIVERED"]
    assert len(delivered) >= 6, last
    assert "FAILED" not in last.values(), last
    m = subprocess.run([sys.executable, str(ROOT / "tools" / "metrics" / "compute.py"), str(run_dir)],
                       capture_output=True, text=True)
    assert m.returncode == 0, m.stderr
    header, row = (run_dir / "metrics.csv").read_text(encoding="utf-8").splitlines()[:2]
    mm = dict(zip(header.split(","), row.split(",")))
    assert int(mm["clearance_violations"]) == 0 and int(mm["stuck_events"]) == 0
