"""dock_line_check.py: lane entries of warehouse_v2 and a claim near / far from the line."""

import importlib.util
import json
import pathlib

import yaml

HERE = pathlib.Path(__file__).resolve().parents[1]
LAYOUT = HERE.parents[1] / "layouts" / "warehouse_v2" / "layout.yaml"


def _mod():
    spec = importlib.util.spec_from_file_location("dock_line_check", HERE / "dock_line_check.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_lane_entries_of_warehouse_v2():
    e = _mod().lane_entries(yaml.safe_load(LAYOUT.read_text(encoding="utf-8")))
    assert set(e) == {"DK1", "DK2", "DK3", "DK4"} and e["DK1"] == (2.0, 23.4)


def _run(tmp_path, claim_y):
    d = tmp_path / f"run_{claim_y}"
    d.mkdir()
    (d / "robot_states.jsonl").write_text(json.dumps({"t": 10.0, "robot_id": "robot_1", "x": 2.0, "y": claim_y}) + "\n")
    (d / "decisions.jsonl").write_text(json.dumps(
        {"t": 10.2, "decision_type": "RESERVATION_GRANT", "trigger": "arrived_at_dock_line", "robot_id": "robot_1",
         "order_id": "o1", "new_decision": "robot_1 -> Q_DK1_2"}) + "\n")
    (d / "orders.jsonl").write_text(json.dumps({"t": 50.0, "order_id": "o1", "state": "DELIVERED"}) + "\n")
    return d


def test_claim_near_the_line_passes_and_far_fails(tmp_path):
    m = _mod()
    assert m.check(_run(tmp_path, 21.0), LAYOUT, 3.0) == 0      # 2.4 m from X_1_5
    assert m.check(_run(tmp_path, 15.0), LAYOUT, 3.0) == 1      # 8.4 m away
