"""M9 hardening tests (T020): pose validity (S4), clock reset (S7), run-record truncation (D2)."""

from __future__ import annotations

import pathlib

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.parameter import Parameter  # noqa: E402

from swarmflow_interfaces.msg import Order, RobotState  # noqa: E402
from swarmflow_orchestrator.orchestrator_node import OrchestratorNode, RunRecord  # noqa: E402
from test_orchestrator_extra import ros  # noqa: E402,F401

FIX = pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard"


def _node(**params):
    base = {"layout_dir": str(FIX), "robots": ["robot_1"], "policy": "fcfs", "run_dir": ""}
    base.update(params)
    return OrchestratorNode(parameter_overrides=[Parameter(k, value=v) for k, v in base.items()])


def _state(mode, reason="", sec=10):
    s = RobotState(robot_id="robot_1", x=1.0, y=1.0, mode=mode, fault_reason=reason, last_vertex="P1")
    s.header.stamp.sec = sec
    return s


def test_unknown_pose_faults_do_not_reach_the_authority(ros):
    n = _node()
    try:
        seen = []
        n.authority.observe_robot = lambda rid, x, y, t: seen.append((rid, t))
        n._on_robot_state(_state(RobotState.MODE_FAULT, "NO_POSE", 10))
        n._on_robot_state(_state(RobotState.MODE_FAULT, "STALE_POSE", 11))
        assert seen == []
        assert n._snapshots["robot_1"].fault_reason == "STALE_POSE"          # the core still sees the fault
        n._on_robot_state(_state(RobotState.MODE_FAULT, "BATTERY", 12))      # other faults carry a valid pose
        n._on_robot_state(_state(RobotState.MODE_IDLE, "", 13))
        assert seen == [("robot_1", 12.0), ("robot_1", 13.0)]
    finally:
        n.destroy_node()


def test_clock_reset_restarts_recovery_and_keeps_orders(ros):
    n = _node()
    try:
        clock = [100.0]
        n._now = lambda: clock[0]
        n._on_order(Order(order_id="o1", pickup_vertex="L1", dropoff_vertex="D1"))
        n._on_robot_state(_state(RobotState.MODE_IDLE, "", 100))
        old_authority = n.authority
        n._recovered, n._window_start = True, 50.0
        n._reported.add("robot_1")
        clock[0] = 200.0
        n._check_clock(200.0)
        n._check_clock(196.0)                       # a jump back of less than 5 s is not a reset
        assert n.authority is old_authority and n._recovered
        clock[0] = 20.0
        n._check_clock(20.0)                        # Gazebo restarted
        assert n.authority is not old_authority and n.core.authority is n.authority
        assert not n._recovered and n._window_start is None
        assert not n._reported and not n._state_stamp and not n._last_seen
        assert "o1" in n._specs                     # orders stay
        n._on_robot_state(_state(RobotState.MODE_IDLE, "", 1))   # a low stamp is accepted again
        assert n._state_stamp["robot_1"] == 1.0
    finally:
        n.destroy_node()


def test_run_record_files_truncated_on_start(tmp_path):
    run = str(tmp_path / "run")
    rec = RunRecord(run)
    for name in RunRecord.FILES:
        rec.write(name, {"old": True})
    rec.close()
    rec = RunRecord(run)
    rec.write("orders", {"new": True})
    rec.close()
    for name in RunRecord.FILES:
        text = (tmp_path / "run" / f"{name}.jsonl").read_text()
        assert text == ('{"new": true}\n' if name == "orders" else "")
