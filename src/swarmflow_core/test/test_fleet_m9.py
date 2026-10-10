"""M9 fleet-core hardening (independent review S5): STUCK robots are not lost forever. Lead test: do not edit."""

from __future__ import annotations

import pathlib

from swarmflow_core import api
from swarmflow_core.api import DecisionType, OrderSpec, RobotMode, RobotSnapshot
from swarmflow_core.fleet import STUCK_RECOVERY_ATTEMPTS, STUCK_RECOVERY_S, FleetCore
from swarmflow_core.graph import load_layout
from swarmflow_core.policies.fcfs import FcfsPolicy
from swarmflow_core.reservations import FcfsReservationAuthority

GRAPH = load_layout(pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard")


def snap(rid, vertex, mode=RobotMode.IDLE, **kw):
    v = GRAPH.vertices[vertex]
    return RobotSnapshot(robot_id=rid, x=v.x, y=v.y, yaw=0.0, mode=mode, last_vertex=vertex, **kw)


def stuck_robot_core():
    c = FleetCore(GRAPH, FcfsPolicy(), FcfsReservationAuthority(GRAPH))
    c.update_robot(snap("robot_1", "P1"))
    c.add_order(OrderSpec("o1", 0.0, None, "L1", "D1"))
    task = c.tick(0.0).dispatches[0].task_id
    c.update_robot(snap("robot_1", "X_1_0", RobotMode.STUCK, task_id=task, order_id="o1"))
    c.task_result(task, False, api.FAILURE_STUCK_TIMEOUT, 60.0)
    park = [a for a in c.tick(60.0).dispatches if a.robot_id == "robot_1"][0]
    c.update_robot(snap("robot_1", "X_1_0", RobotMode.STUCK, task_id=park.task_id))
    c.task_result(park.task_id, False, api.FAILURE_STUCK_TIMEOUT, 130.0)   # the park task fails too
    return c, park


def test_constants():
    assert STUCK_RECOVERY_S == 30.0 and STUCK_RECOVERY_ATTEMPTS == 3


def test_stuck_robot_gets_recovery_park_after_cooldown():
    c, park = stuck_robot_core()
    out = c.tick(130.0)
    assert not [a for a in out.dispatches if a.robot_id == "robot_1"]        # cooling down
    c.update_robot(snap("robot_1", "X_1_0", RobotMode.STUCK))
    assert not [a for a in c.tick(150.0).dispatches if a.robot_id == "robot_1"]
    out = c.tick(130.0 + STUCK_RECOVERY_S)
    again = [a for a in out.dispatches if a.robot_id == "robot_1"]
    assert len(again) == 1 and again[0].order_id == "" and again[0].route[-1] in GRAPH.vertices_of_kind("park")
    assert any(d.decision_type == DecisionType.REROUTE and d.robot_id == "robot_1" and "stuck_recovery" in d.trigger
               for d in out.decisions)


def test_recovery_attempts_are_bounded():
    c, park = stuck_robot_core()
    t = 130.0
    sent = 0
    for _ in range(10):
        t += STUCK_RECOVERY_S
        out = c.tick(t)
        for a in out.dispatches:
            if a.robot_id == "robot_1":
                sent += 1
                c.update_robot(snap("robot_1", "X_1_0", RobotMode.STUCK, task_id=a.task_id))
                c.task_result(a.task_id, False, api.FAILURE_STUCK_TIMEOUT, t + 1.0)
                c.tick(t + 1.0)
    assert sent == STUCK_RECOVERY_ATTEMPTS


def test_recovered_robot_is_available_again():
    c, park = stuck_robot_core()
    out = c.tick(130.0 + STUCK_RECOVERY_S)
    rp = [a for a in out.dispatches if a.robot_id == "robot_1"][0]
    c.update_robot(snap("robot_1", "P1", RobotMode.IDLE, task_id=rp.task_id))
    c.task_result(rp.task_id, True, "", 200.0)
    c.update_robot(snap("robot_1", "P1"))
    c.add_order(OrderSpec("o2", 200.0, None, "L2", "D2"))
    assert [a.order_id for a in c.tick(201.0).dispatches] == ["o2"]
