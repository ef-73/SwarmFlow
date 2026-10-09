"""Station claims (M6 finding: two robots sent to the same loading station gridlock there). Lead test.

Rule (FleetCore): an order is assignable only if (a) no other active order with the same pickup station is still
ASSIGNED or PICKING_UP, and (b) no other active order with the same dropoff station is not yet DELIVERED/FAILED.
Other orders are not blocked (FIFO skips over the blocked one).
"""

from __future__ import annotations

import pathlib

from swarmflow_core.api import OrderSpec, OrderState, RobotMode, RobotSnapshot
from swarmflow_core.fleet import FleetCore
from swarmflow_core.graph import load_layout
from swarmflow_core.policies.fcfs import FcfsPolicy
from swarmflow_core.reservations import FcfsReservationAuthority

GRAPH = load_layout(pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard")


def snap(rid, vertex, mode=RobotMode.IDLE, **kw):
    v = GRAPH.vertices[vertex]
    return RobotSnapshot(robot_id=rid, x=v.x, y=v.y, yaw=0.0, mode=mode, last_vertex=vertex, **kw)


def core3():
    c = FleetCore(GRAPH, FcfsPolicy(), FcfsReservationAuthority(GRAPH))
    for rid, v in (("robot_1", "P1"), ("robot_2", "P2"), ("robot_3", "P3")):
        c.update_robot(snap(rid, v))
    return c


def order(oid, p, d, t=0.0):
    return OrderSpec(order_id=oid, release_t=t, deadline_t=None, pickup_vertex=p, dropoff_vertex=d)


def test_same_pickup_waits_until_first_is_picked_up():
    c = core3()
    c.add_order(order("o1", "L2", "D1", 0.0))
    c.add_order(order("o2", "L2", "D2", 0.1))
    out = c.tick(1.0)
    assert [a.order_id for a in out.dispatches] == ["o1"]
    assert c.order_state("o2") == OrderState.QUEUED
    a = out.dispatches[0]
    c.update_robot(snap(a.robot_id, "L2", RobotMode.LOADING, task_id=a.task_id, order_id="o1"))
    out = c.tick(20.0)
    assert [x.order_id for x in out.dispatches] == []               # still loading: L2 occupied
    c.update_robot(snap(a.robot_id, "X_2_0", RobotMode.NAVIGATING, task_id=a.task_id, order_id="o1"))
    out = c.tick(25.0)
    assert c.order_state("o1") == OrderState.IN_TRANSIT
    assert [x.order_id for x in out.dispatches] == ["o2"]


def test_same_dropoff_waits_until_first_delivered():
    c = core3()
    c.add_order(order("o1", "L1", "D3", 0.0))
    c.add_order(order("o2", "L3", "D3", 0.1))
    out = c.tick(1.0)
    assert [x.order_id for x in out.dispatches] == ["o1"]
    t1 = out.dispatches[0].task_id
    c.task_result(t1, True, "", 60.0)
    out = c.tick(60.0)
    assert c.order_state("o1") == OrderState.DELIVERED
    assert [x.order_id for x in out.dispatches if x.order_id] == ["o2"]


def test_blocked_order_does_not_block_others():
    c = core3()
    c.add_order(order("o1", "L2", "D1", 0.0))
    c.add_order(order("o2", "L2", "D2", 0.1))   # blocked by o1's pickup
    c.add_order(order("o3", "L1", "D3", 0.2))   # free
    out = c.tick(1.0)
    assert sorted(a.order_id for a in out.dispatches if a.order_id) == ["o1", "o3"]


def test_failed_order_releases_its_claims():
    c = core3()
    c.add_order(order("o1", "L2", "D1", 0.0))
    c.add_order(order("o2", "L2", "D1", 0.1))
    t1 = c.tick(1.0).dispatches[0].task_id
    c.task_result(t1, False, "STUCK_TIMEOUT", 70.0)
    out = c.tick(70.0)
    assert c.order_state("o1") == OrderState.FAILED
    assert "o2" in [a.order_id for a in out.dispatches]
