"""T028: assignment-only fleet core for v2 (O-01 ... O-05, O-08, P-01; api.py v2 lifecycle).


Synthetic comb graph (metres): a spine along y = 0, homes south of it, pick branches and a pallet north of it, two
docks with queue lanes.

    PL_1(0,3)   PK_A_2(2,5)            DK2(6,4.2)  PK_B_2(8,5)   DK1(10,4.2)
      |           |                      |           |            |
      |         PK_A_1(2,3)          Q_DK2_1(6,2.6) PK_B_1(8,3)  Q_DK1_1(10,2.6)
      |           |                      |           |            |
      |           |                      |           |          Q_DK1_2(10,1.0)
    X_0_0 ----- X_0_1 ----- X_0_2 ----- X_0_3 ----- X_0_4 ----- X_0_5        (y = 0, x = 0 … 10)
      |           |                                  |
    HM1(0,-2)   HM2(2,-2)                          HM3(8,-1)
"""

from __future__ import annotations

import math

import pytest

from swarmflow_core import api
from swarmflow_core.api import (Edge, OrderSpec, OrderState, RobotMode, RobotSnapshot, Vertex)
from swarmflow_core.fleet_v2 import FleetCoreV2
from swarmflow_core.graph import WarehouseGraph
from swarmflow_core.policies.assign import AssignPolicy

V = {
    "X_0_0": (0, 0, "intersection"), "X_0_1": (2, 0, "intersection"), "X_0_2": (4, 0, "intersection"),
    "X_0_3": (6, 0, "intersection"), "X_0_4": (8, 0, "intersection"), "X_0_5": (10, 0, "intersection"),
    "HM1": (0, -2, "home"), "HM2": (2, -2, "home"), "HM3": (8, -1, "home"),
    "PL_1": (0, 3, "pallet"),
    "PK_A_1": (2, 3, "pick"), "PK_A_2": (2, 5, "pick"), "PK_B_1": (8, 3, "pick"), "PK_B_2": (8, 5, "pick"),
    "DK1": (10, 4.2, "dock"), "Q_DK1_1": (10, 2.6, "queue"), "Q_DK1_2": (10, 1.0, "queue"),
    "DK2": (6, 4.2, "dock"), "Q_DK2_1": (6, 2.6, "queue"),
}
LINKS = [("X_0_0", "X_0_1"), ("X_0_1", "X_0_2"), ("X_0_2", "X_0_3"), ("X_0_3", "X_0_4"), ("X_0_4", "X_0_5"),
         ("X_0_0", "HM1"), ("X_0_1", "HM2"), ("X_0_4", "HM3"), ("X_0_0", "PL_1"),
         ("X_0_1", "PK_A_1"), ("PK_A_1", "PK_A_2"), ("X_0_4", "PK_B_1"), ("PK_B_1", "PK_B_2"),
         ("X_0_5", "Q_DK1_2"), ("Q_DK1_2", "Q_DK1_1"), ("Q_DK1_1", "DK1"), ("X_0_3", "Q_DK2_1"), ("Q_DK2_1", "DK2")]


def make_graph() -> WarehouseGraph:
    verts = {n: Vertex(n, float(x), float(y), k) for n, (x, y, k) in V.items()}
    edges = {}
    for a, b in LINKS:
        d = math.hypot(V[a][0] - V[b][0], V[a][1] - V[b][1])
        for s, e in ((a, b), (b, a)):
            edges[(s, e)] = Edge(s, e, d, 3.0, True, None, {})
    return WarehouseGraph(verts, edges, {}, name="comb")


QUEUE_INDEX = {"Q_DK1_1": ("DK1", 1), "Q_DK1_2": ("DK1", 2), "Q_DK2_1": ("DK2", 1)}


@pytest.fixture
def core():
    return FleetCoreV2(make_graph(), AssignPolicy(), queue_spots=QUEUE_INDEX)


def snap(rid, at, mode=RobotMode.IDLE, task="", order="", nxt="", fault="", payload=""):
    x, y, _ = V[at]
    return RobotSnapshot(rid, float(x), float(y), 0.0, mode, last_vertex=at, task_id=task, order_id=order,
                         next_vertex=nxt, fault_reason=fault, payload_type=payload)


def shelf(oid, picks, dock, t=0.0, prio=0):
    return OrderSpec(oid, t, None, "", dock, "small", prio, order_type=api.ORDER_SHELF, pick_vertices=tuple(picks))


def pallet(oid, spot, dock, t=0.0):
    return OrderSpec(oid, t, None, spot, dock, "pallet", 0, order_type=api.ORDER_PALLET, pallet_id=f"pallet_{spot}")


def states(out, oid):
    return [c.state for c in out.status_changes if c.order_id == oid]


def only(dispatches, rid):
    ds = [d for d in dispatches if d.robot_id == rid]
    assert len(ds) == 1, dispatches
    return ds[0]


# ---- assignment -----------------------------------------------------------------------------------------------

def test_nearest_robot_nearest_next_picks_then_dock(core):
    core.update_robot(snap("robot_1", "HM1"))
    core.update_robot(snap("robot_2", "HM3"))
    core.add_order(shelf("s1", ["PK_A_1", "PK_B_2"], "DK1"))
    out = core.tick(1.0)
    assert states(out, "s1") == [OrderState.QUEUED, OrderState.ASSIGNED]
    a = only(out.dispatches, "robot_2")             # HM3 -> PK_B_2 = 1+3+2 = 6 m beats HM1 -> PK_A_1 = 2+2+3 = 7 m
    assert a.order_id == "s1" and a.route_kind == api.ROUTE_GOALS
    assert tuple(a.route) == ("PK_B_2", "PK_A_1", "DK1")
    d = [x for x in out.decisions if x.decision_type == api.DecisionType.ASSIGN]
    assert d and d[0].robot_id == "robot_2" and "travel_to_first_goal_m" in d[0].cost_keys
    assert d[0].explanation


def test_tie_broken_by_robot_id_and_priority_first(core):
    core.update_robot(snap("robot_2", "HM2"))
    core.update_robot(snap("robot_1", "HM2"))
    core.add_order(shelf("low", ["PK_A_1"], "DK2", t=0.0, prio=0))
    core.add_order(shelf("high", ["PK_A_2"], "DK2", t=0.5, prio=2))
    out = core.tick(1.0)
    assert only(out.dispatches, "robot_1").order_id == "high"      # higher priority is planned first
    assert only(out.dispatches, "robot_2").order_id == "low"


def test_congestion_prefers_robot_whose_first_goal_is_quiet(core):
    # robot_1 (X_0_1) and robot_2 (X_0_4) are both 3 m from PK_A_1 / PK_B_1; robot_3 is busy heading to PK_A_2,
    # so congestion (not the robot-id tie break) decides
    core.update_robot(snap("robot_1", "X_0_1"))
    core.update_robot(snap("robot_2", "X_0_4"))
    core.update_robot(snap("robot_3", "X_0_1", RobotMode.NAVIGATING, task="other", order="x", nxt="PK_A_2"))
    core.add_order(shelf("s1", ["PK_A_1", "PK_B_1"], "DK1"))
    out = core.tick(1.0)
    a = only(out.dispatches, "robot_2")
    assert tuple(a.route) == ("PK_B_1", "PK_A_1", "DK1")
    d = [x for x in out.decisions if x.decision_type == api.DecisionType.ASSIGN][0]
    assert "congestion_m" in d.cost_keys


def test_deterministic(core):
    def run():
        c = FleetCoreV2(make_graph(), AssignPolicy(), queue_spots=QUEUE_INDEX)
        for r, at in (("robot_1", "HM1"), ("robot_2", "HM2"), ("robot_3", "HM3")):
            c.update_robot(snap(r, at))
        for i, (p, dk) in enumerate([("PK_A_2", "DK1"), ("PK_B_1", "DK2"), ("PK_A_1", "DK1")]):
            c.add_order(shelf(f"o{i}", [p], dk, t=float(i)))
        return [(a.robot_id, a.order_id, tuple(a.route)) for t in (1.0, 2.0, 3.0) for a in c.tick(t).dispatches]
    assert run() == run()


# ---- shelf lifecycle ----------------------------------------------------------------------------------------------

def test_shelf_lifecycle_and_home_after_delivery(core):
    core.update_robot(snap("robot_1", "HM1"))
    core.update_robot(snap("robot_2", "HM3"))
    core.add_order(shelf("s1", ["PK_A_1", "PK_B_2"], "DK1"))
    a = only(core.tick(1.0).dispatches, "robot_2")
    core.update_robot(snap("robot_2", "PK_B_2", RobotMode.NAVIGATING, a.task_id, "s1", nxt="PK_A_1"))
    assert states(core.tick(2.0), "s1") == [OrderState.PICKING_UP]
    core.update_robot(snap("robot_2", "PK_A_1", RobotMode.NAVIGATING, a.task_id, "s1", nxt="DK1"))
    assert states(core.tick(3.0), "s1") == [OrderState.IN_TRANSIT]
    core.update_robot(snap("robot_2", "DK1", RobotMode.UNLOADING, a.task_id, "s1"))
    assert states(core.tick(4.0), "s1") == []
    core.task_result(a.task_id, True, "", 9.0)
    core.update_robot(snap("robot_2", "DK1"))
    out = core.tick(9.0)
    assert states(out, "s1") == [OrderState.DELIVERED]
    home = only(out.dispatches, "robot_2")
    assert home.order_id == "" and tuple(home.route) == ("HM3",) and home.route_kind == api.ROUTE_GOALS


def test_single_pick_goes_straight_to_in_transit(core):
    core.update_robot(snap("robot_1", "HM2"))
    core.add_order(shelf("s1", ["PK_A_1"], "DK2"))
    a = only(core.tick(1.0).dispatches, "robot_1")
    core.update_robot(snap("robot_1", "PK_A_1", RobotMode.NAVIGATING, a.task_id, "s1", nxt="DK2"))
    assert states(core.tick(2.0), "s1") == [OrderState.PICKING_UP, OrderState.IN_TRANSIT]


def test_idle_robots_go_home_never_two_to_one_slot(core):
    core.update_robot(snap("robot_1", "X_0_1"))
    core.update_robot(snap("robot_2", "X_0_1"))
    core.update_robot(snap("robot_3", "HM3"))                       # already home: stays
    out = core.tick(1.0)
    targets = {d.robot_id: tuple(d.route) for d in out.dispatches}
    assert set(targets) == {"robot_1", "robot_2"}
    assert {targets["robot_1"], targets["robot_2"]} == {("HM2",), ("HM1",)}
    assert targets["robot_1"] == ("HM2",)                            # lower id gets the nearer slot


# ---- dock queues (O-05) --------------------------------------------------------------------------------------------

def _two_to_dk1(core):
    core.update_robot(snap("robot_1", "HM3"))
    core.update_robot(snap("robot_2", "HM2"))
    core.add_order(shelf("s1", ["PK_B_1"], "DK1"))
    core.add_order(shelf("s2", ["PK_A_1"], "DK1", t=0.1))
    out = core.tick(1.0)
    return only(out.dispatches, "robot_1"), only(out.dispatches, "robot_2")


def test_second_robot_for_a_busy_dock_queues(core):
    a1, a2 = _two_to_dk1(core)
    assert tuple(a1.route) == ("PK_B_1", "DK1")
    assert tuple(a2.route) == ("PK_A_1", "Q_DK1_1")


def test_queue_advances_when_dock_frees_after_arrival(core):
    a1, a2 = _two_to_dk1(core)
    core.update_robot(snap("robot_2", "Q_DK1_1", RobotMode.NAVIGATING, a2.task_id, "s2"))
    core.tick(2.0)
    core.task_result(a2.task_id, True, "", 3.0)                     # arrived at its queue spot
    core.update_robot(snap("robot_2", "Q_DK1_1", RobotMode.IDLE, "", "s2"))
    out = core.tick(3.0)
    assert states(out, "s2") != [OrderState.DELIVERED]               # a queue spot does not deliver
    assert core.order_state("s2") == OrderState.IN_TRANSIT
    assert not [d for d in out.dispatches if d.robot_id == "robot_2"]  # waits: dock still busy, not sent home
    core.task_result(a1.task_id, True, "", 10.0)
    core.update_robot(snap("robot_1", "DK1"))
    out = core.tick(10.0)
    f = only(out.dispatches, "robot_2")
    assert f.order_id == "s2" and tuple(f.route) == ("DK1",)
    core.task_result(f.task_id, True, "", 20.0)
    core.update_robot(snap("robot_2", "DK1"))
    assert states(core.tick(20.0), "s2") == [OrderState.DELIVERED]


def test_queue_advance_cancels_a_running_queue_task_without_failing_the_order(core):
    a1, a2 = _two_to_dk1(core)
    core.update_robot(snap("robot_2", "PK_A_1", RobotMode.NAVIGATING, a2.task_id, "s2", nxt="Q_DK1_1"))
    core.tick(2.0)
    core.task_result(a1.task_id, True, "", 5.0)
    core.update_robot(snap("robot_1", "DK1"))
    out = core.tick(5.0)
    assert a2.task_id in out.cancels
    f = only([d for d in out.dispatches if d.order_id == "s2"], "robot_2")
    assert tuple(f.route) == ("DK1",)
    core.task_result(a2.task_id, False, api.FAILURE_CANCELLED, 5.5)
    out = core.tick(5.5)
    assert states(out, "s2") == [] and core.order_state("s2") == OrderState.IN_TRANSIT


def test_three_robots_one_dock_queue_moves_up():
    c = FleetCoreV2(make_graph(), AssignPolicy(), queue_spots=QUEUE_INDEX)
    for r, at in (("robot_1", "HM3"), ("robot_2", "HM2"), ("robot_3", "HM1")):
        c.update_robot(snap(r, at))
    for i, p in enumerate(["PK_B_1", "PK_A_1", "PK_A_2"]):
        c.add_order(shelf(f"s{i + 1}", [p], "DK1", t=0.1 * i))
    out = c.tick(1.0)
    tails = sorted(tuple(d.route)[-1] for d in out.dispatches)
    assert tails == ["DK1", "Q_DK1_1", "Q_DK1_2"]
    by_tail = {tuple(d.route)[-1]: d for d in out.dispatches}
    for d in out.dispatches:
        if tuple(d.route)[-1] != "DK1":
            c.task_result(d.task_id, True, "", 4.0)
            c.update_robot(snap(d.robot_id, tuple(d.route)[-1], RobotMode.IDLE, "", d.order_id))
    c.tick(4.0)
    head = by_tail["DK1"]
    c.task_result(head.task_id, True, "", 8.0)
    c.update_robot(snap(head.robot_id, "DK1"))
    out = c.tick(8.0)
    moves = {d.robot_id: tuple(d.route) for d in out.dispatches if d.order_id}
    assert moves[by_tail["Q_DK1_1"].robot_id] == ("DK1",)
    assert moves[by_tail["Q_DK1_2"].robot_id] == ("Q_DK1_1",)


# ---- pallets ---------------------------------------------------------------------------------------------------------

def test_pallet_lifecycle_is_forward_only(core):
    core.update_robot(snap("robot_1", "HM1"))
    core.add_order(pallet("p1", "PL_1", "DK2"))
    a = only(core.tick(1.0).dispatches, "robot_1")
    assert tuple(a.route) == ("PL_1", "DK2") and a.route_kind == api.ROUTE_GOALS
    core.update_robot(snap("robot_1", "PL_1", RobotMode.LIFTING, a.task_id, "p1"))
    assert states(core.tick(2.0), "p1") == [OrderState.PICKING_UP]
    core.update_robot(snap("robot_1", "X_0_1", RobotMode.CARRYING, a.task_id, "p1", payload="pallet"))
    assert states(core.tick(3.0), "p1") == [OrderState.IN_TRANSIT]
    core.update_robot(snap("robot_1", "DK2", RobotMode.LIFTING, a.task_id, "p1", payload="pallet"))
    assert states(core.tick(4.0), "p1") == []                       # setting down at the dock: no state change
    core.task_result(a.task_id, True, "", 15.0)
    core.update_robot(snap("robot_1", "DK2"))
    assert states(core.tick(15.0), "p1") == [OrderState.DELIVERED]


# ---- failures and blocked robots (O-08) ------------------------------------------------------------------------------

def test_nav_failure_before_pickup_requeues_and_sends_home(core):
    core.update_robot(snap("robot_1", "HM2"))
    core.add_order(shelf("s1", ["PK_A_2"], "DK2"))
    a = only(core.tick(1.0).dispatches, "robot_1")
    core.task_result(a.task_id, False, api.FAILURE_NAV_PREFIX + "PK_A_2", 5.0)
    core.update_robot(snap("robot_1", "X_0_1"))
    out = core.tick(5.0)
    assert OrderState.QUEUED in states(out, "s1")
    assert any(x.decision_type == api.DecisionType.REASSIGN for x in out.decisions)


def _blocked(core, rid, at, task, order, t, nxt):
    core.update_robot(snap(rid, at, RobotMode.STUCK, task, order, nxt=nxt, fault=api.FAULT_BLOCKED))
    return core.tick(t)


def test_blocked_unloaded_robot_requeues_after_timeout(core):
    core.update_robot(snap("robot_1", "HM2"))
    core.add_order(shelf("s1", ["PK_A_2"], "DK2"))
    a = only(core.tick(1.0).dispatches, "robot_1")
    out = _blocked(core, "robot_1", "X_0_1", a.task_id, "s1", 100.0, "PK_A_2")
    assert not out.cancels and states(out, "s1") == []
    out = _blocked(core, "robot_1", "X_0_1", a.task_id, "s1", 100.0 + api.BLOCKED_REQUEUE_S - 1.0, "PK_A_2")
    assert not out.cancels
    out = _blocked(core, "robot_1", "X_0_1", a.task_id, "s1", 100.0 + api.BLOCKED_REQUEUE_S + 0.1, "PK_A_2")
    assert a.task_id in out.cancels and OrderState.QUEUED in states(out, "s1")


def test_blocked_timer_resets_when_robot_moves_again(core):
    core.update_robot(snap("robot_1", "HM2"))
    core.add_order(shelf("s1", ["PK_A_2"], "DK2"))
    a = only(core.tick(1.0).dispatches, "robot_1")
    _blocked(core, "robot_1", "X_0_1", a.task_id, "s1", 100.0, "PK_A_2")
    core.update_robot(snap("robot_1", "X_0_1", RobotMode.NAVIGATING, a.task_id, "s1", nxt="PK_A_2"))
    core.tick(110.0)
    out = _blocked(core, "robot_1", "X_0_1", a.task_id, "s1", 120.0, "PK_A_2")
    out = _blocked(core, "robot_1", "X_0_1", a.task_id, "s1", 120.0 + api.BLOCKED_REQUEUE_S - 1.0, "PK_A_2")
    assert not out.cancels


def test_blocked_loaded_robot_retries_then_fails(core):
    core.update_robot(snap("robot_1", "HM2"))
    core.add_order(shelf("s1", ["PK_A_1"], "DK2"))
    a = only(core.tick(1.0).dispatches, "robot_1")
    core.update_robot(snap("robot_1", "PK_A_1", RobotMode.NAVIGATING, a.task_id, "s1", nxt="DK2"))
    core.tick(2.0)
    assert core.order_state("s1") == OrderState.IN_TRANSIT
    task, t = a.task_id, 10.0
    for attempt in range(api.MAX_LOADED_RETRIES):
        _blocked(core, "robot_1", "X_0_2", task, "s1", t, "DK2")
        out = _blocked(core, "robot_1", "X_0_2", task, "s1", t + api.BLOCKED_REQUEUE_S + 0.1, "DK2")
        assert task in out.cancels, attempt
        f = only([d for d in out.dispatches if d.order_id == "s1"], "robot_1")
        assert tuple(f.route) == ("DK2",)
        core.task_result(task, False, api.FAILURE_CANCELLED, t + api.BLOCKED_REQUEUE_S + 0.2)
        task, t = f.task_id, t + 100.0
        assert core.order_state("s1") == OrderState.IN_TRANSIT
    _blocked(core, "robot_1", "X_0_2", task, "s1", t, "DK2")
    out = _blocked(core, "robot_1", "X_0_2", task, "s1", t + api.BLOCKED_REQUEUE_S + 0.1, "DK2")
    assert OrderState.FAILED in states(out, "s1")
    assert [c.failure_reason for c in out.status_changes if c.state == OrderState.FAILED] == \
        [api.FAILURE_STUCK_AFTER_PICKUP]


def test_blocked_robot_is_not_given_v1_stuck_recovery(core):
    core.update_robot(snap("robot_1", "HM2"))
    core.add_order(shelf("s1", ["PK_A_1"], "DK2"))
    a = only(core.tick(1.0).dispatches, "robot_1")
    core.update_robot(snap("robot_1", "PK_A_1", RobotMode.NAVIGATING, a.task_id, "s1", nxt="DK2"))
    core.tick(2.0)
    out = _blocked(core, "robot_1", "X_0_2", a.task_id, "s1", 10.0, "DK2")
    out2 = _blocked(core, "robot_1", "X_0_2", a.task_id, "s1", 10.0 + api.BLOCKED_REQUEUE_S - 1.0, "DK2")
    assert not out.dispatches and not out2.dispatches and not out2.cancels
