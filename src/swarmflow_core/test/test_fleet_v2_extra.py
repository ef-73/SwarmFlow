"""T028 extra tests for FleetCoreV2 (reuses the lead's comb graph helpers)."""

from __future__ import annotations

from test_fleet_v2 import (QUEUE_INDEX, V, LINKS, make_graph, only, pallet, shelf, snap, states)  # noqa: F401

from swarmflow_core import api
from swarmflow_core.api import OrderState, RobotMode, Vertex
from swarmflow_core.fleet_v2 import FleetCoreV2
from swarmflow_core.graph import WarehouseGraph
from swarmflow_core.policies.assign import AssignPolicy


def _core():
    return FleetCoreV2(make_graph(), AssignPolicy(), queue_spots=QUEUE_INDEX)


def test_queue_spots_derived_from_graph_names():
    c = FleetCoreV2(make_graph(), AssignPolicy())
    assert c._spots == {"DK1": [(1, "Q_DK1_1"), (2, "Q_DK1_2")], "DK2": [(1, "Q_DK2_1")]}


def test_two_docks_with_queues_at_once():
    c = _core()
    for r, at in (("robot_1", "HM3"), ("robot_2", "HM2"), ("robot_3", "HM1"), ("robot_4", "X_0_3")):
        c.update_robot(snap(r, at))
    c.add_order(shelf("a", ["PK_B_1"], "DK1", t=0.0))
    c.add_order(shelf("b", ["PK_A_1"], "DK1", t=0.1))
    c.add_order(shelf("c", ["PK_A_2"], "DK2", t=0.2))
    c.add_order(shelf("d", ["PK_B_2"], "DK2", t=0.3))
    out = c.tick(1.0)
    tails = {d.order_id: d.route[-1] for d in out.dispatches if d.order_id}
    assert tails == {"a": "DK1", "b": "Q_DK1_1", "c": "DK2", "d": "Q_DK2_1"}
    assert len({d.robot_id for d in out.dispatches}) == 4
    # free DK2 only: DK1's queue must not move
    c.task_result(out.dispatches[[d.order_id for d in out.dispatches].index("c")].task_id, True, "", 5.0)
    c.update_robot(snap("robot_3", "DK2"))
    out = c.tick(5.0)
    moved = {d.order_id: tuple(d.route) for d in out.dispatches if d.order_id}
    assert moved == {"d": ("PK_B_2", "DK2")}
    assert c.order_state("c") == OrderState.DELIVERED


def test_home_slot_freed_and_reused():
    c = _core()
    c.update_robot(snap("robot_1", "X_0_1"))
    c.update_robot(snap("robot_2", "HM3"))
    c.update_robot(snap("robot_3", "HM1"))
    out = c.tick(1.0)
    assert {d.robot_id: tuple(d.route) for d in out.dispatches} == {"robot_1": ("HM2",)}
    # robot_3 leaves HM1 for an order; robot_4 appears and may take HM1 only after it is vacated
    c.add_order(shelf("s1", ["PK_A_1"], "DK2", t=1.5))
    c.update_robot(snap("robot_1", "HM2"))
    c.update_robot(snap("robot_4", "X_0_5"))
    out = c.tick(2.0)
    a = only(out.dispatches, "robot_1")                                     # nearest to the pick: leaves HM2
    assert a.order_id == "s1"
    assert not [d for d in out.dispatches if d.robot_id == "robot_4"]       # HM2 still occupied by robot_1
    c.update_robot(snap("robot_1", "X_0_1", RobotMode.NAVIGATING, a.task_id, "s1", nxt="PK_A_1"))
    out = c.tick(3.0)
    assert {d.robot_id: tuple(d.route) for d in out.dispatches} == {"robot_4": ("HM2",)}


def test_unreachable_pick_is_not_assigned_and_idle_robot_goes_home():
    verts = dict(make_graph().vertices)
    verts["PK_ISLAND"] = Vertex("PK_ISLAND", 20.0, 20.0, "pick")
    g = WarehouseGraph(verts, make_graph().edges, {}, name="island")
    c = FleetCoreV2(g, AssignPolicy(), queue_spots=QUEUE_INDEX)
    c.update_robot(snap("robot_1", "X_0_1"))
    c.add_order(shelf("s1", ["PK_ISLAND"], "DK1"))
    out = c.tick(1.0)
    assert c.order_state("s1") == OrderState.QUEUED
    assert [tuple(d.route) for d in out.dispatches] == [("HM2",)] and out.dispatches[0].order_id == ""


def test_nav_failed_pick_requeues_order_and_robot_goes_home():
    c = _core()
    c.update_robot(snap("robot_1", "HM2"))
    c.add_order(shelf("s1", ["PK_A_2"], "DK2"))
    a = only(c.tick(1.0).dispatches, "robot_1")
    c.task_result(a.task_id, False, api.FAILURE_NAV_PREFIX + "PK_A_2", 5.0)
    c.update_robot(snap("robot_1", "X_0_1"))
    out = c.tick(5.0)
    assert c.order_state("s1") == OrderState.QUEUED
    assert states(out, "s1") == [OrderState.QUEUED]
    home = only(out.dispatches, "robot_1")
    assert home.order_id == "" and tuple(home.route) == ("HM2",)
    assert not [d for d in out.dispatches if d.order_id]            # not re-planned onto the same robot this tick


def test_loaded_nav_failure_retries_with_remaining_goals():
    c = _core()
    c.update_robot(snap("robot_1", "HM2"))
    c.add_order(shelf("s1", ["PK_A_1", "PK_A_2"], "DK2"))
    a = only(c.tick(1.0).dispatches, "robot_1")
    assert tuple(a.route) == ("PK_A_1", "PK_A_2", "DK2")
    c.update_robot(snap("robot_1", "PK_A_1", RobotMode.NAVIGATING, a.task_id, "s1", nxt="PK_A_2"))
    c.tick(2.0)
    c.task_result(a.task_id, False, api.FAILURE_NAV_PREFIX + "PK_A_2", 3.0)
    out = c.tick(3.0)
    f = only([d for d in out.dispatches if d.order_id == "s1"], "robot_1")
    assert tuple(f.route) == ("PK_A_2", "DK2") and f.task_id != a.task_id
    assert c.order_state("s1") == OrderState.PICKING_UP


def test_forward_only_states_never_regress_on_late_snapshots():
    c = _core()
    c.update_robot(snap("robot_1", "HM2"))
    c.add_order(shelf("s1", ["PK_A_1"], "DK2"))
    a = only(c.tick(1.0).dispatches, "robot_1")
    c.update_robot(snap("robot_1", "PK_A_1", RobotMode.NAVIGATING, a.task_id, "s1", nxt="DK2"))
    c.tick(2.0)
    c.update_robot(snap("robot_1", "X_0_1", RobotMode.NAVIGATING, a.task_id, "s1", nxt="DK2"))
    assert states(c.tick(3.0), "s1") == []
    assert c.order_state("s1") == OrderState.IN_TRANSIT
