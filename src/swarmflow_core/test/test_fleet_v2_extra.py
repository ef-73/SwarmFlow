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


# ---- review fixes (H1, M1-M4, L1, L3) ---------------------------------------------------------------------------

import pytest  # noqa: E402

from swarmflow_core.api import RobotSnapshot  # noqa: E402


def test_h1_stale_last_vertex_does_not_credit_a_pick():
    c = _core()
    c.update_robot(snap("r1", "DK1"))                    # just delivered at DK1; last_vertex stays DK1
    c.add_order(shelf("o2", ["DK1"], "DK2"))             # worst case: the pick equals the stale vertex
    tid = only(c.tick(1.0).dispatches, "r1").task_id
    c.update_robot(snap("r1", "DK1", RobotMode.NAVIGATING, tid, "o2", nxt="DK1"))
    out = c.tick(2.0)
    assert states(out, "o2") == [] and c.order_state("o2") == OrderState.ASSIGNED
    c.task_result(tid, False, "NAV_FAILED:DK1", 3.0)
    out = c.tick(3.0)
    assert states(out, "o2") == [OrderState.QUEUED]      # requeued (not loaded), pick not dropped
    # credited only after last_vertex left the baseline, in goal order
    c.update_robot(snap("r1", "HM2"))
    tid = only([d for d in c.tick(4.0).dispatches if d.order_id], "r1").task_id
    c.update_robot(snap("r1", "X_0_1", RobotMode.NAVIGATING, tid, "o2", nxt="DK1"))
    c.tick(5.0)
    c.update_robot(snap("r1", "DK1", RobotMode.NAVIGATING, tid, "o2", nxt="DK2"))
    assert states(c.tick(6.0), "o2") == [OrderState.PICKING_UP, OrderState.IN_TRANSIT]


def test_h1_final_goal_is_never_credited_from_last_vertex():
    c = _core()
    c.update_robot(snap("r1", "HM2"))
    c.add_order(shelf("o", ["PK_A_1"], "DK2"))
    tid = only(c.tick(1.0).dispatches, "r1").task_id
    c.update_robot(snap("r1", "DK2", RobotMode.UNLOADING, tid, "o"))   # skipped the pick in the snapshots
    c.tick(2.0)
    assert c.order_state("o") == OrderState.ASSIGNED


def test_m1_down_waiting_robot_releases_dock_queue_slot():
    c = _core()
    for r, at in (("r1", "HM3"), ("r2", "HM2"), ("r3", "HM1")):
        c.update_robot(snap(r, at))
    for i, p in enumerate(["PK_B_1", "PK_A_1", "PK_A_2"]):
        c.add_order(shelf(f"s{i + 1}", [p], "DK1", t=0.1 * i))
    out = c.tick(1.0)
    q1 = [d for d in out.dispatches if d.route[-1] == "Q_DK1_1"][0]
    c.task_result(q1.task_id, True, "", 4.0)
    c.update_robot(snap(q1.robot_id, "Q_DK1_1", RobotMode.FAULT, "", q1.order_id, fault="X"))
    c.tick(4.0)
    assert c.order_state(q1.order_id) == OrderState.IN_TRANSIT
    out = c.tick(4.0 + api.BLOCKED_REQUEUE_S + 0.1)
    assert c.order_state(q1.order_id) == OrderState.FAILED
    assert [x.failure_reason for x in out.status_changes if x.state == OrderState.FAILED] == \
        [api.FAILURE_STUCK_AFTER_PICKUP]
    assert "Q_DK1_1" not in c._spot_owner


def test_m2_failed_home_slot_expires_and_success_clears():
    c = _core()
    c.update_robot(snap("robot_1", "X_0_1"))
    c.update_robot(snap("robot_2", "HM3"))
    h = only(c.tick(1.0).dispatches, "robot_1")
    assert h.route == ("HM2",)
    c.task_result(h.task_id, False, api.FAILURE_NAV_PREFIX + "HM2", 2.0)
    h = only(c.tick(2.0).dispatches, "robot_1")
    assert h.route == ("HM1",)
    c.task_result(h.task_id, False, api.FAILURE_NAV_PREFIX + "HM1", 3.0)
    assert not c.tick(3.0).dispatches                       # both slots marked bad, HM3 taken
    out = c.tick(3.0 + 121.0)
    assert only(out.dispatches, "robot_1").route == ("HM2",)  # entries expired
    c._bad_home["robot_1"] = {"HM2": 200.0}
    c.add_order(shelf("s1", ["PK_A_1"], "DK2", t=0.0))
    c.update_robot(snap("robot_1", "X_0_1"))
    a = only([d for d in c.tick(130.0).dispatches if d.order_id], "robot_1")
    c.task_result(a.task_id, True, "", 140.0)
    c.tick(140.0)
    assert "robot_1" not in c._bad_home


def test_m3_requeues_are_capped():
    c = _core()
    c.add_order(shelf("s1", ["PK_A_2"], "DK2"))
    reason = api.FAILURE_NAV_PREFIX + "PK_A_2"
    t = 1.0
    for i in range(api.MAX_LOADED_RETRIES):
        c.update_robot(snap("robot_1", "HM2"))
        a = only([d for d in c.tick(t).dispatches if d.order_id], "robot_1")
        c.task_result(a.task_id, False, reason, t + 1)
        assert OrderState.QUEUED in states(c.tick(t + 1), "s1")
        t += 10.0
    c.update_robot(snap("robot_1", "HM2"))
    a = only([d for d in c.tick(t).dispatches if d.order_id], "robot_1")
    c.task_result(a.task_id, False, reason, t + 1)
    out = c.tick(t + 1)
    assert [x.failure_reason for x in out.status_changes if x.state == OrderState.FAILED] == [reason]
    assert any(x.decision_type == api.DecisionType.STUCK_FAIL for x in out.decisions)


def test_m4_home_uses_pose_not_stale_last_vertex():
    c = _core()
    x, y, _ = V["X_0_1"]
    stale = RobotSnapshot("robot_1", float(x), float(y), 0.0, RobotMode.IDLE, last_vertex="HM1")
    hx, hy, _ = V["HM2"]
    parked = RobotSnapshot("robot_2", float(hx), float(hy), 0.0, RobotMode.IDLE, last_vertex="X_0_5")
    c.update_robot(stale)
    c.update_robot(parked)
    out = c.tick(1.0)
    assert [(d.robot_id, tuple(d.route)) for d in out.dispatches] == [("robot_1", ("HM1",))]   # HM2 taken by pose


def test_l1_cancelled_set_is_pruned():
    c = _core()
    c.update_robot(snap("robot_1", "HM3"))
    c.update_robot(snap("robot_2", "HM2"))
    c.add_order(shelf("s1", ["PK_B_1"], "DK1"))
    c.add_order(shelf("s2", ["PK_A_1"], "DK1", t=0.1))
    out = c.tick(1.0)
    a1 = only(out.dispatches, "robot_1")
    a2 = only(out.dispatches, "robot_2")
    c.update_robot(snap("robot_2", "PK_A_1", RobotMode.NAVIGATING, a2.task_id, "s2", nxt="Q_DK1_1"))
    c.task_result(a1.task_id, True, "", 5.0)
    c.update_robot(snap("robot_1", "DK1"))
    assert a2.task_id in c.tick(5.0).cancels
    assert a2.task_id in c._cancelled
    c.tick(5.0 + 61.0)
    assert not c._cancelled
    assert a1.task_id not in c._task_robot


def test_l3_duplicate_order_id_rejected():
    c = _core()
    c.add_order(shelf("s1", ["PK_A_1"], "DK1"))
    with pytest.raises(ValueError):
        c.add_order(shelf("s1", ["PK_A_2"], "DK1"))
    c.tick(1.0)
    with pytest.raises(ValueError):
        c.add_order(shelf("s1", ["PK_A_2"], "DK1"))


def test_h1_robot_starting_on_its_first_pick_is_credited_by_later_goals():
    c = _core()
    c.update_robot(snap("r1", "PK_A_1"))
    c.add_order(shelf("o", ["PK_A_1", "PK_A_2"], "DK2"))
    a = only(c.tick(1.0).dispatches, "r1")
    assert tuple(a.route) == ("PK_A_1", "PK_A_2", "DK2")
    c.update_robot(snap("r1", "PK_A_2", RobotMode.NAVIGATING, a.task_id, "o", nxt="DK2"))
    assert states(c.tick(2.0), "o") == [OrderState.PICKING_UP, OrderState.IN_TRANSIT]
