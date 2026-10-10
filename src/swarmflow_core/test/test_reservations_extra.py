"""Extra reservation tests (T003): spec points not covered by the lead tests."""

from __future__ import annotations

import pathlib

import pytest

from swarmflow_core import api
from swarmflow_core.api import LeaseState, ReleaseReason, ReservationRequest
from swarmflow_core.graph import load_layout
from swarmflow_core.reservations import FcfsReservationAuthority

GRAPH = load_layout(pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard")
OUT = (1.3, 1.1)


def inside(zone="Z_aisle_1"):
    poly = GRAPH.zones[zone].polygon
    return (sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly))


def req(robot, zone="Z_aisle_1", t=0.0, exit_t=20.0):
    z = GRAPH.zones[zone]
    return ReservationRequest("r", robot, zone, z.entries[0].entry, z.entries[-1].entry,
                              earliest_entry_t=t, expected_exit_t=t + exit_t)


def make(**kw):
    a = FcfsReservationAuthority(GRAPH, **kw)
    a.recover([], 0.0)
    return a


def test_custom_ttl_and_retry():
    a = make(lease_ttl_s=2.0, retry_after_s=0.25)
    d = a.request(req("robot_1"), 0.0)
    assert d.lease_expiry_t == pytest.approx(2.0)
    assert a.request(req("robot_2"), 0.1).retry_after_s == pytest.approx(0.25)


def test_ids_and_event_reasons():
    a = make()
    d1 = a.request(req("robot_1", "Z_aisle_1"), 0.0)
    d2 = a.request(req("robot_2", "Z_aisle_2"), 0.0)
    assert (d1.lease_id, d2.lease_id) == ("lease_00001", "lease_00002")
    a.release("robot_1", d1.lease_id, 1.0, ReleaseReason.FAULT)
    assert a.pop_events()[-1].reason == "FAULT"


def test_idempotent_regrant_emits_no_event():
    a = make()
    a.request(req("robot_1"), 0.0)
    a.pop_events()
    a.request(req("robot_1"), 1.0)
    assert a.pop_events() == []


def test_clear_zone_revokes_granted_and_reason():
    a = make()
    d = a.request(req("robot_1"), 0.0)
    a.pop_events()
    assert a.clear_zone("Z_aisle_1", "op", 1.0)
    ev = a.pop_events()
    assert [(e.lease_id, e.state, e.reason) for e in ev] == [(d.lease_id, LeaseState.REVOKED, "CLEAR_ZONE:op")]
    assert a.active_leases() == []


def test_observed_outside_and_late_release_reasons():
    a = make()
    d = a.request(req("robot_1"), 0.0)
    a.expire(6.0)
    a.observe_robot("robot_1", *OUT, 7.0)
    assert a.pop_events()[-1].reason == "OBSERVED_OUTSIDE"
    d = a.request(req("robot_2", t=8.0), 8.0)  # robot_2 never reported a pose: release is trusted
    a.expire(14.0)
    a.pop_events()
    a.release("robot_2", d.lease_id, 15.0)
    assert a.pop_events()[-1].reason == "LATE_RELEASE"


def test_late_release_while_inside_keeps_blocking():
    a = make()
    d = a.request(req("robot_1"), 0.0)
    a.observe_robot("robot_1", *inside(), 1.0)
    a.expire(6.0)
    a.release("robot_1", d.lease_id, 7.0)
    assert not a.request(req("robot_2"), 7.5).granted


def test_release_inside_reason():
    a = make()
    d = a.request(req("robot_1"), 0.0)
    a.observe_robot("robot_1", *inside(), 0.5)
    a.release("robot_1", d.lease_id, 1.0, ReleaseReason.FAULT)
    assert a.pop_events()[-1].reason == "RELEASED_INSIDE:FAULT"


def test_recovered_lease_needs_pose_stamp_not_older_than_recovery():
    a = FcfsReservationAuthority(GRAPH)
    r = api.RobotSnapshot("robot_1", *inside(), 0.0, api.RobotMode.IDLE)
    a.recover([r], 10.0)
    assert a.observe_robot("robot_1", *OUT, 9.0) == []
    ch = a.observe_robot("robot_1", *OUT, 10.5)
    assert [l.state for l in ch] == [LeaseState.RELEASED]


def test_recovered_lease_released_only_via_owner_or_clear():
    a = FcfsReservationAuthority(GRAPH)
    a.recover([api.RobotSnapshot("robot_1", *inside(), 0.0, api.RobotMode.IDLE)], 1.0)
    assert not a.request(req("robot_2"), 100.0).granted  # never TTL-expires into a grant
    assert a.clear_zone("Z_aisle_1", "op", 101.0)
    assert a.request(req("robot_2"), 101.5).granted


def test_boundary_counts_as_inside():
    a = make()
    d = a.request(req("robot_1"), 0.0)
    corner = GRAPH.zones["Z_aisle_1"].polygon[0]
    a.observe_robot("robot_1", *corner, 1.0)
    a.release("robot_1", d.lease_id, 2.0)
    assert a.active_leases()[0].state == LeaseState.OCCUPIED_UNKNOWN


def test_stale_out_of_order_pose_does_not_clear():
    a = make()
    a.request(req("robot_1"), 0.0)
    a.expire(5.0)
    a.observe_robot("robot_1", *inside(), 20.0)
    assert a.observe_robot("robot_1", *OUT, 12.0) == []
    assert not a.request(req("robot_2"), 21.0).granted


def test_release_with_stale_pose_stays_blocked():
    a = make()
    a.observe_robot("robot_1", *OUT, 1.0)
    d = a.request(req("robot_1"), 2.0)
    a.release("robot_1", d.lease_id, 4.0, ReleaseReason.FAULT)
    assert a.pop_events()[-1].state == LeaseState.OCCUPIED_UNKNOWN
    assert not a.request(req("robot_2"), 5.0).granted


def test_release_pose_older_than_fresh_window_is_not_evidence():
    a = make()
    d = a.request(req("robot_1"), 0.0)
    a.observe_robot("robot_1", *OUT, 1.0)
    a.release("robot_1", d.lease_id, 3.0)  # pose is 2 s old > POSE_FRESH_S
    assert a.active_leases()[0].state == LeaseState.OCCUPIED_UNKNOWN


def test_release_with_fresh_outside_pose_frees():
    a = make()
    d = a.request(req("robot_1"), 0.0)
    a.observe_robot("robot_1", *OUT, 2.5)
    a.release("robot_1", d.lease_id, 3.0)
    assert a.active_leases() == []


def test_blocking_release_with_stale_pose_stays_blocking():
    a = make()
    d = a.request(req("robot_1"), 0.0)
    a.observe_robot("robot_1", *OUT, 1.0)
    a.expire(5.0)
    a.release("robot_1", d.lease_id, 6.0)
    assert a.active_leases()[0].state == LeaseState.OCCUPIED_UNKNOWN


# ---- lead additions after the T018 review (escalated to the lead) ---------------------------------------------

def _req2(robot, zone="Z_aisle_1", n=[0]):
    from swarmflow_core.api import ReservationRequest
    n[0] += 1
    z = GRAPH.zones[zone]
    return ReservationRequest(f"x{n[0]}", robot, zone, z.entries[0].entry, z.entries[-1].entry)


def test_no_grant_while_intruder_pose_inside_after_clear_zone():
    from swarmflow_core.reservations import FcfsReservationAuthority
    a = FcfsReservationAuthority(GRAPH)
    a.recover([], 0.0)
    poly = GRAPH.zones["Z_aisle_1"].polygon
    cx, cy = sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly)
    a.observe_robot("robot_2", cx, cy, 1.0)                  # intruder → blocked
    assert a.clear_zone("Z_aisle_1", "operator", 1.2)        # operator clears while robot_2 is still inside
    assert not a.request(_req2("robot_1"), 1.5).granted      # its fresh pose (0.5 s old) still says inside


def test_queue_stale_threshold_scales_with_retry_period():
    from swarmflow_core.reservations import FcfsReservationAuthority
    a = FcfsReservationAuthority(GRAPH, retry_after_s=2.0)  # agents retry every 2 s → head stale only after 5 s
    a.recover([], 0.0)
    d1 = a.request(_req2("robot_1"), 0.0)
    a.request(_req2("robot_2"), 1.0)
    a.release("robot_1", d1.lease_id, 2.0)
    assert not a.request(_req2("robot_3"), 4.5).granted      # robot_2 asked 3.5 s ago: still the head
    assert a.request(_req2("robot_2"), 4.8).granted
