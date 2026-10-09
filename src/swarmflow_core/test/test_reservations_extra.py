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
    d = a.request(req("robot_1", t=8.0), 8.0)
    a.expire(14.0)
    a.release("robot_1", d.lease_id, 15.0)
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
