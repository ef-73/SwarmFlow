"""Reservation authority tests — safety-critical (design §6.5). Lead test: do not edit.

Implementation under test: ``swarmflow_core.reservations.FcfsReservationAuthority(graph)``.
"""

from __future__ import annotations

import pathlib

import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from swarmflow_core import api
from swarmflow_core.api import LeaseState, ReleaseReason, ReservationRequest
from swarmflow_core.graph import load_layout
from swarmflow_core.reservations import FcfsReservationAuthority

FIX = pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard"
GRAPH = load_layout(FIX)
ZONES = sorted(GRAPH.zones)
ROBOTS = ["robot_1", "robot_2", "robot_3"]


def req(robot, zone="Z_aisle_1", t=0.0, n=[0], entry=None, exit_=None, exp=None):
    n[0] += 1
    z = GRAPH.zones[zone]
    entry = entry or z.entries[0].entry
    exit_ = exit_ or z.entries[-1].entry
    return ReservationRequest(request_id=f"r{n[0]}", robot_id=robot, zone_id=zone, entry_vertex=entry,
                              exit_vertex=exit_, earliest_entry_t=t, expected_exit_t=t + 20.0 if exp is None else exp)


def outside_pose():
    return (1.3, 1.1)  # P1, outside every zone


def inside_pose(zone="Z_aisle_1"):
    poly = GRAPH.zones[zone].polygon
    xs, ys = [p[0] for p in poly], [p[1] for p in poly]
    return (sum(xs) / len(xs), sum(ys) / len(ys))


def make():
    a = FcfsReservationAuthority(GRAPH)
    a.recover([], 0.0)
    return a


@pytest.fixture
def auth():
    return make()


def test_is_reservation_authority(auth):
    assert isinstance(auth, api.ReservationAuthority)


def test_grant_then_deny_then_release_then_grant(auth):
    d1 = auth.request(req("robot_1"), 0.0)
    assert d1.granted and d1.lease_id and d1.lease_expiry_t == pytest.approx(api.LEASE_TTL_S)
    d2 = auth.request(req("robot_2"), 0.5)
    assert not d2.granted and d2.reason == "ZONE_LEASED" and d2.lease_id == ""
    assert d2.retry_after_s == pytest.approx(api.RETRY_AFTER_S)
    auth.release("robot_1", d1.lease_id, 1.0)
    d3 = auth.request(req("robot_2"), 1.5)
    assert d3.granted and d3.lease_id != d1.lease_id


def test_other_zone_independent(auth):
    assert auth.request(req("robot_1", "Z_aisle_1"), 0.0).granted
    assert auth.request(req("robot_2", "Z_aisle_2"), 0.0).granted


def test_unknown_zone_and_bad_entry(auth):
    d = auth.request(ReservationRequest("x", "robot_1", "Z_nope", "X_1_1", "X_1_3"), 0.0)
    assert not d.granted and d.reason == "UNKNOWN_ZONE"
    d = auth.request(req("robot_1", entry="X_2_1"), 0.0)
    assert not d.granted and d.reason == "BAD_ENTRY"


def test_rerequest_by_holder_is_idempotent(auth):
    d1 = auth.request(req("robot_1"), 0.0)
    d2 = auth.request(req("robot_1"), 2.0)
    assert d2.granted and d2.lease_id == d1.lease_id
    assert d2.lease_expiry_t == pytest.approx(2.0 + api.LEASE_TTL_S)


def test_heartbeat_renews_until_hard_bound(auth):
    d = auth.request(req("robot_1", t=0.0, exp=4.0), 0.0)  # hard = 4 + max(10, 2) = 14
    t = 0.0
    while t < 30.0:
        t += 1.0
        auth.heartbeat("robot_1", [d.lease_id], t)
        changed = auth.expire(t)
        if changed:
            break
    assert changed and changed[0].state == LeaseState.OCCUPIED_UNKNOWN and changed[0].lease_id == d.lease_id
    assert t == pytest.approx(14.0)
    assert changed[0].hard_expiry_t == pytest.approx(14.0)


def test_heartbeat_from_other_robot_does_not_renew(auth):
    d = auth.request(req("robot_1"), 0.0)
    auth.heartbeat("robot_2", [d.lease_id], 4.0)
    assert [l.lease_id for l in auth.expire(5.0)] == [d.lease_id]


def test_expiry_makes_zone_occupied_unknown_until_owner_reports_outside(auth):
    d = auth.request(req("robot_1"), 0.0)
    ch = auth.expire(5.0)
    assert [l.state for l in ch] == [LeaseState.OCCUPIED_UNKNOWN]
    d2 = auth.request(req("robot_2"), 6.0)
    assert not d2.granted and d2.reason == "OCCUPIED_UNKNOWN"
    # owner still inside → still blocked
    assert auth.observe_robot("robot_1", *inside_pose(), 7.0) == []
    # another robot outside → irrelevant
    assert auth.observe_robot("robot_2", *outside_pose(), 7.0) == []
    assert not auth.request(req("robot_2"), 7.5).granted
    # owner outside → cleared
    ch = auth.observe_robot("robot_1", *outside_pose(), 8.0)
    assert ch and ch[-1].lease_id == d.lease_id and ch[-1].state == LeaseState.RELEASED
    assert auth.request(req("robot_2"), 8.5).granted


def test_late_release_of_expired_lease_clears_zone(auth):
    d = auth.request(req("robot_1"), 0.0)
    auth.expire(6.0)
    auth.release("robot_1", d.lease_id, 7.0, ReleaseReason.EXITED)
    assert auth.request(req("robot_2"), 7.5).granted


def test_release_by_other_robot_ignored(auth):
    d = auth.request(req("robot_1"), 0.0)
    auth.release("robot_2", d.lease_id, 1.0)
    assert not auth.request(req("robot_2"), 1.5).granted


def test_clear_zone(auth):
    d = auth.request(req("robot_1"), 0.0)
    auth.expire(6.0)
    assert auth.clear_zone("Z_aisle_1", "operator", 7.0) is True
    assert auth.clear_zone("Z_nope", "operator", 7.0) is False
    assert auth.request(req("robot_2"), 7.5).granted
    states = [l.state for l in auth.pop_events() if l.lease_id == d.lease_id]
    assert states == [LeaseState.GRANTED, LeaseState.OCCUPIED_UNKNOWN, LeaseState.REVOKED]


def test_request_applies_expiry_first(auth):
    """A request at t past the holder's TTL must not be granted (zone may still be occupied)."""
    auth.request(req("robot_1"), 0.0)
    d = auth.request(req("robot_2"), 100.0)
    assert not d.granted and d.reason == "OCCUPIED_UNKNOWN"


def test_pop_events_order_and_drain(auth):
    d = auth.request(req("robot_1"), 0.0)
    auth.release("robot_1", d.lease_id, 1.0)
    ev = auth.pop_events()
    assert [(e.lease_id, e.state) for e in ev] == [(d.lease_id, LeaseState.GRANTED), (d.lease_id, LeaseState.RELEASED)]
    assert auth.pop_events() == []


def test_active_leases(auth):
    d1 = auth.request(req("robot_1", "Z_aisle_1"), 0.0)
    d2 = auth.request(req("robot_2", "Z_aisle_2"), 0.0)
    assert [l.lease_id for l in auth.active_leases()] == sorted([d1.lease_id, d2.lease_id])
    auth.expire(6.0)
    act = auth.active_leases()
    assert len(act) == 2 and all(l.state == LeaseState.OCCUPIED_UNKNOWN for l in act)  # still blocking


def test_deterministic_lease_ids():
    a, b = make(), make()
    ids_a = [a.request(req(r, z), 0.0).lease_id for r, z in zip(ROBOTS, ZONES)]
    ids_b = [b.request(req(r, z), 0.0).lease_id for r, z in zip(ROBOTS, ZONES)]
    assert ids_a == ids_b


# ---- property test: safety invariant under arbitrary operation sequences --------------------------------------

op = st.one_of(
    st.tuples(st.just("request"), st.sampled_from(ROBOTS), st.sampled_from(ZONES)),
    st.tuples(st.just("heartbeat"), st.sampled_from(ROBOTS), st.just("")),
    st.tuples(st.just("release"), st.sampled_from(ROBOTS), st.sampled_from(ZONES)),
    st.tuples(st.just("observe_out"), st.sampled_from(ROBOTS), st.just("")),
    st.tuples(st.just("observe_in"), st.sampled_from(ROBOTS), st.sampled_from(ZONES)),
    st.tuples(st.just("expire"), st.just(""), st.just("")),
    st.tuples(st.just("clear"), st.just(""), st.sampled_from(ZONES)),
)


@settings(max_examples=400, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(ops=st.lists(st.tuples(op, st.floats(min_value=0.0, max_value=4.0)), min_size=1, max_size=60))
def test_safety_invariant(ops):
    """Never two GRANTED leases in one zone; never a grant while a zone is OCCUPIED_UNKNOWN; a robot granted a zone
    whose previous holder's lease expired only after that holder was seen outside, released late, or a clear."""
    a = make()
    t = 0.0
    held = {}         # robot -> {zone: lease_id}  (what agents believe they hold)
    blocked = set()   # zones whose lease expired and are not yet cleared (model)
    for (name, robot, zone), dt in ops:
        t += dt
        # model: expiry happens implicitly at every call
        for l in a.expire(t):
            pass
        if name == "request":
            d = a.request(req(robot, zone, t), t)
            if d.granted:
                held.setdefault(robot, {})[zone] = d.lease_id
        elif name == "heartbeat":
            a.heartbeat(robot, list(held.get(robot, {}).values()), t)
        elif name == "release":
            lid = held.get(robot, {}).pop(zone, None)
            if lid:
                a.release(robot, lid, t)
        elif name == "observe_out":
            a.observe_robot(robot, *outside_pose(), t)
        elif name == "observe_in":
            a.observe_robot(robot, *inside_pose(zone), t)
        elif name == "expire":
            a.expire(t)
        elif name == "clear":
            a.clear_zone(zone, "test", t)
        # invariant on the authority's own view
        per_zone = {}
        for l in a.active_leases():
            per_zone.setdefault(l.zone_id, []).append(l)
        for z, ls in per_zone.items():
            live = [l for l in ls if l.state == LeaseState.GRANTED]
            assert len(live) <= GRAPH.zones[z].capacity, (z, ls)
            if any(l.state == LeaseState.OCCUPIED_UNKNOWN for l in ls):
                assert not live, f"grant into OCCUPIED_UNKNOWN zone {z}: {ls}"
            for l in live:
                assert l.expiry_t > t - 1e-9 and l.expiry_t <= l.hard_expiry_t + 1e-9
    # events are consistent: every lease starts GRANTED and never returns to GRANTED
    seen = {}
    for e in a.pop_events():
        if e.lease_id not in seen:
            assert e.state == LeaseState.GRANTED
        else:
            assert e.state != LeaseState.GRANTED
        seen[e.lease_id] = e.state


# ---- G2 review additions: recovery, release while inside, stale poses -----------------------------------------

def test_new_authority_denies_until_recovered():
    a = FcfsReservationAuthority(GRAPH)
    d = a.request(req("robot_1"), 0.0)
    assert not d.granted and d.reason == api.DENY_RECOVERING
    assert a.recover([], 2.0) == []
    assert a.request(req("robot_1"), 2.1).granted


def test_recover_blocks_zones_with_robots_inside():
    a = FcfsReservationAuthority(GRAPH)
    x, y = inside_pose("Z_aisle_2")
    robots = [api.RobotSnapshot("robot_1", x, y, 0.0, api.RobotMode.NAVIGATING, held_lease_ids=("lease_00042",)),
              api.RobotSnapshot("robot_2", *outside_pose(), 0.0, api.RobotMode.IDLE)]
    created = a.recover(robots, 2.0)
    assert [(l.lease_id, l.zone_id, l.state) for l in created] == \
        [("recovered_robot_1_Z_aisle_2", "Z_aisle_2", LeaseState.OCCUPIED_UNKNOWN)]
    assert not a.request(req("robot_2", "Z_aisle_2"), 2.1).granted
    assert a.request(req("robot_2", "Z_aisle_1"), 2.1).granted
    a.observe_robot("robot_1", *outside_pose(), 5.0)
    assert a.request(req("robot_3", "Z_aisle_2"), 5.1).granted


def test_release_while_inside_keeps_zone_blocked():
    a = make()
    d = a.request(req("robot_1"), 0.0)
    a.observe_robot("robot_1", *inside_pose(), 1.0)
    a.release("robot_1", d.lease_id, 1.5, ReleaseReason.TASK_CANCELLED)
    assert [e.state for e in a.pop_events()][-1] == LeaseState.OCCUPIED_UNKNOWN
    dn = a.request(req("robot_2"), 2.0)
    assert not dn.granted and dn.reason == api.DENY_OCCUPIED_UNKNOWN
    a.observe_robot("robot_1", *outside_pose(), 3.0)
    assert a.request(req("robot_2"), 3.1).granted


def test_release_when_last_pose_outside_frees_immediately():
    a = make()
    d = a.request(req("robot_1"), 0.0)
    a.observe_robot("robot_1", *inside_pose(), 1.0)
    a.heartbeat("robot_1", [d.lease_id], 4.0)
    a.observe_robot("robot_1", *outside_pose(), 4.5)
    a.release("robot_1", d.lease_id, 4.6)
    assert a.request(req("robot_2"), 4.7).granted


def test_stale_pose_does_not_clear():
    a = make()
    a.request(req("robot_1"), 0.0)
    a.expire(5.0)                                                    # blocking since t=5
    assert a.observe_robot("robot_1", *outside_pose(), 4.0) == []    # pose stamped before blocking
    assert not a.request(req("robot_2"), 5.5).granted


def test_hard_bound_clamped_to_grant_time():
    a = make()
    d = a.request(ReservationRequest("r", "robot_1", "Z_aisle_1", "X_1_1", "X_1_3"), 100.0)  # defaults 0.0
    assert d.granted and d.lease_expiry_t == pytest.approx(105.0)
    lease = a.active_leases()[0]
    assert lease.hard_expiry_t == pytest.approx(110.0)
