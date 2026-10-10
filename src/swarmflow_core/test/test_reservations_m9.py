"""M9 reservation hardening (independent review S2, S3, S6, T1, T2). Lead test: do not edit."""

from __future__ import annotations

import pathlib

from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from swarmflow_core import api
from swarmflow_core.api import LeaseState, ReleaseReason, ReservationRequest
from swarmflow_core.graph import load_layout
from swarmflow_core.reservations import FcfsReservationAuthority

GRAPH = load_layout(pathlib.Path(__file__).resolve().parents[3] / "tests" / "fixtures" / "standard")
ZONES = sorted(GRAPH.zones)
ROBOTS = ["robot_1", "robot_2", "robot_3"]
OUT = (1.3, 1.1)


def inside(zone):
    poly = GRAPH.zones[zone].polygon
    return (sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly))


_n = [0]


def req(robot, zone="Z_aisle_1"):
    _n[0] += 1
    z = GRAPH.zones[zone]
    return ReservationRequest(f"q{_n[0]}", robot, zone, z.entries[0].entry, z.entries[-1].entry)


def make():
    a = FcfsReservationAuthority(GRAPH)
    a.recover([], 0.0)
    return a


# ---- S2: the owner of a blocking lease may get its zone back (it is the only possible occupant) ------------------

def test_owner_of_blocking_lease_is_regranted():
    a = make()
    d = a.request(req("robot_1"), 0.0)
    a.observe_robot("robot_1", *inside("Z_aisle_1"), 1.0)
    a.release("robot_1", d.lease_id, 2.0, ReleaseReason.FAULT)          # stuck inside → blocking
    a.pop_events()
    d2 = a.request(req("robot_1"), 3.0)                                   # park task from inside the zone
    assert d2.granted and d2.lease_id != d.lease_id
    ev = a.pop_events()
    assert [(e.lease_id, e.state) for e in ev] == [(d.lease_id, LeaseState.RELEASED), (d2.lease_id, LeaseState.GRANTED)]
    assert ev[0].reason == "SUPERSEDED"
    assert not a.request(req("robot_2"), 3.5).granted                    # still exclusive


def test_owner_regrant_not_allowed_if_someone_else_blocks():
    a = make()
    a.request(req("robot_1"), 0.0)
    a.observe_robot("robot_2", *inside("Z_aisle_1"), 0.5)                # robot_2 inside without lease (S3)
    a.expire(6.0)
    assert not a.request(req("robot_1"), 6.5).granted


# ---- S3: a robot observed inside a zone without a lease blocks that zone ----------------------------------------

def test_unauthorized_entry_blocks_zone_until_outside():
    a = make()
    ch = a.observe_robot("robot_2", *inside("Z_aisle_2"), 1.0)
    assert len(ch) == 1 and ch[0].state == LeaseState.OCCUPIED_UNKNOWN and ch[0].reason == "UNAUTHORIZED_ENTRY"
    assert ch[0].robot_id == "robot_2" and ch[0].zone_id == "Z_aisle_2"
    d = a.request(req("robot_1", "Z_aisle_2"), 1.5)
    assert not d.granted and d.reason == api.DENY_OCCUPIED_UNKNOWN
    assert a.observe_robot("robot_2", *inside("Z_aisle_2"), 2.0) == []   # no duplicate block
    a.observe_robot("robot_2", *OUT, 3.0)
    assert a.request(req("robot_1", "Z_aisle_2"), 3.5).granted


def test_lease_holder_inside_is_not_unauthorized():
    a = make()
    a.request(req("robot_1"), 0.0)
    assert a.observe_robot("robot_1", *inside("Z_aisle_1"), 1.0) == []


# ---- S6: FCFS means first come, first served ----------------------------------------------------------------------

def test_fifo_queue_respected():
    a = make()
    d1 = a.request(req("robot_1"), 0.0)
    assert not a.request(req("robot_2"), 1.0).granted                    # robot_2 queues first
    assert not a.request(req("robot_3"), 2.0).granted                    # then robot_3
    a.release("robot_1", d1.lease_id, 3.0)
    d3 = a.request(req("robot_3"), 3.1)                                  # robot_3 asks first after the release …
    assert not d3.granted and d3.reason.startswith("QUEUED_BEHIND:robot_2")
    assert a.request(req("robot_2"), 3.2).granted                        # … but robot_2 was first in line


def test_stale_queue_head_is_skipped():
    a = make()
    d1 = a.request(req("robot_1"), 0.0)
    a.request(req("robot_2"), 1.0)                                       # robot_2 queues, then never asks again
    a.request(req("robot_3"), 2.0)
    a.release("robot_1", d1.lease_id, 3.0)
    a.request(req("robot_3"), 3.0)                                       # robot_2 last asked at 1.0
    assert a.request(req("robot_3"), 5.0).granted                        # head stale (> 3 s) → skipped


# ---- T1: physical-occupancy safety property ---------------------------------------------------------------------

op = st.one_of(
    st.tuples(st.just("request"), st.sampled_from(ROBOTS), st.sampled_from(ZONES)),
    st.tuples(st.just("enter"), st.sampled_from(ROBOTS), st.sampled_from(ZONES)),   # robot moves into a zone (any reason)
    st.tuples(st.just("leave"), st.sampled_from(ROBOTS), st.just("")),
    st.tuples(st.just("release"), st.sampled_from(ROBOTS), st.sampled_from(ZONES)),
    st.tuples(st.just("heartbeat"), st.sampled_from(ROBOTS), st.just("")),
    st.tuples(st.just("tick"), st.just(""), st.just("")),
)


@settings(max_examples=300, deadline=None, derandomize=True, suppress_health_check=[HealthCheck.too_slow])
@given(ops=st.lists(st.tuples(op, st.floats(min_value=0.05, max_value=3.0)), min_size=1, max_size=60))
def test_never_grant_into_physically_occupied_zone(ops):
    """With fresh poses reported after every move, the authority never grants zone Z to robot R while another robot's
    latest reported pose is inside Z — whatever that robot did (no lease, expired lease, released inside, …)."""
    a = make()
    t = 0.0
    where = {r: None for r in ROBOTS}          # zone the robot is physically in (None = outside)
    held = {r: {} for r in ROBOTS}
    for (name, robot, zone), dt in ops:
        t += dt
        if name == "request":
            d = a.request(req(robot, zone), t)
            if d.granted:
                others = [o for o in ROBOTS if o != robot and where[o] == zone]
                assert not others, f"granted {zone} to {robot} while {others} inside"
                held[robot][zone] = d.lease_id
        elif name == "enter":
            where[robot] = zone
            a.observe_robot(robot, *inside(zone), t)
        elif name == "leave":
            where[robot] = None
            a.observe_robot(robot, *OUT, t)
        elif name == "release":
            lid = held[robot].pop(zone, None)
            if lid:
                a.release(robot, lid, t)
        elif name == "heartbeat":
            a.heartbeat(robot, list(held[robot].values()), t)
        else:
            a.expire(t)
            for r in ROBOTS:                   # agents publish poses continuously
                a.observe_robot(r, *(inside(where[r]) if where[r] else OUT), t)
