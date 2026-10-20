"""T067: yield policy (pure)."""

from __future__ import annotations

import math

from swarmflow_robot_agent.yield_policy import (YIELD_MAX_S, Body, closest_approach, keep_yielding, should_yield)

E, W, N, S = 0.0, math.pi, math.pi / 2, -math.pi / 2


def body(rid, x, y, yaw, v=0.5, mode="NAVIGATING"):
    return Body(rid, x, y, yaw, v, mode)


def test_closest_approach_head_on():
    d, t, closing = closest_approach(body("a", 0, 0, E), body("b", 4, 0, W))
    assert d < 1e-6 and abs(t - 4.0) < 1e-6 and closing


def test_head_on_exactly_one_robot_yields():
    a, b = body("robot_1", 0, 0, E, 0.5), body("robot_2", 3, 0.2, W, 0.5)        # meet in the middle: a tie
    assert [should_yield(a, b), should_yield(b, a)].count(None) == 1
    assert should_yield(b, a) == "tie"                                           # the higher id yields
    a2, b2 = body("robot_1", 0, 0, E, 0.6), body("robot_2", 3, 0.2, W, 0.3)        # head-on: both reach the
    assert should_yield(a2, b2) is None and should_yield(b2, a2) == "tie"          # meeting point together


def test_crossing_the_later_robot_yields():
    a = body("robot_2", 0, 0, E, 0.5)            # reaches (2, 0) in 4 s
    b = body("robot_1", 2, -1.0, N, 0.5)         # reaches (2, 0) in 2 s
    assert should_yield(a, b) == "later" and should_yield(b, a) is None


def test_following_or_passing_by_is_no_encounter():
    assert should_yield(body("robot_2", 0, 0, E), body("robot_1", 1.5, 0, E, 0.2)) is None          # following
    assert should_yield(body("robot_2", 0, 0, E), body("robot_1", 3, 2.0, W)) is None              # 2 m apart
    assert should_yield(body("robot_2", 0, 0, E), body("robot_1", -2, 0, W)) is None               # moving apart
    assert should_yield(body("robot_2", 0, 0, E), body("robot_1", 9, 0, W)) is None                # too far


def test_never_yield_to_a_waiting_stuck_or_standing_robot():
    me = body("robot_2", 0, 0, E, 0.5)
    for mode in ("WAITING_RESERVATION", "STUCK", "UNLOADING", "LIFTING"):
        assert should_yield(me, body("robot_1", 2, 0, W, 0.5, mode)) is None
    assert should_yield(me, body("robot_1", 2, 0, W, 0.0)) is None


def test_pallets_have_right_of_way():
    me, carrier = body("robot_1", 0, 0, E, 0.6), body("robot_2", 3, 0, W, 0.2, "CARRYING")
    assert should_yield(me, carrier) == "pallet"
    assert should_yield(carrier, me, me_has_pallet=True) is None


def test_wait_ends_when_the_other_has_passed_stopped_or_after_the_limit():
    me = body("robot_2", 0, 0, E, 0.0, "WAITING_RESERVATION")
    coming = body("robot_1", 2, 0.3, W, 0.5)
    assert keep_yielding(me, coming, 10.0, 11.0)
    assert not keep_yielding(me, coming, 10.0, 10.0 + YIELD_MAX_S)
    assert not keep_yielding(me, body("robot_1", -1, 0.3, W, 0.5), 10.0, 11.0)       # passed: moving away
    assert not keep_yielding(me, body("robot_1", 2, 0.3, W, 0.0), 10.0, 11.0)        # stopped
    assert not keep_yielding(me, body("robot_1", 2, 0.3, W, 0.5, "WAITING_RESERVATION"), 10.0, 11.0)
    assert not keep_yielding(me, None, 10.0, 11.0)
