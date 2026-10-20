"""T068: deadlock breaker on the fleet's wait-for graph (any number of robots)."""

from __future__ import annotations

import math

from swarmflow_robot_agent.deadlock import HOLD_S, STEP_S, Breaker, Node, cycle_of, rank_in, waits_for

E, W, N, S = 0.0, math.pi, math.pi / 2, -math.pi / 2


def node(rid, x, y, yaw, mode="STUCK", fault="BLOCKED", key_t=0.0, ahead=()):
    return Node(rid, x, y, yaw, mode, fault, (1, key_t, rid), tuple(ahead))


def graph(*ns):
    return {n.robot_id: n for n in ns}


def triangle(ta=0.0, tb=0.0, tc=0.0):
    """Equilateral, side 1.2 m; each robot faces the next one, turned 10 degrees away from the third."""
    d = math.radians
    return graph(node("a", 0.0, 0.0, d(-10), key_t=ta), node("b", 1.2, 0.0, d(110), key_t=tb),
                 node("c", 0.6, 1.2 * math.sqrt(3) / 2, d(230), key_t=tc))


def head_on(t1=10.0, t2=20.0):
    return graph(node("robot_1", 0.0, 0.0, E, key_t=t1), node("robot_2", 0.8, 0.0, W, key_t=t2))


def test_blocked_robot_waits_for_the_nearest_robot_in_front_only():
    g = graph(node("a", 0, 0, E), node("b", 1.2, 0.2, W, mode="NAVIGATING", fault=""),
              node("c", -1.0, 0, E, mode="NAVIGATING", fault=""))              # c is behind a
    assert waits_for(g["a"], g) == "b"
    g2 = graph(node("a", 0, 0, E), node("b", 0, 1.0, W, mode="NAVIGATING", fault=""))   # to the side, not ahead
    assert waits_for(g2["a"], g2) is None
    g3 = graph(node("a", 0, 0, E, ahead=[(0.5, 0.5), (0.8, 1.0)]), node("b", 0.6, 1.1, W, mode="IDLE", fault=""))
    assert waits_for(g3["a"], g3) == "b"                                        # on its reserved path
    assert waits_for(node("x", 0, 0, E, mode="NAVIGATING", fault=""), g) is None


def test_two_and_three_robot_cycles_are_found_a_chain_is_not():
    g = head_on()
    assert cycle_of("robot_1", g) == frozenset({"robot_1", "robot_2"})
    tri = triangle()
    # three robots 1.2 m apart, each facing the next: a -> b -> c -> a
    assert waits_for(tri["a"], tri) == "b" and waits_for(tri["b"], tri) == "c" and waits_for(tri["c"], tri) == "a"
    assert cycle_of("b", tri) == frozenset({"a", "b", "c"})
    chain = graph(node("a", 0, 0, E), node("b", 1.0, 0, E), node("c", 2.0, 0, E, mode="NAVIGATING", fault=""))
    assert cycle_of("a", chain) is None and cycle_of("b", chain) is None


def test_a_yield_edge_and_a_blocked_edge_make_a_cycle():
    g = graph(node("robot_1", 0, 0, E, mode="WAITING_RESERVATION", fault="YIELD:robot_2"),
              node("robot_2", 1.0, 0.0, W))
    assert cycle_of("robot_1", g) == frozenset({"robot_1", "robot_2"})


def test_a_robot_leading_into_a_cycle_is_not_on_it():
    g = head_on()
    g["robot_3"] = node("robot_3", -1.0, 0.0, E, mode="WAITING_RESERVATION", fault="YIELD:robot_1")
    assert cycle_of("robot_3", g) is None and cycle_of("robot_1", g) == frozenset({"robot_1", "robot_2"})


def test_lowest_priority_member_acts_first_and_only_after_the_hold_time():
    g = head_on(t1=10.0, t2=20.0)                                  # robot_2 reserved later: lowest priority
    assert rank_in("robot_2", {"robot_1", "robot_2"}, g) == 0
    b1, b2 = Breaker("robot_1"), Breaker("robot_2")
    assert b1.update(g, 100.0) is None and b2.update(g, 100.0) is None
    assert b2.update(g, 100.0 + HOLD_S - 0.1) is None
    act = b2.update(g, 100.0 + HOLD_S)
    assert act == (frozenset({"robot_1", "robot_2"}), 0)
    assert b2.update(g, 100.0 + HOLD_S + 1.0) is None               # acts once per cycle
    assert b1.update(g, 100.0 + HOLD_S) is None                     # not robot_1's turn yet
    assert b1.update(g, 100.0 + HOLD_S + STEP_S) is not None         # next member if the cycle is still there


def test_a_vanished_cycle_resets_and_a_new_cycle_starts_over():
    g = head_on()
    b = Breaker("robot_2")
    b.update(g, 0.0)
    clear = graph(node("robot_1", 0, 0, E, mode="NAVIGATING", fault=""), node("robot_2", 3, 0, W))
    assert b.update(clear, 5.0) is None
    assert b.update(g, 6.0) is None and b.update(g, 6.0 + HOLD_S) is not None   # timer restarted at 6.0


def test_every_robot_ranks_the_cycle_the_same_way():
    g = triangle(5.0, 1.0, 3.0)
    ranks = {r: rank_in(r, {"a", "b", "c"}, g) for r in "abc"}
    assert ranks == {"a": 0, "c": 1, "b": 2}                         # latest reservation acts first
