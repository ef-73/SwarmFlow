"""T068: reserved trajectories: timing, space-time conflicts, priority, wait points."""

from __future__ import annotations

import pytest

from swarmflow_robot_agent.trajectory import (Reservation, corridor_clear, first_conflict, outranks, priority_key,
                                              time_parameterise, wait_index)


def line(x0, y0, x1, y1, t0, n=31, dt=0.5):
    """Points from (x0, y0) to (x1, y1), n samples dt apart from t0."""
    return tuple((t0 + k * dt, x0 + (x1 - x0) * k / (n - 1), y0 + (y1 - y0) * k / (n - 1)) for k in range(n))


def test_time_parameterise_samples_the_plan_at_the_nominal_speed():
    pts = time_parameterise([(0, 0), (2, 0), (2, 2)], 0.0, 0.0, 10.0, speed=0.5, dt=0.5)
    assert pts[0] == (10.0, 0.0, 0.0)
    assert pts[1] == pytest.approx((10.5, 0.25, 0.0)) and pts[-1] == pytest.approx((18.0, 2.0, 2.0))
    assert len(pts) == 17                                           # 4 m at 0.25 m per sample
    assert time_parameterise([], 1.0, 2.0, 5.0) == ((5.0, 1.0, 2.0),)
    long = time_parameterise([(0, 0), (100, 0)], 0.0, 0.0, 0.0, horizon=15.0)
    assert long[-1][0] == pytest.approx(15.0)                       # cut at the horizon


def test_time_parameterise_starts_from_the_plan_point_nearest_the_robot():
    pts = time_parameterise([(0, 0), (1, 0), (2, 0), (3, 0)], 2.0, 0.1, 0.0)
    assert all(x >= 2.0 - 1e-9 for _, x, _ in pts)                 # never back to the part already driven


def test_crossing_at_the_same_time_conflicts_and_at_different_times_does_not():
    a = line(0, 0, 6, 0, 0.0)                                       # east along y = 0, at (3, 0) at t = 7.5
    b = line(3, -3, 3, 3, 0.0)                                      # north along x = 3, at (3, 0) at t = 7.5
    assert first_conflict(a, b, 0.0) is not None
    late = line(3, -3, 3, 3, 10.0)                                  # the same crossing, 10 s later
    assert first_conflict(a, late, 0.0) is None


def test_parallel_paths_2m_apart_never_conflict():
    assert first_conflict(line(0, 0, 6, 0, 0.0), line(6, 2, 0, 2, 0.0), 0.0) is None


def test_head_on_conflicts_where_they_meet():
    a, b = line(0, 0, 6, 0, 0.0), line(6, 0.2, 0, 0.2, 0.0)
    c = first_conflict(a, b, 0.0)
    assert c is not None and 2.0 <= a[c][1] <= 3.0                  # before the middle (1 m to meet)


def test_a_standing_robot_conflicts_whenever_the_path_passes_it():
    standing = ((0.0, 4.0, 0.3),)
    c = first_conflict(line(0, 0, 6, 0, 0.0), standing, 0.0)
    assert c is not None and line(0, 0, 6, 0, 0.0)[c][1] == pytest.approx(3.2)


def test_conflicts_beyond_the_window_are_ignored():
    assert first_conflict(line(0, 0, 6, 0, 0.0, n=61), ((0.0, 5.9, 0.0),), 0.0, within=5.0) is None


def test_priority_pallet_first_then_older_reservation_then_lower_id():
    p = Reservation("robot_3", 50.0, True, ())
    old = Reservation("robot_2", 10.0, False, ())
    new = Reservation("robot_1", 20.0, False, ())
    tie = Reservation("robot_4", 10.0, False, ())
    assert sorted([new, tie, old, p], key=priority_key) == [p, old, tie, new]
    assert outranks(old, new) and not outranks(new, old) and outranks(old, tie)


def test_wait_index_is_the_last_point_clear_of_the_other_and_outside_aisles():
    mine = line(0, 0, 6, 0, 0.0, n=25)                              # 0.25 m spacing
    theirs = line(6, 0, 3.5, 0, 0.0, n=11)                          # comes down to x = 3.5
    c = first_conflict(mine, theirs, 0.0)
    w = wait_index(mine, c, theirs)
    assert w is not None and mine[w][1] <= 3.5 - 1.3 + 1e-9         # >= 1.3 m short of its reservation
    assert mine[w + 1][1] > 3.5 - 1.3                               # and the last such point
    in_aisle = lambda x, y: x > 1.0                                 # noqa: E731
    w2 = wait_index(mine, c, theirs, in_aisle)
    assert mine[w2][1] <= 1.0                                       # waits before the aisle


def test_no_wait_point_when_already_in_the_others_way():
    mine = line(3, 0, 6, 0, 0.0, n=13)
    theirs = line(6, 0, 2, 0, 0.0, n=17)                            # its reservation covers where I stand
    c = first_conflict(mine, theirs, 0.0)
    assert c is not None and wait_index(mine, c, theirs) is None


def test_corridor_clear_once_the_other_has_passed():
    corridor = [(x * 0.5, 0.0) for x in range(8)]
    assert not corridor_clear(corridor, ((0.0, 2.0, 0.5),))
    assert corridor_clear(corridor, ((0.0, 2.0, 3.0), (1.0, 2.0, 5.0)))
