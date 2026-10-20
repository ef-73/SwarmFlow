"""T072: coordination highlights: COORD events -> moments -> the ones to show."""

import importlib.util
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("highlights", HERE / "highlights.py")
hl = importlib.util.module_from_spec(spec)
sys.modules["highlights"] = hl
spec.loader.exec_module(hl)

LOG = """\
robot_2-1 | [agent] [INFO] [1.0] [robot_2.robot_agent]: COORD {'type': 'yield_start', 't': 100.0, 'robot_id': 'robot_2', 'other': 'robot_4', 'conflict_in_s': 7.0, 'wait_x': 1.0, 'wait_y': 2.0}
robot_2-1 | [agent] [INFO] [1.0] [robot_2.robot_agent]: COORD {'type': 'yield_end', 't': 109.5, 'robot_id': 'robot_2', 'other': 'robot_4', 'waited_s': 9.5, 'timed_out': False}
robot_1-1 | noise COORD {not a dict
robot_3-1 | [agent] [INFO] [1.0] [robot_3.robot_agent]: COORD {'type': 'yield_start', 't': 50.0, 'robot_id': 'robot_3', 'other': 'robot_5', 'conflict_in_s': 8.0}
robot_3-1 | [agent] [INFO] [1.0] [robot_3.robot_agent]: COORD {'type': 'yield_end', 't': 70.0, 'robot_id': 'robot_3', 'other': 'robot_5', 'waited_s': 20.0, 'timed_out': True}
robot_4-1 | [agent] [INFO] [1.0] [robot_4.robot_agent]: COORD {'type': 'deadlock_cycle', 't': 200.0, 'robot_id': 'robot_4', 'cycle': ['robot_2', 'robot_4'], 'rank': 0}
robot_4-1 | [agent] [INFO] [1.0] [robot_4.robot_agent]: COORD {'type': 'unjam_start', 't': 200.0, 'robot_id': 'robot_4', 'to_x': 13.1, 'to_y': 16.8}
robot_4-1 | [agent] [INFO] [1.0] [robot_4.robot_agent]: COORD {'type': 'unjam_end', 't': 207.0, 'robot_id': 'robot_4', 'backed_up': True}
"""


def test_events_parse_and_sort():
    ev = hl.coord_events(LOG)
    assert [e["type"] for e in ev][:2] == ["yield_start", "yield_end"] and ev[0]["t"] == 50.0
    assert len(ev) == 7


def test_moments_pair_starts_and_ends_and_drop_timeouts():
    ms = hl.moments(hl.coord_events(LOG))
    assert [(m.kind, m.actor, m.others, m.t0, m.t1) for m in ms] == [
        ("yield", "robot_2", ("robot_4",), 100.0, 109.5), ("unjam", "robot_4", ("robot_2",), 200.0, 207.0)]
    assert ms[0].detail["waited_s"] == 9.5


def test_pick_prefers_deadlock_breaks_respects_visibility_wait_and_gap():
    Y = lambda t, w: hl.Moment("yield", t, t + w, "a", ("b",), {"waited_s": w, "conflict_in_s": 6.0})  # noqa: E731
    U = hl.Moment("unjam", 300.0, 307.0, "c", ("d",), {})
    ms = [Y(10, 9), Y(20, 12), Y(100, 2), Y(200, 30), Y(400, 6), U]
    got = hl.pick(ms, lambda m: True)
    assert U in got and Y(100, 2) not in got and Y(200, 30) not in got     # too short / too long
    assert not (Y(10, 9) in got and Y(20, 12) in got)                        # within the gap: only one
    assert [m.t0 for m in got] == sorted(m.t0 for m in got)
    assert hl.pick(ms, lambda m: m.t0 != 300.0 and m.t0 != 20) == [Y(10, 9), Y(400, 6)]


def test_captions_name_the_robots():
    head, sub = hl.caption(hl.Moment("yield", 0, 9, "robot_2", ("robot_4",), {"waited_s": 9.4, "conflict_in_s": 7}))
    assert head == "robot_2 waits for robot_4" and "9 s" in sub
    head, _ = hl.caption(hl.Moment("unjam", 0, 7, "robot_4", ("robot_2",), {}))
    assert "Deadlock" in head and "robot_4" in head
