"""Coordination highlights from a run (T072): the moments robots yielded to each other or broke a deadlock, read from
the robot agents' COORD log lines, and the choice of which to show. Pure Python (no OpenCV, no ROS); unit-tested.

make_highlights.py turns the chosen moments into a captioned clip reel from the recorded cameras (or a top-down
replay where no camera saw them). Every clip is real footage of the run; nothing is staged or re-timed except the
playback speed shown on screen.
"""
from __future__ import annotations

import ast
import re
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

_COORD = re.compile(r"COORD (\{.*?\})\s*$", re.MULTILINE)


def coord_events(log_text: str) -> List[dict]:
    """Every ``COORD {...}`` event the agents logged (yield_start/end, deadlock_cycle, unjam_start/end/skip), sorted
    by time. Lines that do not parse are skipped."""
    out = []
    for m in _COORD.finditer(log_text):
        try:
            e = ast.literal_eval(m.group(1))
        except (ValueError, SyntaxError):
            continue
        if isinstance(e, dict) and "type" in e and "t" in e:
            out.append(e)
    return sorted(out, key=lambda e: float(e["t"]))


@dataclass(frozen=True)
class Moment:
    kind: str                       #: "yield" or "unjam"
    t0: float                       #: start (yield_start / deadlock_cycle)
    t1: float                       #: end (yield_end / unjam_end)
    actor: str                      #: the robot that waited or backed out
    others: Tuple[str, ...]         #: the robot(s) it gave way to
    detail: Dict[str, float]        #: waited_s, conflict_in_s, ...


def moments(events: Iterable[dict]) -> List[Moment]:
    """Yields (start..end per robot) and deadlock breaks (cycle -> unjam_end of the robot that backed out)."""
    out: List[Moment] = []
    open_yield: Dict[str, dict] = {}
    cycle: Dict[str, dict] = {}
    for e in events:
        rid, kind, t = e.get("robot_id", ""), e["type"], float(e["t"])
        if kind == "yield_start":
            open_yield[rid] = e
        elif kind == "yield_end" and rid in open_yield:
            s = open_yield.pop(rid)
            if e.get("waited_s") is not None and not e.get("timed_out", False):
                out.append(Moment("yield", float(s["t"]), t, rid, (s["other"],),
                                  {"waited_s": float(e["waited_s"]), "conflict_in_s": float(s.get("conflict_in_s", 0))}))
        elif kind == "deadlock_cycle":
            cycle[rid] = e
        elif kind == "unjam_end" and rid in cycle and e.get("backed_up"):
            c = cycle.pop(rid)
            others = tuple(r for r in c.get("cycle", ()) if r != rid)
            out.append(Moment("unjam", float(c["t"]), t, rid, others, {}))
    return sorted(out, key=lambda m: m.t0)


def pick(ms: Sequence[Moment], visible: Callable[[Moment], bool], n_yield: int = 3, n_unjam: int = 2,
         min_wait: float = 4.0, max_wait: float = 15.0, gap: float = 20.0) -> List[Moment]:
    """The moments to show, in time order: deadlock breaks first priority, then yields that a viewer can follow
    (waited ``min_wait``..``max_wait`` s), only those ``visible`` (inside a camera's view), none closer than ``gap`` s
    to an already chosen one."""
    chosen: List[Moment] = []

    def free(m: Moment) -> bool:
        return all(m.t1 + gap <= c.t0 or c.t1 + gap <= m.t0 for c in chosen)

    for m in ms:
        if m.kind == "unjam" and len([c for c in chosen if c.kind == "unjam"]) < n_unjam and visible(m) and free(m):
            chosen.append(m)
    ys = sorted((m for m in ms if m.kind == "yield" and min_wait <= m.detail["waited_s"] <= max_wait),
                key=lambda m: -m.detail["waited_s"])
    for m in ys:
        if len([c for c in chosen if c.kind == "yield"]) >= n_yield:
            break
        if visible(m) and free(m):
            chosen.append(m)
    return sorted(chosen, key=lambda m: m.t0)


def caption(m: Moment) -> Tuple[str, str]:
    """(headline, subline) for a moment."""
    others = " and ".join(m.others)
    if m.kind == "yield":
        return (f"{m.actor} waits for {others}",
                f"their planned paths meet in {m.detail['conflict_in_s']:.0f} s; {others} reserved first, so "
                f"{m.actor} stops clear of its path, then goes ({m.detail['waited_s']:.0f} s)")
    return (f"Deadlock: {m.actor} and {others} block each other",
            f"after 8 s the lower-priority robot, {m.actor}, backs out along its own track, and both drive on")
