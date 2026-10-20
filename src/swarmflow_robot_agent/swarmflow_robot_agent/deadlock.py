"""Deadlock breaker (T068): a wait-for graph over the whole fleet, any number of robots. Pure Python, time an argument.

Every robot builds the same graph from what all robots publish (``/fleet/robot_states``, ``/fleet/trajectories``):
each robot waits for at most one other robot, so the graph has out-degree <= 1 and every cycle is found by following
the edges (``cycle_of``), whether it has 2, 3 or 5 members.

- A robot yielding (mode WAITING_RESERVATION, fault YIELD:<b>) waits for b.
- A blocked robot (STUCK + BLOCKED) waits for the nearest robot within ``FRONT_M`` in front of it (within
  ``FRONT_HALF_ANGLE`` of its heading) or on the first ``AHEAD_M`` of its reservation.

A cycle that lasts ``HOLD_S`` is broken by one member at a time, lowest priority first (``Breaker``): member k acts
``HOLD_S + STEP_S * k`` after the cycle appeared, if the cycle is still there. A yielding member stops yielding; a
blocked member backs up (Nav2's own collision-checked BackUp behaviour) and re-plans. Each member acts once per cycle;
if all have acted, the fleet's own blocked-requeue takes over as before. Every robot ranks the members the same way
(priority_key), so they never act at once.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, FrozenSet, Iterable, Mapping, Optional, Sequence, Tuple

FRONT_M = 1.5                   #: a blocked robot waits for a robot this close in front of it ...
FRONT_HALF_ANGLE = math.radians(60)
AHEAD_M = 2.0                   #: ... or on this much of its own reservation
ON_PATH_M = 1.0
HOLD_S = 8.0                    #: a cycle must last this long before anyone acts
STEP_S = 10.0                   #: then one member after another, this far apart
BLOCKED = "BLOCKED"
YIELD_PREFIX = "YIELD:"


@dataclass(frozen=True)
class Node:
    robot_id: str
    x: float
    y: float
    yaw: float
    mode: str
    fault_reason: str
    key: Tuple                                  #: priority_key of its reservation (smaller = higher priority)
    ahead: Tuple[Tuple[float, float], ...] = ()  #: first metres of its reservation (x, y)


def waits_for(n: Node, nodes: Mapping[str, Node]) -> Optional[str]:
    """The robot ``n`` waits for, or None."""
    if n.mode == "WAITING_RESERVATION" and n.fault_reason.startswith(YIELD_PREFIX):
        other = n.fault_reason[len(YIELD_PREFIX):]
        return other if other in nodes and other != n.robot_id else None
    if n.mode != "STUCK" or n.fault_reason != BLOCKED:
        return None
    best, best_d = None, math.inf
    for o in nodes.values():
        if o.robot_id == n.robot_id:
            continue
        d = math.hypot(o.x - n.x, o.y - n.y)
        if d > FRONT_M:
            continue
        bearing = abs(math.atan2(math.sin(math.atan2(o.y - n.y, o.x - n.x) - n.yaw),
                                 math.cos(math.atan2(o.y - n.y, o.x - n.x) - n.yaw)))
        on_path = any(math.hypot(o.x - ax, o.y - ay) < ON_PATH_M for ax, ay in n.ahead)
        if (bearing <= FRONT_HALF_ANGLE or on_path) and (d, o.robot_id) < (best_d, best or ""):
            best, best_d = o.robot_id, d
    return best


def cycle_of(robot_id: str, nodes: Mapping[str, Node]) -> Optional[FrozenSet[str]]:
    """The members of the wait-for cycle ``robot_id`` is on, or None (also None for a robot on a chain that leads
    into a cycle without being part of it: it is freed when the cycle is)."""
    edge = {rid: waits_for(n, nodes) for rid, n in nodes.items()}
    seen = []
    cur = robot_id
    while cur is not None and cur not in seen:
        seen.append(cur)
        cur = edge.get(cur)
    if cur is None or cur not in seen:
        return None
    members = frozenset(seen[seen.index(cur):])
    return members if robot_id in members else None


def rank_in(robot_id: str, members: Iterable[str], nodes: Mapping[str, Node]) -> int:
    """0 = the member that acts first: the lowest priority (largest key)."""
    order = sorted(members, key=lambda r: nodes[r].key, reverse=True)
    return order.index(robot_id)


class Breaker:
    """Per robot: when to act on the cycle this robot is on."""

    def __init__(self, robot_id: str):
        self.robot_id = robot_id
        self._cycle: Optional[FrozenSet[str]] = None
        self._since = 0.0
        self._acted = False

    def update(self, nodes: Mapping[str, Node], t: float) -> Optional[Tuple[FrozenSet[str], int]]:
        """(cycle, rank) when this robot should act now, else None. Acts at most once per cycle."""
        cyc = cycle_of(self.robot_id, nodes) if self.robot_id in nodes else None
        if cyc != self._cycle:
            self._cycle, self._since, self._acted = cyc, t, False
        if cyc is None or self._acted:
            return None
        k = rank_in(self.robot_id, cyc, nodes)
        if t - self._since >= HOLD_S + STEP_S * k:
            self._acted = True
            return cyc, k
        return None


def nodes_from(states: Iterable[Mapping], keys: Mapping[str, Tuple], ahead: Mapping[str, Sequence]) -> Dict[str, Node]:
    """Graph nodes from robot state dicts (robot_id, x, y, yaw, mode, fault_reason)."""
    out = {}
    for s in states:
        rid = s["robot_id"]
        out[rid] = Node(rid, float(s["x"]), float(s["y"]), float(s["yaw"]), str(s["mode"]),
                        str(s.get("fault_reason", "")), tuple(keys.get(rid, (1, math.inf, rid))),
                        tuple(tuple(p) for p in ahead.get(rid, ())))
    return out
