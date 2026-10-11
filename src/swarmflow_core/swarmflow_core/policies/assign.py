"""Assignment-only policy for v2 (improvements_v2 O-01 ... O-03): which robot does which order, and the ordered goal
list. No routes through the graph, no reservations: goal tasks only (``api.ROUTE_GOALS``).

Orders are planned by priority (descending), then release time, then id. For each order every available robot gets a
candidate goal list (shelf: pick points in nearest-next order from the robot, then the dock; pallet: pallet spot, then
the dock). Cost in metres = graph travel to the first goal + ``CONGESTION_M`` per other robot whose ``next_vertex`` is
that first goal or one of its neighbours. Lowest cost wins, ties by robot id. The dock in the goal list is the order's
dock; ``FleetCoreV2`` substitutes a queue spot when the dock is held.

A robot is available to this policy when it is ``IDLE`` and reports neither a task nor an order; the caller marks
busy robots by filling ``task_id`` / ``order_id``. All robots in the snapshot count for congestion.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

from .. import api
from ..api import Assignment, Decision, DecisionType, FleetSnapshot, PlanResult, RobotMode
from ..decisions import make_decision

CONGESTION_M = 5.0   #: cost per other robot heading to (or next to) a candidate's first goal


class AssignPolicy:
    def __init__(self, name: str = "assign"):
        self.name = name

    @staticmethod
    def robot_vertex(robot: api.RobotSnapshot, graph) -> str:
        return robot.last_vertex or graph.nearest_vertex(robot.x, robot.y)

    def plan(self, snapshot: FleetSnapshot, graph) -> PlanResult:
        cache: Dict[Tuple[str, str, str], float] = {}

        def dist(a: str, b: str, payload: str) -> float:
            key = (a, b, payload)
            if key not in cache:
                try:
                    cache[key] = graph.path_length(graph.shortest_path(a, b, payload))
                except (KeyError, ValueError):
                    cache[key] = math.inf
            return cache[key]

        robots = sorted(snapshot.robots, key=lambda r: r.robot_id)
        pool = {r.robot_id: r for r in robots
                if r.mode == RobotMode.IDLE and r.task_id == "" and r.order_id == ""}
        nexts: Dict[str, str] = {r.robot_id: r.next_vertex for r in robots}
        assignments: List[Assignment] = []
        decisions: List[Decision] = []
        orders = sorted(snapshot.open_orders, key=lambda o: (-o.priority, o.release_t, o.order_id))
        for order in orders:
            if not pool:
                break
            if order.order_type == api.ORDER_SHELF:
                picks = list(order.pick_vertices)
            elif order.order_type == api.ORDER_PALLET:
                picks = [order.pickup_vertex]
            else:
                continue
            if not picks:
                continue
            best: Optional[Tuple[float, str, List[str], float, float, float]] = None
            for rid in sorted(pool):
                cand = self._candidate(pool[rid], picks, order, graph, dist)
                if cand is None:
                    continue
                goals, travel, total = cand
                first = goals[0]
                near = {first, *graph.successors(first)}
                crowd = sum(1 for other, nv in nexts.items() if other != rid and nv and nv in near)
                congestion = CONGESTION_M * crowd
                key = (travel + congestion, rid)
                if best is None or key < (best[0], best[1]):
                    best = (travel + congestion, rid, goals, travel, congestion, total)
            if best is None:
                continue
            _, rid, goals, travel, congestion, total = best
            del pool[rid]
            nexts[rid] = goals[0]
            assignments.append(Assignment(rid, order.order_id, f"task_{order.order_id}", tuple(goals),
                                          api.ROUTE_GOALS))
            decisions.append(make_decision(
                t=snapshot.t, event_id=f"{snapshot.t:.3f}-{order.order_id}-ASSIGN", policy=self.name,
                decision_type=DecisionType.ASSIGN, robot_id=rid, order_id=order.order_id,
                trigger="order_released", previous_decision="",
                new_decision=f"{rid} -> {order.order_id} via {goals[0]}",
                cost_keys=("travel_to_first_goal_m", "congestion_m", "route_total_m"),
                cost_values=(travel, congestion, total)))
        return PlanResult(assignments, decisions)

    def _candidate(self, robot, picks: Sequence[str], order, graph, dist):
        """Goal list, travel to the first goal and total length; None if any goal is unreachable."""
        payload = order.payload_type
        cur = self.robot_vertex(robot, graph)
        todo = list(picks)
        goals: List[str] = []
        total = 0.0
        travel = 0.0
        while todo:
            nxt = min(todo, key=lambda p: (dist(cur, p, payload), p))
            d = dist(cur, nxt, payload)
            if math.isinf(d):
                return None
            if not goals:
                travel = d
            total += d
            goals.append(nxt)
            todo.remove(nxt)
            cur = nxt
        d = dist(cur, order.dropoff_vertex, payload)
        if math.isinf(d):
            return None
        total += d
        goals.append(order.dropoff_vertex)
        return goals, travel, total
