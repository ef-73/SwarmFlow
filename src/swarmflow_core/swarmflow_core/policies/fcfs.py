"""FCFS assignment policy (design §6.7, §13.5): orders oldest first, nearest available robot by route distance."""

from __future__ import annotations

from typing import List

from .. import api
from ..api import Assignment, Decision, DecisionType, FleetSnapshot, PlanResult, RobotMode
from ..decisions import make_decision


class FcfsPolicy:
    def __init__(self, name: str = api.POLICY_FCFS):
        self.name = name

    @staticmethod
    def robot_vertex(robot: api.RobotSnapshot, graph) -> str:
        return robot.last_vertex or graph.nearest_vertex(robot.x, robot.y)

    def plan(self, snapshot: FleetSnapshot, graph) -> PlanResult:
        pool = {r.robot_id: r for r in snapshot.robots if r.mode == RobotMode.IDLE and r.task_id == ""}
        assignments: List[Assignment] = []
        decisions: List[Decision] = []
        for order in sorted(snapshot.open_orders, key=lambda o: (o.release_t, o.order_id)):
            if not pool:
                break
            try:
                leg2 = list(graph.shortest_path(order.pickup_vertex, order.dropoff_vertex, order.payload_type))
            except (KeyError, ValueError):
                continue
            best = None
            for rid in sorted(pool):
                start = self.robot_vertex(pool[rid], graph)
                try:
                    leg1 = list(graph.shortest_path(start, order.pickup_vertex, order.payload_type))
                except (KeyError, ValueError):
                    continue
                dist = graph.path_length(leg1)
                if best is None or dist < best[0]:
                    best = (dist, rid, leg1)
            if best is None:
                continue
            dist, rid, leg1 = best
            route = leg1 + leg2[1:]
            del pool[rid]
            assignments.append(Assignment(rid, order.order_id, f"task_{order.order_id}", tuple(route)))
            decisions.append(make_decision(
                t=snapshot.t, event_id=f"{snapshot.t:.3f}-{order.order_id}-ASSIGN", policy=self.name,
                decision_type=DecisionType.ASSIGN, robot_id=rid, order_id=order.order_id,
                trigger="order_released", previous_decision="",
                new_decision=f"{rid} -> {order.order_id} via {order.pickup_vertex}",
                cost_keys=("route_to_pickup_m", "route_total_m"),
                cost_values=(dist, graph.path_length(route))))
        return PlanResult(assignments, decisions)
