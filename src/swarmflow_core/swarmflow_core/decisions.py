"""Decision rendering (design §10): the only producer of ``Decision.explanation``.

Templates use only the decision's own fields (robot, order, costs, trigger, previous/new decision), never free text.
"""

from __future__ import annotations

import dataclasses
from typing import Optional

from . import api
from .api import Decision, DecisionType


def _cost(d: Decision, key: str) -> Optional[float]:
    for k, v in zip(d.cost_keys, d.cost_values):
        if k == key:
            return v
    return None


def _costs_text(d: Decision) -> str:
    if not d.cost_keys:
        return ""
    return " [" + ", ".join(f"{k}={v:.1f}" for k, v in zip(d.cost_keys, d.cost_values)) + "]"


def render(decision: Decision) -> str:
    d = decision
    t = d.decision_type
    robot = d.robot_id or "no robot"
    order = d.order_id or "no order"
    if t == DecisionType.ASSIGN:
        to_pickup, total = _cost(d, "route_to_pickup_m"), _cost(d, "route_total_m")
        if to_pickup is not None and total is not None:
            return (f"{robot} assigned order {order}: nearest available robot, {to_pickup:.1f} m to pickup "
                    f"(route {total:.1f} m).")
        return f"{robot} assigned order {order}: nearest available robot{_costs_text(d)}."
    if t == DecisionType.REASSIGN:
        return (f"Order {order} returned to the queue after {robot} could not complete it ({d.trigger}); "
                f"it will be re-planned{_costs_text(d)}.")
    if t == DecisionType.REROUTE:
        return (f"{robot} rerouted for order {order}: {d.previous_decision} replaced by {d.new_decision} "
                f"({d.trigger}){_costs_text(d)}.")
    if t == DecisionType.PRIORITY:
        return (f"Order {order} priority changed: {d.previous_decision} -> {d.new_decision} "
                f"({d.trigger}){_costs_text(d)}.")
    if t == DecisionType.RESERVATION_GRANT:
        return f"{robot} granted {d.new_decision} for {d.previous_decision} (trigger {d.trigger}){_costs_text(d)}."
    if t == DecisionType.RESERVATION_DENY:
        retry = _cost(d, "retry_after_s")
        tail = f"; retry in {retry:.1f} s" if retry is not None else ""
        return f"{robot} denied for {d.previous_decision}: {d.new_decision} (trigger {d.trigger}){tail}."
    if t == DecisionType.CONGESTION_RESPONSE:
        return (f"Congestion response for {robot} / {order}: {d.previous_decision} -> {d.new_decision} "
                f"({d.trigger}); predicted improvement {d.predicted_improvement_s:.1f} s{_costs_text(d)}.")
    if t == DecisionType.DISRUPTION_RESPONSE:
        return (f"Disruption response for {robot} / {order}: {d.previous_decision} -> {d.new_decision} "
                f"({d.trigger}); predicted improvement {d.predicted_improvement_s:.1f} s{_costs_text(d)}.")
    if t == DecisionType.STUCK_FAIL:
        return f"Order {order} failed on {robot}: {d.new_decision} (trigger {d.trigger}){_costs_text(d)}."
    return f"{t.value} for {robot} / {order}: {d.previous_decision} -> {d.new_decision} ({d.trigger})."


def make_decision(**fields) -> Decision:
    """Build a :class:`Decision` with ``explanation`` rendered from the other fields."""
    fields.pop("explanation", None)
    d = Decision(**fields)
    return dataclasses.replace(d, explanation=render(d))


def reservation_decision(t: float, policy: str, req: api.ReservationRequest,
                         dec: api.ReservationDecision) -> Decision:
    base = dict(t=t, policy=policy, robot_id=req.robot_id, trigger=f"request:{req.request_id}",
                previous_decision=f"zone {req.zone_id} via {req.entry_vertex}")
    if dec.granted:
        return make_decision(event_id=f"{t:.3f}-{req.request_id}-GRANT", decision_type=DecisionType.RESERVATION_GRANT,
                             new_decision=dec.lease_id, **base)
    return make_decision(event_id=f"{t:.3f}-{req.request_id}-DENY", decision_type=DecisionType.RESERVATION_DENY,
                         new_decision=dec.reason, cost_keys=("retry_after_s",), cost_values=(dec.retry_after_s,),
                         **base)
