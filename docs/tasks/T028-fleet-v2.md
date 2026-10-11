---
id: T028
title: Assignment-only fleet core v2 (goal tasks, shelf/pallet orders, dock queues, blocked handling, home slots)
workstream: B
status: review
claimed_by: "sf-implementer"
branch: ws-b/T028-fleet-v2
model: sonnet
risk: safety-critical
---

## Goal
A pure-Python fleet core for v2 that only **assigns**: which robot does which order and the ordered goal list
(improvements_v2 §2, O-01 … O-05, O-08, P-01). It follows the v2 lifecycle in the `swarmflow_core.api` module
docstring exactly. The v1 `FleetCore` stays untouched (it is the reservations baseline).

## Design (lead)
- `swarmflow_core/fleet_v2.py`: `FleetCoreV2(graph, policy, queue_spots=None)` with the same surface as
  `FleetCore`: `add_order`, `update_robot`, `task_result(task_id, success, failure_reason, t)`, `tick(t) ->
  TickOutput` (reuse `fleet.TickOutput`), `order_state`, `orders`. Every `Assignment` it emits has
  `route_kind = api.ROUTE_GOALS`. `queue_spots` maps queue vertex → (dock, queue_index); when None, derive it from
  the graph's `queue` vertices, whose names are `Q_<dock>_<index>` (layout semantic check guarantees it).
- `swarmflow_core/policies/assign.py`: `AssignPolicy(name="assign")` implementing `api.Policy`. Orders by
  priority (desc), then release time, then id. For each order, every available robot gets a candidate goal list:
  shelf → pick points in nearest-next order from the robot (graph distance, ties by name), then the dock; pallet →
  [pallet spot, dock]. Cost (metres) = travel to the first goal + congestion; congestion = `CONGESTION_M` (e.g.
  5.0) × number of other robots whose `next_vertex` is the candidate's first goal or one of its graph neighbours.
  Lowest cost wins, ties by robot id. The ASSIGN decision carries cost keys `travel_to_first_goal_m`,
  `congestion_m`, `route_total_m` (rendered by `decisions.render`; add a template branch if needed in
  `decisions.py`).
- Dock occupancy (O-05): a dock is held by the one task whose last goal is that dock, from dispatch until its
  result. A new route for a held dock ends at the lowest free queue spot instead; when a dock frees, its queue
  robots each move up one spot (index 1 → the dock, k → k−1): if a queue robot's task is still running it is
  cancelled (`TickOutput.cancels`) and a follow-up task is dispatched; the `CANCELLED` result of a task the core
  cancelled itself changes nothing. A task that ends at a queue spot never delivers.
- Order states forward-only; transitions from robot snapshots (task/order ids must match): shelf → PICKING_UP at
  the first pick (`last_vertex`), IN_TRANSIT at the last pick (both in one tick for one pick); pallet → LIFTING
  while ASSIGNED gives PICKING_UP, CARRYING gives IN_TRANSIT; DELIVERED on success of a task whose last goal is the
  order's dock.
- Failures: non-cancel failure before pickup → requeue (REASSIGN decision) and send the robot home; after pickup →
  follow-up task to the remaining goals, counted as a retry. Blocked (`STUCK` + `FAULT_BLOCKED`): timer from the
  first report, reset when the mode leaves STUCK; after `BLOCKED_REQUEUE_S`: cancel, then requeue + home (nothing
  on board) or follow-up with the remaining goals (`MAX_LOADED_RETRIES`, then FAILED `STUCK_AFTER_PICKUP` + home).
  No v1 stuck-recovery park tasks for blocked robots.
- Home rule: an available robot not on a `home` vertex gets a goal task to the nearest free home slot (free = no
  robot there and not the target of another home task); robots are handled in id order.

## Owned files
- src/swarmflow_core/swarmflow_core/fleet_v2.py
- src/swarmflow_core/swarmflow_core/policies/assign.py
- src/swarmflow_core/swarmflow_core/decisions.py
- src/swarmflow_core/test/test_fleet_v2_extra.py

## Lead tests (do not edit)
- src/swarmflow_core/test/test_fleet_v2.py

## Forbidden
- AGENTS.md §4; no Gazebo; no ROS imports in swarmflow_core; do not change `fleet.py`, `api.py` or other modules.

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t028
docker compose run --rm --no-deps dev python3 -m pytest -q src/swarmflow_core
```

## Definition of done
- Lead tests and all existing `src/swarmflow_core` tests pass; your extra tests cover at least: two docks with
  queues at once, a home slot freed and re-used, an order whose pick is unreachable (requeued, robot home).
- `scripts/ci.sh` green on the branch rebased onto `main`; reviewer pass (safety-critical card).

## Inputs / contracts used
- `src/swarmflow_core/swarmflow_core/api.py` (v2 docstring, constants), `fleet.py` (v1 pattern), `graph.py`,
  `policies/fcfs.py`, `decisions.py`; docs/improvements_v2.md §2.
