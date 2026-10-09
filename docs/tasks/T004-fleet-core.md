---
id: T004
title: Fleet core — FCFS policy, order lifecycle, decisions, fake backend
workstream: B
status: open
claimed_by: "sf-implementer"
branch: ws-b/T004-fleet-core
model: sonnet
risk: normal
---

## Goal
The orchestrator's pure-Python brain: `FcfsPolicy`, `FleetCore` (order book + policy + stuck/park rules),
`decisions.render`, and an in-memory `FakeBackend` that runs 10 orders × 3 robots end to end with the real
reservation authority (WS-B acceptance, design §14.2).

## Owned files
- src/swarmflow_core/swarmflow_core/policies/**
- src/swarmflow_core/swarmflow_core/fleet.py
- src/swarmflow_core/swarmflow_core/decisions.py
- src/swarmflow_core/swarmflow_core/backends/**
- src/swarmflow_core/test/test_fleet_extra.py

## Lead tests (do not edit)
- src/swarmflow_core/test/test_fleet.py

## Forbidden
- AGENTS.md §4; no ROS imports; no wall clock; no randomness except a seeded `random.Random` if you need one.
- Do not edit `api.py`, `graph.py`, `reservations.py`, fixtures.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-t004
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider src/swarmflow_core/test
```

## Specification
Read the `api.py` module docstring (implementation map, **order lifecycle**, move-only tasks) — it is binding.

**`policies/fcfs.py` — `FcfsPolicy(name=api.POLICY_FCFS)`** (`policies/__init__.py` may be empty).
- `plan(snapshot, graph)`: open orders sorted by `(release_t, order_id)`; for each, the **available** robot with the
  smallest route distance from its vertex to the pickup (ties → `robot_id`). Robot vertex = `last_vertex`, or
  `graph.nearest_vertex(x, y)` if empty. Available = mode IDLE and `task_id == ""` (the `FleetCore` additionally
  marks robots on preemptible park tasks as available by passing them with mode IDLE and empty task id).
- Route = `shortest_path(robot_vertex, pickup) + shortest_path(pickup, dropoff)[1:]` (payload type of the order).
- `task_id` = `f"task_{order_id}"` (FleetCore may suffix `_<n>` on reassignment — see below; the policy itself is
  stateless and deterministic).
- One `Decision` per assignment: `DecisionType.ASSIGN`, `policy=self.name`, `trigger="order_released"`,
  `previous_decision=""`, `new_decision=f"{robot_id} -> {order_id} via {pickup}"`, `cost_keys=("route_to_pickup_m",
  "route_total_m")` with values, `event_id=f"{snapshot.t:.3f}-{order_id}-ASSIGN"`, `explanation=render(decision)`.

**`decisions.py`** — `render(decision) -> str`: one template per `DecisionType` using only the decision's fields
(robot, order, costs, trigger, previous/new decision). Never free text. E.g. ASSIGN: "robot_2 assigned order o003:
nearest available robot, 7.4 m to pickup (route 24.9 m)." Also `make_decision(**fields)` that fills `explanation`,
and `reservation_decision(t, policy, req: ReservationRequest, dec: ReservationDecision) -> Decision`
(RESERVATION_GRANT / RESERVATION_DENY, trigger `f"request:{req.request_id}"`, new_decision lease id or deny reason).

**`fleet.py` — `FleetCore(graph, policy, authority, traffic_control=True)`**
- `add_order(order)`, `update_robot(snapshot)` (latest per robot), `task_result(task_id, success, failure_reason, t)`
  (or a `api.TaskResult` via `task_result_obj`), `tick(t) -> TickOutput`, `order_state(order_id) -> OrderState | None`
  (None before release), `orders() -> dict`.
- `TickOutput` dataclass: `dispatches: list[Assignment]`, `cancels: list[str]` (task ids),
  `status_changes: list[api.OrderStatusChange]`, `decisions: list[Decision]`.
- Each tick, in this order: (1) release orders with `release_t <= t` → QUEUED; (2) apply queued task results and
  robot snapshot transitions (lifecycle rules in api.py); (3) failures: `STUCK_TIMEOUT` → FAILED + `STUCK_FAIL`
  decision; other failure not picked up → QUEUED again + `REASSIGN` decision (the order can be re-planned in the
  same tick, its new task id gets a `_<n>` suffix); picked up → FAILED (`STUCK_AFTER_PICKUP`) + `STUCK_FAIL`;
  the failed robot gets a park task; (4) plan with the policy (snapshot of available robots + QUEUED orders) →
  ASSIGNED + dispatches + decisions; (5) **park rule**: every available robot that is not on a park vertex and got
  no order gets a move-only task (`order_id ""`, `task_id f"park_{robot_id}_{n}"`) to the nearest **free** park
  vertex (not the last vertex of another robot, not the target of another park task; ties by name).
- A robot executing a park task is **available**: if the policy assigns it an order, the tick emits
  `cancels=[park task id]` and the new dispatch together. A robot is never sent a park task twice for the same
  stay; a robot already standing on a park vertex is left alone.
- A robot whose snapshot mode is STUCK / FAULT, or which has an outstanding (non-park) task, is not available.
- `traffic_control=False` (Baseline A) changes nothing in assignment; it is recorded for the adapter.
- Status changes are emitted exactly once per transition, in order; deterministic for the same inputs.

**`backends/fake.py` — `FakeBackend(graph, spawn: dict[robot_id, vertex], authority=None, speed_mps=0.5)`**
implements `api.Backend` (`snapshot`, `dispatch`, `cancel`, `poll_results`) plus `step(t)`. Each robot follows its
route vertex by vertex at `speed_mps` (straight lines, no collisions), dwells `api.LOAD_DWELL_S` in LOADING at the
pickup and UNLOADING at the dropoff (move-only tasks: no dwell), and reports IDLE at the end. With an `authority`:
at a hold vertex before a zone entry the robot requests a lease (WAITING_RESERVATION while denied, retry every
`RETRY_AFTER_S`), heartbeats its leases every `HEARTBEAT_PERIOD_S`, and releases once ≥ `RELEASE_MARGIN_M` outside
the zone polygon. `last_vertex` = last vertex reached.

`run_fake(core, backend, orders, until_t, dt=0.1) -> FakeRunResult` with fields `order_ids` (all orders added),
`decisions` (all, incl. reservation decisions), `max_robots_in_zone` (max over samples of robots whose pose is inside
one zone polygon), `finish_t` (time all orders reached DELIVERED/FAILED, else `until_t`). The loop: `authority.recover`
with the initial snapshots at t=0; every step: `backend.step(t)`, feed every robot snapshot to `core.update_robot`
and `authority.observe_robot`, feed `poll_results()` to `core.task_result`, `out = core.tick(t)`, apply cancels then
dispatches to the backend; stop early when all orders are terminal.

## Definition of done
- `src/swarmflow_core/test/test_fleet.py` passes unchanged; whole `src/swarmflow_core/test` passes; add your own
  tests in `test_fleet_extra.py` (render templates for every v1 decision type, lifecycle edge cases).
- Commit "T004: fleet core", card status `review`. Report the pytest summary and any spec ambiguity you resolved.

## Inputs / contracts used
- docs/design.md §6.3, §6.7, §10, §13.3, §13.5; api.py; graph.py; reservations.py; tests/fixtures/
