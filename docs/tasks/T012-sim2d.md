---
id: T012
title: 2D kinematic backend skeleton (sim2d)
workstream: C
status: open
claimed_by: "sf-implementer"
branch: ws-c/T012-sim2d
model: sonnet
risk: low
---

## Goal
`sim2d/` runs the v1 fleet layer (FleetCore + FCFS + reservation authority) against simple kinematic robots that are
driven by the **real** robot-agent core, loading `layouts/standard/generated/sim2d.json`. Deterministic. Seed for the
v2 benchmark backend (design §6.7, §12.3, §13.2).

## Owned files
- sim2d/**

## Lead tests (do not edit)
- sim2d/tests/test_sim2d.py

## Forbidden
- AGENTS.md §4; no ROS imports; stdlib + PyYAML only; do not edit `swarmflow_core` or `swarmflow_robot_agent`.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-t012
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider sim2d
```

## Specification
Package `sim2d/sim2d/` (+ `sim2d/pyproject.toml`, name `swarmflow-sim2d`), exporting `Sim2DBackend` and `run`.
- `Sim2DBackend.from_json(path, robots, speed_mps=0.5, turn_rate=1.5, seed=0)`: builds a `WarehouseGraph` from the
  sim2d.json (or loads `nav_graph.yaml` + `zones.yaml` next to it with `swarmflow_core.graph.load_layout` — your
  choice; document it), spawns robots at the layout's spawn vertices, one
  `swarmflow_robot_agent.agent_core.AgentCore` per robot (`agent(robot_id)` returns it). Implements `api.Backend`
  (`snapshot`, `dispatch`, `cancel`, `poll_results`) and `step(t, dt)`.
- Kinematics: a robot executing a `Navigate` action turns in place towards the next pose at `turn_rate`, then drives
  straight at `speed_mps`; reports pose to its AgentCore (`on_pose`) every step and `on_nav_result(True)` on reaching
  the last pose. `CancelNav` stops it. No collisions (it is a kinematic model).
- Protocol wiring: `RequestReservation` → the run's `FcfsReservationAuthority.request` (response delivered on the
  next step), `Release` → `authority.release`, heartbeats every `api.HEARTBEAT_PERIOD_S` with `heartbeat_lease_ids()`,
  `on_orchestrator_heartbeat(t, epoch="epoch:sim2d")` every 1 s; `TaskFinished` → a `TaskResult` for
  `poll_results`. `Navigate` vertices/poses come from AgentCore.
- `run(layout_json, orders, policy, robots, until_t, dt=0.1, seed=0) -> Sim2DResult`: authority (policy `fcfs`) or
  none (`independent`, agents with `traffic_control=False`); FleetCore + `FcfsPolicy(name=policy)`; `authority.recover`
  at t=0; loop like `swarmflow_core.backends.fake.run_fake` (snapshots → `core.update_robot` + `authority.observe_robot`,
  results → `core.task_result`, `core.tick`, cancels then dispatches); stop when all orders are terminal.
  `Sim2DResult`: `order_states` (order_id → OrderState), `decisions`, `max_robots_in_zone`, `finish_t`,
  `trajectory_digest` (sha256 of all robot poses rounded to 3 decimals, per step).
- Determinism: no wall clock, no unseeded randomness, sorted iteration everywhere.

## Definition of done
- Lead tests pass unchanged; add your own tests. Commit "T012: sim2d skeleton", card status `review`.

## Inputs / contracts used
- docs/design.md §6.3, §6.7, §12.3; api.py; graph.py; fleet.py; reservations.py; backends/fake.py (pattern);
  src/swarmflow_robot_agent/swarmflow_robot_agent/agent_core.py; layouts/standard/generated/sim2d.json
