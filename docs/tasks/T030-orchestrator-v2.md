---
id: T030
title: Orchestrator node runs the v2 fleet core (policy "assign") with v2 order and task fields
workstream: B
status: claimed
claimed_by: "sf-implementer"
branch: ws-b/T030-orchestrator-v2
model: sonnet
risk: normal
---

## Goal
`swarmflow_orchestrator` drives `FleetCoreV2` + `AssignPolicy` when `policy:=assign`, converting the v2 fields both
ways, so the full v2 loop runs over ROS (light mode, then Gazebo). `fcfs` / `independent` are unchanged.

## Design (lead)
- `policy` parameter accepts `assign` (constant string `"assign"`, = `AssignPolicy().name`). With `assign`:
  `self.core = FleetCoreV2(self.graph, AssignPolicy())`; no reservation traffic (keep the authority object so the
  services still answer, but agents in goal mode never ask).
- `_on_order`: fill `order_type` (code → string via `api.ORDER_TYPE_CODES`), `pick_vertices` (tuple), `pallet_id`.
  If the core rejects an order (`ValueError`, e.g. a transport order in v2), publish one `OrderStatus` FAILED with
  `failure_reason = "UNSUPPORTED_ORDER: <message>"`, record it in `orders.jsonl`, and log a warning.
- `_goal_of`: `goal.route_kind = api.ROUTE_KIND_CODES[a.route_kind]`; order `order_type`, `pick_vertices`,
  `pallet_id`.
- `_on_robot_state`: `angular_speed` into the snapshot.
- Run records (`orders.jsonl`, decisions) unchanged in format; add `order_type` to the order records.
- `orchestrator.launch.py`: nothing to change if the policy comes from `SWARMFLOW_POLICY`; check it.

## Owned files
- src/swarmflow_orchestrator/swarmflow_orchestrator/orchestrator_node.py
- src/swarmflow_orchestrator/launch/orchestrator.launch.py
- src/swarmflow_orchestrator/test/test_orchestrator_v2_extra.py

## Lead tests (do not edit)
- src/swarmflow_orchestrator/test/test_orchestrator_v2.py

## Forbidden
- AGENTS.md §4; no Gazebo; v1 behaviour unchanged (all existing orchestrator tests pass unchanged).

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t030 SWARMFLOW_GUI=none
scripts/lock.sh acquire build T030 30 && docker compose run --rm dev bash -c "colcon build --symlink-install --parallel-workers 2 && colcon test --parallel-workers 1 --packages-select swarmflow_orchestrator && colcon test-result --verbose"; scripts/lock.sh release build
```

## Definition of done
- Lead test and all existing orchestrator tests pass (3 runs in a row for the new test).
- `scripts/ci.sh` green on the branch rebased onto `main`.

## Inputs / contracts used
- `swarmflow_core.fleet_v2`, `policies.assign` (T028), `api.py` v2, `Order.msg`, `DispatchTask.action`,
  `RobotState.msg`.
