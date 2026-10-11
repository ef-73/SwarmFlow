---
id: T029
title: Robot agent goal mode (ROUTE_GOALS, dwell by vertex kind, blocked reporting, pallet modes, angular speed)
workstream: D
status: open
claimed_by: "sf-implementer"
branch: ws-d/T029-agent-goals
model: sonnet
risk: safety-critical
---

## Goal
The robot agent executes v2 goal tasks exactly as the `swarmflow_core.api` v2 docstring says, while v1 waypoint
tasks behave exactly as before (improvements_v2 O-01, O-02, P-01, N-03 reporting, F-06 modes, N-05 angular speed).

## Design (lead)
- `AgentCore.start_task(task_id, order_id, route, t, pickup="", dropoff="", route_kind=api.ROUTE_WAYPOINTS,
  order_type=api.ORDER_TRANSPORT)`. `ROUTE_GOALS`: every goal must be a graph vertex (no edge check), no holds,
  no reservations (whatever `traffic_control` is), no v1 STUCK_TIMEOUT failure.
- One `Navigate` per goal with a single pose (vertex x, y; yaw = the vertex yaw if it has one, else the heading
  from the current pose to the goal).
- Goal kinds: no-dwell goals (`pick`, `queue`, `home`, `pallet` when the order is not a pallet order,
  intersections) are reached when the pose is within `api.GOAL_TOLERANCE_M` **or** Nav2 succeeds — then the next
  goal is sent at once (or the task finishes); a late duplicate signal is ignored. Dwell goals wait for Nav2
  success: `dock` → mode `UNLOADING` for `UNLOAD_DWELL_BOX_S` (shelf/transport) or `UNLOAD_DWELL_PALLET_S`
  (pallet); `loading`/`delivery` → v1 `LOAD_DWELL_S` (mode LOADING or UNLOADING); `pallet` goal of a pallet
  order → mode `LIFTING` for `PALLET_LIFT_S` (new module constant, 3.0 s), then mode `CARRYING` for the rest of the
  task until the dock dwell. `last_vertex` = last goal reached, `next_vertex` = current goal,
  `remaining_route` = current goal and the ones after it; between tasks `last_vertex` is kept.
- Blocked: distance to the current goal not reduced by `STUCK_MIN_PROGRESS_M` for `BLOCKED_REPORT_S` → mode
  `STUCK`, `fault_reason = api.FAULT_BLOCKED`, task kept, navigation not cancelled; progress → back to the moving
  mode. A Nav2 failure → same STUCK/BLOCKED report and the same goal re-sent after a jittered 1–3 s.
- `on_pose(..., angular_speed=0.0)` keyword; `state().angular_speed`.
- `agent_node.py`: pass `goal.route_kind` and `goal.order.order_type` (codes → api strings via
  `ROUTE_KIND_CODES` / `ORDER_TYPE_CODES`), measure angular speed (yaw difference over time, or odometry if
  already subscribed), publish `RobotState.angular_speed`; behaviour for `ROUTE_WAYPOINTS` unchanged.

## Owned files
- src/swarmflow_robot_agent/swarmflow_robot_agent/agent_core.py
- src/swarmflow_robot_agent/swarmflow_robot_agent/agent_node.py
- src/swarmflow_robot_agent/test/test_agent_goals_extra.py

## Lead tests (do not edit)
- src/swarmflow_robot_agent/test/test_agent_goals.py

## Forbidden
- AGENTS.md §4; no Gazebo; no changes to v1 waypoint behaviour (all existing robot-agent tests must pass unchanged).

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t029 SWARMFLOW_GUI=none
docker compose run --rm --no-deps dev bash -c "cd src/swarmflow_robot_agent && PYTHONPATH=.:../swarmflow_core python3 -m pytest -q test/test_agent_goals.py test/test_agent_core*.py"
scripts/lock.sh acquire build T029 30 && docker compose run --rm dev bash -c "colcon build --symlink-install --parallel-workers 2 && colcon test --packages-select swarmflow_robot_agent && colcon test-result --verbose"; scripts/lock.sh release build
```

## Definition of done
- Lead tests and every existing robot-agent test pass (colcon test, incl. `test_agent_node.py`).
- Extra tests: goal-mode cancel mid-dwell, a nav failure during CARRYING keeps CARRYING-vs-STUCK reporting
  consistent, a waypoint task after a goal task still requests reservations.
- `scripts/ci.sh` green on the branch rebased onto `main`; reviewer pass (safety-critical).

## Inputs / contracts used
- `api.py` v2 docstring and constants; `swarmflow_interfaces` (DispatchTask.route_kind, Order.order_type,
  RobotState.angular_speed); design §6.3.
