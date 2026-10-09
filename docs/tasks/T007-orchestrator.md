---
id: T007
title: Orchestrator ROS adapter (swarmflow_orchestrator)
workstream: B
status: open
claimed_by: "sf-implementer"
branch: ws-b/T007-orchestrator
model: sonnet
risk: safety-critical
---

## Goal
Package `src/swarmflow_orchestrator/` (ament_python): a thin rclpy node around `swarmflow_core.fleet.FleetCore` and
`FcfsReservationAuthority` that implements the fleet side of design §6.5/§6.6, writes the run record (§13.5), plus a
launch file that also serves the global `/map`.

## Owned files
- src/swarmflow_orchestrator/**

## Lead tests (do not edit)
- src/swarmflow_orchestrator/test/test_orchestrator_node.py

## Forbidden
- AGENTS.md §4; no Gazebo/Nav2; do not edit contracts or `swarmflow_core` (report a change request instead).

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-t007
scripts/lock.sh acquire build T007 20     # release afterwards: scripts/lock.sh release build
docker compose run --rm -T dev colcon build --symlink-install --parallel-workers 2
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider src/swarmflow_orchestrator/test
```

## Specification
**`swarmflow_orchestrator/orchestrator_node.py` — `OrchestratorNode(**kwargs)`**, rclpy Node named `orchestrator`.
Parameters: `layout_dir` (generated dir), `robots` (string array, e.g. `["robot_1","robot_2","robot_3"]`),
`policy` (`fcfs` | `independent`), `run_dir` (`""` = no files), `tick_hz` (5.0), `state_log_hz` (2.0).
- Builds `graph = load_layout(layout_dir)`, `authority = FcfsReservationAuthority(graph)`,
  `FcfsPolicy(name=policy)`, `FleetCore(graph, policy, authority, traffic_control=(policy == "fcfs"))`.
- **Epoch**: at start `epoch = f"epoch:{<node clock ns at start>}-{os.getpid()}"`; `/fleet/orchestrator_heartbeat`
  (`std_msgs/Header`, 1 Hz, `stamp` = node clock, `frame_id` = epoch). Agents use the epoch to detect restarts.
- **Recovery**: for `api.RECOVERY_GRACE_S` after start, collect RobotStates; then `authority.recover(snapshots, t)`
  (until then the authority denies with `RECOVERING`).
- Subscriptions: `/fleet/orders` (`Order` → `OrderSpec`; release_t = msg `release_time` if non-zero else now;
  deadline zero → None), `/fleet/robot_states` (→ `RobotSnapshot` + `authority.observe_robot(robot_id, x, y,
  stamp)` with the header stamp), `/fleet/reservation_heartbeat`, `/fleet/reservation_release`.
- Services: `/fleet/request_reservation` → `authority.request` (+ a `RESERVATION_GRANT`/`RESERVATION_DENY`
  DecisionEvent via `decisions.reservation_decision`). With policy `independent` every request is answered DENIED,
  reason `TRAFFIC_CONTROL_OFF`, and no DecisionEvent (Baseline A agents never ask). `/fleet/clear_zone` →
  `authority.clear_zone`.
- Tick (`tick_hz`): `authority.expire(t)`; `out = core.tick(t)`; for each cancel → cancel that task's action goal;
  for each dispatch → `DispatchTask` goal to `/<robot_id>/dispatch_task` (`order` filled from the OrderSpec; empty
  Order for move-only tasks); results → `core.task_result(task_id, success, failure_reason, t)`. A goal rejected by
  the agent → treat as failure `"REJECTED"` (core requeues the order). Never block a callback on a future.
- Publish: `/fleet/order_status` (every `OrderStatusChange`, `eta` zero), `/fleet/decisions` (every Decision; the
  `explanation` comes from `decisions.render`), `/fleet/reservations` (every `authority.pop_events()` lease, state via
  `api.LEASE_STATE_CODES`, `lease_expiry` = expiry_t for GRANTED else zero).
  QoS per the api.py docstring: reliable everywhere; `/fleet/order_status` and `/fleet/reservations`
  transient-local depth 100.
- Time: node clock everywhere (follows `use_sim_time`), converted to float seconds.
- **Run record** (design §13.5) when `run_dir` is set: create the dir; append JSON lines (one object per line,
  `sort_keys=True`, flushed): `decisions.jsonl` (all Decision fields; enums as strings), `orders.jsonl` (every
  status change: t, order_id, state name, robot_id, failure_reason), `robot_states.jsonl` (each robot at
  `state_log_hz`: t, robot_id, x, y, yaw, mode name, linear_speed, task_id, order_id, held_lease_ids),
  `reservations.jsonl` (lease events).
- Console entry point `orchestrator_node = swarmflow_orchestrator.orchestrator_node:main`.

**`launch/orchestrator.launch.py`**: args `layout` (standard), `layouts_root` (/ws/layouts), `policy`
(env `SWARMFLOW_POLICY`, default fcfs), `robots` (env `SWARMFLOW_ROBOTS`, default robot_1,robot_2,robot_3),
`run_id` (env `SWARMFLOW_RUN_ID`, default `""` → `f"{SWARMFLOW_SCENARIO or 'adhoc'}-{policy}-latest"`),
`runs_root` (/ws/runs). Starts the orchestrator (`use_sim_time: true`, `run_dir = runs_root/run_id`) and a global
`nav2_map_server` (lifecycle-managed, autostart, `use_sim_time: true`) publishing `/map` from the layout's
`generated/map.yaml` (node name `global_map_server`, topic `/map`). `package.xml` exec_depends incl. `nav2_map_server`
and `nav2_lifecycle_manager` (present in the robot image; the dev image does not need them to build).

## Definition of done
- Lead tests pass unchanged; add tests for the run record format and for cancel/reject handling.
- Commit "T007: orchestrator adapter", card status `review`. Report test summaries and every ambiguity resolved.

## Inputs / contracts used
- docs/design.md §6.5, §6.6, §10, §13.5; api.py (docstring: lifecycle, QoS); fleet.py, reservations.py,
  decisions.py; swarmflow_interfaces.
