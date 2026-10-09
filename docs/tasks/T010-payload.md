---
id: T010
title: Package pose-follower (swarmflow_payload)
workstream: F
status: review
claimed_by: "sf-implementer"
branch: ws-f/T010-payload
model: sonnet
risk: normal
---

## Goal
Packages visibly ride on the robots (design §7.4): a pure core that decides package poses, and a ROS node in the
`payload` service (sim image) that feeds it robot states and ground-truth poses and moves the Gazebo models.

## Owned files
- src/swarmflow_payload/**

## Lead tests (do not edit)
- src/swarmflow_payload/test/test_payload_core.py

## Forbidden
- AGENTS.md §4; never run Gazebo (the lead integrates in sim); do not edit other packages or contracts.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-t010
scripts/lock.sh acquire build T010 20     # release afterwards; wait/retry if held
docker compose run --rm -T dev colcon build --symlink-install --parallel-workers 2
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider src/swarmflow_payload/test
```

## Specification
**`swarmflow_payload/payload_core.py`** (no ROS / gz imports): `POOL_SIZE = 12`, `PLATFORM_Z = 0.45` (box centre:
platform top 0.30 m + half the 0.30 m package), `RECYCLE_AFTER_S = 10.0`.
`PayloadCore(graph, bounds=(x_min, y_min, x_max, y_max), drop_offsets={delivery: m})`:
- `initial_poses()` → `{pkg_NN: (x, y, z, yaw)}`, pool pose `x = x_min − 3.0`, `y = y_min + 0.6·i`, `z = 0.15`
  (same as the generated world).
- `on_robot_pose(robot_id, x, y, yaw, t)` (ground truth), `on_robot_state(dict with robot_id, mode (name), order_id,
  t)`.
- Load: first state with mode `LOADING` for an order the robot has no package for → take the lowest free package
  (pool exhausted → append a message containing "pool" to `self.warnings`, no package). Follow from then on.
- Unload: mode `UNLOADING` starts the dwell; after `api.LOAD_DWELL_S` the package is placed on the floor at the drop
  pose (robot's dropoff station = the delivery vertex nearest the robot pose; pose + `drop_offsets[station]` along the
  station yaw, z 0.15, yaw = station yaw), stops following, `payload_state.loaded = False`; recycled to its pool pose
  `RECYCLE_AFTER_S` after it was dropped.
- A robot whose state goes to IDLE / STUCK / FAULT without completing UNLOADING → its package returns to the pool.
- `tick(t)` → `{pkg: (x, y, z, yaw)}` for packages whose pose must be set now: carried packages every tick (robot
  pose, z = PLATFORM_Z), dropped/recycled packages once.
- `package_of(robot_id)`, `payload_state(robot_id)` → dict with PayloadState fields (`loaded`, `order_id`,
  `payload_type` "small", `size_x/size_y` 0.40, `footprint` = padded footprint polygon in base_link
  `[(0.35, 0.30), (0.35, −0.30), (−0.35, −0.30), (−0.35, 0.30)]` (v1 constant, design §7.2)).
**`swarmflow_payload/payload_node.py`** — `PayloadNode(**kwargs)` named `payload`; params `layout_dir`,
`layout_file` (layout.yaml for bounds and drop offsets), `world` ("standard"), `robots` (string array), `rate_hz` 20.
Subscribes `/fleet/robot_states` and `/<robot>/odom` (`nav_msgs/Odometry`, ground truth in the map frame — the robot's
Gazebo OdometryPublisher reports the absolute world pose); publishes `/<robot>/payload_state` (`PayloadState`, frame
`base_footprint`) at 5 Hz. Moves models with gz-transport Python bindings (present in the sim image:
`from gz.transport13 import Node`, `from gz.msgs10.pose_v_pb2 import Pose_V`, `from gz.msgs10.boolean_pb2 import
Boolean`): one `request("/world/<world>/set_pose_vector", Pose_V, Pose_V, Boolean, timeout_ms)` per tick containing all
poses from `core.tick(t)` (do not call it when the dict is empty). Keep the gz call behind a small `PoseSetter` class so
the node can be tested with a fake. Import gz lazily (the dev image has no gz Python; tests use the fake). Console entry
`payload_node`; `launch/payload.launch.py` (use_sim_time true; args layout, layouts_root, robots from env
`SWARMFLOW_ROBOTS`).
- The gz API names above are **[U]** (taken from the installed modules, not yet exercised); the lead verifies them in
  sim. Mark them clearly in a comment.

## Definition of done
- Lead tests pass unchanged; add a node test (wall clock) with a fake PoseSetter: RobotState LOADING + odom → a
  `payload_state` with `loaded=True` and the fake receives the package pose. Commit "T010: payload follower",
  card status `review`.

## Inputs / contracts used
- docs/design.md §7.2, §7.4; swarmflow_interfaces (PayloadState, RobotState); api.py; graph.py;
  layouts/standard/layout.yaml (+ generated world: package pool)
