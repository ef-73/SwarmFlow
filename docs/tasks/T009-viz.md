---
id: T009
title: Foxglove layout + swarmflow_viz marker node
workstream: E
status: open
claimed_by: "sf-implementer"
branch: ws-e/T009-viz
model: sonnet
risk: low
---

## Goal
Foxglove shows the fleet (design §11.1): a `swarmflow_viz` node publishes robot, zone and station markers, and
`viz/foxglove/swarmflow_v1.json` is a ready-to-import layout.

## Owned files
- src/swarmflow_viz/**
- viz/**

## Lead tests (do not edit)
- src/swarmflow_viz/test/test_viz.py

## Forbidden
- AGENTS.md §4; no Gazebo; do not edit contracts or other packages.

## Commands to run
```bash
export MSYS_NO_PATHCONV=1 PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"
export COMPOSE_PROJECT_NAME=swarmflow-t009
scripts/lock.sh acquire build T009 20     # release afterwards; wait/retry if held
docker compose run --rm -T dev colcon build --symlink-install --parallel-workers 2
docker compose run --rm -T dev python3 -m pytest -q -p no:cacheprovider src/swarmflow_viz/test
```

## Specification
**`swarmflow_viz/markers.py`** (no node, pure builders returning `visualization_msgs/MarkerArray`, frame `map`,
`header.stamp` from `stamp_sec`):
- `ZONE_COLORS = {"FREE": (0.2, 0.8, 0.2, 0.35), "GRANTED": (1.0, 0.6, 0.0, 0.45), "OCCUPIED_UNKNOWN":
  (0.9, 0.1, 0.1, 0.55)}` (r, g, b, a; values rounded to 2 decimals).
- `robot_markers(states, stamp_sec)`: `states` = dicts with RobotState field names (x, y, yaw, mode as name, robot_id,
  …). Per robot: ns `robots`, id = robot number (`robot_3` → 3), CUBE 0.60 × 0.50 × 0.25 at the pose (z 0.175),
  colour per robot (robot_1 blue, robot_2 orange, robot_3 green); ns `robot_labels`, same id, TEXT_VIEW_FACING
  `"<robot_id> <MODE>"` (+ ` <order_id>` if any) at z 0.8.
- `zone_markers(graph, lease_states: dict[zone -> api.LeaseState], stamp_sec)`: per zone (sorted by name, id = index)
  ns `zones` TRIANGLE_LIST (two triangles per rectangle; or a LINE_STRIP outline) coloured FREE / GRANTED /
  OCCUPIED_UNKNOWN (RELEASED/REVOKED/EXPIRED → FREE); ns `zone_labels` TEXT `"<zone> <state>"`.
- `station_markers(graph, stamp_sec)`: CYLINDER pads + TEXT_VIEW_FACING label per station vertex.
**`swarmflow_viz/viz_node.py`** — `VizNode(**kwargs)` named `swarmflow_viz`, parameter `layout_dir`; subscribes
`/fleet/robot_states` and `/fleet/reservations` (transient-local depth 100, reliable); keeps the latest state per
robot and lease state per zone (by lease events); publishes `/fleet/robot_markers` at 5 Hz and `/fleet/zone_markers`
(zones + stations) at 1 Hz. Console entry `viz_node`. A launch file `launch/viz.launch.py` (use_sim_time true,
layout_dir from args `layout`, `layouts_root`).
**`viz/foxglove/swarmflow_v1.json`**: a Foxglove layout (studio layout export format: `configById`, `layout`,
`globalVariables`, `userNodes`, `playbackConfig`) with: a 3D panel (fixed frame `map`; topics `/map`,
`/fleet/robot_markers`, `/fleet/zone_markers`, `/robot_1/plan`, `/robot_2/plan`, `/robot_3/plan`), a decision log
(RawMessages or Table on `/fleet/decisions`), an order table on `/fleet/order_status`, and a Plot of robot speeds from
`/fleet/robot_states` (`.linear_speed` per robot via message path filters `{robot_id=="robot_1"}`). Add a short
`viz/foxglove/README.md`: connect to `ws://localhost:8765`, import the layout.

## Definition of done
- Lead test passes unchanged; add a node test (wall clock): publish 2 RobotStates → a MarkerArray arrives on
  `/fleet/robot_markers` with 2 robots. Commit "T009: viz", card status `review`.

## Inputs / contracts used
- docs/design.md §11.1; swarmflow_interfaces; api.py; graph.py; tests/fixtures/robot_states_standard.jsonl
