---
id: T054
title: C++ TF / scan relay for the viz container (X-01)
workstream: E
status: review
claimed_by: "sf-implementer"
branch: ws-e/T054-cpp-relay
model: sonnet
risk: normal
---

## Goal
The `viz` container uses ~0.65 core (target 0.3, improvements_v2 X-01). Measured split: `tf_relay` (Python) ~0.34,
`viz_node` ~0.23. The relay handles every TF and scan message of every robot in rclpy. Replace it with a C++ node of
the same behaviour (default user decision recorded in docs/user_todo.md: convert swarmflow_viz, WS-E keeps it).

## Design (lead)
- Convert `src/swarmflow_viz` to `ament_cmake` + `ament_cmake_python`: the Python package `swarmflow_viz` installs as
  before (`ament_python_install_package`), the two Python nodes keep working as executables in
  `lib/swarmflow_viz/` (`viz_node`, and keep `tf_relay` too), config/launch installed as now.
- New `src/swarmflow_viz/src/tf_relay.cpp` → executable `tf_relay_cpp`, same behaviour as `swarmflow_viz/tf_relay.py`
  (read it): parameter `robots` (string array); per robot subscribe `/<r>/tf`, `/<r>/tf_static` (transient local,
  deep queue) and `/<r>/scan` (sensor QoS); republish on `/tf`, `/tf_static` (latched union of all statics) and
  `/viz/<r>/scan`; prefix every frame with `<r>/` except the shared `map` and frames already prefixed; drop
  transforms whose child ends with `wheel_link`; scans: prefix frame, non-finite ranges → 0.0.
- `launch/viz.launch.py` starts `tf_relay_cpp` instead of the Python relay.
- `package.xml`: build/exec deps for rclcpp, tf2_msgs, sensor_msgs, geometry_msgs; keep the Python exec deps;
  test deps as needed.

## Owned files
- src/swarmflow_viz/** (except the lead test below and the existing tests)

## Lead tests (do not edit)
- src/swarmflow_viz/test/test_tf_relay_cpp.py
- src/swarmflow_viz/test/test_viz.py
- src/swarmflow_viz/test/test_viz_node.py
- src/swarmflow_viz/test/test_viz_qos.py
- src/swarmflow_viz/test/test_fleet_view_t021.py

## Forbidden
- AGENTS.md section 4; no Gazebo; no behaviour change of viz_node; no new packages outside src/swarmflow_viz.

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t054 SWARMFLOW_GUI=none
scripts/lock.sh acquire build T054 30 && docker compose run --rm dev bash -c "colcon build --symlink-install --parallel-workers 2 && colcon test --parallel-workers 1 --packages-select swarmflow_viz && colcon test-result --verbose"; scripts/lock.sh release build
```
(If the dev image lacks rclcpp headers for tf2_msgs/sensor_msgs, stop and report; do not edit Dockerfiles.)

## Definition of done
- All swarmflow_viz tests pass (colcon test), including the lead test; the sim image build is not needed here
  (the lead measures CPU in Gazebo: target viz ≤ 0.3 core with 4 robots).
- `scripts/ci.sh` green on the branch rebased onto `main`.

## Inputs / contracts used
- `swarmflow_viz/tf_relay.py` (reference behaviour), `launch/viz.launch.py`, docs/results/v1_baseline.md (CPU split).
