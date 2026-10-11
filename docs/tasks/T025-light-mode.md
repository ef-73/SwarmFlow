---
id: T025
title: Light mode - kinematic robots driven by the real orchestrator and robot agents, no Gazebo (X-09 / C-03)
workstream: C
status: open
claimed_by: "sf-implementer"
branch: ws-c/T025-light-mode
model: sonnet
risk: normal
---

## Goal
A Gazebo-free run mode that costs about one core even for 10 robots: the **unchanged** `swarmflow_orchestrator`
node, scenario engine and one **unchanged** `swarmflow_robot_agent` node per robot drive simple kinematic robots
with basic local avoidance, publishing the same topics as the Gazebo stack so Foxglove (`viz` + `foxglove_bridge`)
shows it. Used to watch and tune assignment before Gazebo (improvements_v2 X-09, C-03; milestones M11).

## Design (lead)
- `sim2d/sim2d/light_world.py` — pure Python, no ROS: `LightWorld(max_speed, max_turn, footprint=(0.70, 0.60),
  stop_dist=..., ...)` with `add_robot(id, x, y, yaw)`, `set_goals(id, [(x, y, yaw_or_None), ...])` (replaces the
  current goals), `cancel(id)`, `step(dt)`, `pose(id) -> (x, y, yaw)`, `speed(id)`, `state(id)` in
  `"idle" | "moving" | "blocked" | "arrived"`, `robot_ids()` (sorted). Unicycle: turn in place when the heading error is
  large, else drive and steer. Avoidance: slow/stop when another footprint is ahead within `stop_dist` (state
  `blocked`); a head-on standoff is broken deterministically (e.g. the higher id side-steps to its right, waits until
  clear, continues). Deterministic: no wall clock, no unseeded randomness.
- `sim2d/sim2d/light.py` — `python3 -m sim2d.light --layout-dir D --robots r1,r2 --scenario S --run-dir R
  [--rtf X] [--duration-s T]`: one process that publishes `/clock` (sim time advanced at `rtf` × real time) and, per
  robot, serves `nav2_msgs/action/NavigateThroughPoses` at `/<robot>/navigate_through_poses` (feeding the poses into
  `LightWorld`; abort on cancel/preempt like `fake_nav2.py`), publishes `/<robot>/tf` (`odom -> base_footprint`),
  `/<robot>/tf_static` (`map -> odom` identity) and `/<robot>/plan` (`nav_msgs/Path` of the remaining goals). It also
  starts, unchanged and with `use_sim_time: true`, the orchestrator node, the scenario engine and one agent node per
  robot (namespaced, `/tf -> tf` remapped as in `swarmflow_nav/launch/robot.launch.py`), in-process or as child
  processes (your choice; measure CPU). Spawn poses from the layout's `sim2d.json`. Exits 0 after `--duration-s`
  sim seconds (if given) and leaves the same run record the Gazebo stack leaves (`orders.jsonl`,
  `robot_states.jsonl`, `decisions.jsonl`, `config.yaml`), so `tools/metrics/compute.py` works on it.
- `docker/compose.light.yaml` — `docker compose -f docker/compose.light.yaml up` starts the light process (robot
  image, `sim2d/` and `runs/` mounted), `viz` (robot image is fine: it falls back without gz bindings) and
  `foxglove_bridge` with the same command as `docker/compose.yaml`. No service labelled `swarmflow.sim=1`.

## Owned files
- sim2d/sim2d/**
- sim2d/tests/** (except the lead tests below)
- sim2d/pyproject.toml
- sim2d/README.md
- docker/compose.light.yaml

## Lead tests (do not edit)
- sim2d/tests/test_light_world.py
- sim2d/tests/test_light_ros.py

## Forbidden
- AGENTS.md §4; no Gazebo; no edits to `src/**` (if an unchanged node cannot be driven this way, stop and report why)
- The existing `sim2d/tests/test_sim2d.py`, `test_kinematics.py` must keep passing unchanged.

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t025 SWARMFLOW_GUI=none
scripts/lock.sh acquire build T025 30
docker compose run --rm dev bash -c "colcon build --symlink-install --parallel-workers 2 && source install/setup.bash && python3 -m pytest -q sim2d"
scripts/lock.sh release build
```

## Definition of done
- Both lead test files pass in the dev container (3 times in a row for `test_light_ros.py`), existing sim2d tests pass.
- CPU: report `process_time` of a 600 sim-s, 3-robot `v1_demo` run at rtf 1 (target ≤ 0.6 core average) and the
  `LightWorld` cost per robot per step.
- `sim2d/README.md`: how to run light mode, what it simplifies (no lidar, no Nav2, simple avoidance; clearance
  violations possible only if the head-on rule fails), and its CPU numbers.
- `scripts/ci.sh` green on the branch rebased onto `main`.

## Inputs / contracts used
- design §6.7 (Backend), §8.3; `src/swarmflow_robot_agent/swarmflow_robot_agent/fake_nav2.py` (pattern),
  `agent_node.py` (TF and action names), `swarmflow_nav/launch/robot.launch.py` (remaps),
  `swarmflow_orchestrator/launch/orchestrator.launch.py`, `swarmflow_scenarios/launch/scenario_engine.launch.py`
  (parameters), `tests/integration/demo_run.sh` (run record), `tools/metrics/compute.py`.
