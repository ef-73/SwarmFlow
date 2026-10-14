---
id: T042
title: Pallet docking in the robot agent - pre-dock, drive under, lift, carry footprint, deep drop, back out (F-02, F-04)
workstream: D
status: claimed
claimed_by: "sf-implementer"
branch: ws-d/T042-pallet-docking
model: sonnet
risk: safety-critical
---

## Goal
A robot on a pallet order picks up the pallet from its stand and sets it down at the truck dock in Gazebo
(improvements_v2 F-02, F-04). Nav2 cannot plan between pallet legs 15 cm from the body, so the final metre in and
out is a straight closed-loop drive commanded by the agent.

## Design (lead)
`agent_core.py` (goal mode, pallet orders only; shelf orders and v1 unchanged):
- Constants: `PREDOCK_M = 1.2`, `DOCK_AHEAD_M = 0.7`, `UNDOCK_M = 0.9`, `DOCK_TOL_LAT_M = 0.03`,
  `DOCK_TOL_LON_M = 0.02`, `DOCK_TIMEOUT_S = 30.0`, `PALLET_LIFT_S` (exists).
- New actions: `DockDrive(target: (x, y, yaw), reverse: bool)`, `StopDrive()`, `SetFootprint(kind: "carry" |
  "normal")`.
- Pallet goal (`pallet` vertex of a pallet order): `Navigate` to the pre-dock pose = pallet pose − `PREDOCK_M`
  along the pallet yaw, heading = pallet yaw. On Nav2 success → mode LIFTING + `DockDrive(pallet pose, forward)`.
  Reached when the pose is within `DOCK_TOL_LAT_M` across and `DOCK_TOL_LON_M` along the target heading →
  `StopDrive`, lift dwell `PALLET_LIFT_S`, then `SetFootprint("carry")` **before** the next `Navigate`, mode
  CARRYING, `last_vertex` = the pallet vertex.
- Dock goal of a pallet order: on Nav2 success → `DockDrive(dock pose + DOCK_AHEAD_M along the dock yaw,
  forward)` (mode stays CARRYING) → reached → `StopDrive`, mode UNLOADING for `UNLOAD_DWELL_PALLET_S` → mode
  LIFTING + `DockDrive(reverse)` to the deep-drop pose − `UNDOCK_M` → reached → `StopDrive`,
  `SetFootprint("normal")`, task success, `last_vertex` = the dock.
- A dock drive without `STUCK_MIN_PROGRESS_M` progress for `DOCK_TIMEOUT_S` → `StopDrive`, task failure
  `FAILURE_NAV_PREFIX + <vertex>`; cancel during a drive → `StopDrive` + CANCELLED (no `CancelNav`). A pallet-order
  task without a pallet goal (dock-queue follow-up) starts in CARRYING and keeps the carry footprint.

`agent_node.py`:
- `DockDrive`: 20 Hz closed loop on the TF pose publishing `geometry_msgs/Twist` on the robot's `cmd_vel`
  (namespace-relative, the same topic the velocity smoother feeds): forward/back speed ≤ 0.15 m/s, slowing near the
  target, heading and lateral correction (e.g. pure pursuit on the target line), ≤ 0.5 rad/s; `StopDrive` publishes a
  zero twist and ends the loop. Never runs while a Nav2 goal is active.
- `SetFootprint`: set the `footprint` parameter of `/<rid>/local_costmap/local_costmap` and
  `/<rid>/global_costmap/global_costmap` (carry: padded 1.25 × 1.05 m, normal: the current 0.70 × 0.60 string from
  nav2_params.yaml) with async parameter clients; log a warning if the call fails, do not block the agent.

## Owned files
- src/swarmflow_robot_agent/swarmflow_robot_agent/agent_core.py
- src/swarmflow_robot_agent/swarmflow_robot_agent/agent_node.py
- src/swarmflow_robot_agent/test/test_pallet_docking_extra.py
- src/swarmflow_robot_agent/test/test_agent_goals_extra.py

## Lead tests (do not edit)
- src/swarmflow_robot_agent/test/test_pallet_docking.py
- src/swarmflow_robot_agent/test/test_agent_goals.py

## Forbidden
- AGENTS.md section 4; no Gazebo; v1 and shelf-order behaviour unchanged (all existing agent tests pass).

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t042 SWARMFLOW_GUI=none
docker compose run --rm --no-deps dev bash -c "cd src/swarmflow_robot_agent && PYTHONPATH=.:../swarmflow_core python3 -m pytest -q test"
scripts/lock.sh acquire build T042 30 && docker compose run --rm dev bash -c "colcon build --symlink-install --parallel-workers 2 && colcon test --parallel-workers 1 --packages-select swarmflow_robot_agent && colcon test-result --verbose"; scripts/lock.sh release build
```

## Definition of done
- Lead tests and all existing robot-agent tests pass (pytest and colcon test).
- `scripts/ci.sh` green on the branch rebased onto `main`; reviewer pass (safety-critical: direct velocity commands).

## Inputs / contracts used
- api.py v2 docstring (pallet lifecycle, dwell), design 7.1 (robot geometry), T041 pallet geometry (deck 1.20 x 1.00,
  legs 15 cm from the body).
