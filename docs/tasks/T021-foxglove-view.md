---
id: T021
title: Foxglove view that matches Gazebo, per-robot overlay and fleet dashboard
workstream: E
status: done
claimed_by: "claude-lead"
branch: lead/T021-foxglove-view
model: opus
risk: normal
---

## Goal
Foxglove is the main view of a run (user decision 2026-10-10, `docs/improvements_v2.md` §7 V-01…V-08, V-05b,
V-10): it shows the same warehouse as Gazebo, every robot in its Gazebo colour with its lidar, footprint outline,
path on the floor and current goal, symbols for places, packages where Gazebo has them, and a per-robot status
dashboard. The Gazebo window no longer starts by default. Lead-implemented (needs Gazebo to verify).

## Owned files
- src/swarmflow_viz/**
- viz/foxglove/**
- docker/compose.yaml
- docker/gui.sh
- README.md
- docs/evidence/T021_*
- docs/tasks/T021-foxglove-view.md

## Lead tests (do not edit)
- none

## Forbidden
- AGENTS.md §4
- No change to Nav2, robot agent, orchestrator or interfaces: the view only reads existing topics.

## Commands to run
```bash
export COMPOSE_PROJECT_NAME=swarmflow SWARMFLOW_GUI=none
docker compose run --rm dev colcon test --packages-select swarmflow_viz
scripts/ci.sh
```

## Definition of done
- `tf_relay` merges `/robot_N/tf(_static)` into `/tf(_static)` with `robot_N/` frame prefixes; scans on `/viz/robot_N/scan`.
- `viz_node` publishes: the world from `generated/world.sdf` (`/viz/scene`, latched), packages at their Gazebo
  poses, robot bodies/labels/footprints locked to their TF frames, paths and goals per robot (`/viz/robots`),
  and a `diagnostic_msgs/DiagnosticArray` dashboard (`/viz/fleet_dashboard`).
- Gazebo window (user request, same session): status rings, paths and goals as gz markers; a "SwarmFlow robots"
  TopicEcho panel with one status line per robot (`config/gazebo_gui.config`, loaded by `docker/gui.sh`).
- `gazebo_gui` only starts with `--profile gui`; `foxglove_bridge` forwards only the topics the layout uses.
- `viz/foxglove/swarmflow_v2.json` imports into current Foxglove (checked in Chrome) and shows the above; a
  screenshot comparison Foxglove vs Gazebo at the same moment is in `docs/evidence/`.
- Unit tests for the SDF parser, frame prefixing, markers and dashboard; `scripts/ci.sh` green.
- Agent-log entry written (AGENTS.md §8).

## Inputs / contracts used
- design §7.1, §7.6, §11.1; `src/swarmflow_description/urdf/swarmflow_bot.urdf.xacro`;
  `src/swarmflow_gazebo/launch/sim.launch.py` (robot colours); `layouts/*/generated/world.sdf`
- interfaces: `RobotState`, `Order`, `OrderStatus`, `PayloadState` (read only)
