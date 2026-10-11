---
id: T022
title: Gazebo robot dashboard panel (C++ gz-gui plugin)
workstream: E
status: claimed
claimed_by: "claude-lead"
branch: lead/T022-gazebo-dashboard
model: opus
risk: low
---

## Goal
The Gazebo window shows a real "SwarmFlow robots" panel (one row per robot: status colour, robot colour, mode, goal,
order, load, speed, update age, fault) without manual setup, replacing the T021 TopicEcho stopgap (user request
2026-10-11). The T021 Foxglove layout is re-imported and checked once more.

## Owned files
- src/swarmflow_gz_panel/**
- src/swarmflow_viz/**
- docker/gui.sh
- README.md
- docs/evidence/T022_*
- docs/tasks/T022-gazebo-dashboard.md

## Lead tests (do not edit)
- none

## Forbidden
- AGENTS.md §4
- No change to Nav2, robot agent, orchestrator or interfaces.

## Commands to run
```bash
export COMPOSE_PROJECT_NAME=swarmflow SWARMFLOW_GUI=none
docker compose run --rm dev pytest src/swarmflow_viz/test -q
scripts/ci.sh
```

## Definition of done
- `swarmflow_gz_panel` builds a gz-gui 8 plugin `SwarmFlowDashboard` in the sim image and builds nothing where
  gz-gui 8 is missing (dev image, CI).
- `swarmflow_viz` publishes the board as JSON on gz `/swarmflow/dashboard`; the Gazebo GUI config loads the panel.
- Seen working in the Gazebo window (screenshot in `docs/evidence/`); `scripts/ci.sh` green; agent-log entry.

## Inputs / contracts used
- T021 (`gz_overlay`, `config/gazebo_gui.config`, `docker/gui.sh`); gz-gui 8 plugin API (sim image headers)
