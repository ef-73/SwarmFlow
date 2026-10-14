---
id: T052
title: warehouse_v2 spawns robot_1 ... robot_8 (one per home slot)
workstream: F
status: review
claimed_by: "sf-mechanical"
branch: ws-f/T052-v2-eight-spawns
model: haiku
risk: low
---

## Goal
The light-mode scaling check (M11/M15) needs more than 5 robots; the layout has 8 home slots but spawns 5. Gazebo
only spawns the robots named in SWARMFLOW_ROBOTS, so nothing else changes.

## Owned files
- tools/layoutgen/build_warehouse_v2.py
- layouts/warehouse_v2/layout.yaml
- layouts/warehouse_v2/generated/**

## Lead tests (do not edit)
- tools/layoutgen/tests/test_v2_spawns.py

## Forbidden
- AGENTS.md section 4; no other change.

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t052
docker compose run --rm --no-deps dev bash -c "python3 tools/layoutgen/build_warehouse_v2.py && python3 tools/layoutgen/generate.py layouts/warehouse_v2/layout.yaml --out layouts/warehouse_v2/generated && python3 tools/layoutgen/generate.py layouts/standard/layout.yaml --out layouts/standard/generated && python3 -m pytest -q tools/layoutgen"
```
(Check how build_warehouse_v2.py writes layout.yaml first; if it takes arguments, use them.)

## Definition of done
- In build_warehouse_v2.py the spawn list covers robot_1 … robot_8 at HM1 … HM8 (same yaw as now); layout.yaml
  and generated files regenerated; all layoutgen tests pass; `git status` shows only the owned files changed.

## Inputs / contracts used
- tools/layoutgen/build_warehouse_v2.py (spawn list, line ~125)
