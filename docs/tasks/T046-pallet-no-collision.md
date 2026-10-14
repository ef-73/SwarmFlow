---
id: T046
title: Pallet models without collision geometry (they tumbled after set-down)
workstream: F
status: open
claimed_by: "sf-mechanical"
branch: ws-f/T046-pallet-no-collision
model: haiku
risk: low
---

## Goal
In Gazebo (M16 session p2) two of four pallets set down at a dock ended up tilted or 0.5 m in the air: the models
have collision boxes but no gravity, so contact with the floor pushed them away for good. The payload node controls
the pallet pose and the GPU lidar sees visuals, so the pallets need no collision geometry (like the v1 packages).

## Owned files
- tools/layoutgen/layoutgen/generate.py
- layouts/warehouse_v2/generated/world.sdf

## Lead tests (do not edit)
- tools/layoutgen/tests/test_pallet_models.py

## Forbidden
- AGENTS.md section 4; no other change to the generator's output; `layouts/standard/**` unchanged.

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t046
docker compose run --rm --no-deps dev bash -c "python3 tools/layoutgen/generate.py layouts/warehouse_v2/layout.yaml --out layouts/warehouse_v2/generated && python3 tools/layoutgen/generate.py layouts/standard/layout.yaml --out layouts/standard/generated && python3 -m pytest -q tools/layoutgen"
```

## Definition of done
- In `_pallet_model` (generate.py) stop emitting the `<collision>` elements and update its docstring to say
  "collision off"; regenerate both layouts; all layoutgen tests pass; `git status` shows only generate.py,
  layouts/warehouse_v2/generated/world.sdf and this card changed.

## Inputs / contracts used
- tools/layoutgen/layoutgen/generate.py (`_pallet_model`)
