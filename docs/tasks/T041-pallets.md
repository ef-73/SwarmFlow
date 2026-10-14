---
id: T041
title: Pallet-stand models in the v2 world and a pallet follower in the payload node (F-01, F-03)
workstream: F
status: review
claimed_by: "sf-implementer"
branch: ws-f/T041-pallets
model: sonnet
risk: normal
---

## Goal
Pallets exist in Gazebo at the `PL_*` stations of v2 layouts, and the payload node moves a pallet with the robot
that carries it (raised by `LIFT_Z`), then sets it down where the robot unloaded (improvements_v2 F-01, F-03).

## Design (lead)
- `tools/layoutgen`: for every `pallet` station of a layout emit model `pallet_<station>` at the station pose
  (x, y, 0, 0, 0, yaw), `static` false, gravity off: a deck box 1.20 (along yaw) x 1.00 x 0.06 m with its underside at
  0.38 m, and 4 legs 0.08 x 0.08 x 0.38 m at the deck corners (inset 0.04 m). Collision on. Regenerate
  `layouts/warehouse_v2/generated`; `standard` output must stay byte-identical (it has no pallet stations).
- `src/swarmflow_payload/swarmflow_payload/pallet_core.py` (pure, like `payload_core.py`): `PalletCore(spots)` where
  spots = {model_name: (x, y, z, yaw)}; `LIFT_Z = 0.05`; `CARRY_FOOTPRINT` = padded 1.25 x 1.05 rectangle (list of
  (x, y)); `on_order(order_id, order_type, pallet_id)`, `on_robot_state(state_dict, t)`, `tick(t) -> {model: pose}`
  (only poses that changed), `initial_poses()`, `payload_state(robot_id) -> dict` with keys loaded, order_id,
  payload_type ("pallet" or ""), footprint. Semantics in the lead test: LIFTING does not move the pallet; CARRYING
  makes it follow the robot pose at `LIFT_Z`; it stays lifted during UNLOADING and is set down (z 0) at the robot's
  pose when the robot leaves UNLOADING for any mode, then never moves again.
- `payload_node.py`: on v2 layouts (pallet stations present) also run `PalletCore`: subscribe `/fleet/orders`
  (order_type, pallet_id), feed robot states, send the changed pallet poses with the same gz `set_pose_vector` call
  as packages, and publish `PayloadState` (loaded, payload_type, footprint) for pallet carriers. v1 package handling
  unchanged.

## Owned files
- tools/layoutgen/** (except lead tests)
- layouts/warehouse_v2/generated/**
- src/swarmflow_payload/swarmflow_payload/**
- src/swarmflow_payload/test/test_pallet_core_extra.py

## Lead tests (do not edit)
- tools/layoutgen/tests/test_pallet_models.py
- src/swarmflow_payload/test/test_pallet_core.py

## Forbidden
- AGENTS.md section 4; no Gazebo; `layouts/standard/**` unchanged; no `layouts/schema/**` changes.

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t041 SWARMFLOW_GUI=none
docker compose run --rm --no-deps dev bash -c "python3 tools/layoutgen/generate.py layouts/warehouse_v2/layout.yaml --out layouts/warehouse_v2/generated && python3 tools/layoutgen/generate.py layouts/standard/layout.yaml --out layouts/standard/generated && python3 -m pytest -q tools/layoutgen"
scripts/lock.sh acquire build T041 30 && docker compose run --rm dev bash -c "colcon build --symlink-install --parallel-workers 2 && colcon test --parallel-workers 1 --packages-select swarmflow_payload && colcon test-result --verbose"; scripts/lock.sh release build
```

## Definition of done
- Lead tests + all existing layoutgen and payload tests pass; `git status` shows no change under `layouts/standard`.
- `scripts/ci.sh` green on the branch rebased onto `main`.

## Inputs / contracts used
- `layouts/warehouse_v2/layout.yaml` (PL_* stations), `payload_core.py` / `payload_node.py` (patterns), api.py v2
  (ORDER_PALLET, modes LIFTING / CARRYING), `Order.msg` (order_type, pallet_id).
