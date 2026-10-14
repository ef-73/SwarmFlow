---
id: T047
title: The truck takes a delivered pallet away (pallet leaves the dock after TRUCK_AFTER_S)
workstream: F
status: claimed
claimed_by: "sf-implementer"
branch: ws-f/T047-truck-takes-pallet
model: sonnet
risk: low
---

## Goal
A pallet set down at a truck dock stays there for ever today, so the next pallet (or a shelf robot) for that dock
runs into it. In a warehouse the truck loads it. After `TRUCK_AFTER_S` (30 s sim) the pallet moves to an off-map
pool, like the v1 packages, and never moves again.

## Design (lead)
- `pallet_core.py`: module constant `TRUCK_AFTER_S = 30.0`; `PalletCore(spots, pool_origin=(-3.0, 0.0))` (keyword
  argument with that default; existing calls keep working). Remember the set-down time per pallet; in `tick(t)`
  when `t >= set_down_t + TRUCK_AFTER_S` report the pool pose `(pool_origin.x, pool_origin.y + 1.5 * k, 0.0, 0.0)`
  once (k = 0, 1, 2 … in the order pallets are taken). Carried pallets are never taken.
- `payload_node.py`: pass a pool origin west of the layout bounds (x_min − 3.0, y_min), as packages use.

## Owned files
- src/swarmflow_payload/swarmflow_payload/pallet_core.py
- src/swarmflow_payload/swarmflow_payload/payload_node.py
- src/swarmflow_payload/test/test_pallet_core_extra.py

## Lead tests (do not edit)
- src/swarmflow_payload/test/test_pallet_truck.py
- src/swarmflow_payload/test/test_pallet_core.py

## Forbidden
- AGENTS.md section 4; no Gazebo; v1 package behaviour unchanged.

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t047 SWARMFLOW_GUI=none
docker compose run --rm --no-deps dev bash -c "cd src/swarmflow_payload && PYTHONPATH=.:../swarmflow_core python3 -m pytest -q test/test_pallet_truck.py test/test_pallet_core.py test/test_pallet_core_extra.py test/test_payload_core.py"
```

## Definition of done
- Lead tests and existing payload tests pass; `scripts/ci.sh` green on the branch rebased onto `main`.

## Inputs / contracts used
- `pallet_core.py`, `payload_core.py` (pool pattern), `payload_node.py`.
