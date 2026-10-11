---
id: T026
title: v2 warehouse layout (two blocks, central aisle, docks with queues, homes, pallets) + generator support
workstream: F
status: claimed
claimed_by: "sf-implementer"
branch: ws-f/T026-warehouse-v2
model: sonnet
risk: normal
---

## Goal
`layouts/warehouse_v2/layout.yaml` and its generated files exist, built by `tools/layoutgen`, and match the v2
layout decided with the user (improvements_v2 §1, L-01 … L-06). The `standard` layout is unchanged byte for byte.

## Geometry (lead decision; map frame x east, y north, metres)
- Bounds x 0–20, y 0–32 (walls outside). `schema_version: 2`, `name: warehouse_v2`, `resolution_m: 0.05`,
  `zones: []`, `holds: []`.
- Racks (height 2.0): west block `R_W1`…`R_W5` x 0–8, east block `R_E1`…`R_E5` x 12–20 (touching the outer walls, so
  storage aisles are dead ends). Row r (1…5): y_min = 8.0 + 3.2 (r−1), y_max = y_min + 1.0. Central aisle x 8–12.
- Storage aisle a (1…4) between rows a and a+1. Pick points `PK_<W|E><a>_<S|N>_<ss>`: face S at y = aisle y_min +
  0.45, face N at y = aisle y_max − 0.45; slots ss = 01…04 at x = 7, 5, 3, 1 (W) and 13, 15, 17, 19 (E) (01 nearest
  the central aisle); yaw π (W) / 0 (E).
- Docks `DK1`…`DK4` at x = 2.0, 6.5, 13.5, 18.0, y = 30.6, yaw π/2. Queue spots `Q_DK<k>_<i>`, i = 1…3, same x,
  y = 30.6 − 1.6 i, `queue_of: DK<k>`, `queue_index: i`, yaw π/2. Exit corridors (southbound, between dock lanes)
  at x = 4.25 and x = 15.75, plus the central x 8–12.
- Home slots `HM1`…`HM8` at y = 1.5, x = 1.5 + 2.4 (k−1), yaw π/2. Spawn robot_1…robot_5 at HM1…HM5.
- Pallet stands `PL_W1` (2.5, 5.5), `PL_W2` (5.5, 5.5), `PL_E1` (14.5, 5.5), `PL_E2` (17.5, 5.5), yaw π/2.
- Keep-right `aisles`: `central` (x 8–12, y 3–21.8, axis y); one per storage aisle (`W1`…`E4`, axis x, the aisle
  rectangle); `cross_north` (x 0–20, y 21.8–25.0, axis x); `cross_south` (x 0–20, y 0–3.0, axis x).
- Route graph (travel estimates and goal names only; robots plan their own paths): intersections `X_i_j` along the
  central aisle centre (x = 10) at every aisle mouth, at the pallet area, home strip and north cross aisle; aisle
  mouths at x = 7.5 (W) / 12.5 (E) on the aisle centre line; every pick point linked to its aisle's mouth chain and
  to its neighbours in the same aisle; docks, queue spots, exit corridors, homes, pallets linked so that every
  station reaches every other (bidirectional lanes, `clear_width_m` = the real corridor width).

## Generator work (tools/layoutgen)
- Accept the v2 station types and `aisles`: graph kinds, vertex order (nav_graph.md), `is_parking_spot` /
  `is_charger` on `HM*`, sidecar `schema_version: 2` for v2 layouts (1 for v1, so `standard` is unchanged) with
  `aisles` and `queue_of` / `queue_index`, world colours for the new types (floor markers; pallet *models* come in
  M16), `sim2d.json` spawn table.
- `validate_semantics`: `queue_of` names an existing dock; queue name equals `Q_<dock>_<queue_index>`.
- Optional: author `layout.yaml` with a small builder script under `tools/layoutgen/` (committed), not by hand.

## Owned files
- layouts/warehouse_v2/**
- tools/layoutgen/** (except the lead tests below)

## Lead tests (do not edit)
- tools/layoutgen/tests/test_warehouse_v2.py

## Forbidden
- AGENTS.md §4; no Gazebo; no edits to `layouts/schema/**` (contract) or `layouts/standard/**`.

## Commands to run
```bash
export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH" COMPOSE_PROJECT_NAME=swarmflow-t026 SWARMFLOW_GUI=none
docker compose run --rm --no-deps dev bash -c "python3 tools/layoutgen/generate.py layouts/warehouse_v2/layout.yaml --out layouts/warehouse_v2/generated && python3 tools/layoutgen/generate.py layouts/standard/layout.yaml --out layouts/standard/generated && git status --short layouts && python3 -m pytest -q tools/layoutgen tests/unit"
```
(no colcon build needed; no build lock needed for this pure-Python run)

## Definition of done
- All layoutgen tests (lead and existing) and `tests/unit` pass; `layouts/standard/generated` unchanged.
- `layouts/warehouse_v2/generated/*` committed and fresh; map.pgm < 1 MB.
- `scripts/ci.sh` green on the branch rebased onto `main`.

## Inputs / contracts used
- docs/improvements_v2.md §1; layouts/schema/* (v2, decision 002); design §6.1, §7.2;
  `src/swarmflow_core/swarmflow_core/graph.py` (loader, read-only).
