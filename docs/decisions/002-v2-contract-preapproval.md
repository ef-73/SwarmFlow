# 002 — v2 contract changes pre-approved (gate G6)

- **Date:** 2026-10-11 · **Decided by:** project owner
- **Decision:** apply the v2 contract changes below on one branch, `lead/contract-v2` (milestone M12),
  with the exact diff logged for review.
- **Approved changes** (v2 plan items L-07, O-01, O-04, F-06):
  1. Layout schema: station types `pick`, `dock`, `queue`, `home`, `pallet`; zones optional; zone `capacity` may be > 1;
     a `lanes` entry for keep-right lane masks.
  2. `DispatchTask.route`: an ordered list of goal vertices (meaning only; the field stays `string[]`).
  3. `Order.msg` / `OrderSpec`: order type (shelf | pallet), 1..N pick vertices, the truck dock, pallet id; the change
     carries into `DispatchTask` (it embeds `Order`).
  4. `RobotState.msg`: `angular_speed`; modes `MODE_LIFTING`, `MODE_CARRYING`. BLOCKED is reported as `MODE_STUCK`
     with `fault_reason = "blocked"` (no new constant).
  5. `swarmflow_core/api.py` protocols and `tests/fixtures/` updated to match 1–4.
- **Not approved here (stop and ask):** any other contract change, including O-06 (deferred).
- **Checks:** `scripts/ci.sh` green on the branch; independent reviewer with no open high findings; the v1 demo with
  `traffic_control: true` still reaches the M9 level (≥ 15 deliveries / 10 min, 0 stuck).

## Exact diff landed (2026-10-11, `lead/contract-v2`, for the user's review)

Reviewed by an independent Sonnet reviewer: 0 blocker, 3 high, 5 medium, 5 low findings. All high and medium
findings were fixed on the branch (lifecycle rules for cancels, dwell by vertex kind, forward-only order states,
blocked vs v1 stuck rules, shelf edge cases, schema rules tying station names to types, a v2 order fixture); the low
ones are notes for the M13/M15 cards.

1. Layout schema: `schema_version` 1 or 2; station types `pick` (`PK_*`), `dock` (`DK*`), `queue` (`Q_DK<n>_<i>`
   with required `queue_of` / `queue_index`), `home` (`HM*`), `pallet` (`PL_*`), each name tied to its type; zone
   `capacity` ≥ 1; zones "optional" = the list may be empty (the key stays). **Naming change:** the approved "lanes
   entry for keep-right lane masks" is a top-level **`aisles`** list (axis-aligned rectangles with a travel axis and
   `keep: right`), because `lanes` already names the graph edges. Sidecar `zones.yaml` gets the same `aisles` and the
   new vertex kinds; `nav_graph.md`: home slots are RMF parking spots, v2 vertex order.
2. `DispatchTask`: `route_kind` (`ROUTE_WAYPOINTS` = v1, `ROUTE_GOALS`).
3. `Order`: `order_type` (`TYPE_TRANSPORT` / `TYPE_SHELF` / `TYPE_PALLET`), `pick_vertices`, `pallet_id`;
   `payload_type` may be `"pallet"`.
4. `RobotState`: `angular_speed`; `MODE_LIFTING` = 7, `MODE_CARRYING` = 8; BLOCKED = `MODE_STUCK` + `"BLOCKED"`.
5. `api.py`: `API_VERSION` 2; the matching enum values, code tables and dataclass fields (all with v1 defaults);
   constants `UNLOAD_DWELL_BOX_S` 5, `UNLOAD_DWELL_PALLET_S` 10, `GOAL_TOLERANCE_M` 0.25, `BLOCKED_REPORT_S` 10,
   `BLOCKED_REQUEUE_S` 30, `MAX_LOADED_RETRIES` 3, `FAULT_BLOCKED`; the v2 lifecycle in the module docstring.
   `tests/fixtures/orders_v2_sample.yaml` (shelf and pallet orders). A v2 graph fixture follows from the generated
   `warehouse_v2` layout (M13) on a second branch under this same approval.
