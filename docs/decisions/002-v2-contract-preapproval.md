# 002 — v2 contract changes pre-approved (gate G6)

- **Date:** 2026-10-11 · **Decided by:** the user, in chat ("Pre-approve now"; run all of improvements §9, steps 0–7)
- **Decision:** the lead applies the v2 contract changes below on one branch, `lead/contract-v2` (milestone M12),
  without stopping for approval, and logs the exact diff for the user to review afterwards.
- **Approved changes** (from [`improvements_v2.md`](../improvements_v2.md) L-07, O-01, O-04, F-06 and §13):
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
