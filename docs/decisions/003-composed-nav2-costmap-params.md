# 003 — Composed Nav2: costmap parameters were never applied (v1); fixed for v2

- **Date:** 2026-10-14 (M14) · **Decided by:** maintainers
- **Finding:** `robot.launch.py` attaches the Nav2 parameter file to each composable node. launch_ros turns that file
  into parameter overrides for that node only, so the **costmap sub-nodes** (`/<rid>/local_costmap/local_costmap`,
  `/<rid>/global_costmap/global_costmap`, created inside the controller and planner servers) never received theirs.
  Every v1 run since M4 used Nav2's default costmap layers (`static_layer`, `obstacle_layer`, `inflation_layer`)
  and default costmap parameters — e.g. no 0.70 × 0.60 footprint (default `robot_radius` 0.1 m), the default
  inflation, and the global obstacle layer that M8 believed it had removed. Seen in the M14 compose logs
  (`Using plugin ...`) and confirmed with `ros2 param get` after the fix.
- **Consequence for v1 results:** the v1 numbers (M5–M9, `docs/results/v1_baseline.md`) are valid as measurements of
  v1 *as it ran*, but several v1 tuning conclusions about costmaps were drawn from settings that were not active.
  The v1 clearance violations at station exits are consistent with robots that planned with a 0.1 m radius.
- **Decision:** v2 (`nav_profile` v2) passes one merged parameter file to the Nav2 container process with
  `--params-file`, which applies to every node in it, sub-nodes included. v1 keeps its launch unchanged so the
  recorded baseline stays reproducible; E-01 compares v1 *reservations* on the v2 profile anyway.
- **Not changed:** the original design notes and past run records.
