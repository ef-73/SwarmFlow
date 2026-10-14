# v1 baseline on the standard layout (X-10, also the M12 v1 regression run)

What v2 has to beat. Code: `main` at `a40c15d` (v1 behaviour; contract-v2 additions inactive; joint states at 10 Hz,
X-01). Scenario `v1_demo` (seed 1, 3 robots, Poisson 3 orders/min, FCFS + aisle reservations), Gazebo headless,
ground-truth localization, 3 runs of the full 600 sim-s window (`tests/integration/batch_runs.sh v1_baseline_b 3`).
Run 2026-10-14; raw records in `runs/v1_baseline_b-{1,2,3}/` (not committed).

| Metric | run 1 | run 2 | run 3 | mean ± sd |
|---|---|---|---|---|
| Deliveries / 600 sim-s | 15 | 15 | 15 | **15.0 ± 0.0** |
| Throughput (orders / sim-min) | 1.50 | 1.50 | 1.50 | 1.50 ± 0.00 |
| Mean order latency (s, queued → delivered) | 213.7 | 212.9 | 209.9 | 212.2 ± 2.0 |
| Mean waiting per robot (s) | 56.6 | 53.5 | 54.6 | 54.9 ± 1.6 |
| Stuck events / failed orders | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| Clearance violations (padded footprints overlap) | 4 | 1 | 0 | 1.7 ± 2.1 |
| Min footprint distance (m, padded 0.70 × 0.60) | −0.12 | −0.15 | +0.13 | −0.05 ± 0.15 |
| Near misses (< 0.30 m episodes) | 4 | 5 | 3 | 4.0 ± 1.0 |
| "Control loop missed" warnings | 0 | 35 | 43 | — |

**M12 regression gate:** ≥ 15 deliveries / 10 min, 0 stuck → **passed** in all 3 runs.

**Clearance violations:** every overlap is robot_1 / robot_2 at the D1 station exit in the east main aisle around
(18, 4), outside any reservation zone; the padded footprints (5 cm per side) overlap by up to 0.17 m, so the bodies
come within a few cm or touch. This is v1 behaviour (local avoidance only, no rule at station exits) and is one of
the things v2 local navigation (M14) must fix.

**CPU (mean of 30 s `docker stats` samples, % of one core):**

| gazebo | robot_N (each) | viz | orchestrator | payload | scenario_engine | foxglove_bridge |
|---|---|---|---|---|---|---|
| 190–228 | 134–155 | 70–72 | 51–59 | 34–36 | 19–22 | 11–20 |

An idle robot container alone uses ~1.25 cores (Nav2 container ~0.8, robot agent ~0.36, bridge ~0.18; measured
with `top` in the container). viz stays at ~0.7 core after the joint-state fix: the cost is per-message rclpy work in
`tf_relay` (~0.34) and `viz_node` (~0.23), not the wheel transforms (X-01 open, see log).

**Excluded:** a first batch (`v1_baseline-*`, 2026-10-13) stopped on wall time while the sim ran at 0.65–0.8 × real
time and shared the CPU with two test containers; it is why `demo_run.sh` now runs until the sim window is covered.
