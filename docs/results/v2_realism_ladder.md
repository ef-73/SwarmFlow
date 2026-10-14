# v2 realism ladder (I-01, I-03)

How much does the privileged information of the simulation help? The same scenario at four levels of realism:
`v2_steady` (seed 2: 1.6 shelf + 0.25 pallet orders/min, 4 robots, `assign` policy, v2 navigation profile with
the OtherRobotsLayer and keep-right), 3 runs of the full 600 sim-s window per rung, Gazebo headless, 2026-10-14.
Raw records `runs/ladder_r<rung>_<k>/` (not committed); table by `tools/metrics/ladder_table.py runs ladder`.

| Rung | Runs | Deliveries / 600 s | Orders / min | Mean latency (s) | Stuck | Failed | Clearance violations | Near misses (< 0.30 m) | Min footprint distance (m) | Localisation error mean / worst max (m) |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 3 | 8.00 ± 0.00 | 0.81 ± 0.02 | 91.17 ± 3.75 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.00 ± 0.00 | 1.33 ± 0.58 | 0.15 ± 0.07 | 0 (ground truth) |
| 2 | 3 | 7.33 ± 1.15 | 0.73 ± 0.12 | 90.03 ± 4.96 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.33 ± 0.58 | 0.67 ± 0.58 | 0.12 ± 0.22 | 0.12 / 0.49 |
| 3 | 3 | 8.00 ± 0.00 | 0.80 ± 0.00 | 96.52 ± 5.87 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.33 ± 0.58 | 1.67 ± 0.58 | 0.16 ± 0.16 | 0.12 / 0.45 |
| 4 | 3 | 5.00 ± 1.73 | 0.50 ± 0.17 | 88.58 ± 6.19 | 1.67 ± 1.53 | 1.67 ± 1.53 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.37 ± 0.06 | 3.44 / 27.33 |

- Rung 1: localization=ground_truth, netem=, lidar_noise=0.01, lidar_range=8.0
- Rung 2: localization=amcl, netem=, lidar_noise=0.01, lidar_range=8.0
- Rung 3: localization=amcl, netem=delay 50ms 20ms loss 2%, lidar_noise=0.01, lidar_range=8.0
- Rung 4: localization=amcl, netem=delay 50ms 20ms loss 2%, lidar_noise=0.03, lidar_range=5.0

Localisation error = |reported pose (AMCL, via the robot agent) − Gazebo ground truth| over all `/fleet/robot_states`
samples (`tests/integration/loc_error.py`). AMCL is the tuned v2 profile (`nav2_params_v2.yaml`): Nav2 defaults
lost a robot in the first 4-robot trial (1 delivery, 5 stuck, 5 failed; mean error of one robot 3.5 m; that trial
predates the harness fix below and was not repeated), while a
single robot alone stayed within 0.03–0.38 m and its wheel odometry matched ground truth to 0.1 m over 54 m.

## What it shows

These numbers replace a first version that was contaminated by a harness bug: `robot_4` (a profiled compose
service) survived `docker compose down` and carried its AMCL and Nav2 state into the next run, which showed up as
"robot_4 lost 21–24 m". Every run now starts from recreated containers (T055); 7 of the 12 ladder runs were redone.

- **AMCL with a good lidar is as good as ground truth here.** Rungs 2–3: every robot's mean error 0.02–0.18 m, the
  worst single sample 0.49 m in all 6 runs; deliveries 7.3 and 8.0 vs 8.0 (−8 % and 0 %), 0 stuck, 0 failed.
  `v2_steady` is demand-limited (≈ 8 orders released per window), so this says "no capacity cost at this load".
- **Network delay and loss (rung 3: 50 ± 20 ms, 2 % loss on robot egress) changed nothing measurable**: shared
  positions at 5 Hz tolerate it.
- **A degraded lidar breaks AMCL (rung 4: 3 cm noise, 5 m range).** In all 3 runs two or three of the four robots
  lost their position (mean errors 2.4–12.4 m, worst 27 m); deliveries fell to 5.0 ± 1.7 with 1.7 stuck robots and
  failed orders per run. At 5 m range many poses in the long aisles see only parallel rack faces, and AMCL slides
  along them. A lost robot is also a safety hazard: the others avoid it where it is *reported* (OtherRobotsLayer).
  No contact happened (padded footprints never overlapped), mostly because lost robots stopped.

## I-03 decision: AMCL is the default on v2 layouts

The criterion was "switch to AMCL if its throughput is within 10 % of ground truth". It is (rungs 2–3), with no
localisation failure in 6 runs, so `SWARMFLOW_LOCALIZATION` now defaults to `auto`: AMCL (tuned v2 profile) on v2
layouts, ground truth on the v1 `standard` layout, whose results and nav profile were recorded with it (T056).
Ground truth stays one variable away for benchmarks that should exclude localisation.

Follow-up (logged, not built): localisation health monitoring — a robot reports FAULT/LOCALIZATION when its AMCL
covariance or scan match is bad, and the OtherRobotsLayer stamps a larger uncertainty area around it. Needed before
running with a lidar as poor as rung 4.
