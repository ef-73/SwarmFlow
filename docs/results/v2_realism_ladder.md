# v2 realism ladder (I-01, I-03)

How much does the privileged information of the simulation help? The same scenario at four levels of realism:
`v2_steady` (seed 2: 1.6 shelf + 0.25 pallet orders/min, 4 robots, `assign` policy, v2 navigation profile with
the OtherRobotsLayer and keep-right), 3 runs of the full 600 sim-s window per rung, Gazebo headless, 2026-10-14.
Raw records `runs/ladder_r<rung>_<k>/` (not committed); table by `tools/metrics/ladder_table.py runs ladder`.

| Rung | Runs | Deliveries / 600 s | Orders / min | Mean latency (s) | Stuck | Failed | Clearance violations | Near misses (< 0.30 m) | Min footprint distance (m) | Localisation error mean / worst max (m) |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 3 | 7.00 ± 1.00 | 0.70 ± 0.10 | 92.79 ± 1.70 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.00 ± 0.00 | 1.00 ± 1.00 | 0.17 ± 0.13 | 0 (ground truth) |
| 2 | 3 | 7.67 ± 1.53 | 0.77 ± 0.15 | 89.66 ± 4.72 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.33 ± 0.58 | 1.33 ± 0.58 | 0.11 ± 0.17 | 4.06 / 23.75 |
| 3 | 3 | 8.00 ± 0.00 | 0.80 ± 0.00 | 98.12 ± 6.41 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.33 ± 0.58 | 1.67 ± 0.58 | 0.06 ± 0.09 | 2.17 / 20.91 |
| 4 | 3 | 7.00 ± 1.73 | 0.70 ± 0.17 | 83.62 ± 7.04 | 0.00 ± 0.00 | 0.00 ± 0.00 | 0.00 ± 0.00 | 1.67 ± 2.08 | 0.16 ± 0.17 | 1.37 / 39.01 |

- Rung 1: localization=ground_truth, netem=, lidar_noise=0.01, lidar_range=8.0
- Rung 2: localization=amcl, netem=, lidar_noise=0.01, lidar_range=8.0
- Rung 3: localization=amcl, netem=delay 50ms 20ms loss 2%, lidar_noise=0.01, lidar_range=8.0
- Rung 4: localization=amcl, netem=delay 50ms 20ms loss 2%, lidar_noise=0.03, lidar_range=5.0

Localisation error = |reported pose (AMCL, via the robot agent) − Gazebo ground truth| over all `/fleet/robot_states`
samples (`tests/integration/loc_error.py`). AMCL is the tuned v2 profile (`nav2_params_v2.yaml`): Nav2 defaults
lost a robot in the first 4-robot trial (1 delivery, 5 stuck, 5 failed; mean error of one robot 3.5 m), while a
single robot alone stayed within 0.03–0.38 m and its wheel odometry matched ground truth to 0.1 m over 54 m.

## What it shows

- **Throughput does not separate the rungs.** `v2_steady` is demand-limited (≈ 8 orders released in the window), so
  every rung delivers what is released: 7.0 / 7.7 / 8.0 / 7.0 orders, 0 stuck, 0 failed in all 12 runs. A
  saturating scenario would be needed to see a capacity cost.
- **Localisation is where realism bites.** With good odometry most robots stay at 0.13–0.21 m mean error (rungs 2–3)
  — already close to the 0.25 m goal tolerance. With the degraded lidar (rung 4, 3 cm noise, 5 m range) errors of
  1.0–3.8 m appear on two to four of the four robots.
- **A lost robot is a safety hazard, not a throughput loss.** In 4 of the 6 rung-2/3 runs one robot (always robot_4)
  was reported 21–24 m from its true pose for the whole run while it stood idle; deliveries were unaffected because
  it received no task. Other robots avoid a robot where it is *reported* (OtherRobotsLayer), so a lost robot is
  invisible to them. Cause not diagnosed (suspected: AMCL converging wrongly at start among the identical home
  slots, and no correction while the robot never moves).
- **Network delay and loss (rung 3: 50 ± 20 ms, 2 % loss on robot egress) changed nothing measurable** at this
  load: shared positions at 5 Hz tolerate it.

## I-03 decision: ground truth stays the default

The criterion was "switch to AMCL if its throughput is within 10 % of ground truth". Throughput is within 10 %, but
the lost-robot result makes AMCL unsafe as a default until localisation health is monitored (e.g. a robot reports
FAULT/LOCALIZATION when its AMCL covariance or scan match is bad, and the OtherRobotsLayer then stamps a large
uncertainty area). Logged as a follow-up, not built.
