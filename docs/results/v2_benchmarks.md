# v2 benchmarks (E-01, E-02, C-01)

All runs 2026-10-14, Gazebo headless, full 600 sim-s window, ground-truth localization (the AMCL default came after
these runs, I-03). Every run starts from freshly created containers: an earlier version of these tables was
contaminated by a harness bug (below) and has been replaced. Raw records in `runs/` (not committed). v1 numbers: [`v1_baseline.md`](v1_baseline.md); realism: [`v2_realism_ladder.md`](v2_realism_ladder.md).

## Throughput and safety

| Setup | Layout / scenario | Robots | Runs | Deliveries / 600 s | Orders / min / robot | Stuck / failed | Clearance violations | Near misses | Min footprint distance (m) |
|---|---|---|---|---|---|---|---|---|---|
| **v1** FCFS + aisle reservations | standard / `v1_demo` (3 orders/min) | 3 | 3 | 15.0 ± 0.0 | 0.50 | 0 / 0 | 1.7 ± 2.1 | 4.0 ± 1.0 | −0.05 ± 0.15 |
| **v2** assign + local navigation, prediction on | warehouse_v2 / `v2_demo` (≈ 4.5 orders/min, saturating) | 4 | 3 | 14.7 ± 1.5 | 0.37 | 0 / 0 | 1.0 ± 1.0 | 11.0 ± 4.6 | −0.02 ± 0.04 |
| v2, **prediction off** (current footprints only) | warehouse_v2 / `v2_demo` | 4 | 3 | 12.7 ± 3.5 | 0.32 | 1.3 / 1.3 | 1.7 ± 0.6 | 10.3 ± 6.7 | −0.07 ± 0.04 |
| v2, prediction on | warehouse_v2 / `v2_demo` | 5 | 1 | 13 | 0.26 | 2 / 2 | 9 | 27 | −0.09 |
| v2, prediction on | warehouse_v2 / `v2_steady` (demand-limited) | 4 | 3 | 8.0 ± 0.0 | — | 0 / 0 | 0 | 1.3 ± 0.6 | +0.15 ± 0.07 |

Clearance violation = the padded footprints (0.70 × 0.60 m, 5 cm per side) overlap; the robot bodies touch only below
about −0.10 m. No run reached that.

**Does v2 beat v1?** Not on deliveries per robot, and the two are not the same job: v2 orders have 1–3 pick points
plus a dock 20–30 m away on a 20 × 32 m building (v1: one pickup and one drop on a 22 × 12 m floor), and about 1 in 9
v2 orders is a pallet move with a 10 s unload. What v2 does better: no aisle holds (waiting per robot 21–31 s vs 55 s
in v1), and physical contact never occurred while v1's footprints overlapped at station exits in 2 of 3 runs.
**A fifth robot does not help on this layout:** one run delivered 13 (vs 14.7 with 4), with 2 robots stuck and 9
padded-footprint overlaps: the docks and the central aisle are the bottleneck, not the number of robots.

**± prediction (N-05):** with the other robots' 2 s predicted positions in the costmaps, all 3 runs delivered 13–16
orders with no stuck robot; without, one of 3 runs got 4 robots stuck (9 deliveries, 4 failed orders), and the
padded footprints came closer in every run (−0.07 vs −0.02 m mean). Near misses are about equal. The effect is
modest with n = 3, but every safety number points the same way; prediction stays on.

**v1 reservations on the v2 layout** (planned comparison) does not apply: the v2 layout has no reservation zones by
design (orchestrator only assigns, user decision 3).

## E-02: demo runs

The three `v2_demo` prediction-on runs above are the 10-minute demos with 4 robots: 13, 15, 16 deliveries, 0 stuck,
0 failed, no physical contact (deepest padded overlap −5.4 cm). Pallet orders run with physical pallets (M16).

## C-01: CPU per container (% of one core, mean of 30 s samples)

foxglove_bridge rises from ~10 to ~31 % while a Foxglove client is connected (the reruns had one open).

| Container | 4 robots (`v2_demo`) | 5 robots (`v2_demo`) | 4 robots, AMCL rung |
|---|---|---|---|
| gazebo | 198–210 | 190 | 200–208 |
| each robot (Nav2 + agent + bridge) | 99–125 | 104–110 | 109–146 |
| viz | 67–71 | 73 | 70–73 |
| orchestrator | 45–47 | 43 | 51 |
| scenario_engine | 17–18 | 16 | 20 |
| foxglove_bridge | 10–31 | 33 | 31–32 |
| **total** | ≈ 8.0 cores | ≈ 8.9 cores | ≈ 8.5 cores |

Each robot costs ~1.1–1.2 cores (Nav2 ~0.8, robot agent ~0.35 in Python); the world size adds little. Light mode
(T025/T036): 3 robots 0.32 core, 8 robots on `v2_demo` 0.59 core (300 sim-s at rtf 1).

## Known limits

- viz stays at ~0.65 core (target 0.3, X-01): needs a C++ TF/scan relay (package choice in `docs/user_todo.md`).
- AMCL has no localisation health monitor: with a degraded lidar (realism rung 4) robots got lost, and a lost robot
  is avoided where it is reported, not where it is.

## Harness bug found and fixed (T055)

`robot_4`, `robot_5` and `gazebo_gui` are in compose profiles; a plain `docker compose down` does not stop profiled
services, so `robot_4` kept running between runs with its old Nav2, AMCL and agent state. The first version of
these tables and of the realism ladder included such runs (a robot "lost 21–24 m" in the AMCL rungs, a collapsed
no-prediction run, a five-robot run at 19 deliveries). Every run script now stops all profiles and recreates every
container (`up --force-recreate`), each run's `health.txt` shows equal container uptimes, and the 14 affected runs
were redone under the same ids.
