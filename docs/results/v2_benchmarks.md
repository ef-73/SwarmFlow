# v2 benchmarks (E-01, E-02, C-01)

All runs 2026-10-14, Gazebo headless, full 600 sim-s window, ground-truth localization unless noted. Raw records in
`runs/` (not committed). v1 numbers: [`v1_baseline.md`](v1_baseline.md); realism: [`v2_realism_ladder.md`](v2_realism_ladder.md).

## Throughput and safety

| Setup | Layout / scenario | Robots | Runs | Deliveries / 600 s | Orders / min / robot | Stuck / failed | Clearance violations | Near misses | Min footprint distance (m) |
|---|---|---|---|---|---|---|---|---|---|
| **v1** FCFS + aisle reservations | standard / `v1_demo` (3 orders/min) | 3 | 3 | 15.0 ± 0.0 | 0.50 | 0 / 0 | 1.7 ± 2.1 | 4.0 ± 1.0 | −0.05 ± 0.15 |
| **v2** assign + local navigation, prediction on | warehouse_v2 / `v2_demo` (≈ 4.5 orders/min, saturating) | 4 | 3 | 12.7 ± 0.6 | 0.32 | 0 / 0 | 0.7 ± 1.2 | 6.3 ± 3.5 | +0.02 ± 0.07 |
| v2, **prediction off** (current footprints only) | warehouse_v2 / `v2_demo` | 4 | 3 | 10.0 ± 5.2 | 0.25 | 1.3 / 1.3 | 1.3 ± 0.6 | 3.3 ± 2.3 | −0.07 ± 0.03 |
| v2, prediction on | warehouse_v2 / `v2_demo` | 5 | 1 | 19 | 0.38 | 1 / 1 | 1 | 12 | −0.02 |
| v2, prediction on | warehouse_v2 / `v2_steady` (demand-limited) | 4 | 3 | 7.0 ± 1.0 | — | 0 / 0 | 0 | 1.0 ± 1.0 | +0.17 ± 0.13 |

Clearance violation = the padded footprints (0.70 × 0.60 m, 5 cm per side) overlap; the robot bodies touch only below
about −0.10 m. No run reached that.

**Does v2 beat v1?** Not on deliveries per robot, and the two are not the same job: v2 orders have 1–3 pick points
plus a dock 20–30 m away on a 20 × 32 m building (v1: one pickup and one drop on a 22 × 12 m floor), and about 1 in 9
v2 orders is a pallet move with a 10 s unload. What v2 does better: no aisle holds (waiting per robot 12–31 s vs 55 s
in v1), physical contact never occurred while v1's footprints overlapped at station exits in 2 of 3 runs, and a fifth robot
raised throughput by 50 % (12.7 → 19), with one robot stuck once.

**± prediction (N-05):** with the other robots' 2 s predicted positions in the costmaps, all 3 runs delivered 12–13
orders with no stuck robot; without, one of 3 runs collapsed (4 deliveries, 4 robots stuck and their orders failed)
and footprint overlaps doubled. Fewer near misses without prediction are an artefact of the collapsed run (less
traffic). Prediction stays on.

**v1 reservations on the v2 layout** (planned comparison) does not apply: the v2 layout has no reservation zones by
design (orchestrator only assigns, user decision 3).

## E-02: demo runs

The three `v2_demo` prediction-on runs above are the 10-minute demos with 4 robots: 12, 13, 13 deliveries, 0 stuck,
0 failed, no physical contact (deepest padded overlap −5.4 cm). Pallet orders run with physical pallets (M16).

## C-01: CPU per container (% of one core, mean of 30 s samples)

| Container | 4 robots (`v2_demo`) | 5 robots (`v2_demo`) | 4 robots, AMCL rung |
|---|---|---|---|
| gazebo | 198–210 | 190 | 200–208 |
| each robot (Nav2 + agent + bridge) | 99–125 | 104–110 | 109–146 |
| viz | 62–66 | 63 | 68–70 |
| orchestrator | 45–47 | 43 | 51 |
| scenario_engine | 17–18 | 16 | 20 |
| foxglove_bridge | 10–11 | 9 | 11 |
| **total** | ≈ 8.0 cores | ≈ 8.9 cores | ≈ 8.5 cores |

Each robot costs ~1.1–1.2 cores (Nav2 ~0.8, robot agent ~0.35 in Python); the world size adds little. Light mode
(T025/T036): 3 robots 0.32 core, 8 robots on `v2_demo` 0.59 core (300 sim-s at rtf 1).

## Known limits

- viz stays at ~0.65 core (target 0.3, X-01): needs a C++ TF/scan relay (package choice in `docs/user_todo.md`).
- A lost AMCL robot is invisible to the others (realism ladder); ground truth stays the default.
