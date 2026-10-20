# v2 benchmarks (E-01, E-02, C-01)

All runs 2026-10-14, Gazebo headless, full 600 sim-s window, ground-truth localization (I-03: AMCL
was tried as the default and reverted, see the realism ladder). Every run starts from freshly created containers: an earlier version of these tables was
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
| viz (Python TF relay) | 67–71 | 73 | 70–73 |
| viz (C++ relay, T054) | 35–39 | — | 35 |
| orchestrator | 45–47 | 43 | 51 |
| scenario_engine | 17–18 | 16 | 20 |
| foxglove_bridge | 10–31 | 33 | 31–32 |
| **total** | ≈ 8.0 cores | ≈ 8.9 cores | ≈ 8.5 cores |

Each robot costs ~1.1–1.2 cores (Nav2 ~0.8, robot agent ~0.35 in Python); the world size adds little. Light mode
(T025/T036): 3 robots 0.32 core, 8 robots on `v2_demo` 0.59 core (300 sim-s at rtf 1).

## Known limits

- viz: see "v2.1" below for the X-01b result.
- AMCL has no localisation health monitor: with a degraded lidar (realism rung 4) robots got lost, and a lost robot
  is avoided where it is reported, not where it is.

## Harness bug found and fixed (T055)

`robot_4`, `robot_5` and `gazebo_gui` are in compose profiles; a plain `docker compose down` does not stop profiled
services, so `robot_4` kept running between runs with its old Nav2, AMCL and agent state. The first version of
these tables and of the realism ladder included such runs (a robot "lost 21–24 m" in the AMCL rungs, a collapsed
no-prediction run, a five-robot run at 19 deliveries). Every run script now stops all profiles and recreates every
container (`up --force-recreate`), each run's `health.txt` shows equal container uptimes, and the 14 affected runs
were redone under the same ids.

## v2.1 anti-jamming (M18, 2026-10-15)

**Decision rule** (fixed before the first run; keep a change only if all hold, against the baseline): stuck episodes
per run at least 25 % lower (baseline mean 8.25 → ≤ 6.2); mean deliveries ≥ baseline mean − 1 (15.0 → ≥ 14.0); no
contact (min footprint distance > −0.10 m; baseline worst −0.054); clearance violations ≤ baseline mean + 1
(1.75 → ≤ 2.75). n = 3 per variant, so differences smaller than the run-to-run spread are reported as such. A losing
change stays available as a parameter; the default does not change.

All runs `v2_demo`, 4 robots, ground truth,
10 sim-min, fresh containers, current images; `stuck episodes` = entries into `STUCK` (new metric, T059). A run counts
only if it covered the whole 600 s window (the harness stops at a wall-time cap when the sim runs slower than real
time); excluded runs are listed.

| Variant | Runs | Deliveries | Stuck episodes / run (where) | Stuck → failed | Clearance violations | Near misses | Min footprint distance | Robot CPU |
|---|---|---|---|---|---|---|---|---|
| Baseline (v2.0 defaults, 2026-10-14) | 4 | 13 / 15 / 16 / 16 → **15.0** | 17 / 8 / 4 / 4 → **8.25** (27 of 33 in storage aisles) | 0 | 1.75 | 11.5 | −0.054 | 115–126 % |
| J-01 5 s look-ahead, fading cost | 3 | 15 / 11 / 13 → **13.0** | 9 / 13 / 18 → **13.3** (storage 21, docks 9, central 10) | 1.0 | 1.67 | 11.3 | **−0.104** | 118–120 % |
| J-02 footprint-aware MPPI | 1 (stopped) | **4** | **36** (31 in storage aisles) | 5 | 1 | 7 | −0.007 | 121 % |
| J-04 prediction along plans | 2 | 15 / 12 → **13.5** | 11 / 21 → **16.0** (storage 28) | 0.5 | 1.0 | 9.5 | −0.095 | 123–125 % |

Decision rule (stuck −25 %, deliveries ≥ 14.0, min distance > −0.10 m, clearance ≤ 2.75): **no variant passes; the
v2.0 navigation defaults stay.** All three remain switchable (README table) for further work.

- **J-01** (5 s look-ahead). At 0.6 m/s, 5 s of straight-line prediction lays cost 3 m ahead of every moving robot;
  footprints of several robots overlap into walls even with the fade to cost 20, and robots stopped for robots they
  would never have met (one run had 10 stuck episodes in the 4 m central aisle; the baseline had 1 in 33). The 2 s
  horizon stays.
- **J-02** (footprint-aware controller). 21 of the 33 baseline episodes are a robot inside a storage aisle turned across
  it, e.g. leaving a pick point nose-first toward the opposite rack. Scoring the whole footprint was meant to stop that,
  but pick points put the robot's side 0.15 m from the rack, inside Nav2's inscribed-inflation band (cost 253), so with
  the footprint scored every pose at or near a pick point looks like a near-collision: 4 deliveries, 36 episodes, and
  the two remaining runs were cancelled. The isolated scenarios (`pick_turn`, `pick_turn_pair`, new) passed 3/3 with
  and without it in ~20 s, and head_on / pass_parked / crossing / dead_end kept their M14 results with it on (dead_end
  closest 7.3 cm, M14 6.6 cm). The real fix is geometric: pick points ~0.1 m further from the racks (layout generator,
  WS-F), then footprint scoring can be retried.
- **J-04** (prediction along each robot's published plan). Works as designed (unit-tested; each plan has 7
  subscribers in a 4-robot run) but did not reduce stuck episodes: most episodes are robots stalling near racks, which
  better prediction of other robots does not address.
- J-03 (combining the winners) was not run: there were no winners.

**Excluded runs.** `j04_2` (covered 253 of 600 s: the sim slowed to ~0.4 real-time factor), `ctl_1` and `ctl_2`
(two control runs of the unchanged defaults at the end of the session: 489 and 440 s covered, robot CPU 142–149 %).
`j02_2` was stopped in its first minutes and deleted.

**Finding: stuck episodes depend strongly on CPU pressure.** The same code (`ab2bb53`) on the same images gave 9–21
stuck episodes per run at 118–125 % CPU per robot and 23 and 49 at 142–149 %, as the laptop's load rose over the
afternoon (Gazebo, four Nav2 stacks and other desktop applications saturate all 16 threads: processor utility
104 %). When Nav2 misses control deadlines, robots hesitate and the stuck timer fires. Comparisons on this machine are
only fair between runs at the same load: each run's `cpu_mean.txt` is the check. A comparison study should interleave
variants and controls or run fewer robots per sim.

**X-01b (viz CPU).** Profiling `viz_node` (cProfile, 180 s of a 4-robot `v2_demo`) showed most of its time in
deserializing the robots' Nav2 plans (six Python objects per pose) and in rebuilding the static scene every 10 s. The
node now subscribes to plans raw and reads x, y straight from the CDR bytes (0.71 ms vs 11.1 ms for a 400-pose plan,
tested against rclpy's own deserialization) and builds the scene once (T062). Check run `x01b_1` (600 s, 4 robots):
**viz 0.36 core**, at the same high host load as `ctl_1`/`ctl_2` (robot CPU ~148 %), where the old node used 0.39 and
0.50. The 0.30 target is **not met**; what remains is rclpy's executor overhead and the C++ TF/scan relay in the same
container. (`x01b_1` itself covered 427 of 600 sim-s: 10 deliveries, 42 stuck episodes, the overloaded host again; not counted above. CPU is sampled over wall time, so the viz figure still holds.)

## Dock line: places claimed on arrival (2026-10-19)

**Change.** A robot used to reserve its dock or queue spot the moment its order was assigned, so places in a dock's
line went by dispatch order. Now its route ends at the lane entry in front of the dock's queue (`X_1_5` for DK1), and
it claims the dock (if free) or the lowest free queue spot only when its picks are done and it is within 3 m of that
entry, in order of arrival (`FleetCoreV2._claim_line`). Capacity is unchanged (dock + 3 queue spots per dock), so an
arriving robot always finds a place. Every claim is checked by `tools/metrics/dock_line_check.py`.

**Agent bug found on the way.** When the agent sends the next goal to Nav2 just as the previous one finishes, Nav2 can
report the *new* goal as succeeded while the robot is still at the old one. The agent skipped its last goal and ended
the task at its last pick; the old fleet core then counted the order as delivered 20+ m from its dock. The agent now
treats a Nav2 success more than 1 m from the goal as spurious and re-sends the goal. Every earlier v2 run had 1–2 such
orders; the v2 rows below are recounted from the raw runs, counting only orders unloaded at their dock.

| Setup (v2_demo, 4 robots, 600 s) | Runs | Deliveries at the dock | Counted before but not at the dock | Stuck / failed | Min footprint distance |
|---|---|---|---|---|---|
| v2.0, prediction on (`e01_pred_r1_*`) | 3 | 11 / 14 / 15 → **13.3 ± 2.1** (published 14.7) | 2 / 1 / 1 | 0 / 0 | −0.02 ± 0.04 m |
| v2.0, prediction off (`e01_nopred_r1_*`) | 3 | 11 / 9 / 13 → **11.0 ± 2.0** (published 12.7) | 2 / 0 / 3 | 1.3 / 1.3 | −0.07 ± 0.04 m |
| Dock line + agent fix (`dl_*`) | 3 | 9 / 15 / 16 → **13.3 ± 3.8** | 0 / 0 / 0 | 1.0 / 1.0 | −0.06 ± 0.05 m |

Claim check on the three `dl_*` runs: 42 claims, every one 2.9–3.4 m from its lane entry, no delivery without a
claim. Throughput is unchanged within run-to-run spread. `dl_1` had two long jams that the other two runs did not: a
robot that reached DK1 0.33 m off the dock pose after a queue advance and whose controller could not close the gap
(Nav2 needs 0.25 m; 150 s, then the order failed), and two robots nose to nose at the mouth of aisle E1 for the rest
of the run; it also had the only body contact of these runs (−0.12 m padded, 2 cm). `dl_2` and `dl_3` had 0 failed
orders and 7 and 9 stuck episodes, none at the docks. Robot CPU was 93–110 % per robot (an idle host).

**Not counted.** `dlbug_1`, `dlbug_2`: the first build, which could claim a place from a task that ended away from the
line (the agent bug above); `dlbug_2` claimed DK1 from 18 m away and jammed. `dlrace_1`: the second build, without the
agent fix: a task ended at its last pick, the robot was sent back out of a dead-end aisle and jammed at its mouth.

## Reservations, deadlock breaker, smoother motion (T066–T070, 2026-10-19/20)

**Pallets, boxes, spinning (T066).**
- **Pallets.** A pallet order now stops 1.2 m before the dock and drives straight onto the dock pad. The pallet is set
  down on the dock pose itself, centred and square; before, it landed 0.7 m past the pad, slightly turned. The truck
  takes it after 6 s (was 30 s), before the next robot in the line arrives.
- **Boxes.** Shelf orders now show their boxes in Gazebo: one box per pick, stacked on the robot's deck, taken by the
  truck when the unload ends (`ShelfBoxCore`).
- **The "spinning stuck robots".** They were robots U-turning at a dead-end pick. Turning in place did not count as
  progress, so after 10 s the agent reported STUCK. Turning towards the goal now counts as progress. STUCK episodes
  with more than 90° of turning per run: before (`dl_*`) 3 / 2 / 3; after, all off (`o069_*`) 0 / 0 / 0. The U-turns
  themselves remain, because aisles are dead ends (turn-on-the-spot episodes 9.3 per run before, 10.3 after).

**Pick points moved away from the racks: reverted.**
- Moving pick points 0.75 m from the rack face (`b066_1` first build: 5 deliveries) or 0.6 m (`pick06_1`: 9) left a
  passing robot too little room in the 2.2 m aisles: two robots in one aisle could not pass each other.
- Back to 0.45 m.

**Stop-in-place yield (T067): not adopted.**
- The rule: when another robot comes closer than 1 m within 4 s at constant velocity, the later robot stops where it
  is.
- v1: 15 / 15 / 10 deliveries, 0 / 0 / 3 failed.
- v2 (no yield inside aisles): 10 / 16, 4 / 0 failed. In `y067v1_3` and `y167_1` it fired too late and stopped the
  robot *in* the other robot's path. Nav2 cannot route around a robot 0.8 m ahead, so both stayed blocked for 5 min.

**Reserved trajectories + deadlock breaker (T068).** Built on the literature:
- **Reactive avoidance deadlocks in corridors.** ORCA-style planners stall when agents meet head-on in a corridor
  ([AAAI SoCS paper](https://ojs.aaai.org/index.php/SOCS/article/download/43087/50647/47192),
  [Robotica](https://www.cambridge.org/core/journals/robotica/article/addressing-the-deadlock-problem-of-the-optimal-reciprocal-collision-avoidance-algorithm-in-distributed-multirobot-navigation/70CEA27CFAE13D30DD3ABCD189F9019D)).
- **Space-time reservations with priority to the earlier reservation**
  ([WHCA*](https://w9-pathfinding.readthedocs.io/latest/mapf/WHCAStar.html)).
- **Lower priority waits out of the way**, as in decentralised AGV warehouse control
  ([Draganjac et al., IEEE TASE 2016](https://researchgate.net/profile/Damjan-Miklic/publication/309056924_Decentralized_Control_of_Multi-AGV_Systems_in_Autonomous_Warehousing_Applications/links/5be55f684585150b2ba910b3/Decentralized-Control-of-Multi-AGV-Systems-in-Autonomous-Warehousing-Applications.pdf)).
- **Confined bidirectional lanes are where schedule negotiation struggles**
  ([Open-RMF maintainers](https://discourse.openrobotics.org/t/scalability-of-openrmf-for-large-multi-robot-deployments-in-dynamic-environments-e-g-airports/53697)).

How it works:
- **Reservations.**
  - Every robot publishes its Nav2 plan timed at 0.5 m/s (0.5 s samples, 15 s) on `/fleet/trajectories`, stamped with
    the time it started its current goal.
  - A robot yields only when its path meets a higher-priority robot's reserved path at a similar time (closer than
    1 m, within 1.5 s plus 15 % of the look-ahead, conflict within 8 s). Paths that cross at different times are no
    conflict.
  - Priority: pallet first, then the older reservation, then robot id. This is a total order, so waiting can never
    form a cycle, for any number of robots.
  - The robot waits at the last point of its own path that is 1.3 m clear of the other's reservation and outside
    storage aisles. With no such point it does not stop.
- **Deadlock breaker.**
  - Every robot builds the same wait-for graph from what all robots publish: yield edges, plus "blocked behind the
    nearest robot in front". Each robot waits for at most one other, so every cycle (2, 3 or more robots) is found by
    following edges.
  - A cycle that lasts 8 s is broken one member at a time, lowest priority first, then one every 10 s.
  - The acting robot backs up 0.6 m along its own recent track (space it just drove through), only if no robot is
    there or behind it; otherwise it skips its turn and the next member acts.
  - Nav2's BackUp behaviour was tried first and refused every time: its costmap check collides with the other robot's
    padded mark when two robots are jammed nose to nose (`r068v0_1`).

**Smoother motion (T069).**
- Realistic acceleration limits in MPPI and the velocity smoother: 1.5 m/s² and 2.5 rad/s² (were 3.0 / 3.5 and
  2.5 / 3.2).
- MPPI turn-rate sampling noise 0.3 (was 0.4).
- Measured from every robot's wheel odometry at 20 Hz (`tests/integration/smooth_probe.py`).
- The first profile also added Nav2's TwirlingCritic and a stronger path smoother (`s069v0_1`: 0 deliveries). The
  critic penalises the turn rate itself (it is meant for omnidirectional robots), so robots could not U-turn at aisle
  ends and cut rack corners. Both were dropped.

**Results.** `v2_demo`, 600 s, fresh containers. All runs below have the smoothness probe running in robot_1's
container. It costs about 25 % of a core, which matters: the earlier all-off runs without it (`b066_*`) gave
16 / 13 / 15. So a new all-off control was run at the same load (`o069_*`).

| Setup | Robots | Deliveries / 10 min | Failed orders | Contacts (< −0.10 m) | RMS heading accel (rad/s²) | RMS accel (m/s²) | Steering reversals / m |
|---|---|---|---|---|---|---|---|
| All off (`o069_*`) | 4 | 11 / 15 / 8 → 11.3 ± 3.5 | 1 / 0 / 5 | 0 | 0.38 | 0.18 | 0.45 |
| Breaker (`u068_*`) | 4 | 11 / 13 / 12 → 12.0 ± 1.0 | 1 / 0 / 1 | 1 | 0.40 | 0.20 | 0.48 |
| Reservations + breaker (`r068_*`) | 4 | 15 / 11 / 11 → 12.3 ± 2.3 | 0 / 1 / 1 | 1 | 0.39 | 0.20 | 0.47 |
| Smoothing + breaker (`s069_*`) | 4 | 13 / 13 / 14 → **13.3 ± 0.6** | 0 / 0 / 0 | 1 | **0.30** | **0.15** | **0.42** |
| All off (`f066_*`) | 5 | 8 / 14 → 11.0 | 4 / 3 | 0 | 0.42 | 0.22 | 0.49 |
| Reservations + breaker (`g068_*`) | 5 | 17 / 17 → **17.0** | **0 / 0** | 0 | 0.40 | 0.22 | 0.46 |

- **Breaker.** In `u068_2` it found 3 cycles and resolved each one; in two of them the lowest-ranked robot correctly
  skipped (no safe way back) and the next one backed out.
- **Yields.** 10–19 per run, about 9 s each.
- **The three contacts.**
  - `u068_2`: two robots had been jammed at −0.104 m for 14 s before the breaker acted. Its reverse drive deepened
    that by 6 mm while straightening, then cleared to +0.55 m.
  - `r068_3` (−0.114 m) and `s069_2` (−0.101 m) happened in ordinary driving at walking pace, with neither feature
    acting.

**Decision.**
- The rule fixed before the runs (mean deliveries ≥ control − 1, failed orders, permanent jams and contacts not
  higher) is met by reservations + breaker with 5 robots. With 4 robots, every setting fails only on contacts: 1 vs 0
  in 3 runs, none caused by the features.
- The project owner adopted all three as defaults on these numbers: `SWARMFLOW_YIELD`, `SWARMFLOW_UNJAM` and
  `SWARMFLOW_SMOOTH` are on unless set to `0`.

**Not counted.**
- `b066_1` / `pick06_1`: pick-distance builds, reverted.
- `y067v1_*`, `y167_*`: stop-in-place yield (above).
- `r068v0_1`, `u068v0_1`: the breaker build with Nav2 BackUp, which never moved.
- `s069v0_1`: the first smoothing profile.
