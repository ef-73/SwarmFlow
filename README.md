# SwarmFlow

**A multi-robot warehouse digital twin: real robot software, a simulated warehouse, and a fleet of robots that
anticipate each other instead of waiting for a traffic controller.**

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-F58113)
![Nav2](https://img.shields.io/badge/Nav2-MPPI_%2B_Smac-3E7CB1)
![Docker](https://img.shields.io/badge/Docker-one_command-2496ED)
![Python | C++](https://img.shields.io/badge/Python_%7C_C%2B%2B-19k_lines-555)
![Tests](https://img.shields.io/badge/tests-315_passing-2EA44F)
![Release](https://img.shields.io/badge/release-v2.1.0-orange)

[![Four robots in the 3D Gazebo warehouse; one drives under a pallet and lifts it](docs/evidence/swarmflow_3d_preview.gif)](docs/evidence/swarmflow_3d_demo.mp4)

*A real Gazebo run, filmed by three cameras in simulation time: four robots pick shelf orders in the aisles, and one
drives under a pallet, lifts it, carries it to a truck dock and sets it down.
**[Watch the full 1080p video](docs/evidence/swarmflow_3d_demo.mp4)** (66 s: 9 orders, one of them a pallet, delivered in
6 sim minutes, no contact).*

---

## The question

> *Can predictive traffic coordination improve warehouse throughput and reduce congestion compared with robots that
> navigate independently?*

SwarmFlow is built to answer that with measurements, not demos. Every scenario is seeded and repeatable, every run
starts from fresh containers, and every metric is computed from the recorded robot states.

## Highlights

| | |
|---|---|
| **2× throughput from coordination** | In a shared narrow corridor, coordinated robots delivered **7.0** orders vs **3.3** for independent navigation (v1, n = 3). |
| **No jams with prediction** | Robots that predict where the others will be in 2 s: **0** stuck robots and **0** failed orders in every busy 10-minute run; without prediction, one run jammed four robots at once. |
| **Zero collisions** | No physical contact in any v2 demo run (4 robots, saturating order stream). |
| **Precise pallet handling** | Robots drive under pallets, lift, carry and set them down: **8 / 8** deliveries, **0.0 cm** sideways docking error. |
| **Runs on a laptop** | `docker compose up` on Windows brings up Gazebo, 4 Nav2 robots, the fleet layer and a Foxglove dashboard. A light mode runs **8 robots in 0.6 of a CPU core**. |
| **Honest science** | Three anti-jamming ideas were tested against a decision rule fixed before the first run. None passed, and the results, including why, are published. |

## How it works

![SwarmFlow architecture: the fleet layer assigns orders; each robot runs its own agent and Nav2 and avoids the others using their shared, predicted states](docs/evidence/architecture.svg)

- **The fleet layer decides *who does what*.** Each order goes to the robot with the lowest cost: travel distance
  plus a congestion penalty for robots already heading to the same spot. Pick lists are ordered nearest-next. Each
  dock has a queue lane, and a robot claims its place in it only when it arrives at the line, so places go first
  come, first served.
- **Each robot decides *how to get there*.** Every robot stamps the others into its own costmap: their footprints
  now, and where they will be over the next 2 seconds. It keeps right in every aisle and plans around everything
  else itself. There is no central traffic control, and avoidance keeps working if the fleet layer goes down.
- **Same software as a real robot.** ROS 2 Jazzy and Nav2 (Smac planner, MPPI controller), a custom differential-drive
  chassis with a 360° lidar, physics in Gazebo Harmonic.

## Engineering highlights

Problems found and solved along the way, each with evidence in the repo:

- **Robots were invisible to each other.** Every robot's lidar sits in a slot at the same height, so a scan passes
  straight through the other robots. The fix: a custom C++ Nav2 costmap layer that draws other robots from their
  shared, predicted states.
- **Nav2 was silently ignoring its parameters.** In composed mode, the costmap sub-nodes never received their
  parameter file and ran on Nav2 defaults. Found by checking parameters inside the running stack, fixed in the launch
  file.
- **A test-harness bug skewed the benchmarks.** Docker services behind compose profiles survived `compose down` and
  carried state between runs. It was caught by comparing container uptimes; the harness was fixed and all 14 affected
  runs were redone.
- **The visualisation used too much CPU.** Profiling showed most of its time went into decoding route plans, so the
  node now reads coordinates straight from the serialized bytes: **15× faster** per plan, tested byte-for-byte
  against ROS's own decoder.
- **Stalls depend on CPU pressure.** The same code gave 4–21 stuck episodes per run at normal load and 23–49 on an
  overloaded host. Every run now records its CPU load, and only runs at matching load are compared.

## Results

![Top-down replay of a 10-minute run: four robots, 16 deliveries](docs/evidence/swarmflow_demo.gif)

*Top-down replay of a full 10-minute run at 10×, drawn from the recorded robot states: 16 deliveries, no physical
contact ([full-quality video](docs/evidence/v2_demo_replay_g7_gt_1.webm)).*

| Setup | Robots | Deliveries / 10 min | Stuck / failed | Closest approach (padded footprints) |
|---|---|---|---|---|
| v1: central reservations, standard layout | 3 | 15.0 ± 0.0 | 0 / 0 | −0.05 ± 0.15 m |
| v2: robots coordinate locally, with prediction | 4 | 14.7 ± 1.5 | 0 / 0 | −0.02 ± 0.04 m |
| v2: robots coordinate locally, without prediction | 4 | 12.7 ± 3.5 | 1.3 / 1.3 | −0.07 ± 0.04 m |

Footprints are padded 5 cm per side, so bodies touch only below −0.10 m. v2 orders are harder (1–3 picks plus a dock 20–30 m away; about 1 in 9 is a pallet move), and v2 robots wait 21–31 s
per run instead of 55 s behind v1's aisle holds. Full tables: [v2 benchmarks](docs/results/v2_benchmarks.md),
[realism ladder](docs/results/v2_realism_ladder.md) (localisation error, network delay and loss, lidar noise),
[v1 baseline](docs/results/v1_baseline.md).

**v2.1 experiments** (anti-jamming: 5 s look-ahead, footprint-aware control, prediction along each robot's plan):
none beat the baseline under the pre-registered rule. The data located the real cause: pick points sit too close to
the racks for a 0.6 m-wide robot to turn freely, which is the next change.
[Write-up](docs/results/v2_benchmarks.md#v21-anti-jamming-m18-2026-10-15).

## Run it

Windows 11 with Docker Desktop (WSL2), about 20 GB of disk. A cold build takes about 10 minutes.

```powershell
git clone https://github.com/ef-73/SwarmFlow.git
cd SwarmFlow
docker compose up
```

Then open Foxglove on `ws://localhost:8765`. To run the v2 warehouse with 4 robots, set `SWARMFLOW_LAYOUT=warehouse_v2`,
`SWARMFLOW_POLICY=assign`, `SWARMFLOW_SCENARIO=v2_demo`, `SWARMFLOW_ROBOTS=robot_1,robot_2,robot_3,robot_4` and
`SWARMFLOW_TRAFFIC_CONTROL=false`. Headless measured run:
`tests/integration/demo_run.sh <id> 10 v2_demo assign 0`.

Everything else (Gazebo window, configuration, experiments, architecture, limitations) is in the
**[full usage guide](docs/USAGE.md)**.

## How it was built

Built in 8 days, from an empty repository to v2.1, with a contract-first, test-gated process:

- **Contracts first.** Message types, the layout schema and the fleet-layer API were frozen before parallel work
  began; changes need an explicit, written decision ([decisions](docs/decisions/)).
- **A referee, not trust.** One CI script ([`scripts/ci.sh`](scripts/ci.sh)) checks frozen contracts, hygiene, the
  build and **315 tests** on every change. Nothing reaches `main` red.
- **AI-assisted.** Most code was written by AI coding agents working in parallel against tests written up front; the
  project owner set direction, made the design calls and reviewed every release.

Design: [`docs/design.md`](docs/design.md). Contributing: [`CONTRIBUTING.md`](CONTRIBUTING.md).

## Repository map

| Path | What it holds |
|---|---|
| `src/swarmflow_core/` | Fleet layer as a pure-Python library (assignment, costs, decisions); no ROS dependency |
| `src/swarmflow_nav/` | Nav2 bringup and the C++ costmap layers (other robots with prediction, keep-right lanes) |
| `src/swarmflow_robot_agent/` | Per-robot agent: goals, picks, pallet docking, stuck detection |
| `src/swarmflow_viz/`, `viz/` | Foxglove scene and dashboard, Gazebo overlay |
| `layouts/`, `tools/layoutgen/` | Warehouse layouts generated from one definition (world, map, nav graph) |
| `sim2d/` | Light mode: the same fleet logic without Gazebo |
| `tools/metrics/` | Metrics, replay data and video rendering |
| `tests/`, `scripts/ci.sh` | Fixtures, unit and integration tests, the CI referee |

## License

MIT, see [`LICENSE`](LICENSE).
