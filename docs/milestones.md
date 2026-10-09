# SwarmFlow — Lead milestones (autonomous execution plan)

**Status:** proposed 2026-10-09 — waiting for the user to approve the review suggestions (R1–R16 below) and
gates G0–G1. **Owner:** Claude lead agent (Opus 5.5). **Implements:** [`design.md`](design.md) v1 (§15) under
[`AGENTS.md`](../AGENTS.md).

This file is the lead's work plan: what I do on my own, what I hand to cheaper subagents, how every result is
verified, and where I must stop for the user. Status of each milestone is updated here as work lands.

---

## 1. Review of design.md + AGENTS.md (2026-10-09)

Overall the plan is sound: the scope is small, contracts come first, the risky parts (multi-robot Nav2, Docker on
Windows) are front-loaded, and there is a fallback. The issues below are things that would actually break or slow the
plan. **Blockers** stop work on this machine today.

| # | Severity | Issue | Suggested fix |
|---|---|---|---|
| R1 | **Blocker** | **Docker is not installed** on this machine (`docker` not found; no Docker Desktop install). Every build, test and sim in AGENTS.md runs in Docker. | User installs Docker Desktop (WSL2 backend, Ubuntu integration on). The 5-day clock starts after that (M1). |
| R2 | **Blocker** | **No git remote, no `gh`.** PRs and GitHub Actions CI (design §14.3, AGENTS §1) cannot exist. | Either (a) user creates a private GitHub repo + installs `gh`, or (b) run "local CI": `scripts/ci.sh` runs the same checks in the `dev` image; branches are reviewed and merged locally. Recommend (b) now, (a) whenever convenient. |
| R3 | **Blocker** | `core.autocrlf=true`: git will check out shell scripts/launch files with CRLF, which break inside Linux containers (`/bin/bash^M`). Already warned on the last commits. | Add `.gitattributes` with `* text=auto eol=lf` (and `*.ps1 eol=crlf`) before any script lands. |
| R4 | High | `docker compose up` "from the repo root" can't work: compose lives at `docker/compose.yaml`. And the `dev` service that AGENTS §6 commands use is not in the §8.3 service table. | Root `compose.yaml` that `include`s `docker/compose.yaml`; add a `dev` service under a `tools` profile so plain `up` never starts it. |
| R5 | High | **Contradiction with D1:** §5.4/§6.1 use `rmf_building_map_tools` to make the world + nav graph, but AGENTS §10 bans RMF dependencies in v1. | v1 layoutgen writes the SDF world (racks are boxes) and the nav graph in RMF's nav-graph YAML format itself; RMF tools only in v2. Copy the format from an `rmf_demos` nav graph, mark [U] until Baseline C loads it. |
| R6 | High | **`docs/agent_log.md` will conflict on every merge**: all branches append to the end of one file. | One file per entry: `docs/agent_log/<YYYY-MM-DD>T<HHMM>Z-<agent>-<task>.md`; `docs/agent_log.md` becomes the index/format doc. |
| R7 | High | **No task queue.** 24/7 agents need a way to pick work without two taking the same task. | `docs/tasks/<id>.md` task cards (AGENTS §7 format) with `status: open/claimed/done` and `claimed_by`; claiming = a commit on your branch that flips the status. Lead writes cards; only the lead assigns contract-adjacent tasks. |
| R8 | Medium | Sim lock path differs between Windows-host agents (`C:\Users\ethan\.swarmflow`) and WSL/Codex agents (`/home/<user>/.swarmflow`); `flock` doesn't exist in Git Bash. Two agents could race the `docker ps` check. | `SWARMFLOW_LOCK_DIR` = the Windows path for everyone (`/mnt/c/Users/ethan/.swarmflow` from WSL); use atomic `mkdir` as the lock. Simpler still: **only the lead runs Gazebo** (subagents never do — see §3). |
| R9 | Medium | Bind-mounting a Windows-path repo into Linux containers is slow for `colcon build` (and heats the CPU). | Named Docker volumes for `build/`, `install/`, `log/`; only `src/` is bind-mounted. Measure in M1. |
| R10 | Medium | Thermal: Gazebo + 3 Nav2 + GUI software rendering on a laptop CPU (Core Ultra 9 285H). | Cap Gazebo real-time factor (e.g. 0.5) when hot — all metrics are in sim time, so results stay valid; GUI container optional; GPU via `/dev/dxg` on the WSLg route. |
| R11 | Medium | Day 1 overloads the critical path: env spike **and** contract freeze **and** four lanes starting against unfrozen contracts. | Draft and freeze contracts *before* Day 1 (M2 can start now on paper and be build-checked as soon as Docker exists). |
| R12 | Medium | Codex cloud agents can't reach the local Docker engine, and "no installs outside Docker" is ambiguous for their own sandboxes. | Clarify: the install ban is about *the user's machine*. Codex gets only sim-free tasks whose tests run with plain `pytest` (pure-Python `swarmflow_core`, layoutgen, 2D sim). |
| R13 | Low | Pose-follower via `set_pose` service calls at 20 Hz may jitter or lag across the bridge. | Keep as planned (handoff C13), but list Gazebo's `DetachableJoint` system as the fallback **[U]** if jitter is visible on Day 3. |
| R14 | Low | AGENTS says only the user merges `main`, but the lead fast-forwarded `main` twice on request. | Add: "the lead may fast-forward `main` only when the user explicitly asks in chat, and only after local CI passes." |
| R15 | Low | User review is the real bottleneck for 6 lanes × 24/7. | WIP limit: ≤ 2 unmerged branches per workstream; the lead batches reviews with a one-paragraph summary per branch. |
| R16 | Low | Subagent model use is not specified anywhere. | Adopt §3 of this file and reference it from AGENTS.md. |

Nothing in the review changes the v1 scope or the user decisions D1/D2.

## 2. Gates (where I stop and wait for the user)

| Gate | Before | User does |
|---|---|---|
| **G0** | M0 edits land | Approves/rejects R1–R16; answers: GitHub remote yes/no; may the lead fast-forward `main` after local CI? |
| **G1** | M1 | Installs Docker Desktop (WSL2 backend), starts it once. |
| **G2** | contracts become frozen (end of M2) | Reviews the contract branch; approves the freeze. |
| **G3** | first Gazebo run with GUI (M4) | Is present/watching; confirms the GUI shows on Windows. |
| **G4** | Day-2 go/no-go (M5) | Watches the gate demo; decides GO or fallback cuts. |
| **G5** | v1 tag (M8) | Records demo video, merges, tags `v1.0.0`. |

Between gates I work without asking, inside AGENTS.md rules.

## 3. Subagent model routing and quality control

### 3.1 Which model does what

| Model | Use for | Never for |
|---|---|---|
| **Haiku 4.5** (`haiku`) — cheapest | Mechanical work with a mechanical check: transcribing `.msg/.srv/.action` files from design §6.6; package boilerplate (`package.xml`, `setup.py`, `CMakeLists.txt`) from a template I give; `.gitattributes`/`.gitignore`/README skeletons; cross-doc consistency sweeps (grep for stale terms, broken links, section refs); generating fixtures by running a script I wrote. | Logic, design choices, anything without an automatic check, reviews. |
| **Sonnet 5.5** (`sonnet`) — mid | Implementation against a written spec **and lead-written tests**: FCFS assignment, reservation authority, layoutgen, order generator, robot-agent state machine + fake Nav2 action server, 2D sim skeleton, Foxglove layout, metrics script, first drafts of Dockerfiles; independent "find the bugs" reviews of other agents' diffs and of docs. | Editing contracts, deciding interfaces, running Gazebo, judging its own work done. |
| **Opus 5.5** (me, lead) | Contracts (`api.py`, interfaces, schema), the acceptance tests every subagent must pass, the reservation safety property tests, Docker networking/GUI spike, all Gazebo/Nav2 bringup and debugging, Nav2 tuning, every final review and merge prep, design decisions, user communication. | — |
| **Fable 5.1** (`fable`) | Not used by default. Only with user OK, for a problem Opus has failed on twice. | — |

Read-only codebase searches use the `Explore` agent on `haiku`.

### 3.2 How quality is kept with cheaper models

1. **Tests before dispatch.** For every Sonnet/Haiku coding task I first commit the task card (AGENTS §7) and the
   acceptance tests on the lead branch. The subagent's job is to make *my* tests pass; it may add tests, never edit mine.
2. **Isolation.** Each subagent runs in its own git worktree (`isolation: "worktree"`) on its `ws-x/…` branch.
3. **Mechanical gate (run by me, not the subagent)** on every result:
   - `git diff --name-only` ⊆ the card's owned files; lead tests byte-identical (hash check);
   - `scripts/ci.sh` green in the `dev` image (build, tests, no-ROS-imports, contract-diff);
   - for Haiku transcription: generated interfaces compared field-by-field with design §6.6 (`ros2 interface show` output diff).
4. **Review depth by risk.**
   - Low risk (boilerplate, docs, fixtures): mechanical gate only.
   - Normal (most Sonnet code): mechanical gate + I read the diff.
   - **Safety-critical** (reservation authority, robot-agent hold/enter logic, orchestrator-down behaviour): mechanical gate +
     property tests I wrote + an *independent* Sonnet reviewer told to find bugs + my adjudication.
5. **Escalation ladder.** Fail → one retry on the same model with the exact failure output → escalate one tier
   (Haiku → Sonnet → Opus/me). No third attempt on the same tier.
6. **Boundaries.** Subagents never run Gazebo (so the sim lock only has one user: me), never touch contracts,
   `.github/`, `AGENTS.md`, or `main`. At most **3 subagents in parallel**, and only one of them building in Docker at a
   time (thermal).
7. **Logging.** I write the agent-log entry for each subagent task, including model used, attempts and verification result,
   so the user can see where cheap models worked and where they didn't.

## 4. Milestones

Day numbers refer to design §15; the 5-day clock starts at M1 (Docker available).

### M0 — Plan consolidation *(now; no Docker needed)*
- **Goal:** design.md + AGENTS.md consistent, with the approved R-items applied.
- **Work:** apply approved R1–R16 (edits to design §5.4/§6.1/§8/§14, AGENTS §4–§8); add `.gitattributes`; split the
  agent log (R6); add `docs/tasks/` card template (R7); reference §3 from AGENTS.md.
  Routing: edits by me; consistency sweep by **Haiku**; independent doc review by **Sonnet** ("find contradictions and
  under-specified items an implementing agent would hit").
- **Verify:** Haiku sweep finds no stale RMF-in-v1 terms, no broken section references; Sonnet review findings each
  resolved or answered; `git diff` limited to Lead-owned paths.
- **Exit:** G0 approved; plan marked "final" in design.md header.

### M1 — Environment bootstrap *(Day 1 morning; needs G1)*
- **Goal:** `dev` and `sim` images build; containers really exchange ROS data; GUI route chosen.
- **Work (me):** `docker/dev.Dockerfile`, `docker/sim.Dockerfile`, root `compose.yaml` + `docker/compose.yaml` with `dev`
  service; named volumes (R9); `tests/integration/test_multi_container.sh`; GUI spike WSLg-from-PowerShell, then noVNC;
  `scripts/sim_lock.sh`; `scripts/ci.sh` (R2b). **Sonnet** drafts Dockerfiles from my spec; I own the networking/GUI spike.
- **Verify:** image builds logged with time; hello-multi-container ≥ 50/100 messages received; headless `gz sim -s`
  RTF and CPU measured idle; GUI visible (needs G3 for the user to confirm visually; until then I verify the client
  connects and renders via a screenshot from the noVNC route).
- **Exit:** numbers recorded in the agent log; `SWARMFLOW_GUI` default decided.

### M2 — Contract freeze *(Day 1; drafting can start in M0)*
- **Goal:** the four contracts exist, build, and are frozen.
- **Work:** `src/swarmflow_interfaces/` (msg/srv/action transcription → **Haiku**, field list and review → me);
  `layouts/schema/layout.schema.json` + `layouts/standard/layout.yaml` (me); `src/swarmflow_core/swarmflow_core/api.py`
  (me); synthetic fixtures (script by me, run by **Haiku**); contract-diff check in `scripts/ci.sh`.
- **Verify:** `colcon build` of interfaces; `ros2 interface show` matches design §6.6 field-by-field; schema validates the
  standard layout and rejects 3 deliberately broken copies; `api.py` imports with no ROS; `ci.sh` flags an edit to a
  frozen path.
- **Exit:** G2.

### M3 — Sim-free lanes in parallel *(Day 1 afternoon → Day 2)*
Each item = one task card + lead tests + one subagent (Sonnet unless noted), mechanical gate + review per §3.2.

| Lane | Task | Key lead-written acceptance tests |
|---|---|---|
| B | FCFS assignment + `ReservationAuthority` (§6.5) in `swarmflow_core` | Property tests: never two live leases on one zone; expiry → `OCCUPIED_UNKNOWN`; release-on-exit; deterministic given time. **Safety-critical review.** |
| B | `swarmflow_orchestrator` ROS adapter | Launch test with a fake agent: order → `DispatchTask` goal → `ASSIGN` decision event. |
| D | Robot agent + fake `NavigateThroughPoses` server | Route split at zone-entry vertices; holds until grant; orchestrator-down rule 6; one retry then `STUCK`. **Safety-critical review.** |
| F | layoutgen: SDF world, Nav2 map, nav graph (RMF format), `zones.yaml`, `sim2d.json` | Every §7.2 inequality asserted; nav graph vertices match stations; map pixels match racks. |
| F | Order generator (seeded) | Same seed → identical stream. |
| E | Foxglove layout + `swarmflow_viz` markers from fixtures | Layout JSON loads; marker node test on fixture stream. |
| C | 2D sim skeleton (`Backend` protocol) | FCFS run on standard layout deterministic across 2 runs. |

- **Exit:** all lane branches pass `ci.sh` and are queued for user review.

### M4 — Single robot in Gazebo *(Day 1–2; me; sim lock)*
- **Work:** chassis xacro (Sonnet drafts from §7.1 table, I verify in sim), standard world, single-robot Nav2 with our
  footprint (§7.2 settings), `/clock` + `use_sim_time`.
- **Verify:** robot reaches 5 random goals incl. through a 1.30 m storage aisle; LiDAR sees racks; CPU/RTF logged.
- **Exit:** G3 (user sees it in the GUI).

### M5 — Three robots + Day-2 gate *(Day 2; me)*
- **Work:** Option N namespacing from the Nav2 multi-robot example, composition on; robot agent + orchestrator in the
  loop for one order.
- **Verify:** design §15.2 gate criteria, run 3× in a row.
- **Exit:** G4 — GO, or apply §15.3 cuts in order.

### M6 — End-to-end deliveries *(Day 3)*
- **Work:** 3 robots, order stream, pose-follower packages (Sonnet implements, I integrate), decision events in Foxglove.
- **Verify:** ≥ 10 deliveries / 10 min unattended, packages visibly on robots, `decisions.jsonl` consistent with order states.

### M7 — Corridor scene, metrics, one-command compose *(Day 4)*
- **Work:** Baseline A vs B scenario, metrics script (Sonnet), `docker compose up` from PowerShell, README draft (Sonnet,
  reviewed by me).
- **Verify:** n = 3 runs per policy in `runs/`; Baseline A shows ≥ 1 stuck-timeout or higher wait; fresh-clone test:
  delete volumes, `docker compose up`, stack healthy.

### M8 — Hardening and v1 *(Day 5)*
- **Work:** fix list from M6–M7, docs, architecture diagram, cleanup. No new features.
- **Verify:** design §15.4 definition of done, item by item, in the agent log.
- **Exit:** G5.

## 5. Status

| Milestone | Status |
|---|---|
| M0 | waiting for G0 |
| M1 | blocked on G1 (Docker not installed) |
| M2 | can start drafting after G0 |
| M3–M8 | not started |
