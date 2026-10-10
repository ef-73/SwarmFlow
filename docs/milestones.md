# SwarmFlow — Lead milestones (autonomous execution plan)

**Status:** active 2026-10-09 — G0 and G1 passed. Lead session instructions: [`lead_session.md`](lead_session.md). (G0: user approved all of R1–R16, gave the GitHub remote, authorized the lead
to fast-forward `main`.) **Owner:** Claude lead agent (Opus 5.5).
**Implements:** [`design.md`](design.md) v1 (§15) under [`AGENTS.md`](../AGENTS.md).

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
| R2 | **Blocker** | **No git remote, no `gh`.** PRs and GitHub Actions CI (design §14.3, AGENTS §6) cannot exist. | Either (a) user creates a private GitHub repo + installs `gh`, or (b) run "local CI": `scripts/ci.sh` runs the same checks in the `dev` image; branches are reviewed and merged locally. Recommend (b) now, (a) whenever convenient. |
| R3 | **Blocker** | `core.autocrlf=true`: git will check out shell scripts/launch files with CRLF, which break inside Linux containers (`/bin/bash^M`). Already warned on the last commits. | Add `.gitattributes` with `* text=auto eol=lf` (and `*.ps1 eol=crlf`) before any script lands. |
| R4 | High | `docker compose up` "from the repo root" can't work: compose lives at `docker/compose.yaml`. And the `dev` service that AGENTS §6 commands use is not in the §8.3 service table. | Root `compose.yaml` that `include`s `docker/compose.yaml`; add a `dev` service under a `tools` profile so plain `up` never starts it. |
| R5 | High | **Contradiction with D1:** §5.4/§6.1 use `rmf_building_map_tools` to make the world + nav graph, but AGENTS §11 bans RMF dependencies in v1. | v1 layoutgen writes the SDF world (racks are boxes) and the nav graph in RMF's nav-graph YAML format itself; RMF tools only in v2. Copy the format from an `rmf_demos` nav graph, mark [U] until Baseline C loads it. |
| R6 | High | **`docs/agent_log.md` will conflict on every merge**: all branches append to the end of one file. | One file per entry: `docs/agent_log/<YYYY-MM-DD>T<HHMM>Z-<agent>-<task>.md`; `docs/agent_log.md` becomes the index/format doc. |
| R7 | High | **No task queue.** 24/7 agents need a way to pick work without two taking the same task. | `docs/tasks/<id>.md` task cards (AGENTS §7 format) with `status: open/claimed/done` and `claimed_by`; claiming = a commit on your branch that flips the status. Lead writes cards; only the lead assigns contract-adjacent tasks. |
| R8 | Medium | Sim lock path differs between Windows-host agents (`C:\Users\ethan\.swarmflow`) and WSL/Codex agents (`/home/<user>/.swarmflow`); `flock` doesn't exist in Git Bash. Two agents could race the `docker ps` check. | `SWARMFLOW_LOCK_DIR` = the Windows path for everyone (`/mnt/c/Users/ethan/.swarmflow` from WSL); use atomic `mkdir` as the lock. Simpler still: **only the lead runs Gazebo** (subagents never do — see §3). |
| R9 | Medium | Bind-mounting a Windows-path repo into Linux containers is slow for `colcon build` (and heats the CPU). | Named Docker volumes for `build/`, `install/`, `log/`; only `src/` is bind-mounted. Measure in M1. |
| R10 | Medium | Thermal: Gazebo + 3 Nav2 + GUI software rendering on a laptop CPU (Core Ultra 9 285H). | Cap Gazebo real-time factor (e.g. 0.5) when hot — all metrics are in sim time, so results stay valid; GUI container optional; GPU via `/dev/dxg` on the WSLg route. |
| R11 | Medium | Day 1 overloads the critical path: env spike **and** contract freeze **and** four lanes starting against unfrozen contracts. | Draft and freeze contracts *before* Day 1 (M2 can start now on paper and be build-checked as soon as Docker exists). |
| R12 | Medium | Codex cloud agents can't reach the local Docker engine, and "no installs outside Docker" is ambiguous for their own sandboxes. | Clarify: the install ban is about *the user's machine*. Codex gets only sim-free tasks whose tests run with plain `pytest` (pure-Python `swarmflow_core`, layoutgen, 2D sim). |
| R13 | Low | Pose-follower via `set_pose` service calls at 20 Hz may jitter or lag across the bridge. | Keep as planned (handoff C13), but list Gazebo's `DetachableJoint` system as the fallback **[U]** if jitter is visible on Day 3. |
| R14 | Low | AGENTS says only the user merges `main`, but the lead fast-forwarded `main` twice on request. | Superseded by decision D4: the user authorized the lead to fast-forward `main` after `scripts/ci.sh` passes. |
| R15 | Low | User review is the real bottleneck for 6 lanes × 24/7. | WIP limit: ≤ 2 unmerged branches per workstream; the lead batches reviews with a one-paragraph summary per branch. |
| R16 | Low | Subagent model use is not specified anywhere. | Adopt §3 of this file and reference it from AGENTS.md. |

Nothing in the review changes the v1 scope or the user decisions D1/D2. **All of R1–R16 were applied on
2026-10-09** (R1 as design §8.0 with decision D3 pending; R2 with the GitHub remote plus `scripts/ci.sh`).

## 1.1 M0 verification round (2026-10-09)

Two read-only subagents checked the edited documents, and I adjudicated every finding.

| Reviewer | Findings | Accepted | Result |
|---|---|---|---|
| **Haiku 4.5**: mechanical sweep for stale RMF/log/merge/sim wording, § refs, tables, links | 9 | 9 | Stale "user merges" ×4, non-lead sim wording ×4, 2 wrong § refs fixed; tables and links clean |
| **Sonnet 5.5**: independent review for contradictions, gaps and technical errors | 20 (1 blocker, 6 high) | 20 | All fixed in design.md / AGENTS.md (see below) |

The Sonnet fixes that changed the design:
- **Builds:** runtime images build the workspace at image build time, so a fresh clone runs.
- **Interfaces:** orchestrator liveness topic; `/fleet/clear_zone` service; global `/map`; marker topics.
- **Zones:** separate hold vertices outside each zone, so a waiting robot never blocks the exit; edges keyed by vertex-name pairs; generated layouts committed and checked for freshness.
- **Packages:** fixed pool of 12, ground-truth pose, 3 s dwell; the `payload` service moves to the `sim` image.
- **v1 evaluation (design §13.5):** Baseline A switch, stuck rule owner, corridor scenario, run-record layout.
- **CI (design §14.3):** git checks run on the host; the lead runs `main`'s copy of `ci.sh`; hygiene and lead-test hash checks; per-worktree compose project and a build lock.
- **Cards:** the lead assigns every card; Lead-delegated cards are allowed.
- **Fixtures:** Gazebo recordings go to `tests/fixtures_gz/`.
- **Sims:** Day-2 sim sessions are sequenced.
- **GUI:** `SWARMFLOW_GUI=none` for agents.

Takeaway for routing: the cheap sweep was exhaustive on mechanical checks. The judgment review needed Sonnet, and it
was worth it: it found the one blocker.

## 2. Gates

Only two gates **block** me. The others are **async**: I decide using written criteria, record the evidence
(screenshots, logs, numbers) in an agent-log entry, notify the user, and keep going. The user can overrule any async
decision afterwards.

| Gate | When | Type | What happens |
|---|---|---|---|
| **G0** | before M0 | ✅ passed 2026-10-09 | R1–R16 approved, remote given, lead may fast-forward `main`. |
| **G1** | before M1 | ✅ passed 2026-10-09 | Docker Desktop installed and running (decision D3, design §8.0). The user runs the installer (admin rights + accepting Docker's license); I verify the result (`docker run hello-world`, WSL2 backend, `docker compose version`). |
| **G2** | end of M2 | async | Contracts frozen when `scripts/ci.sh` passes and the independent Sonnet review has no unresolved blocker/high findings. |
| **G3** | first GUI run (M4) | async | Verified by screenshot (noVNC route: captured in the built-in browser; WSLg route: window capture). User glances when convenient. |
| **G4** | Day-2 go/no-go (M5) | async | GO if design §15.2 criteria pass 3 runs in a row (logged); otherwise I apply the §15.3 cuts in order and log each. |
| **G5** | v1 release (after M9) | **blocking** | User watches the demo, records/approves the video. I prepare everything incl. the `v1.0.0` tag command; tagging stays with the user (AGENTS.md §4). |

Between gates I work without asking, inside AGENTS.md rules. I also stop and ask if: a fix would change a frozen
contract, the v1 scope or a user decision; something needs a new install on the host; or the same failure survives
three different fixes.

## 2.1 Running autonomously

Once G1 passes, one instruction — "run milestones M1–M9" — is enough. During the run:

- **Progress:** each milestone ends with a commit to `main`, a push to `origin`, an updated §5 status table here, and an
  agent-log entry. The user can follow everything on GitHub.
- **Session length:** long runs are summarized automatically as context fills; this file plus the agent log are the
  durable state, so a new session can continue from them ("continue the milestones in docs/milestones.md").
- **Machine:** the laptop must stay on and awake (keep-awake can be requested); Docker must be running.
- **Permissions:** Docker/WSL commands may trigger permission prompts depending on the session's permission mode; an
  allowlist for `docker`, `wsl` and `scripts/*.sh` avoids stalls.
- **Realistic pace:** the 5-day plan assumed a human watching the sim for parts of each day. Autonomously, the
  Gazebo/Nav2 debugging (M4–M5) is the slowest part because only one sim runs at a time; sim-free lanes run in parallel.

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
- **Exit:** G0 approved ✅; plan marked "final" in design.md header ✅; review findings resolved; pushed to `origin/main`.

### M1 — Environment bootstrap *(Day 1 morning; needs G1)*
- **Goal:** `dev` and `sim` images build; containers really exchange ROS data; GUI route chosen.
- **Work (me):** `docker/dev.Dockerfile`, `docker/sim.Dockerfile`, root `compose.yaml` + `docker/compose.yaml` with `dev`
  service; named volumes (R9); `tests/integration/test_multi_container.sh`; GUI spike WSLg-from-PowerShell, then noVNC;
  `scripts/sim_lock.sh`; `scripts/ci.sh` (R2b). **Sonnet** drafts Dockerfiles from my spec; I own the networking/GUI spike.
- **Verify:** image builds logged with time; hello-multi-container ≥ 50/100 messages received; headless `gz sim -s`
  RTF and CPU measured idle; GUI client connects and renders, proven by a screenshot (async G3).
- **Exit:** numbers recorded in the agent log; `SWARMFLOW_GUI` default decided.

### M2 — Contract freeze *(Day 1; drafting can start in M0)*
- **Goal:** the four contracts exist, build, and are frozen.
- **Work:** `src/swarmflow_interfaces/` (msg/srv/action transcription → **Haiku**, field list and review → me);
  `layouts/schema/` (`layout.schema.json`, `zones.schema.json`, nav-graph subset description — me, after checking an
  `rmf_demos` jazzy nav graph); `layouts/standard/layout.yaml` (WS-F card, **Sonnet**, against the schema and the §7.2
  tables); `src/swarmflow_core/swarmflow_core/api.py` (me); synthetic fixtures (script by me, run by **Haiku**);
  `scripts/ci.sh` host-side checks (me).
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
- **Work, as sequenced sim sessions (≤ 20 min each):** ① robot agent + orchestrator driving robot_1 alone through one
  order with a zone hold; ② Option N namespacing for 3 robots from the Nav2 multi-robot example, composition on, each
  reaching goals; ③ the gate run with all 3 robots. Between sessions: fix offline, rebuild, re-run unit tests.
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
- **Exit:** → M9.

### M9 — Reflection and improvements *(after M8, before G5)*
- **Goal:** v1 is as good as it can be without changing its scope, and the lessons are written down for v2.
- **Work:**
  1. **Reflect**, from evidence rather than memory: agent-log entries (failures, retries, escalations, model
     outcomes), CI history, run metrics in `runs/`, design §17 risks vs what actually happened, open **[U]** tags,
     README fresh-clone test. Independent second opinion: a **Sonnet** reviewer reads the repo and the log and lists
     weaknesses; a **Haiku** sweep finds stale docs and TODOs.
  2. **Write** `docs/retrospective_v1.md`: what worked, what didn't, model-routing results (where cheap models held
     up, where they needed escalation), and a ranked improvement list with effort and risk.
  3. **Implement autonomously** every improvement that stays inside the guardrails: bug fixes, flaky tests,
     robustness (timeouts, healthchecks, retries), performance and thermal tuning, docs/README accuracy, test coverage,
     CI hardening, cleanup. Each follows the normal card → tests → `ci.sh` → integrate path, and the v1 definition of
     done (design §15.4) is re-run afterwards.
  4. **Do not implement**, only list for the user: anything that changes a frozen contract, the v1 scope, a user
     decision (D1–D4), or needs a host install; and anything that belongs to v2.
- **Verify:** §15.4 definition of done still passes after the improvements; CI green; the retrospective lists every
  improvement as done (with commit) or deferred (with reason).
- **Exit:** G5 (the user reviews the retrospective together with the demo).

## 5. Status

| Milestone | Status |
|---|---|
| M0 | ✅ done 2026-10-09 — R1–R16 applied; 29 review findings fixed (§1.1) |
| M1 | ✅ done 2026-10-09 — images build (dev 27 s, sim 241 s, robot 227 s); DDS 90/100 msgs across containers; GUI default `wslg` + NVIDIA d3d12 (G3 evidence pending robots, M4); colcon on named volume 4.3× faster than bind mount |
| M2 | ✅ done 2026-10-09 — contracts frozen; G2 passed (Sonnet review: 3 high fixed, 0 open) |
| M3 | ✅ done 2026-10-09 — T001–T012 integrated (all sim-free lanes incl. metrics and 2D sim skeleton) |
| M4 | ✅ done 2026-10-09 — 7/7 goals incl. 1.30 m aisle, 3 cm final error; G3 passed (docs/evidence/); decision 001 ground-truth localization |
| M5 | ✅ done 2026-10-09 — G4 GO: 3 robots 14/14 goals; gate run ×3 each 3/3 delivered through aisle holds |
| M6 | ✅ done 2026-10-09 — 14 and 10 deliveries / 10 min unattended; packages visible; decisions consistent; contract change request (hold positions) for the user |
| M7 | ✅ done 2026-10-09 — corridor n=3: FCFS 6.0 vs independent 3.0 deliveries, wait 56 vs 118 s; fresh clone + one-command up verified; cold build 10 min |
| M8 | ✅ done 2026-10-10 — controller starvation and in-aisle reversals fixed; corridor n=3: FCFS 7.0 ± 0.0 vs independent 3.3 deliveries, 0 stuck; 10-min demo 15 deliveries, 0 stuck; §15.4 checked (tag + video with the user) |
| M9 | ✅ done 2026-10-10 — retrospective (`docs/retrospective_v1.md`); T018–T020 safety hardening (each reviewed twice), CI referee fixes, docs/ports; M9 demo 15 deliveries, 0 stuck; §15.4 re-checked → **G5 waiting for the user** |
