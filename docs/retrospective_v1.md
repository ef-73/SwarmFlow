# SwarmFlow v1 — retrospective (M9)

Written by the lead (Opus 5.5) from evidence: `docs/agent_log/`, `docs/results/`, `runs/` (not committed), CI history,
two independent M9 reviews (a Sonnet repository review and a Haiku stale-docs sweep, 2026-10-09) and the per-card
safety reviews. Numbers are quoted from those sources; nothing here is from memory alone.

## 1. What v1 is

`docker compose up` from a Windows terminal starts Gazebo Harmonic (headless server + GUI via WSLg on the GPU), three
custom diff-drive robots each with Nav2 and a SwarmFlow robot agent, the SwarmFlow orchestrator (FCFS assignment +
exclusive aisle reservations with leases, heartbeats, recovery), a scenario engine, a package pose-follower, marker
publisher and Foxglove bridge. 17 task cards (T001–T017) plus the M9 hardening cards (T018–T020) were implemented by
subagents and integrated by the lead behind `scripts/ci.sh`.

## 2. What worked

- **Contract-first freeze (M2).** Interfaces, schema, library API and fixtures froze once and never needed a contract
  change afterwards; every later card built against them. The G2 Sonnet review found 3 high-severity contract gaps
  (orchestrator restart, release-while-inside, missing task-result channel) *before* any implementation depended on
  them.
- **Lead tests before dispatch + mechanical gate.** Every Sonnet implementer card passed its lead tests on the first
  attempt (14 of 14 in M3–M7). Owned-files / lead-test-hash checks caught one out-of-scope commit (a `.hypothesis/`
  database) and nothing else slipped through.
- **Independent safety reviews.** For every safety-critical card (T003, T005, T007, T018, T019) the Sonnet reviewer
  found defects the lead tests missed — stale-pose clearing, a route starting inside a zone without a lease, recovery
  under sim time, lost-lease races, a zone-boundary "inside" test. Each needed one retry; for T018–T020 a second
  review after the retry still found defects, which the lead fixed per the escalation ladder.
- **Fake backend → 2D sim → Gazebo ladder.** Most behaviour was exercised in seconds without Gazebo; Gazebo time was
  reserved for integration questions.
- **One-command start.** Fresh clone + `docker compose up` from PowerShell verified in M7; cold image build ≈ 10 min.

## 3. What did not work (and what it cost)

| Problem | Found | Cost | Root cause | Fix |
|---|---|---|---|---|
| Wheel odometry drift (robot really against a rack while Nav2 said "goal reached") | M4 session 1 | 1 sim session | DiffDrive odometry with slip used as ground truth | Gazebo OdometryPublisher as odom (decision 001) |
| Two robots sent to one loading station gridlock | M6 demo 1 | 1 demo run | FCFS assigns by robot distance only | Station claims (T014) |
| Corridor scenario produced no head-on traffic in Gazebo | M7 run 1 | 2 runs + 2 cards | Choreography tuned on the fake backend's timing; claims + park rule changed real timing | T017: tuned on the 2D sim with the real agent at Gazebo speed |
| Nav2 controller starved (20 Hz loop at 5–6 Hz) → "Failed to make progress" → stuck timeouts | M7 analysis (visible since M5) | stuck events in every batch | 3 × MPPI (20 Hz × 1000 × 40) on a thermally limited laptop | MPPI 15 Hz × 800 × 30 (M8) |
| Robots turned around inside an aisle | M8 | 1 discarded batch | Nav2's NavigateThroughPoses BT removes passed waypoints only every 3 s; plus stale global-costmap marks | Custom BT (remove passed goals every tick), global costmap static-only (M8) |
| Parallel ROS tests cross-talked; a subagent released a lock it did not hold; a stale lock after sleep | M5–M8 | ~1 h of hung CI, one corrupted measurement | Shared DDS domain; no lock ownership; no stale rule | Sequential `colcon test`, lock owner token, stale-lock removal |
| A stopped batch left an orphan run that tore down the next run | M8 | 1 run | TaskStop does not kill grandchildren in Git Bash | Check `ps` after stopping; batch scripts should own their children (open item) |
| GitHub Actions failed on 3 of 13 pushes while local CI was green | M7 | unknown (logs need sign-in) | Not diagnosed (most likely wall-clock ROS tests on a 2-vCPU runner) | Failures now surface as public annotations; next failure diagnosable |

## 4. Model routing results

| Model | Used for | Outcome |
|---|---|---|
| Haiku 4.5 (`sf-mechanical`) | T001 interface transcription; T015 one-line QoS fix; M9 stale-docs sweep | All exact. T015 was outside the routing table ("never logic") — it worked, but its test was never checked against the failing config (review P3). |
| Sonnet 5.5 (`sf-implementer`) | 17 cards incl. all safety-critical code, Foxglove layout, README draft, scenario searches | Lead tests passed first time on every card; safety-critical cards needed 1 retry each, always triggered by the reviewer, never by the gate. |
| Sonnet 5.5 (`sf-reviewer`) | G2 contract review, 5 safety reviews, M9 repository review | Highest-value spend of the run: found every safety defect that reached review. |
| Opus 5.5 (lead) | Contracts, lead tests, Docker/GUI spike, all Gazebo/Nav2 work, CI, integration, escalations | All sim debugging stayed with the lead; no Fable escalation needed. |

Lesson: cheap models are reliable *against good tests*; the expensive failure mode is the tests' blind spots, and only
an adversarial reviewer found those. Future safety cards should start with the reviewer's failure scenarios as lead
tests, and a second review after the retry.

## 5. Measured v1 results

| Run | Deliveries | Stuck | Failed | Overlaps | Source |
|---|---|---|---|---|---|
| 10-min demo, M6 (before tuning) | 14, 10 | some | — | — | `docs/agent_log/…M6.md` |
| 10-min demo, M8 | 15 | 0 | 0 | 3 (trunk junction) | `runs/m8b-demo-1` |
| 10-min demo, M9 (after T018–T020) | 15 | 0 | 0 | 3 (same junction) | `runs/m9-demo-1` |
| Corridor FCFS, n = 3, M7 → M8 | 6.00 ± 4.30 → **7.00 ± 0.00** | 0.67 → 0 | 0.67 → 0 | 0 → 0 | `docs/results/m7_…`, `m8_corridor_n3.md` |
| Corridor independent, n = 3, M7 → M8 | 3.00 ± 0.00 → 3.33 ± 3.79 | 1.00 → 2.67 | 1.00 → 2.67 | 1.33 → 2.00 | same |

M8 mean wait: FCFS 36 ± 5 s vs independent 102 ± 95 s. The intervals are wide at n = 3; the claim is the direction,
and FCFS became fully repeatable once the controller stopped starving.

## 6. Improvement list (M9)

Sources: M9 Sonnet repository review (S = safety, R = robustness, C = CI, D = docs/data, T = tests), Haiku stale-docs
sweep, second reviews of T018–T020, and the lead's own findings during M8/M9.

| # | Improvement | Effort / risk | Status |
|---|---|---|---|
| S1 | Agent drops a lost lease before entry, cancels and re-requests | M / safety | **done** T019 (`1b6fc9b` and earlier) |
| S2, S3, S6 | Owner re-grant of blocking leases, unauthorized entry blocks a zone, stale FIFO queue heads | M / safety | **done** T018 (`9ba48fc` and earlier) |
| S4 | No RobotState before a valid pose; stale pose = FAULT; orchestrator ignores unknown poses for zone clearing and recovery | M / safety | **done** T019 + T020 |
| S5 | Bounded STUCK recovery park tasks | S / safety | **done** T020 (`fe15d3a`) |
| S7 | Simulator clock reset: revoke all leases, new authority, recovery window | M / safety | **done** T020 |
| 2nd reviews | 4 + 3 + 3 further defects after the retries (stale-pose dispatch, lease kept on unknown pose, clock-back pose freeze, owner blocks lost on denial, fleet-wide reset from one robot, dead park ids) | S each | **done** by the lead (ladder: retry used) |
| D2 | Run files of a reused run id: moved to `.prev` | S | **done** T020 |
| D3 | Metrics count only the scenario window | S | **done** `ba2f0dd` |
| C1, C2 | CI: fresh test results, fail on 0 tests, compose/xacro/launch checks, CRLF types, no referee fallback | S | **done** `lead/ci-m9` |
| C-new | Referee piped through `bash -s` lost its own verdict (exit 0 after FAIL); 0-tests guard always failed | S / high value | **done** `0fb9c8d` and parent |
| Sec | noVNC and Foxglove ports bound to 127.0.0.1 | S | **done** `lead/docs-m9` |
| Docs | Mojibake, design service table, README wording, Baseline A scenario, M8 results, known limitations | S | **done** |
| Perf | MPPI load, global costmap, NavigateThroughPoses BT | M | **done** M8 |
| R1 | Healthchecks + restart policies for robot/orchestrator containers | M | deferred: a restart policy interacts with the S7/S4 recovery paths and needs Gazebo time to verify; v1.1 |
| R2 | Spawn verification in the gazebo healthcheck | S | deferred: same reason |
| T3 | Replace fixed sleeps in orchestrator node tests with condition waits | S | partly (new T020 tests use direct calls); rest deferred |
| CI-id | `ci.sh` does not treat `ws-b/T018b-…` as a task branch (checks run by hand with `CI_BRANCH`) | S | deferred: accept `T\d+[a-z]?` in v1.1; never use suffixed branch names meanwhile |
| Batch | Batch scripts should own and kill their children (orphan run in M8) | S | deferred to v2 benchmark harness (WS-C) |
| Rate | demo/gate scripts should report "Control loop missed" counts | S | deferred (counted by hand in M8 log) |
| D6, D8 | "half-width" column in design table; LICENSE file | S | D6 deferred; LICENSE needs the user's choice |
| n ≥ 3 | 10-minute demo repeated n ≥ 3 | 1 h sim | deferred: v2 item 1 (Baseline B n ≥ 5) |
| GH | 3 of 13 GitHub Actions failures undiagnosed | ? | annotations in place; diagnose on next failure |

Process lessons (for v2 cards): run the reviewer's failure scenarios as lead tests; always review again after a retry
(it found 10 more defects here); never use the Write tool's view of a file as proof a tool didn't mangle escapes
(heredoc and sed both corrupted `\1` this run); keep the machine awake during Gazebo batches (one demo lost to sleep).

## 7. For the user (needs a decision, not implemented)

1. **Layout contract change** (frozen `tests/fixtures/`, `layouts/schema`): move trunks to x = 3.2 / 18.8 and holds
   to x = 4.6 / 17.4, and/or add reservation zones at aisle–trunk junctions. Evidence: clearance overlaps at holds
   (M6) and at the east trunk junction (3 in the M8 demo, about 1 cm from contact between real bodies).
2. **Recovery with a robot that never reports a valid pose** keeps the whole fleet in RECOVERING (safe stop, no
   operator override). An override service (e.g. `/fleet/force_recover`) is an interface change.
3. **Lease ids restart at `lease_00001`** after an orchestrator restart/clock reset; a generation prefix needs an
   api/interface decision (robots already guard by epoch and robot id).
4. **LICENSE** choice.
5. **G5:** watch/record the demo video, link it from the README, import `viz/foxglove/swarmflow_v1.json` into
   Foxglove once (unverified), then tag: `git tag -a v1.0.0 -m "SwarmFlow v1" && git push origin v1.0.0`.
6. A Windows Firewall prompt for "Docker Desktop Backend" appeared during M7 and was left unanswered (not needed for
   localhost use).
