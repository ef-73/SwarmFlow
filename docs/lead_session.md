# Lead session instructions (start or resume the v1 run)

You are the **SwarmFlow lead agent** (Claude Opus). Your job: complete v1 by executing milestones **M1–M9** in
[`docs/milestones.md`](milestones.md) autonomously, then stop at gate **G5** for the user.

## 1. Read first, in this order
1. [`AGENTS.md`](../AGENTS.md) — rules. They bind you too (you are "the lead"). §10 = subagent rules.
2. [`docs/milestones.md`](milestones.md) — gates (§2), autonomous-run rules (§2.1), routing (§3), milestones (§4),
   **status table (§5): resume from the first milestone not marked done.**
3. [`docs/design.md`](design.md) — the spec. Read each section a milestone cites before working on it.
4. The newest entries in [`docs/agent_log/`](agent_log/) — what happened last.

## 2. Environment facts (verified 2026-10-09)
- Windows 11, Intel Core Ultra 9 285H (16 threads), 63 GB RAM, NVIDIA GPU present; **CPU thermal limit**.
- Docker Desktop 29.8.2 (WSL2 backend), Compose v5.5.1; Docker VM sees 16 CPUs / 33 GB. `hello-world` ran.
- If `docker` is not on PATH in Git Bash: `export PATH="/c/Program Files/Docker/Docker/resources/bin:$PATH"`.
  In PowerShell: `$env:Path = "C:\Program Files\Docker\Docker\resources\bin;" + $env:Path`.
- WSL 2.5.10 with WSLg; an `Ubuntu` distro exists (not used by the project).
- Remote `origin` = `https://github.com/ef-73/SwarmFlow.git`; pushing works with the user's stored credentials.
  No `gh` CLI (don't install it; GitHub Actions runs on push).
- Main checkout: `C:\Users\ethan\Documents\Projects\SwarmFlow` (branch `main`). Keep it clean; you are likely in a
  worktree under `.claude/worktrees/`.

## 3. How to integrate (decision D4)
1. Work on a branch (`lead/...`, or the task card's branch for subagent work).
2. Rebase onto current `main`, run `main`'s `scripts/ci.sh` (once it exists; before that, the checks the milestone
   names), and require green.
3. `git -C C:/Users/ethan/Documents/Projects/SwarmFlow merge --ff-only <branch>`, then
   `git -C C:/Users/ethan/Documents/Projects/SwarmFlow push origin main`.
4. If `main` is ever red, fix or revert it before anything else.

## 4. Working loop per milestone
1. Mark the milestone in progress in §5 of `docs/milestones.md`.
2. Write task cards (`docs/tasks/`) and lead tests for delegated work; dispatch subagents per `AGENTS.md` §10
   (`sf-mechanical` = Haiku, `sf-implementer` / `sf-reviewer` = Sonnet; ≤ 3 in parallel; worktree isolation).
3. Do the lead-only work yourself (contracts, Docker/GUI spike, all Gazebo/Nav2 sessions under the sim lock).
4. Verify against the milestone's **Verify** list; collect evidence (command output, numbers, screenshots).
5. Integrate (§3), write an agent-log entry file, update §5 status, push.
6. Async gates (G2–G4): decide by the written criteria, record the evidence in the log, continue.

## 5. Stop and ask the user only if
- a fix would change a frozen contract, the v1 scope, or a user decision (D1–D4);
- something must be installed on the Windows host (outside Docker);
- the same failure survives three different fixes;
- you reach G5 (end of M9).

## 6. Practical notes
- Thermal: one Gazebo at a time, ≤ 20 min sessions, `SWARMFLOW_GUI=none` unless capturing GUI evidence,
  2 build workers, Gazebo RTF cap when hot (design §8.5).
- Long run: context is summarized automatically; this file + `docs/milestones.md` + `docs/agent_log/` are the durable
  state. To resume in a new session: "Read docs/lead_session.md and continue."
- Ask the desktop app to keep the machine awake if that tool is available.
- Docker image pulls/builds are allowed (inside Docker). Host installs are not.
