# Contributing to SwarmFlow

Thanks for your interest. This page covers how to build, test and send changes. The design is in
[`docs/design.md`](docs/design.md); how to run demos and experiments is in [`docs/USAGE.md`](docs/USAGE.md).

## Setup

You need Docker (Docker Desktop with the WSL2 backend on Windows, or Docker Engine on Linux) and about 20 GB of disk.
Nothing else is installed on the host: every dependency lives in a Docker image (`docker/*.Dockerfile`) or in a package
manifest (`package.xml`, `pyproject.toml`) built inside the `dev` image.

## Build and test

All commands run from the repo root. Give each git worktree its own compose project, so build volumes don't collide:

```bash
export COMPOSE_PROJECT_NAME=swarmflow-$(basename "$(git rev-parse --show-toplevel)" | tr A-Z a-z) SWARMFLOW_GUI=none
docker compose run --rm dev colcon build --symlink-install --parallel-workers 2
docker compose run --rm dev colcon test --packages-select <pkg>
docker compose run --rm dev pytest src/swarmflow_core -q
scripts/ci.sh
```

`scripts/ci.sh` is the referee, and GitHub Actions runs the same script on every push and pull request. It checks:

- frozen contracts are untouched;
- hygiene: no file over 5 MB, no run output or rosbags, LF line endings, a secret-pattern scan;
- generated layouts are up to date;
- the full build and every test pass.

A pull request is merged only when it passes on top of the current `main`.

## Rules of the road

- **Contracts are frozen.** These paths change only through a dedicated `lead/contract-*` branch and a written
  decision in [`docs/decisions/`](docs/decisions/):
  - `src/swarmflow_interfaces/**` (ROS messages, services, actions)
  - `layouts/schema/**`
  - `src/swarmflow_core/swarmflow_core/api.py`
  - `tests/fixtures/**`

  If a contract is missing something, open an issue and describe the change instead of working around it silently.
- **`swarmflow_core` stays ROS-free.** It must not import `rclpy` or any ROS package (a test enforces this). Library
  code takes time as an argument; every ROS node uses `use_sim_time: true`.
- **Tests come with behaviour.** New behaviour needs tests. Never disable or weaken a test or a CI check to get a
  branch through.
- **One Gazebo at a time.** Gazebo is heavy. Take `scripts/sim_lock.sh acquire` before starting it and release it
  when done, keep sessions short (≤ 20 min), and run headless (`SWARMFLOW_GUI=none`) unless you're watching. Builds
  are serialised with `scripts/lock.sh build`. Most work needs no simulator: use the unit tests, fake backends,
  recorded fixtures and the 2D light mode (`sim2d/`).
- **Line endings are LF** (`.gitattributes`); never commit CRLF shell scripts.
- **Don't commit** secrets, run output (`runs/`), rosbags (`*.db3`, `*.mcap`) or binaries over 5 MB.
- **Check, don't guess.** Don't invent topic names, parameters or APIs: check inside the container
  (`ros2 interface show`, `ros2 param list`), or mark the claim as unverified.

## Branches and pull requests

- Branch from `main`, one change per branch, named `<area>/<short-slug>`, e.g. `nav/keep-right-tuning`.
- Keep branches small (aim for under 400 changed lines, excluding generated files).
- Rebase onto `main` before opening the PR. `main` is only ever fast-forwarded.
- Describe in the PR what changed, how you verified it (commands and results), and anything you could not verify.

## Repository layout

The top-level map is in the [README](README.md#repository-map). Package boundaries follow the workstreams in
[design §14.2](docs/design.md).
