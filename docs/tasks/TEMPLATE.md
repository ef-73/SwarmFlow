---
id: T000
title: <short imperative title>
workstream: <A|B|C|D|E|F|Lead>
status: open            # open | claimed | review | done | abandoned
claimed_by: ""
branch: <ws-x/T000-slug>
model: <haiku|sonnet|opus>   # lead's routing choice, docs/milestones.md §3
risk: <low|normal|safety-critical>
---

## Goal
<one or two sentences: what exists when this is done>

## Owned files
- <exact paths/globs this task may create or edit>

## Lead tests (do not edit)
- <paths of tests the lead committed before dispatch, or "none">

## Forbidden
- AGENTS.md §4
- <task-specific, e.g. "no Gazebo", "no new dependencies">

## Commands to run
```bash
docker compose run --rm dev pytest <path> -q
scripts/ci.sh
```

## Definition of done
- <observable acceptance criteria>
- `scripts/ci.sh` green on the branch rebased onto `main`
- Agent-log entry written (AGENTS.md §8)

## Inputs / contracts used
- <design.md sections, contract files, fixtures>
