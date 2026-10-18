#!/bin/bash
# Run demo_run.sh several times in a row (sim lock per run) and print one metrics table.
# Owns its children (X-06): on exit or Ctrl-C the whole process group is stopped and the stack taken down,
# so an interrupted batch never leaves an orphan run that tears down the next one (M8).
# Usage: tests/integration/batch_runs.sh <prefix> <n> [minutes=10] [scenario=v1_demo] [policy=fcfs]
set -uo pipefail
cd "$(git rev-parse --show-toplevel)"
prefix="${1:?prefix}"; n="${2:?n}"; minutes="${3:-10}"; scenario="${4:-v1_demo}"; policy="${5:-fcfs}"
stop() { trap - EXIT INT TERM; pkill -TERM -P $$ 2>/dev/null; docker compose --profile extra_robots --profile gui down >/dev/null 2>&1
         scripts/sim_lock.sh release >/dev/null 2>&1; }
trap stop EXIT INT TERM
for i in $(seq 1 "$n"); do
  id="$prefix-$i"
  echo "=== run $i/$n: $id"
  tests/integration/demo_run.sh "$id" "$minutes" "$scenario" "$policy" 0 &
  wait $! || echo "run $id exited with $?"
done
head -1 "runs/$prefix-1/metrics.csv" 2>/dev/null
for i in $(seq 1 "$n"); do tail -n +2 "runs/$prefix-$i/metrics.csv" 2>/dev/null; done
