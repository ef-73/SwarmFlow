#!/bin/bash
# Gazebo sim lock (AGENTS.md §5): lead-only, one sim at a time (thermal limit).
#   scripts/sim_lock.sh acquire <task-id> [expected-minutes]   (max 20 min per session)
#   scripts/sim_lock.sh release | status
# Authoritative check: no running container labelled swarmflow.sim=1 (any compose project).
# A lock older than 30 min with no labelled container running is stale and removed (logged to stderr).
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
cmd="${1:-status}"

running() { docker ps --filter label=swarmflow.sim=1 -q; }

case "$cmd" in
  acquire)
    task="${2:-unknown}"; minutes="${3:-20}"
    if [ -n "$(running)" ]; then
      echo "sim_lock: a swarmflow.sim=1 container is running:" >&2
      docker ps --filter label=swarmflow.sim=1 --format '  {{.Names}} ({{.Status}})' >&2
      exit 1
    fi
    age="$("$here/lock.sh" age sim)"
    if [ "$age" -ge 30 ]; then
      echo "sim_lock: STALE lock (${age} min, no sim container running) removed:" >&2
      "$here/lock.sh" status sim >&2; "$here/lock.sh" release sim >/dev/null
    fi
    "$here/lock.sh" acquire sim "$task" "$minutes" ;;
  release)
    if [ -n "$(running)" ]; then
      echo "sim_lock: refusing to release while sim containers run; docker compose down first" >&2; exit 1
    fi
    "$here/lock.sh" release sim ;;
  status)
    "$here/lock.sh" status sim
    echo "running sim containers: $(running | wc -l)" ;;
  *) echo "usage: $0 acquire <task> [minutes] | release | status" >&2; exit 2 ;;
esac
