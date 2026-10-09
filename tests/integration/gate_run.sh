#!/bin/bash
# Day-2 go/no-go gate run (design §15.2), lead-only (takes the sim lock): full stack with 3 robots + agents +
# orchestrator, one order per robot through a storage-aisle hold; PASS when every order is DELIVERED.
# Usage: tests/integration/gate_run.sh <run_id> [timeout_s]   → runs/<run_id>/ holds the run record.
set -uo pipefail
export MSYS_NO_PATHCONV=1
cd "$(git rev-parse --show-toplevel)"
run_id="${1:?run_id}"; timeout_s="${2:-300}"
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-swarmflow-$(basename "$(pwd)")}"
export SWARMFLOW_RUN_ID="$run_id" SWARMFLOW_AGENT=true SWARMFLOW_GUI="${SWARMFLOW_GUI:-none}"
export SWARMFLOW_ROBOTS=robot_1,robot_2,robot_3
rm -rf "runs/$run_id"
scripts/sim_lock.sh acquire "gate:$run_id" 15 >/dev/null || exit 2
cleanup() { docker compose down >/dev/null 2>&1; scripts/sim_lock.sh release >/dev/null; }
trap cleanup EXIT
docker compose up -d gazebo robot_1 robot_2 robot_3 orchestrator >/dev/null 2>&1
# wait until all three robots appear in the orchestrator's state log, then 20 s more (recovery grace + Nav2 up)
for i in $(seq 1 60); do
  n=$(cut -d'"' -f1- "runs/$run_id/robot_states.jsonl" 2>/dev/null | grep -o '"robot_id": "robot_[0-9]"' | sort -u | wc -l)
  [ "$n" -ge 3 ] && break; sleep 2
done
sleep 20
for o in "o1 L1 D1" "o2 L2 D2" "o3 L3 D3"; do
  set -- $o
  docker compose exec -T orchestrator /entrypoint.sh ros2 topic pub --once -w 1 /fleet/orders \
    swarmflow_interfaces/msg/Order "{order_id: $1, pickup_vertex: $2, dropoff_vertex: $3, payload_type: small}" >/dev/null 2>&1
done
start=$(date +%s); ok=0
while [ $(( $(date +%s) - start )) -lt "$timeout_s" ]; do
  d=$(grep -c '"state": "DELIVERED"' "runs/$run_id/orders.jsonl" 2>/dev/null); d=${d:-0}
  [ "$d" -ge 3 ] && { ok=1; break; }
  sleep 5
done
echo "run $run_id: delivered $(grep -c '"state": "DELIVERED"' "runs/$run_id/orders.jsonl" 2>/dev/null)/3 in $(( $(date +%s) - start )) s wall"
grep -o '"decision_type": "[A-Z_]*"' "runs/$run_id/decisions.jsonl" 2>/dev/null | sort | uniq -c | sed 's/^/  /'
w=$(grep -c '"mode": "WAITING_RESERVATION"' "runs/$run_id/robot_states.jsonl" 2>/dev/null); echo "  WAITING_RESERVATION samples (2 Hz): $w"
[ $ok = 1 ] && echo "GATE PASS" || echo "GATE FAIL"
[ $ok = 1 ]
