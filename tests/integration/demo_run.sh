#!/bin/bash
# Unattended demo run (design §15.4 item 2; milestone M6), lead-only (takes the sim lock):
# full v1 stack + scenario for <minutes> of wall time, optional GUI screenshots, then metrics.
# Usage: tests/integration/demo_run.sh <run_id> [minutes=10] [scenario=v1_demo] [policy=fcfs] [screenshot=1]
set -uo pipefail
export MSYS_NO_PATHCONV=1
cd "$(git rev-parse --show-toplevel)"
run_id="${1:?run_id}"; minutes="${2:-10}"; scenario="${3:-v1_demo}"; policy="${4:-fcfs}"; shot="${5:-1}"
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-swarmflow-$(basename "$(pwd)")}"
export SWARMFLOW_RUN_ID="$run_id" SWARMFLOW_SCENARIO="$scenario" SWARMFLOW_POLICY="$policy" SWARMFLOW_AGENT=true
export SWARMFLOW_GUI=wslg SWARMFLOW_GIT_SHA="$(git rev-parse HEAD)"
robots="$(python3 -c "import yaml,sys; print(','.join(yaml.safe_load(open('scenarios/$scenario.yaml'))['robots']))" 2>/dev/null \
  || python -c "import yaml,sys; print(','.join(yaml.safe_load(open('scenarios/$scenario.yaml'))['robots']))")"
export SWARMFLOW_ROBOTS="$robots"
[ "$policy" = independent ] && export SWARMFLOW_TRAFFIC_CONTROL=false || export SWARMFLOW_TRAFFIC_CONTROL=true
rm -rf "runs/$run_id"
scripts/sim_lock.sh acquire "demo:$run_id" $(( minutes + 6 )) >/dev/null || exit 2
cleanup() { docker compose down >/dev/null 2>&1; scripts/sim_lock.sh release >/dev/null; }
trap cleanup EXIT
services="gazebo orchestrator scenario_engine payload viz foxglove_bridge"
for r in ${robots//,/ }; do services="$services $r"; done
docker compose up -d $services >/dev/null 2>&1
echo "[$(date -u +%H:%M:%S)] up: $services"
end=$(( $(date +%s) + minutes * 60 ))
shots=0
while [ "$(date +%s)" -lt "$end" ]; do
  sleep 30
  d=$(grep -c '"state": "DELIVERED"' "runs/$run_id/orders.jsonl" 2>/dev/null); d=${d:-0}
  docker stats --no-stream --format '{{.Name}} {{.CPUPerc}}' | sed "s/$COMPOSE_PROJECT_NAME-//" | awk '{s+=$2} END {printf "cpu %.0f%% ", s}'
  echo "[$(date -u +%H:%M:%S)] delivered $d"
  if [ "$shot" = 1 ] && [ $shots = 0 ] && [ $(( end - $(date +%s) )) -lt 90 ]; then
    docker compose up -d --no-deps gazebo_gui >/dev/null 2>&1; shots=1
  fi
done
if [ "$shot" = 1 ]; then
  mkdir -p "runs/$run_id/screens"
  for cam in "11 -5.5 12 -0.298 0.298 0.641 0.641" "11 6.15 19 -0.5 0.5 0.5 0.5"; do
    set -- $cam
    docker compose exec -T gazebo_gui /entrypoint.sh bash -c "gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 --req 'pose: {position: {x: $1, y: $2, z: $3}, orientation: {x: $4, y: $5, z: $6, w: $7}}' >/dev/null; sleep 3; rm -f ~/.gz/gui/pictures/*; gz service -s /gui/screenshot --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 5000 --req 'data: \"\"' >/dev/null; sleep 4" >/dev/null 2>&1
    f=$(docker compose exec -T gazebo_gui bash -c 'ls ~/.gz/gui/pictures/*.png 2>/dev/null | head -1' | tr -d '\r')
    [ -n "$f" ] && docker compose cp "gazebo_gui:$f" "runs/$run_id/screens/$(date +%s).png" >/dev/null 2>&1
  done
fi
docker compose exec -T orchestrator /entrypoint.sh bash -c 'timeout 5 ros2 topic echo --once /fleet/robot_markers visualization_msgs/msg/MarkerArray --field markers 2>/dev/null | grep -c "ns: robots"' 2>/dev/null | head -1 | sed 's/^/robot markers seen: /'
docker compose logs > "runs/$run_id/compose.log" 2>&1
docker stats --no-stream --format '{{.Name}} {{.CPUPerc}} {{.MemUsage}}' | sed "s/$COMPOSE_PROJECT_NAME-//" > "runs/$run_id/cpu_end.txt"
docker compose run --rm --no-deps -T dev python3 tools/metrics/compute.py "runs/$run_id" >/dev/null 2>&1
cat "runs/$run_id/metrics.csv" 2>/dev/null
