#!/bin/bash
# Record the 3D demo video material (takes the sim lock): v2 warehouse, 4 robots, the v2_demo order stream plus one
# pallet order at the start, three Gazebo cameras rendering on the GPU (docker/compose.record.yaml).
# Usage: tools/metrics/record_demo.sh <run_id> [sim_minutes=6] [pallet_spot=PL_W2] [dock=DK2]
# Then:  docker compose run --rm --no-deps -T --entrypoint python3 viz /ws/tools/metrics/make_demo_video.py /ws/runs/<id>
#        (make_demo_video.py needs the runs/ and tools/ mounts; see its docstring)
set -uo pipefail
export MSYS_NO_PATHCONV=1
cd "$(git rev-parse --show-toplevel)"
run_id="${1:?run_id}"; sim_min="${2:-6}"; spot="${3:-PL_W2}"; dock="${4:-DK2}"
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-swarmflow-$(basename "$(pwd)" | tr A-Z a-z)}"
export SWARMFLOW_RUN_ID="$run_id" SWARMFLOW_SCENARIO=v2_demo SWARMFLOW_POLICY=assign SWARMFLOW_AGENT=true
export SWARMFLOW_LAYOUT=warehouse_v2 SWARMFLOW_ROBOTS=robot_1,robot_2,robot_3,robot_4 SWARMFLOW_TRAFFIC_CONTROL=false
export SWARMFLOW_GUI=none SWARMFLOW_GIT_SHA="$(git rev-parse HEAD)"
dc() { docker compose -f compose.yaml -f docker/compose.record.yaml "$@"; }
py="$(command -v python3 || command -v python)"
rm -rf "runs/$run_id"; mkdir -p "runs/$run_id/video"
scripts/sim_lock.sh acquire "video:$run_id" 20 >/dev/null || exit 2
cleanup() { dc --profile extra_robots --profile gui down >/dev/null 2>&1; scripts/sim_lock.sh release >/dev/null; }
trap cleanup EXIT
dc --profile extra_robots --profile gui down >/dev/null 2>&1
dc up -d --force-recreate gazebo >/dev/null 2>&1
for _ in $(seq 1 40); do [ "$(docker inspect -f '{{.State.Health.Status}}' "$COMPOSE_PROJECT_NAME-gazebo-1" 2>/dev/null)" = healthy ] && break; sleep 3; done
dc exec -d -T gazebo /entrypoint.sh bash -c "python3 /ws/tools/metrics/record_cams.py /ws/runs/$run_id/video > /ws/runs/$run_id/video/record.log 2>&1"
dc up -d orchestrator scenario_engine payload robot_1 robot_2 robot_3 robot_4 >/dev/null 2>&1
echo "[$(date -u +%H:%M:%S)] up, recording"
cap=$(( $(date +%s) + 17 * 60 ))
sim_info() {  # "<sim seconds left in our window> <pallet delivered 0|1>"
  "$py" - "runs/$run_id" "$sim_min" <<'PY' 2>/dev/null || echo "9999 0"
import json, pathlib, sys, yaml
d = pathlib.Path(sys.argv[1]); cfg = yaml.safe_load((d / "config.yaml").read_text())
end = cfg["start_time_sim_s"] + (cfg.get("start_delay_s") or 0) + float(sys.argv[2]) * 60
last = json.loads((d / "robot_states.jsonl").read_text().splitlines()[-1])["t"]
done = any('"pallet"' in l and '"DELIVERED"' in l for l in (d / "orders.jsonl").read_text().splitlines())
print(int(max(0, end - last)), int(done))
PY
}
injected=0
while [ "$(date +%s)" -lt "$cap" ]; do
  sleep 10
  if [ $injected = 0 ] && [ -f "runs/$run_id/config.yaml" ]; then
    dc exec -T orchestrator /entrypoint.sh bash -c "cd /ws/tools/scenarios && python3 -m scenarios.order_cli pallet --spot $spot --dock $dock --id video_pallet" \
      && injected=1 && echo "[$(date -u +%H:%M:%S)] pallet order $spot -> $dock published"
  fi
  read -r left pallet <<< "$(sim_info)"
  d=$(grep -c '"state": "DELIVERED"' "runs/$run_id/orders.jsonl" 2>/dev/null); d=${d:-0}
  echo "[$(date -u +%H:%M:%S)] delivered $d, pallet delivered $pallet, sim left ${left}s, $(tail -1 runs/$run_id/video/record.log 2>/dev/null)"
  [ "$left" -le 0 ] && break
done
dc exec -T gazebo pkill -INT -f record_cams.py
for _ in $(seq 1 30); do grep -q '^done' "runs/$run_id/video/record.log" && break; sleep 2; done
tail -1 "runs/$run_id/video/record.log"
dc logs > "runs/$run_id/compose.log" 2>&1
docker compose run --rm --no-deps -T dev python3 tools/metrics/compute.py "runs/$run_id" >/dev/null 2>&1
cat "runs/$run_id/metrics.csv" 2>/dev/null
