#!/bin/bash
# Hello multi-container test (design §8.1 step 4): container A publishes a counter at 10 Hz,
# container B must RECEIVE >= 50 messages in 10 s (a listed-but-silent topic fails).
# Runs two `dev` containers on the compose project network. Usage: tests/integration/test_multi_container.sh
set -uo pipefail
export MSYS_NO_PATHCONV=1  # Git Bash on Windows: do not rewrite /topic args into C:/... paths
cd "$(git rev-parse --show-toplevel)"
export COMPOSE_PROJECT_NAME="${COMPOSE_PROJECT_NAME:-swarmflow-$(basename "$(pwd)")}"
MIN="${MIN_MESSAGES:-50}"
talker="${COMPOSE_PROJECT_NAME}-hello-talker"

cleanup() { docker rm -f "$talker" >/dev/null 2>&1 || true; }
trap cleanup EXIT
cleanup

docker compose run -d --rm --no-deps --name "$talker" dev \
  ros2 topic pub -r 10 /swarmflow_hello std_msgs/msg/Int32 "{data: 1}" >/dev/null
sleep 3  # talker process start-up

count=$(docker compose run --rm --no-deps dev bash -c \
  'sleep 2; timeout -s INT 10 ros2 topic echo /swarmflow_hello std_msgs/msg/Int32 --field data 2>/dev/null | grep -c "^1$"' \
  | tr -d '\r' | tail -1)
count="${count:-0}"
echo "hello-multi-container: received $count messages in 10 s (need >= $MIN)"
if [ "$count" -ge "$MIN" ]; then echo "PASS"; exit 0; else echo "FAIL"; exit 1; fi
