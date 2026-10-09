#!/bin/bash
# Build the colcon workspace at image build time (design §8.2: runtime images contain the built workspace).
# Throttled per AGENTS.md §5. Usage: build_ws.sh [colcon package-selection args]
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd /ws
if [ -z "$(find src -name package.xml -print -quit 2>/dev/null)" ]; then
  echo "build_ws.sh: no packages in src/, skipping colcon build"
  exit 0
fi
export MAKEFLAGS=-j2
colcon build --parallel-workers 2 --event-handlers console_direct- "$@"
rm -rf build log
