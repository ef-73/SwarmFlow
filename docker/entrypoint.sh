#!/bin/bash
# Source ROS and the workspace overlay (if built), then run the command.
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
