# swarmflow/robot — dev + Nav2 + ros_gz_bridge + foxglove_bridge + our robot packages (design §8.2).
# One image for robot_1..robot_3, configured by env vars (ROBOT_ID, ROBOT_NAMESPACE, SPAWN_VERTEX).
FROM dev

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

COPY src/ /ws/src/
COPY layouts/ /ws/layouts/
COPY docker/build_ws.sh /usr/local/bin/build_ws.sh
RUN build_ws.sh
