# swarmflow/dev — ROS 2 Jazzy build/test image (design §8.2). No Gazebo, no GUI, no GPU.
# Used by agents, CI (scripts/ci.sh), unit tests and the orchestrator / scenario_engine services.
# Base image digest and package notes: docker/versions.lock.md
FROM ros:jazzy-ros-base@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-launch-testing \
        ros-jazzy-launch-testing-ament-cmake \
        ros-jazzy-launch-pytest \
        ros-jazzy-ament-cmake-pytest \
        ros-jazzy-nav2-msgs \
        ros-jazzy-visualization-msgs \
        ros-jazzy-nav-msgs \
        ros-jazzy-geometry-msgs \
        ros-jazzy-xacro \
        python3-pytest \
        python3-pytest-timeout \
        python3-hypothesis \
        python3-yaml \
        python3-jsonschema \
        python3-numpy \
        python3-pil \
        python3-pip \
        git \
        procps \
    && rm -rf /var/lib/apt/lists/*

# Pure-Python roots (sim2d, tools/*) import swarmflow_core from source without a colcon build (design §8.2).
ENV PYTHONPATH=/ws/src/swarmflow_core

WORKDIR /ws
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
