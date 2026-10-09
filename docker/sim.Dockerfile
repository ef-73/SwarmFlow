# swarmflow/sim — dev + Gazebo Harmonic + ros_gz (+ Xvfb/VNC/noVNC for the browser GUI route, design §8.4).
# Used by the gazebo, gazebo_gui and payload services. Built FROM the dev image (compose additional_contexts).
FROM dev

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        libgl1-mesa-dri \
        mesa-utils \
        xvfb \
        x11vnc \
        novnc \
        websockify \
        openbox \
    && rm -rf /var/lib/apt/lists/*

COPY src/ /ws/src/
COPY layouts/ /ws/layouts/
COPY docker/build_ws.sh /usr/local/bin/build_ws.sh
COPY docker/gui.sh /usr/local/bin/gui.sh
RUN build_ws.sh
