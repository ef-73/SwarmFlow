#!/bin/bash
# Gazebo GUI client launcher for the gazebo_gui service (design §8.4).
# SWARMFLOW_GUI=wslg|vnc|none (agents and CI always use none).
set -e
mode="${SWARMFLOW_GUI:-wslg}"
# SwarmFlow GUI layout with the robot dashboard panel (T021), if installed
gui_cfg=(); cfg=/ws/install/swarmflow_viz/share/swarmflow_viz/config/gazebo_gui.config
[ -f "$cfg" ] && gui_cfg=(--gui-config "$cfg")
case "$mode" in
  none)
    echo "gui.sh: SWARMFLOW_GUI=none, GUI disabled"; exit 0 ;;
  wslg)
    export DISPLAY="${DISPLAY:-:0}" QT_QPA_PLATFORM=xcb
    if [ ! -S /tmp/.X11-unix/X0 ]; then
      echo "gui.sh: WSLg X11 socket /tmp/.X11-unix/X0 not found; use SWARMFLOW_GUI=vnc" >&2; exit 1
    fi
    exec gz sim -g -v 2 "${gui_cfg[@]}" ;;
  vnc)
    export DISPLAY=:1 QT_QPA_PLATFORM=xcb
    unset WAYLAND_DISPLAY  # x11vnc exits if it sees a Wayland session
    Xvfb :1 -screen 0 "${VNC_GEOMETRY:-1600x900}x24" -nolisten tcp &
    sleep 1
    openbox &
    x11vnc -display :1 -forever -shared -nopw -quiet -rfbport 5900 &
    websockify --web /usr/share/novnc 6080 localhost:5900 &
    echo "gui.sh: noVNC at http://localhost:6080/vnc.html"
    exec gz sim -g -v 2 "${gui_cfg[@]}" ;;
  *)
    echo "gui.sh: unknown SWARMFLOW_GUI=$mode (expected wslg|vnc|none)" >&2; exit 2 ;;
esac
