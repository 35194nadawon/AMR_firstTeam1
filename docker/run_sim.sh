#!/bin/bash
# Launches the full Storagy simulation inside the noVNC desktop session.
# Auto-started by /etc/xdg/autostart/storagy-sim.desktop, but you can also
# run it manually from a terminal inside the desktop.
set -e

export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1   # software OpenGL: works without a GPU

# Wait until the VNC X server (:1) is up before starting GUI apps.
for _ in $(seq 1 60); do
    [ -e /tmp/.X11-unix/X1 ] && break
    sleep 1
done
sleep 3

source /opt/ros/humble/setup.bash
source /opt/storagy_sim_origin_ws/install/setup.bash

cd /opt/storagy_sim_origin_ws
echo "==================================================================="
echo " Starting Storagy simulation (Gazebo + Nav2 + YOLO + LLM + Web)"
echo " Web dashboard will be available at: http://localhost:8090"
echo "==================================================================="
exec ros2 launch storagy full_bringup.launch.py
