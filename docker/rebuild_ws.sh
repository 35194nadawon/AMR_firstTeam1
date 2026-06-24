#!/bin/bash
# Rebuilds the ROS 2 workspace inside the container. Run this after editing
# anything other than plain Python scripts in the volume-mounted ./src
# (launch files, worlds, maps, URDF, message definitions, ...):
#
#   docker compose exec storagy-sim rebuild_ws.sh
#
# then restart the simulation (close the sim terminal in the noVNC desktop
# and run run_sim.sh again, or `docker compose restart`).
set -e

source /opt/ros/humble/setup.bash
cd /opt/storagy_sim_origin_ws

# storagy_llm's setup.py installs this file; recreate the placeholder if the
# host checkout doesn't have one (the real key comes from the environment).
[ -f src/storagy_llm/storagy_llm/.env ] \
    || printf 'OPENAI_API_KEY=\n' > src/storagy_llm/storagy_llm/.env

colcon build --symlink-install \
    --packages-select storagy_interfaces storagy storagy_llm storagy_hide

echo
echo "[rebuild_ws] done — restart the simulation to pick up the changes."
