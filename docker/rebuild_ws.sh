#!/bin/bash
# Rebuilds the ROS 2 workspace inside the container. Run this after editing
# anything other than plain Python scripts in the volume-mounted ./src
# (launch files, worlds, maps, URDF, message definitions, ...):
#
#   docker compose exec storagy-sim rebuild_ws.sh
#
# Pull/merge로 layout.json이 갱신됐으면 rebuild 시 Nav2 PGM·points·대시보드 PNG를
# layout 기준으로 자동 동기화한 뒤 colcon build 합니다 (SDF/layout.json은 유지).
#
# then restart the simulation (close the sim terminal in the noVNC desktop
# and run run_sim.sh again, or `docker compose restart`).
set -e

WS=/opt/storagy_sim_origin_ws
source /opt/ros/humble/setup.bash
cd "${WS}"

# Pull/merge로 받은 layout.json → Nav2 PGM·points·대시보드 PNG 동기화
GEN="${WS}/tools/generate_2026_amr_world.py"
LAYOUT="${WS}/src/storagy/worlds/2026_amr_layout.json"
if [ -f "${GEN}" ] && [ -f "${LAYOUT}" ]; then
    echo "[rebuild_ws] syncing Nav2 map from ${LAYOUT}..."
    python3 "${GEN}" --from-layout
elif [ -f "${LAYOUT}" ] && [ ! -f "${GEN}" ]; then
    echo "[rebuild_ws] WARN: ${GEN} not found — Nav2 map not synced"
fi

# storagy_llm's setup.py installs this file; recreate the placeholder if the
# host checkout doesn't have one (the real key comes from the environment).
[ -f src/storagy_llm/storagy_llm/.env ] \
    || printf 'OPENAI_API_KEY=\n' > src/storagy_llm/storagy_llm/.env

colcon build --symlink-install \
    --packages-select storagy_interfaces storagy storagy_llm storagy_hide storagy_guide

echo
echo "[rebuild_ws] done — restart the simulation to pick up the changes."
