# Storagy ROS 2 simulation — runs on any OS (Mac/Windows/Ubuntu), no GPU needed.
#
# Base provides a full Linux desktop (MATE) reachable from a web browser via
# noVNC, so Gazebo and RViz windows show up in the browser. Works on amd64
# and arm64 (Apple Silicon).
FROM tiryoh/ros2-desktop-vnc:humble

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# 1. System dependencies
#    - Gazebo Harmonic (gz-sim8) + ros_gz (Harmonic) from the OSRF apt repo
#    - Nav2 + slam_toolbox + simulation support packages from the ROS repo
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release ca-certificates \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-humble-ros-gzharmonic \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-cv-bridge \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-rviz2 \
        python3-pip \
 && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. Python dependencies (CPU-only PyTorch — no CUDA pulled in)
# ---------------------------------------------------------------------------
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu \
        torch torchvision \
 && pip3 install --no-cache-dir -r /tmp/requirements.txt

# ---------------------------------------------------------------------------
# 3. Build the ROS 2 workspace
#    Only the simulation packages are built; Nav2/SLAM come from apt above.
# ---------------------------------------------------------------------------
ENV WS=/opt/storagy_sim_origin_ws
WORKDIR ${WS}
COPY . ${WS}

# A placeholder .env lets `colcon build` succeed (storagy_llm's setup.py
# installs this file). The real OPENAI_API_KEY is injected at runtime via the
# container environment and takes precedence over this empty value.
RUN mkdir -p ${WS}/src/storagy_llm/storagy_llm \
 && printf 'OPENAI_API_KEY=\n' > ${WS}/src/storagy_llm/storagy_llm/.env

RUN source /opt/ros/humble/setup.bash \
 && colcon build --symlink-install \
        --packages-select storagy_interfaces storagy storagy_llm storagy_hide \
 && chmod -R a+rX ${WS}

# ---------------------------------------------------------------------------
# 4. Helper scripts + auto-start of the simulation inside the desktop session
#    rebuild_ws.sh re-runs colcon build — use it after editing files in the
#    volume-mounted ./src on the host.
# ---------------------------------------------------------------------------
RUN install -m 755 ${WS}/docker/run_sim.sh /usr/local/bin/run_sim.sh \
 && install -m 755 ${WS}/docker/rebuild_ws.sh /usr/local/bin/rebuild_ws.sh \
 && mkdir -p /etc/xdg/autostart \
 && cat > /etc/xdg/autostart/storagy-sim.desktop <<'EOF'
[Desktop Entry]
Type=Application
Name=Storagy Simulation
Comment=Launch the full Storagy simulation
Exec=mate-terminal --maximize --title="Storagy Simulation" -- bash -lc "/usr/local/bin/run_sim.sh; echo; echo '[simulation process exited] press Enter to close'; read"
X-MATE-Autostart-enabled=true
Terminal=false
EOF

EXPOSE 80 8090
