# M20 autonomy stack image. Single image, one service per node group in compose.
# Base pinned by digest in production (ADR-011) — tag used here for readability.
# On Jetson, swap base for an L4T/JetPack-matched ROS image (see OQ-13).
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-colcon-common-extensions \
      ros-humble-nav2-bringup ros-humble-robot-localization \
      ros-humble-foxglove-bridge ros-humble-rosbridge-suite \
      ros-humble-pointcloud-to-laserscan ros-humble-slam-toolbox \
      ros-humble-nav2-collision-monitor ros-humble-nav2-smac-planner \
      ros-humble-nav2-mppi-controller ros-humble-nav2-rotation-shim-controller \
      ros-humble-nav2-regulated-pure-pursuit-controller ros-humble-rmw-cyclonedds-cpp \
      python3-yaml python3-pil python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# M20-Brain runtime (docs/AI_NAV_BRIEF.md): ONNX policy inference for m20_ai_nav,
# VLM clients for m20_agent. NO torch / mujoco / SB3 here — training is offline
# (tools/ai_nav/train.py), only the exported .onnx ships. Pin versions for ADR-011.
RUN pip3 install --no-cache-dir "onnxruntime==1.23.2" "anthropic>=2.4,<3" "openai>=1.60,<4"

WORKDIR /ws
COPY src ./src
# Vendor deps are pinned in tools/vendor.repos (ADR-003) but NOT built into the core
# stack image by default: they need their own toolchains (drdds, mujoco, onnxruntime,
# Isaac) and belong to later phases (bridge/SLAM/RL). Keeping them out keeps the
# Phase-0 artifact lean and buildable. Enable when wanted: --build-arg IMPORT_VENDOR=true
ARG IMPORT_VENDOR=false
COPY tools/vendor.repos tools/vendor.repos
RUN if [ "$IMPORT_VENDOR" = "true" ]; then \
      (pip install vcstool --break-system-packages 2>/dev/null || pip install vcstool); \
      mkdir -p src/vendor && vcs import src/vendor < tools/vendor.repos || true; \
    fi

# drdds: vendor joint-interface messages (ADR-015) — a small pure-msg package that the
# bridge (and the MuJoCo sim node) depend on. Always baked in, pinned to sdk_deploy.
ARG DRDDS_SHA=80e3d40084c4ed151ba6f88b0d55cf1d480aa45e
RUN git clone --filter=blob:none --sparse https://github.com/DeepRoboticsLab/sdk_deploy.git /tmp/sdk \
    && git -C /tmp/sdk sparse-checkout set src/drdds \
    && git -C /tmp/sdk checkout ${DRDDS_SHA} \
    && cp -r /tmp/sdk/src/drdds src/drdds \
    && rm -rf /tmp/sdk

RUN source /opt/ros/humble/setup.bash && \
    apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -y || true && \
    colcon build --symlink-install && \
    for p in m20_msgs m20_commander m20_locomotion_bridge m20_missions m20_behaviors \
             m20_navigation m20_ai_nav m20_agent m20_station m20_bringup; do \
      test -d install/$p || { echo "MISSING PACKAGE $p"; exit 1; }; done && \
    test -f install/m20_ai_nav/share/m20_ai_nav/models/m20_nav_policy.onnx && \
    test -f install/m20_navigation/share/m20_navigation/config/collision_monitor.yaml && \
    rm -rf /var/lib/apt/lists/*

# M20-Brain deployment pieces: params overlay tool, SLAM configs, service entries
COPY tools/nav/apply_overlay.py /ws/tools/apply_overlay.py
COPY tools/slam/localization_params.yaml tools/slam/mapper_params.yaml /ws/config/slam/
COPY docker/nav_entry.sh /nav_entry.sh
COPY docker/perception_entry.sh /perception_entry.sh
COPY docker/agent_entry.sh /agent_entry.sh
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/healthcheck.py /healthcheck.py
RUN chmod +x /entrypoint.sh /nav_entry.sh /perception_entry.sh /agent_entry.sh
ENTRYPOINT ["/entrypoint.sh"]
