# M20 autonomy stack image. Single image, one service per node group in compose.
# Base pinned by digest in production (ADR-011) — tag used here for readability.
# On Jetson, swap base for an L4T/JetPack-matched ROS image (see OQ-13).
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-colcon-common-extensions \
      ros-humble-nav2-bringup ros-humble-robot-localization \
      ros-humble-foxglove-bridge ros-humble-rosbridge-suite \
      ros-humble-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*

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
    rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/healthcheck.py /healthcheck.py
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
