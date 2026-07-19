#!/usr/bin/env bash
# Idempotent runtime-dependency check for docker-commander-1.
# WHY: on this disk-tight box we apt-install into the RUNNING container instead of
# rebuilding the image; if the container is ever recreated those installs vanish.
# Every bringup script calls this so a stale container self-heals (or fails LOUDLY
# offline) instead of silently missing nodes. Keep docker/Dockerfile in sync — it is
# the reproducible record; this script is the live-container stopgap.
C=docker-commander-1
DEPS="ros-humble-pointcloud-to-laserscan ros-humble-rmw-cyclonedds-cpp"

for d in $DEPS; do
  if ! docker exec $C dpkg -s "$d" >/dev/null 2>&1; then
    echo "[container_deps] $d missing — installing..."
    docker exec $C bash -c "apt-get update -qq >/dev/null && apt-get install -y -qq $d >/dev/null" \
      || { echo "[container_deps] FAILED to install $d (offline?) — aborting"; exit 1; }
    echo "[container_deps] $d installed"
  fi
done
echo "[container_deps] ok"
