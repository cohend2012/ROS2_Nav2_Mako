#!/usr/bin/env bash
# Entry for the `agent` service: language missions over /m20/mission/run.
#   M20_AGENT      claude | cosmos | offline (default offline; claude/cosmos fall back to it)
#   M20_LANDMARKS  landmark file in m20_agent/config (oil_gas_field_landmarks.yaml |
#                  office_landmarks.yaml) or an absolute path (e.g. /data/my_office.yaml)
#   /data          taught places ("this is the kitchen") + mission reports, persisted
set -euo pipefail
# ROS setup files are not nounset-safe (AMENT_TRACE_SETUP_FILES etc.): under set -u
# they abort the script SILENTLY. Relax -u only while sourcing (found in the
# container verification build, 2026-11-05).
set +u
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
set -u
LM=${M20_LANDMARKS:-oil_gas_field_landmarks.yaml}
case "$LM" in /*) ;; *) LM="$(ros2 pkg prefix m20_agent)/share/m20_agent/config/$LM" ;; esac
[ -f "$LM" ] || { echo "[agent] landmark file not found: $LM"; exit 1; }
mkdir -p /data/agent
echo "[agent] backend=${M20_AGENT:-offline} landmarks=$LM"
exec ros2 run m20_agent agent --ros-args -p backend:="${M20_AGENT:-offline}" \
  -p landmarks:="$LM" -p taught_places:=/data/taught_places.yaml -p report_dir:=/data/agent
