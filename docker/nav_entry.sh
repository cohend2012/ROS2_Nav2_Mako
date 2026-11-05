#!/usr/bin/env bash
# Entry for the `nav` service (docker/compose.yaml). Builds this site's Nav2 params at
# container start, then launches classical Nav2 or an AI mode.
#   M20_SITE        outdoor | office          (office = real footprint overlay, 0.3 m/s)
#   M20_NAV_MODE    nav2 | follow_path | standalone | direct   (nav2 = classical, default)
#   M20_NAV_VARIANT dwb | shim_rpp | rpp_cm | lattice_rpp | mppi   (classical variants)
#   M20_CM          1|0 collision_monitor after the smoother (default: 1 in the office)
#   M20_MODEL       ONNX policy for AI modes (default: the package's m20_nav_policy.onnx)
set -euo pipefail
# ROS setup files are not nounset-safe (AMENT_TRACE_SETUP_FILES etc.): under set -u
# they abort the script SILENTLY. Relax -u only while sourcing (found in the
# container verification build, 2026-11-05).
set +u
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
set -u
CFG="$(ros2 pkg prefix m20_navigation)/share/m20_navigation/config"
SITE=${M20_SITE:-outdoor}; MODE=${M20_NAV_MODE:-nav2}; VARIANT=${M20_NAV_VARIANT:-dwb}
if [ -z "${M20_CM:-}" ]; then
  if [ "$SITE" = office ]; then CM=1; else CM=0; fi
else
  CM=$M20_CM
fi
OVER=()
if [ "$SITE" = office ]; then OVER+=("$CFG/nav2_office_overlay.yaml"); fi
if [ "$VARIANT" != dwb ]; then
  if [ ! -f "$CFG/office_variants/$VARIANT.yaml" ]; then
    echo "[nav] unknown M20_NAV_VARIANT=$VARIANT"; exit 1
  fi
  OVER+=("$CFG/office_variants/$VARIANT.yaml")
fi
if [ ${#OVER[@]} -gt 0 ]; then
  python3 /ws/tools/apply_overlay.py "$CFG/nav2_params.yaml" "${OVER[@]}" /tmp/nav2_params.yaml
else
  cp "$CFG/nav2_params.yaml" /tmp/nav2_params.yaml
fi
CM_ARG=false
if [ "$CM" = 1 ]; then CM_ARG=true; fi
echo "[nav] site=$SITE mode=$MODE variant=$VARIANT collision_monitor=$CM_ARG"
exec ros2 launch m20_ai_nav ai_nav.launch.py mode:="$MODE" site:="$SITE" \
  params_file:=/tmp/nav2_params.yaml collision_monitor:="$CM_ARG" \
  ${M20_MODEL:+model_path:=$M20_MODEL}
