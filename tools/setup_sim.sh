#!/usr/bin/env bash
# Reproducible setup for the MuJoCo sim PREVIEW ("see the robot").
# No sudo. Installs mujoco (pip --user) and fetches the PINNED vendor M20 model.
# One-time GUI prerequisite (mesa, needs sudo): tools/install_gui_deps.sh
set -e
SIM_DIR="${M20_SIM_DIR:-$HOME/m20_sim}"
SDK_SHA="80e3d40084c4ed151ba6f88b0d55cf1d480aa45e"   # sdk_deploy pin (tools/vendor.repos)
DESC="$SIM_DIR/sdk_deploy/src/M20_sdk_deploy/M20_description"

echo "==> Ensuring pip is available..."
python3 -m pip --version >/dev/null 2>&1 || {
  curl -sS https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py
  python3 /tmp/get-pip.py --user
}

echo "==> Installing mujoco + pillow (pip --user)..."
python3 -c "import mujoco" 2>/dev/null || python3 -m pip install --user mujoco
python3 -c "import PIL"    2>/dev/null || python3 -m pip install --user pillow

echo "==> Fetching pinned M20 model ($SDK_SHA)..."
if [ ! -d "$DESC" ]; then
  mkdir -p "$SIM_DIR"; cd "$SIM_DIR"
  git clone --filter=blob:none --sparse https://github.com/DeepRoboticsLab/sdk_deploy.git
  cd sdk_deploy
  git sparse-checkout set src/M20_sdk_deploy/M20_description
  git checkout "$SDK_SHA"
fi

# Install the test arena next to M20.xml so its <include> + ../meshes resolve.
SCENE_SRC="$(cd "$(dirname "$0")" && pwd)/sim/scene_test.xml"
if [ -f "$SCENE_SRC" ]; then
  cp "$SCENE_SRC" "$DESC/m20_mjcf/mjcf/scene_test.xml"
  echo "==> Installed test arena (scene_test.xml)"
fi

echo "==> Validating model loads..."
python3 - "$DESC/m20_mjcf/mjcf/M20.xml" <<'PY'
import sys, mujoco
m = mujoco.MjModel.from_xml_path(sys.argv[1])
print(f"    OK: M20 bodies={m.nbody} joints={m.njnt} actuators={m.nu} dofs={m.nv}")
PY
echo "==> Done. View it with:  make sim-view   (or bash tools/view_sim.sh)"
