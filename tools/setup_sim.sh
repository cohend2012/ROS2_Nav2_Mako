#!/usr/bin/env bash
# Reproducible setup for the MuJoCo sim PREVIEW ("see the robot").
# No sudo. Installs mujoco (pip --user) and fetches the PINNED vendor M20 model.
# One-time GUI prerequisite (mesa, needs sudo): tools/install_gui_deps.sh
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"   # resolve before any cd
SIM_DIR="${M20_SIM_DIR:-$HOME/m20_sim}"
SDK_SHA="80e3d40084c4ed151ba6f88b0d55cf1d480aa45e"   # sdk_deploy pin (tools/vendor.repos)
DESC="$SIM_DIR/sdk_deploy/src/M20_sdk_deploy/M20_description"
# Ubuntu 24.04+ marks the system Python externally managed (PEP 668); --user
# installs are still isolated from apt's packages, so allow them explicitly.
python3 -c 'import sys,sysconfig,os; sys.exit(not os.path.exists(os.path.join(sysconfig.get_path("stdlib"),"EXTERNALLY-MANAGED")))' \
  && export PIP_BREAK_SYSTEM_PACKAGES=1

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

# Install every scene (test arena, mapping arena, oil & gas field) next to M20.xml
# so its <include> + ../meshes resolve. Re-run after editing tools/sim/*.xml.
SCENES="$SCRIPT_DIR/sim"
for s in "$SCENES"/*.xml; do
  cp "$s" "$DESC/m20_mjcf/mjcf/"
  echo "==> Installed scene $(basename "$s")"
done

echo "==> Validating model + scenes load..."
python3 - "$DESC/m20_mjcf/mjcf" "$SCENES"/*.xml <<'PY'
import os, sys, mujoco
d = sys.argv[1]
for f in ["M20.xml"] + [os.path.basename(p) for p in sys.argv[2:]]:
    m = mujoco.MjModel.from_xml_path(os.path.join(d, f))
    print(f"    OK: {f} bodies={m.nbody} joints={m.njnt} actuators={m.nu} dofs={m.nv}")
PY
echo "==> Done. View it with:  make sim-view   (or bash tools/view_sim.sh)"
