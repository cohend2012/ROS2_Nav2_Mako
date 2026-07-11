#!/usr/bin/env bash
# One-time GUI prerequisite: OpenGL/mesa libraries for MuJoCo rendering under WSLg.
# Needs sudo. A minimal WSL Ubuntu ships no OpenGL libs; WSLg only provides the
# D3D12 interop in /usr/lib/wsl/lib, so mesa supplies the GL-on-D3D12 bridge that
# both the MuJoCo window (GLX) and the offscreen renderer (EGL) require.
set -e
echo "==> Installing mesa / OpenGL libraries..."
sudo apt-get update
sudo apt-get install -y \
  libgl1 libglx-mesa0 libgl1-mesa-dri libglx0 \
  libegl1 libegl-mesa0 libgbm1 libgles2 \
  libosmesa6 libglfw3
echo
echo "==> Done. Now run:  make sim-setup   then   make sim-view"
