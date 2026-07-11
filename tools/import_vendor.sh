#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p src/vendor
pip install vcstool --quiet 2>/dev/null || true
vcs import src/vendor < tools/vendor.repos
