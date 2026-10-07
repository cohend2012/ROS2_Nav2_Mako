#!/usr/bin/env bash
# One-time setup for the splat demo on a fresh machine (Linux or WSL2 Ubuntu).
# Safe to re-run: every step skips what is already in place.
#
#   bash tools/splat/setup_splat_demo.sh --splat <path or https URL to gaussians_indoor.ply>
#   bash tools/splat/setup_splat_demo.sh --check        # report only, change nothing
#
# Options:
#   --splat PATH|URL   the splat PLY (not in git, ~213 MB); checked against the sha256 in the scene yaml
#   --urdf-zip PATH    M20 URDF zip (optional; default: fetch the pinned DeepRoboticsLab/URDF_model)
#   --scene NAME       scene to prepare (default indoor_splat)
#   --rebuild          rebuild the Docker images even if they exist
#   --check            only report what is missing
#
# Needs: Docker with the NVIDIA Container Toolkit (`docker run --gpus all` works), git, python3,
# ~35 GB free disk for the images. Then run:  M20_EXPLORE=1 bash tools/splat/splat_sim_up.sh
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
SIM_DIR="$HOME/m20_sim"
URDF_SHA="75824b516ecc8fa3f8f7ce5d6577e1e86d6b614f"     # DeepRoboticsLab/URDF_model pin
SPLAT_SRC="" URDF_ZIP="" SCENE="indoor_splat" REBUILD=0 CHECK=0
while [ $# -gt 0 ]; do
  case "$1" in
    --splat) SPLAT_SRC="$2"; shift 2 ;;
    --urdf-zip) URDF_ZIP="$2"; shift 2 ;;
    --scene) SCENE="${2%.xml}"; shift 2 ;;
    --rebuild) REBUILD=1; shift ;;
    --check) CHECK=1; shift ;;
    -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
    *) echo "unknown option: $1 (see --help)"; exit 2 ;;
  esac
done

ok()   { echo "  [ok]   $*"; }
todo() { echo "  [todo] $*"; MISSING=$((MISSING + 1)); }
fail() { echo "  [FAIL] $*"; exit 1; }
step() { echo; echo "==> $*"; }
MISSING=0
SCENE_YAML="$REPO/tools/sim/$SCENE.scene.yaml"
[ -f "$SCENE_YAML" ] || fail "no scene yaml $SCENE_YAML"
SPLAT_FILE=$(awk '/^  file:/{print $2; exit}' "$SCENE_YAML")
SPLAT_SHA=$(awk '/^  sha256:/{print $2; exit}' "$SCENE_YAML")

# ---------------------------------------------------------------- 1. prerequisites
step "1/6 prerequisites"
for c in docker git python3; do
  command -v "$c" >/dev/null || fail "$c not found — install it first"
done
docker info >/dev/null 2>&1 || fail "Docker is not running (WSL: start Docker Desktop, enable WSL integration)"
ok "docker, git, python3"
if docker run --rm --gpus all ubuntu:22.04 nvidia-smi -L >/dev/null 2>&1; then
  ok "GPU in containers ($(docker run --rm --gpus all ubuntu:22.04 nvidia-smi -L | head -1 | cut -c1-60))"
else
  fail "'docker run --gpus all' fails — install the NVIDIA driver + NVIDIA Container Toolkit"
fi
FREE_GB=$(df -Pk "$(docker info -f '{{.DockerRootDir}}' 2>/dev/null || echo /)" 2>/dev/null | awk 'NR==2{print int($4/1048576)}')
[ "${FREE_GB:-0}" -ge 35 ] && ok "disk: ${FREE_GB} GB free" \
  || echo "  [warn] only ${FREE_GB:-?} GB free for Docker; the images need ~25 GB"

# ---------------------------------------------------------------- 2. Docker images
step "2/6 Docker images (first build: 30-60 min, mostly m20_splat)"
have_img() { docker image inspect "$1" >/dev/null 2>&1; }
build() {  # build <tag> <dockerfile>
  echo "  building $1 from $2 ..."
  docker build -f "$REPO/$2" -t "$1" "$REPO"
}
if [ $REBUILD = 1 ] || ! have_img m20_autonomy:latest; then
  if [ $CHECK = 1 ]; then todo "image m20_autonomy"; else
    build m20_autonomy:latest docker/Dockerfile
    build m20_autonomy:latest docker/Dockerfile.autonomy-cyclone      # CycloneDDS layer
    REBUILD=1                                                          # dependants must rebuild
  fi
else ok "m20_autonomy"; fi
for pair in m20_sim:docker/Dockerfile.sim m20_splat:docker/Dockerfile.splat; do
  img="${pair%%:*}:latest" df="${pair#*:}"
  if [ $REBUILD = 1 ] || ! have_img "$img"; then
    if [ $CHECK = 1 ]; then todo "image $img"; else build "$img" "$df"; fi
  else ok "${img%:latest}"; fi
done

# ---------------------------------------------------------------- 3. commander
step "3/6 commander container"
if docker ps --format '{{.Names}}' | grep -q '^docker-commander-1$'; then ok "docker-commander-1 running"
elif [ $CHECK = 1 ]; then todo "commander (docker compose up -d commander)"
else (cd "$REPO/docker" && docker compose up -d commander) && ok "commander started"; fi

# ---------------------------------------------------------------- 4. MuJoCo model + scenes
step "4/6 MuJoCo robot model + scenes"
MJCF_DIR="$SIM_DIR/sdk_deploy/src/M20_sdk_deploy/M20_description/m20_mjcf/mjcf"
if [ -f "$MJCF_DIR/$SCENE.xml" ] && cmp -s "$REPO/tools/sim/$SCENE.xml" "$MJCF_DIR/$SCENE.xml"; then
  ok "vendor model + $SCENE.xml installed"
elif [ $CHECK = 1 ]; then todo "sim model (bash tools/setup_sim.sh)"
else bash "$REPO/tools/setup_sim.sh"; fi

# ---------------------------------------------------------------- 5. splat
step "5/6 splat ($SPLAT_FILE)"
DEST="$SIM_DIR/splats/$SPLAT_FILE"
sha_ok() { [ -f "$1" ] && [ "$(sha256sum "$1" | cut -d' ' -f1)" = "$SPLAT_SHA" ]; }
if sha_ok "$DEST"; then ok "$DEST (sha256 matches)"
elif [ $CHECK = 1 ]; then todo "splat: re-run with --splat <path or URL to $SPLAT_FILE>"
elif [ -z "$SPLAT_SRC" ]; then
  todo "splat missing — get $SPLAT_FILE from the team and re-run with --splat <path or URL>"
else
  mkdir -p "$SIM_DIR/splats"
  case "$SPLAT_SRC" in
    http://*|https://*) echo "  downloading ..."; curl -fL --progress-bar -o "$DEST.part" "$SPLAT_SRC" && mv "$DEST.part" "$DEST" ;;
    *) [ -f "$SPLAT_SRC" ] || fail "no such file: $SPLAT_SRC"; cp "$SPLAT_SRC" "$DEST" ;;
  esac
  sha_ok "$DEST" && ok "$DEST (sha256 matches)" \
    || { rm -f "$DEST"; fail "$SPLAT_SRC does not match the scene's sha256 — wrong splat? (geometry + map were made from $SPLAT_SHA)"; }
fi

# ---------------------------------------------------------------- 6. URDF (viewer fallback drawing)
step "6/6 M20 URDF (~/m20_sim/m20_urdf)"
URDF_DIR="$SIM_DIR/m20_urdf"
if [ -f "$URDF_DIR/M20.urdf" ]; then ok "$URDF_DIR/M20.urdf"
elif [ $CHECK = 1 ]; then todo "URDF (fetched automatically on a normal run)"
else
  TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
  if [ -n "$URDF_ZIP" ]; then
    python3 -m zipfile -e "$URDF_ZIP" "$TMP"
  else
    git -C "$TMP" clone -q --filter=blob:none --sparse https://github.com/DeepRoboticsLab/URDF_model.git src
    git -C "$TMP/src" sparse-checkout set M20/urdf && git -C "$TMP/src" checkout -q "$URDF_SHA"
  fi
  U=$(find "$TMP" -name M20.urdf -not -path '*/.git/*' | head -1)
  [ -n "$U" ] || fail "no M20.urdf found in ${URDF_ZIP:-URDF_model}"
  mkdir -p "$URDF_DIR" && cp -r "$(dirname "$U")"/. "$URDF_DIR"/
  ok "$URDF_DIR/M20.urdf"
fi

# ---------------------------------------------------------------- summary
echo
if [ $MISSING -eq 0 ]; then
  echo "All set. Launch the demo (stays in the foreground, Ctrl-C stops it):"
  echo "  M20_EXPLORE=1 bash tools/splat/splat_sim_up.sh"
  echo "then open http://localhost:8080"
else
  echo "$MISSING item(s) still to do (see [todo] above)."; exit 1
fi
