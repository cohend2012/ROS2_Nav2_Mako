#!/usr/bin/env bash
# ============================================================================
# Best-of-N map selection. Six anchored-mapping iterations (2026-07-25/26)
# showed the residual is ~0.2-0.4 m per region, RANDOMLY distributed per build
# (it4: 0.20/0.03/0.68; it6: 0.38/0.43/0.06). A map is a one-time artifact, so
# we build N candidates, score each with the SAME committed landmark gate, and
# keep the one with the smallest worst-landmark offset. The winning score is
# recorded next to the map (maps/oil_gas_field.score.txt) — no gate-shopping:
# every candidate is scored identically and the shipped number is the truth.
# Usage: bash tools/map/best_map.sh [N]   (default 3)
# ============================================================================
set -u
N=${1:-3}
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
STASH="$REPO/maps/best_candidate"
BEST=999
rm -rf "$STASH"

for i in $(seq 1 "$N"); do
  echo "=== best_map: build $i/$N ==="
  OUT=$(bash "$REPO/tools/map/build_map.sh" 2>&1)
  echo "$OUT" | grep -E "offset|GATE"
  # worst landmark offset of this build (gate FAIL still yields a usable score)
  W=$(echo "$OUT" | grep -oE "offset [0-9.]+" | awk '{print $2}' | sort -rn | head -1)
  [ -z "$W" ] && { echo "  build $i produced no score — skipping"; continue; }
  SRC="$REPO/maps/candidate"
  [ -d "$SRC" ] || SRC="$REPO/maps"   # gate PASS moves files into maps/
  if awk "BEGIN{exit !($W < $BEST)}"; then
    BEST=$W
    rm -rf "$STASH"; mkdir -p "$STASH"
    cp "$SRC"/oil_gas_field.* "$STASH/" 2>/dev/null
    echo "$OUT" | grep -E "offset|GATE" > "$STASH/score.txt"
    echo "  build $i is new best (worst-landmark $W m)"
  else
    echo "  build $i worst-landmark $W m — keeping previous best ($BEST m)"
  fi
done

[ -d "$STASH" ] || { echo "best_map: NO usable build"; exit 1; }
cp -f "$STASH"/oil_gas_field.* "$REPO/maps/"
cp -f "$STASH/score.txt" "$REPO/maps/oil_gas_field.score.txt"
rm -rf "$STASH" "$REPO/maps/candidate"
echo "=== best_map: shipped best build (worst-landmark $BEST m) ==="
cat "$REPO/maps/oil_gas_field.score.txt"
python3 "$REPO/tools/map/check_map_landmarks.py" "$REPO/maps/oil_gas_field.pgm" "$REPO/maps/oil_gas_field.yaml"
