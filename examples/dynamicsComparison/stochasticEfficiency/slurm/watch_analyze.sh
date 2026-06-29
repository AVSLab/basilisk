#!/bin/bash
# Refresh the analysis + figures from whatever results exist RIGHT NOW, on a
# loop, so you can watch the Pareto/bias plots sharpen while the sweep runs.
#
# Runs on a LOGIN NODE (it is light: it only reads cached samples and replots,
# never simulates). For anything heavier, submit submit_analyze.sbatch instead.
#
# Usage:
#   ./watch_analyze.sh            # one pass
#   ./watch_analyze.sh 300        # re-analyse every 300 s until Ctrl-C
set -euo pipefail
HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$HERE/env.sh"
source "$HERE/config.sh"
cd "$STOCHEFF_DIR"

INTERVAL="${1:-0}"

one_pass() {
    "$PY" runComparison.py --rebuildManifest \
        --orbits "$ORBITS" --stationaryStd "$STATIONARY_STD"
    "$PY" analyzeResults.py --moment std --cores "$ANALYZE_CORES" \
        --relTarget "$REL_TARGET" --negligibleFrac "$NEGLIGIBLE_FRAC"
    "$PY" plotResults.py --moment std
    echo "[$(date '+%H:%M:%S')] refreshed figures in $STOCHEFF_RESULTS_DIR"
}

if [[ "$INTERVAL" == "0" ]]; then
    one_pass
else
    echo "Re-analysing every ${INTERVAL}s (Ctrl-C to stop)..."
    while true; do one_pass; sleep "$INTERVAL"; done
fi
