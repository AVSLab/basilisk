#!/bin/bash
#SBATCH --job-name=stocheff-analyze
#SBATCH --output=%x-%j.out
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --partition=amilan
#SBATCH --qos=normal
#SBATCH --time=01:00:00

# Analyse + plot the results that EXIST RIGHT NOW. Safe to run at any time --
# during the sweep, between stages, or at the end -- because the analysis reads
# partial results via sampleStore.loadMerged (consolidated file + all shards,
# de-duplicated by seed) and never re-simulates. Run it repeatedly to watch the
# Pareto/bias figures sharpen as samples stream in.
#
# It does NOT consolidate (that deletes shards); it only rebuilds the manifest
# from whatever sample files are present, then analyses + plots.
set -euo pipefail
HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$HERE/env.sh"
source "$HERE/config.sh"

cd "$STOCHEFF_DIR"

echo "== rebuild manifest from current sample files =="
srun "$PY" runComparison.py --rebuildManifest \
    --orbits "$ORBITS" --stationaryStd "$STATIONARY_STD"

echo "== analyse (std + mean), bias-cost on $ANALYZE_CORES cores =="
srun "$PY" analyzeResults.py --moment std  --cores "$ANALYZE_CORES" \
    --relTarget "$REL_TARGET" --negligibleFrac "$NEGLIGIBLE_FRAC"
srun "$PY" analyzeResults.py --moment mean --cores "$ANALYZE_CORES" || true

echo "== plots (bias-vs-cost is the headline accuracy figure) =="
srun "$PY" plotResults.py --moment std
srun "$PY" plotResults.py --moment mean || true

echo "Analysis done. Figures + JSON in $STOCHEFF_RESULTS_DIR"
