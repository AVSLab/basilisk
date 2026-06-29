#!/bin/bash
#SBATCH --job-name=stocheff-plan
#SBATCH --output=%x-%j.out
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --partition=amilan
#SBATCH --qos=normal
#SBATCH --time=00:30:00

# STAGE 2 (single job): turn the pilot into the precision-targeted plan.
# Reads the pilot biases, computes per-config target N (so MC error <= REL_TARGET
# x bias) and the reference N (precise enough to resolve the smallest bias), and
# writes PLAN_FILE. Cheap; no simulation.
set -euo pipefail
HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$HERE/env.sh"
source "$HERE/config.sh"

cd "$STOCHEFF_DIR"
# Make sure the manifest reflects the pilot's (possibly sharded) sample files.
srun "$PY" runComparison.py --rebuildManifest \
    --orbits "$ORBITS" --stationaryStd "$STATIONARY_STD"

srun "$PY" planBudget.py \
    --relTarget "$REL_TARGET" --negligibleFrac "$NEGLIGIBLE_FRAC" \
    --profileN "$PROFILE_N" --perTaskSeconds "$PER_TASK_SECONDS" \
    --out "$PLAN_FILE"

echo "Plan written to $PLAN_FILE"
