#!/bin/bash
#SBATCH --job-name=stocheff-pilotref
#SBATCH --output=%x-%j.out
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --partition=amilan
#SBATCH --qos=normal
#SBATCH --time=04:00:00

# Pilot reference (single job): PILOT_N_REF samples of the scipy ground truth,
# enough for planBudget.py to estimate sigma(FoM) and the SDE biases.
set -euo pipefail
HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$HERE/env.sh"
source "$HERE/config.sh"

cd "$STOCHEFF_DIR"
srun "$PY" runComparison.py --reference \
    --orbits "$ORBITS" --stationaryStd "$STATIONARY_STD" \
    --nSamples "$PILOT_N_REF" --saveEvery 200

echo "Pilot reference done."
