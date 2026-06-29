#!/bin/bash
#SBATCH --job-name=stocheff-pilot
#SBATCH --output=%x-%A_%a.out
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --partition=amilan
#SBATCH --qos=normal
#SBATCH --time=04:00:00

# STAGE 1 (job array): a cheap uniform pilot whose only purpose is to MEASURE
# each config's bias and per-run cost, so planBudget.py can size the real run.
# Includes the reference (a pilot reference, PILOT_N_REF samples). One array task
# = one (config, shard) pair; SLURM_ARRAY_TASK_ID -> --taskListIndex.
set -euo pipefail
HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$HERE/env.sh"
source "$HERE/config.sh"

cd "$STOCHEFF_DIR"
# The pilot uses a uniform N for the grid and a separate (larger) N for the
# reference. We run the grid array here; the pilot reference is a 1-task add-on
# handled by run_study.sh (it just calls --reference). For a uniform pilot the
# reference is task index == (#grid tasks) when --includeReference is set, but to
# keep it simple run_study submits the pilot reference separately.
srun "$PY" runComparison.py \
    --taskListIndex "$SLURM_ARRAY_TASK_ID" \
    --nShards "$PILOT_SHARDS" \
    --orbits "$ORBITS" --stationaryStd "$STATIONARY_STD" \
    --nSamples "$PILOT_N" --saveEvery 100

echo "Pilot task $SLURM_ARRAY_TASK_ID done."
