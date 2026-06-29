#!/bin/bash
#SBATCH --job-name=stocheff-launchprod
#SBATCH --output=%x-%j.out
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --partition=amilan
#SBATCH --qos=normal
#SBATCH --time=00:15:00

# Bridge job: runs AFTER the plan exists. It reads the plan's task count, submits
# the production array sized to it, and then submits a re-runnable analyze job
# that depends on the production array. This indirection is needed because a
# Slurm --array size cannot reference a file that does not exist at submit time;
# here the plan is guaranteed to exist (this job depends on submit_plan).
set -euo pipefail
HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$HERE/env.sh"
source "$HERE/config.sh"

cd "$STOCHEFF_DIR"
NTASKS="$("$PY" runComparison.py --planFile "$PLAN_FILE" --printTaskCount)"
LAST=$(( NTASKS - 1 ))
echo "Production: $NTASKS array tasks from plan $PLAN_FILE"

COMMON=( --account="$STOCHEFF_ACCOUNT" --partition="$PARTITION" --qos="$QOS"
         --export=ALL )

PROD_ID=$(sbatch "${COMMON[@]}" --time="$TIME_GRID" \
    --array=0-"$LAST"%"$ARRAY_THROTTLE" \
    "$HERE/submit_production.sbatch" | awk '{print $NF}')
echo "submitted production array: $PROD_ID"

# Final analyze after the whole production array finishes OK.
ANA_ID=$(sbatch "${COMMON[@]}" --time="$TIME_ANALYZE" \
    --dependency=afterok:"$PROD_ID" \
    "$HERE/submit_analyze.sbatch" | awk '{print $NF}')
echo "submitted final analyze: $ANA_ID (after $PROD_ID)"
echo "$PROD_ID" > "$STOCHEFF_RESULTS_DIR/.production_jobid"
