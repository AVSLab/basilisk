#!/bin/bash
# One-command launcher for the full precision-targeted study on Alpine.
#
# Stages, chained by Slurm dependencies:
#   1. PILOT      array  : cheap uniform sweep to MEASURE biases + per-run cost
#      + pilot reference  : (runs alongside the pilot array)
#   2. PLAN       1 job  : planBudget.py -> per-config target N + shard counts so
#                          MC error <= REL_TARGET x bias, and reference precise
#                          enough to resolve the smallest bias  (writes plan.json)
#   3. launch     1 job  : reads the plan, submits the PRODUCTION array sized to
#                          it, and an analyze job depending on it
#   4. PRODUCTION array  : the real run (per-config N, reference sharded)
#   5. ANALYZE    1 job  : final figures + JSON  (afterok production)
#
# You can ALSO run analysis any time results are streaming in:
#   sbatch <slurm flags> submit_analyze.sbatch
# or locally on a login node:  ./watch_analyze.sh
#
# Usage:
#   cd <study>/slurm
#   # edit env.sh (account, BASILISK_ROOT) and config.sh (REL_TARGET etc.)
#   ./run_study.sh --dry-run     # show the plan of submissions
#   ./run_study.sh               # submit the whole chain
#   ./run_study.sh --from-plan   # skip pilot+plan, (re)launch production from an
#                                # existing plan.json (e.g. after editing it)

set -euo pipefail
HERE="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Exported so submitted sbatch scripts (which Slurm copies to a spool dir, losing
# their own location) can find env.sh/config.sh in the real slurm/ dir. Inherited
# by jobs via --export=ALL, and by the launch_production bridge job too.
export STOCHEFF_SLURM_DIR="$HERE"

MODE="full"
[[ "${1:-}" == "--dry-run" ]] && MODE="dry"
[[ "${1:-}" == "--from-plan" ]] && MODE="fromplan"

source "$HERE/env.sh"
source "$HERE/config.sh"
cd "$STOCHEFF_DIR"

COMMON=( --account="$STOCHEFF_ACCOUNT" --partition="$PARTITION" --qos="$QOS"
         --export=ALL )

sub() {  # echo, then sbatch (or just echo in dry mode); prints job id
    if [[ "$MODE" == "dry" ]]; then echo "DRYRUN sbatch $*" >&2; echo "DRYRUN"; return; fi
    sbatch "$@" | awk '{print $NF}'
}

echo "Account=$STOCHEFF_ACCOUNT qos=$QOS partition=$PARTITION"
echo "Results -> $STOCHEFF_RESULTS_DIR ; plan -> $PLAN_FILE"

if [[ "$MODE" == "fromplan" ]]; then
    if [[ ! -f "$PLAN_FILE" ]]; then echo "No plan at $PLAN_FILE"; exit 1; fi
    echo "Launching production directly from existing plan $PLAN_FILE."
    LID=$(sub "${COMMON[@]}" --time="$TIME_PLAN" "$HERE/launch_production.sbatch")
    echo "launch-production job: $LID  (submits the production array + final analyze)"
    exit 0
fi

# --- stage 1: pilot array + pilot reference (concurrent) ---
PILOT_TASKS="$("$PY" runComparison.py --nShards "$PILOT_SHARDS" --printTaskCount)"
PLAST=$(( PILOT_TASKS - 1 ))
echo "Pilot: $PILOT_TASKS array tasks (nShards=$PILOT_SHARDS)"
PILOT_ID=$(sub "${COMMON[@]}" --time="$TIME_PILOT" \
    --array=0-"$PLAST"%"$ARRAY_THROTTLE" "$HERE/submit_pilot.sbatch")
echo "pilot array: $PILOT_ID"
PREF_ID=$(sub "${COMMON[@]}" --time="$TIME_PILOT" "$HERE/submit_pilot_ref.sbatch")
echo "pilot reference: $PREF_ID"

# --- stage 2: plan (after pilot + pilot ref) ---
if [[ "$MODE" == "dry" ]]; then
    echo "DRYRUN plan depends on afterok:$PILOT_ID:$PREF_ID"
    echo "DRYRUN launch-production depends on afterok:<plan>"
    exit 0
fi
PLAN_ID=$(sub "${COMMON[@]}" --time="$TIME_PLAN" \
    --dependency=afterok:"$PILOT_ID":"$PREF_ID" "$HERE/submit_plan.sbatch")
echo "plan job: $PLAN_ID"

# --- stage 3: launch production (after plan) ---
LAUNCH_ID=$(sub "${COMMON[@]}" --time="$TIME_PLAN" \
    --dependency=afterok:"$PLAN_ID" "$HERE/launch_production.sbatch")
echo "launch-production job: $LAUNCH_ID  (submits the production array + final analyze)"
echo
echo "Watch:   squeue --me"
echo "Anytime: sbatch ${COMMON[*]} --time=$TIME_ANALYZE $HERE/submit_analyze.sbatch"
echo "         (or ./watch_analyze.sh on a login node to refresh figures live)"
