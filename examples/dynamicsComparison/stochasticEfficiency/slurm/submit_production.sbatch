#!/bin/bash
#SBATCH --job-name=stocheff-prod
#SBATCH --output=%x-%A_%a.out
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --partition=amilan
#SBATCH --qos=normal
#SBATCH --time=23:00:00

# STAGE 3 (job array): the precision-targeted production run. The plan
# (PLAN_FILE) sets each config's target N and shard count -- INCLUDING the
# reference, which is sharded here because it needs ~1e6 samples. One array task
# = one (config, shard) entry of the plan's task list.
#
# Fully resumable + re-runnable: each shard skips seeds it already has, so a
# timed-out task or a re-submit only computes what is missing.
set -euo pipefail
# Slurm copies this script to a spool dir, so ${BASH_SOURCE[0]} is NOT the
# slurm/ folder. Use the dir exported by run_study.sh / SLURM_SUBMIT_DIR, with
# BASH_SOURCE as a last resort.
HERE="${STOCHEFF_SLURM_DIR:-${SLURM_SUBMIT_DIR:-$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )}}"
source "$HERE/env.sh"
source "$HERE/config.sh"

cd "$STOCHEFF_DIR"
srun "$PY" runComparison.py \
    --planFile "$PLAN_FILE" \
    --taskListIndex "$SLURM_ARRAY_TASK_ID" \
    --orbits "$ORBITS" --stationaryStd "$STATIONARY_STD" \
    --saveEvery 200

echo "Production task $SLURM_ARRAY_TASK_ID done."
