#!/bin/bash
# Study parameters for the Alpine run, in one place. Sourced by all sbatch
# scripts AFTER env.sh. Override any of these from the shell before submitting.

# --- physical scenario (defaults already baked into the code; here for clarity) ---
export ORBITS="${ORBITS:-2.0}"                 # number of orbital periods
export STATIONARY_STD="${STATIONARY_STD:-0.3}" # OU noise strength

# --- stage 1: PILOT (cheap, fixed N) -------------------------------------------
# A small uniform sweep used only to *measure* each config's bias and per-run
# cost, which planBudget.py turns into the precision-targeted plan. It does not
# need to be accurate, just enough to estimate biases (a few hundred each).
export PILOT_N="${PILOT_N:-300}"          # samples per config in the pilot
export PILOT_N_REF="${PILOT_N_REF:-600}"  # samples for the pilot reference
export PILOT_SHARDS="${PILOT_SHARDS:-4}"  # shards per config in the pilot

# --- stage 2: PRECISION PLAN (planBudget.py) -----------------------------------
# REL_TARGET: resolve each integrator's bias to this relative MC precision.
#   0.10 => the Monte-Carlo error on every (resolved) bias is <=10% of that bias.
# NEGLIGIBLE_FRAC: biases below this fraction of sigma(FoM) are only *upper
#   bounded* (negligible for ranking), not resolved to REL_TARGET -- this is what
#   keeps the budget finite (resolving a ~0 bias to 10% needs N->inf).
# PER_TASK_SECONDS: target wall time per array task; planBudget shards each config
#   so one shard fits this (keep < the QOS walltime). 10 h is safe under 'normal'.
export REL_TARGET="${REL_TARGET:-0.10}"
export NEGLIGIBLE_FRAC="${NEGLIGIBLE_FRAC:-0.005}"
export PROFILE_N="${PROFILE_N:-3000}"     # samples per profile config (CRN: cheap)
export PER_TASK_SECONDS="${PER_TASK_SECONDS:-36000}"   # 10 h
export PLAN_FILE="${PLAN_FILE:-$STOCHEFF_RESULTS_DIR/plan.json}"

# --- Slurm resources -----------------------------------------------------------
export QOS="${QOS:-normal}"                # normal (1 day) | long (7 days)
export PARTITION="${PARTITION:-amilan}"
export TIME_PILOT="${TIME_PILOT:-04:00:00}"  # walltime per PILOT array task
export TIME_GRID="${TIME_GRID:-23:00:00}"    # walltime per PRODUCTION array task
export TIME_PLAN="${TIME_PLAN:-00:30:00}"
export TIME_ANALYZE="${TIME_ANALYZE:-01:00:00}"
# Concurrency throttle on the array (%N): cap simultaneously-running tasks to be
# friendly on the shared cluster. Each task uses 1 core.
export ARRAY_THROTTLE="${ARRAY_THROTTLE:-60}"
# Cores assumed when reporting parallel wall-time estimates in the analysis.
export ANALYZE_CORES="${ANALYZE_CORES:-64}"
