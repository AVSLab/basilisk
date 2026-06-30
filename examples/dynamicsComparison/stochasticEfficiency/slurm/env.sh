#!/bin/bash
# Environment setup for the stochastic-efficiency study on CU Boulder Alpine.
#
# Sourced by every Slurm job script. Edit the few marked variables for your
# account / install, then everything else is automatic.
#
# Alpine specifics this encodes (https://curc.readthedocs.io):
#   * Slurm scheduler; load the slurm/alpine module on compute nodes.
#   * LMOD module system; Python via miniforge (mamba) or a prebuilt venv.
#   * Write results to /scratch/alpine/$USER (large, fast) NOT /home (10 GB cap).

set -euo pipefail

# ============================ EDIT THESE ===================================
# Your CURC allocation account, e.g. ucb123_asc1.  Find yours with:
#   sacctmgr -p show assoc user=$USER format=account
export STOCHEFF_ACCOUNT="${STOCHEFF_ACCOUNT:-ucb-general}"

# Absolute path to the Basilisk checkout on Alpine (contains the built .venv or
# the dist3 install).  EDIT to where you cloned/built Basilisk.
export BASILISK_ROOT="${BASILISK_ROOT:-/projects/$USER/stochastic_mc/basilisk}"

# How to make Basilisk importable. Two supported modes:
#   "venv"  -> activate $BASILISK_ROOT/.venv (a prebuilt virtualenv with MuJoCo)
#   "conda" -> activate a conda env named $STOCHEFF_CONDA_ENV
export STOCHEFF_PYMODE="${STOCHEFF_PYMODE:-venv}"
export STOCHEFF_CONDA_ENV="${STOCHEFF_CONDA_ENV:-basilisk}"

# Modules the .venv was BUILT against. These MUST be loaded at runtime too, or
# the venv's Python / Basilisk's compiled .so files (linked against this gcc's
# libstdc++) will fail to import on a fresh compute node. Match build_basilisk.sbatch.
export STOCHEFF_GCC_MODULE="${STOCHEFF_GCC_MODULE:-gcc/13.2.0}"
export STOCHEFF_PY_MODULE="${STOCHEFF_PY_MODULE:-python/3.10.2}"
# ===========================================================================

# Study directory (this file lives in <study>/slurm/).
export STOCHEFF_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

# Results go to scratch by default (purged after ~90 days; copy out what you keep).
export STOCHEFF_RESULTS_DIR="${STOCHEFF_RESULTS_DIR:-/scratch/alpine/$USER/stocheff/results}"
mkdir -p "$STOCHEFF_RESULTS_DIR"

# Keep BLAS/OpenMP single-threaded: we parallelise across Slurm tasks, not within
# a realization, so per-task multithreading would only cause oversubscription.
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
# Numba caches compiled kernels (the profile arm's NumbaModel); give it a stable,
# node-local-or-scratch cache dir so the JIT compile happens once, not per task.
export NUMBA_CACHE_DIR="${NUMBA_CACHE_DIR:-$STOCHEFF_RESULTS_DIR/.numba_cache}"
mkdir -p "$NUMBA_CACHE_DIR"

# --- module + interpreter setup ---
module purge
module load slurm/alpine

if [[ "$STOCHEFF_PYMODE" == "conda" ]]; then
    module load miniforge
    conda activate "$STOCHEFF_CONDA_ENV"
    export PY=python
else
    # venv mode: load the SAME gcc + python modules the venv was built against
    # (the venv's interpreter and Basilisk's .so files depend on them), THEN
    # activate the prebuilt virtualenv which carries compiled MuJoCo + Basilisk.
    module load "$STOCHEFF_GCC_MODULE" "$STOCHEFF_PY_MODULE"
    # shellcheck disable=SC1091
    source "$BASILISK_ROOT/.venv/bin/activate"
    export PY="$BASILISK_ROOT/.venv/bin/python"
fi

# Sanity: confirm Basilisk imports before burning a job.
"$PY" -c "from Basilisk.simulation import svIntegrators, mujoco; print('Basilisk import OK')"

echo "STOCHEFF_DIR=$STOCHEFF_DIR"
echo "STOCHEFF_RESULTS_DIR=$STOCHEFF_RESULTS_DIR"
echo "PY=$PY"
