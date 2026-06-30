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

# Absolute path to the Basilisk checkout (contains the built .venv / dist3).
# Derived automatically from this script's location (slurm/ is 4 levels below the
# repo root: <root>/examples/dynamicsComparison/stochasticEfficiency/slurm), so it
# is correct wherever you cloned Basilisk. Override by exporting BASILISK_ROOT.
export BASILISK_ROOT="${BASILISK_ROOT:-$( cd "$( dirname "${BASH_SOURCE[0]}" )/../../../.." && pwd )}"

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
# We run env.sh in two contexts:
#   * inside a Slurm job (compute node)  -> modules MUST load; be strict.
#   * on the submit/login node (dry-run, run_study.sh sizing) -> CURC often
#     blocks compiler modules on login nodes, so be soft: warn and continue, the
#     real jobs load them fine on compute nodes (the build proved this).
if [[ -n "${SLURM_JOB_ID:-}" ]]; then
    STRICT=1            # on a compute node
else
    STRICT=0            # on the login/submit node
fi

# module commands shouldn't abort the (set -e) script on the login node.
load_mod() {  # load_mod <module>...; returns nonzero instead of exiting on failure
    module load "$@" 2>/dev/null
}

module purge 2>/dev/null || true
load_mod slurm/alpine || true

if [[ "$STOCHEFF_PYMODE" == "conda" ]]; then
    load_mod miniforge || true
    conda activate "$STOCHEFF_CONDA_ENV" || true
    export PY=python
else
    # venv mode: the ONLY runtime requirement is the gcc module -- not the
    # compiler, but its libstdc++.so.6. Basilisk's .so files were built with
    # gcc/13.2.0 and need GLIBCXX_3.4.32, which Alpine's system libstdc++ lacks;
    # loading the gcc module puts the newer libstdc++ on the library path.
    # (Verified by diag_modules.sbatch: gcc-only imports OK; no-modules fails
    # with the GLIBCXX error.) The .venv carries its own Python, so the python
    # module is NOT needed and is loaded only best-effort.
    if ! load_mod "$STOCHEFF_GCC_MODULE"; then
        if [[ "$STRICT" == "1" ]]; then
            echo "ERROR: could not load $STOCHEFF_GCC_MODULE on the compute node" \
                 "(needed for libstdc++/GLIBCXX_3.4.32)." >&2
            exit 1
        fi
        echo "WARN: could not load $STOCHEFF_GCC_MODULE here" \
             "(expected on a login node; compute-node jobs will load it)." >&2
    fi
    # Python module is optional (the venv has its own interpreter); ignore failure.
    load_mod "$STOCHEFF_PY_MODULE" || true
    if [[ -f "$BASILISK_ROOT/.venv/bin/activate" ]]; then
        # shellcheck disable=SC1091
        source "$BASILISK_ROOT/.venv/bin/activate"
    elif [[ "$STRICT" == "1" ]]; then
        echo "ERROR: no venv at $BASILISK_ROOT/.venv (BASILISK_ROOT wrong?)." >&2
        exit 1
    else
        echo "WARN: no venv at $BASILISK_ROOT/.venv -- check BASILISK_ROOT." >&2
    fi
    export PY="$BASILISK_ROOT/.venv/bin/python"
fi

# Sanity: confirm Basilisk imports. Strict (fail) inside a job; best-effort on
# the login node, where the compiler module may be unavailable.
if "$PY" -c "from Basilisk.simulation import svIntegrators, mujoco; print('Basilisk import OK')"; then
    :
else
    if [[ "$STRICT" == "1" ]]; then
        echo "ERROR: Basilisk import failed on the compute node." >&2
        exit 1
    fi
    echo "WARN: Basilisk import not verified on this login node (likely the gcc" \
         "module can't load here). It will be verified inside each job." >&2
fi

echo "STOCHEFF_DIR=$STOCHEFF_DIR"
echo "STOCHEFF_RESULTS_DIR=$STOCHEFF_RESULTS_DIR"
echo "PY=$PY"
