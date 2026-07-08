#!/bin/bash
#SBATCH --job-name=bsk-build
#SBATCH --output=bsk-build-%j.out
#SBATCH --partition=amilan
#SBATCH --qos=normal
#SBATCH --account=ucb-general
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=08:00:00

# One-shot, unattended build of Basilisk on Alpine for the stochastic study.
#
# Submit it and walk away:
#     sbatch build_basilisk.sbatch
#     tail -f bsk-build-<JOBID>.out      # watch progress
#
# Why a batch job (not interactive): it survives disconnects, queues for the
# memory it needs, and logs everything to a file -- no copy/paste required.
#
# The two things that broke earlier builds are handled here:
#   1. GLIBC error from the conan protoc binary  -> build with --vizInterface False
#      (the study uses only svIntegrators + mujoco, not Vizard/protobuf).
#   2. OOM "Killed signal terminated program cc1plus" -> cap compile parallelism
#      to BUILD_JOBS so each g++ has enough RAM (template-heavy files need ~3-4 GB
#      each; with --mem=32G, 4 parallel compiles is safe).
#
# Fully resumable: the C++ build is incremental, so if this is killed (walltime,
# preemption) just resubmit -- it continues from where it stopped.

set -euo pipefail

# ---- EDIT IF YOUR PATHS DIFFER ----
BSK_ROOT="/projects/$USER/stochastic_mc/basilisk"
GCC_MODULE="gcc/13.2.0"
PY_MODULE="python/3.10.2"
# Compile parallelism. ~ mem_GB / 4, capped at cpus-per-task. With --mem=32G and
# 4 cores, 4 is safe. Drop to 2 (or 1) if you still see "Killed ... cc1plus".
BUILD_JOBS="${BUILD_JOBS:-4}"
# -----------------------------------

echo "=== node resources ==="
echo "host=$(hostname)  cores=$(nproc)  mem=$(free -g | awk '/Mem:/{print $2}')GB"
echo "SLURM_CPUS_PER_TASK=${SLURM_CPUS_PER_TASK:-?}  SLURM_MEM_PER_NODE=${SLURM_MEM_PER_NODE:-?}MB"
echo "BUILD_JOBS=$BUILD_JOBS"

# ---- environment ----
module purge
module load slurm/alpine "$GCC_MODULE" "$PY_MODULE"
gcc --version | head -1
python --version

# keep conan + pip caches OFF /home (10 GB cap); /projects is large
export CONAN_HOME="/projects/$USER/.conan2"
export PIP_CACHE_DIR="/projects/$USER/.pipcache"
mkdir -p "$CONAN_HOME" "$PIP_CACHE_DIR"

# THROTTLE the C++ build to fit memory. cmake.build() in conanfile.py honors
# CMAKE_BUILD_PARALLEL_LEVEL; CONAN_CPU_COUNT bounds any conan-side parallelism.
export CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS"
export CONAN_CPU_COUNT="$BUILD_JOBS"

cd "$BSK_ROOT"
source .venv/bin/activate

echo "=== starting build (mujoco ON, vizInterface OFF) ==="
date
python conanfile.py --mujoco True --vizInterface False
status=$?
date

if [[ $status -eq 0 ]]; then
    echo "=== build OK; verifying the import the study needs ==="
    # Also assert the specific stochastic integrators the study's SDE arm uses
    # (SRA1/SOSRA/W2Ito2) are present, so a build against stale integrator
    # sources is caught here rather than at run time.
    python -c "
from Basilisk.simulation import svIntegrators, mujoco
missing = [n for n in ('svStochasticIntegratorSRA1','svStochasticIntegratorSOSRA','svStochasticIntegratorW2Ito2')
           if not hasattr(svIntegrators, n)]
assert not missing, 'missing integrators: %s' % missing
print('Basilisk OK')
"
else
    echo "=== build FAILED (exit $status). If it was OOM (Killed cc1plus), resubmit"
    echo "    with a smaller parallelism, e.g.:  sbatch --export=BUILD_JOBS=2 build_basilisk.sbatch"
fi
exit $status
