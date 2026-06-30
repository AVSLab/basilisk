#!/bin/bash
#SBATCH --job-name=stocheff-diag
#SBATCH --output=stocheff-diag-%j.out
#SBATCH --partition=amilan
#SBATCH --qos=normal
#SBATCH --account=ucb-general
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=1
#SBATCH --time=00:05:00

# Diagnostic: determine the MINIMAL module set the prebuilt .venv needs on a
# compute node to import Basilisk. Tries progressively, reports each result.
# (Put in a file so terminal line-wrapping can't corrupt a --wrap string.)
#
# Run from the slurm/ dir:  sbatch diag_modules.sbatch
# Then:                     cat stocheff-diag-<JOBID>.out

VENV="/projects/$USER/stochastic_mc/basilisk/.venv"
IMPORT='from Basilisk.simulation import svIntegrators, mujoco; print("   IMPORT_OK")'

echo "HOST=$(hostname)"
echo

echo "=== what python modules exist (no compiler loaded) ==="
module purge 2>/dev/null
module --default avail python 2>&1 | grep -iE "python|conda|forge" || true
echo

echo "=== A) gcc/13.2.0 only, then activate venv ==="
module purge 2>/dev/null
module load gcc/13.2.0 2>&1
source "$VENV/bin/activate"
echo "   which python: $(which python)"
python -c "$IMPORT" 2>&1 | tail -3
echo

echo "=== B) gcc/13.2.0 + python/3.10.2, then activate venv ==="
deactivate 2>/dev/null || true
module purge 2>/dev/null
module load gcc/13.2.0 2>&1
module load python/3.10.2 2>&1   # may fail; we want to SEE the error
echo "   (python module load exit: $?)"
source "$VENV/bin/activate"
echo "   which python: $(which python)"
python -c "$IMPORT" 2>&1 | tail -3
echo

echo "=== C) no modules at all, just the venv (does libstdc++ already suffice?) ==="
deactivate 2>/dev/null || true
module purge 2>/dev/null
source "$VENV/bin/activate"
python -c "$IMPORT" 2>&1 | tail -3
echo

echo "=== done ==="
