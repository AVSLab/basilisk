#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

r"""
Bookkeeping-mass sweep for :ref:`scenarioCompareRwPanels`.

MuJoCo requires every wheel body to carry a positive mass, while the BSM
balanced-wheel effector is ideally massless; the scenario uses a 1e-6 kg
bookkeeping mass. This sweep reruns the comparison with that mass reduced to
confirm it is what sets the ~1e-10 rad cross-engine agreement floor.

Results are written to ``results/sweepRwBookkeepingMass.json``.
"""

import json
import os

import numpy as np

import _comparisonValidation
import scenarioCompareRwPanels as srp

from Basilisk.utilities import macros

resultsPath = os.path.join(os.path.dirname(__file__), "results")


def attitudeFloor(rwMass, dt=0.02, tf=120.0, recordDt=0.5):
    """Max cross-engine hub-attitude principal angle with the given wheel mass [kg]."""
    _comparisonValidation.validateTaskHorizon(
        "reaction-wheel bookkeeping-mass sweep", tf, dt)
    originalMass = srp.RW_MASS
    try:
        srp.RW_MASS = rwMass
        bsmState, _, _, _ = srp.runBSM(dt, tf, recordDt)
        mjState, _, _, _ = srp.runMujoco(dt, tf, recordDt)
    finally:
        srp.RW_MASS = originalMass
    bsmTimes = np.asarray(bsmState.times())*macros.NANO2SEC
    mujocoTimes = np.asarray(mjState.times())*macros.NANO2SEC
    sigmaBSM = np.array(bsmState.sigma_BN)
    sigmaMujoco = np.array(mjState.sigma_BN)
    sampleInterval = _comparisonValidation.recorderSampleInterval(
        dt, recordDt)
    _comparisonValidation.validateMatchingHistories(
        "reaction-wheel bookkeeping-mass BSM/MuJoCo",
        bsmTimes, mujocoTimes, tf, sampleInterval)
    _comparisonValidation.validateHistory(
        "reaction-wheel bookkeeping-mass BSM",
        bsmTimes, tf, sampleInterval, attitude=sigmaBSM)
    _comparisonValidation.validateHistory(
        "reaction-wheel bookkeeping-mass MuJoCo",
        mujocoTimes, tf, sampleInterval, attitude=sigmaMujoco)
    return float(np.max(srp.relativePrincipalAngle(sigmaBSM, sigmaMujoco)))


def run(rwMasses=(1.0e-6, 1.0e-8, 1.0e-10), dt=0.02, tf=120.0,
        recordDt=0.5, saveJson=True, resultsDir=None):
    """Run the bookkeeping-mass sweep and return the generated metrics."""
    if not srp.couldImportMujoco:
        raise ImportError("Build Basilisk with --mujoco to run this sweep.")

    rows = []
    for rwMass in rwMasses:
        floor = attitudeFloor(rwMass, dt, tf, recordDt)
        rows.append({"rwMass": rwMass, "hubAttitudePrincipalAngleMax": floor})
        print("RW_MASS {:.0e} kg -> attitude floor {:.3e} rad".format(rwMass, floor))

    metrics = {
        "scenario": "sweepRwBookkeepingMass",
        "configuration": {
            "dt": dt,
            "tf": tf,
            "recordDt": recordDt,
            "note": "scenarioCompareRwPanels with swept MuJoCo wheel bookkeeping mass",
        },
        "rows": rows,
    }
    if saveJson:
        targetResults = resultsPath if resultsDir is None else resultsDir
        os.makedirs(targetResults, exist_ok=True)
        outFile = os.path.join(targetResults, "sweepRwBookkeepingMass.json")
        with open(outFile, "w") as f:
            json.dump(metrics, f, indent=2)
        print("Wrote " + outFile)
    return metrics


if __name__ == "__main__":
    run()
