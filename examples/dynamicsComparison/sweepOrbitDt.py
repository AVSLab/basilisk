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
Integrator-step sweep for :ref:`scenarioCompareOrbit`.

Repeats the approximately two-orbit Keplerian comparison over a ladder of RK4
steps at one common, exactly aligned final epoch to separate the two error
sources the base scenario reports at a single ``dt``:

#. the per-engine error against the analytic Kepler solution, expected to shrink as
   :math:`dt^4` (RK4 truncation); and
#. the cross-engine BSM-vs-MuJoCo difference, expected to be ``dt``-independent
   (formulation/round-off seeded, not time-stepping).

Results are written to ``results/sweepOrbitDt.json``.
"""

import json
import os

import numpy as np

import _comparisonValidation
import scenarioCompareOrbit as sco

from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeGravBody

resultsPath = os.path.join(os.path.dirname(__file__), "results")


def comparisonRow(dt, tf, recordDt, bsmTimes, mujocoTimes,
                  posBSM, posMujoco, truth):
    """Validate one sweep rung and compute its position-error metrics.

    Args:
        dt (float): RK4 integration step [s].
        tf (float): requested propagation horizon [s].
        recordDt (float): recorder sampling interval [s].
        bsmTimes (array-like): BSM sample timestamps [s].
        mujocoTimes (array-like): MuJoCo sample timestamps [s].
        posBSM (array-like): BSM inertial positions [m].
        posMujoco (array-like): MuJoCo inertial positions [m].
        truth (array-like): analytic inertial positions at ``bsmTimes`` [m].

    Returns:
        dict: integration step and maximum analytic/cross-engine position errors.

    Raises:
        ValueError: if either history is incomplete, malformed, or sampled at
            timestamps different from the other engine.
    """
    sampleInterval = _comparisonValidation.recorderSampleInterval(
        dt, recordDt)
    _comparisonValidation.validateMatchingHistories(
        "orbit-step BSM/MuJoCo", bsmTimes, mujocoTimes, tf, sampleInterval)
    _comparisonValidation.validateHistory(
        "orbit-step BSM", bsmTimes, tf, sampleInterval,
        position=posBSM, analyticPosition=truth)
    _comparisonValidation.validateHistory(
        "orbit-step MuJoCo", mujocoTimes, tf, sampleInterval,
        position=posMujoco)
    return {
        "dt": dt,  # [s]
        "bsmVsAnalyticMax": float(np.max(
            np.linalg.norm(truth - posBSM, axis=1))),  # [m]
        "mujocoVsAnalyticMax": float(np.max(
            np.linalg.norm(truth - posMujoco, axis=1))),  # [m]
        "crossParadigmPosMax": float(np.max(
            np.linalg.norm(posBSM - posMujoco, axis=1))),  # [m]
    }


def run(dts=(40.0, 20.0, 10.0, 5.0, 2.5), simDuration=None,
        recordDt=60.0, saveJson=True, resultsDir=None):
    """Run the step-size sweep and return the generated metrics.

    Args:
        dts (sequence, optional): RK4 integration steps [s].
        simDuration (float, optional): propagation horizon [s]. Defaults to two
            periods of the configured orbit.
        recordDt (float, optional): recorder sampling interval [s].
        saveJson (bool, optional): write ``sweepOrbitDt.json``. Defaults to True.
        resultsDir (str, optional): explicit artifact directory. Defaults to
            the scenario ``results`` folder.

    Returns:
        dict: sweep configuration, error rows, and fitted truncation orders.
    """
    if not sco.couldImportMujoco:
        raise ImportError("Build Basilisk with --mujoco to run this sweep.")

    mass = 750.0  # [kg]
    mu = simIncludeGravBody.BODY_DATA["earth"].mu  # [m^3/s^2]
    dts = tuple(dts)
    if not dts:
        raise ValueError("At least one integration step is required.")

    rN, vN, oe0 = sco.initialOrbitState(mu)
    orbitPeriod = 2.0*np.pi*np.sqrt(oe0.a**3/mu)  # [s]
    requestedTf = (
        2.0*orbitPeriod if simDuration is None else float(simDuration)
    )  # [s]
    sampleIntervals = tuple(
        _comparisonValidation.recorderSampleInterval(dt, recordDt)
        for dt in dts
    )  # [s]
    tf = _comparisonValidation.alignedHorizon(
        requestedTf, dts + sampleIntervals)  # [s]

    rows = []
    for dt in dts:
        bsmRec = sco.runBSM(mass, mu, dt, tf, recordDt)
        mjRec = sco.runMujoco(mu, dt, tf, recordDt)
        bsmTimes = np.array(bsmRec.times())*macros.NANO2SEC
        mujocoTimes = np.array(mjRec.times())*macros.NANO2SEC
        posBSM = np.array(bsmRec.r_BN_N)
        posMujoco = np.array(mjRec.r_BN_N)
        truth = sco.keplerTruth(mu, oe0.a, oe0, bsmTimes)
        rows.append(comparisonRow(
            dt, tf, recordDt, bsmTimes, mujocoTimes,
            posBSM, posMujoco, truth))
        print("dt {:5.1f} s: BSM vs Kepler {:.3e} m, cross-engine {:.3e} m".format(
            dt, rows[-1]["bsmVsAnalyticMax"], rows[-1]["crossParadigmPosMax"]))

    # Fitted truncation order between successive rungs of the analytic-error ladder.
    orders = []
    for lo, hi in zip(rows[1:], rows[:-1]):
        orders.append(np.log(hi["bsmVsAnalyticMax"]/lo["bsmVsAnalyticMax"])
                      / np.log(hi["dt"]/lo["dt"]))
    print("pairwise fitted truncation orders:", ["{:.2f}".format(p) for p in orders])

    metrics = {
        "scenario": "sweepOrbitDt",
        "configuration": {
            "requestedTf": requestedTf,
            "tf": tf,
            "recordDt": recordDt,
            "orbitPeriod": orbitPeriod,
            "orbitCount": tf/orbitPeriod,
            "orbit": "same elements as scenarioCompareOrbit",
        },
        "rows": rows,
        "pairwiseTruncationOrders": [float(p) for p in orders],
    }
    if saveJson:
        targetResults = resultsPath if resultsDir is None else resultsDir
        os.makedirs(targetResults, exist_ok=True)
        outFile = os.path.join(targetResults, "sweepOrbitDt.json")
        with open(outFile, "w") as f:
            json.dump(metrics, f, indent=2)
        print("Wrote " + outFile)
    return metrics


if __name__ == "__main__":
    run()
