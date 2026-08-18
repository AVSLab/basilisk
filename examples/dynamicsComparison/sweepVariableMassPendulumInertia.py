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

r"""Sweep the MuJoCo slosh-bob inertia and integration step in deep space.

The BSM spherical pendulum treats its bob as a point mass. MuJoCo requires a
positive centroidal inertia for the articulated bob body. This control varies that
initial inertia while retaining the same bob mass, depletion law, damping,
initial conditions, and burn. Repeating the sweep at two RK4 steps separates
the artificial-inertia contribution from the finite-step disagreement between
the two pendulum realizations.
"""

import json
import os

import matplotlib.pyplot as plt
import numpy as np

from Basilisk.utilities import macros

import _comparePlots
import _comparisonValidation
import scenarioCompareVariableMass as variableMass


fileName = os.path.basename(os.path.splitext(__file__)[0])
resultsPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")

BOB_INERTIAS = (1.0e-8, 1.0e-5, 1.0e-4, 1.0e-3)  # [kg*m^2]
TIME_STEPS = (0.05, 0.025, 0.0125)  # [s]


def _bsmReference(dt, simDuration):
    """Propagate one deep-space BSM reference at ``dt``."""
    simulation, recorders, _ = variableMass.buildBSM(
        dt, record=True, useThruster=True, inOrbit=False,
        simDuration=simDuration)
    simulation.ConfigureStopTime(macros.sec2nano(simDuration))
    simulation.ExecuteSimulation()
    bsm = variableMass.pullBSM(recorders, variableMass.earthMu())
    _comparisonValidation.validateHistory(
        "variable-mass inertia-sweep BSM",
        bsm["t"], simDuration, dt,
        attitude=bsm["sigma_BN"])
    return bsm


def _mujocoAttitudeError(bsm, dt, bobInertia, simDuration):
    """Return the maximum deep-space attitude difference for one MuJoCo bob inertia."""
    initialState = {
        "r_BN_N": bsm["r_BN_N"][0],
        "v_BN_N": bsm["v_BN_N"][0],
        "sigma_BN": bsm["sigma_BN"][0],
        "omega_BN_B": bsm["omega_BN_B"][0],
    }
    simulation, recorders, _ = variableMass.buildMujoco(
        dt,
        record=True,
        initialState=initialState,
        useThruster=True,
        inOrbit=False,
        pendulumBobInertia=bobInertia,
    )
    simulation.ConfigureStopTime(macros.sec2nano(simDuration))
    simulation.ExecuteSimulation()
    mujoco = variableMass.pullMujoco(recorders, variableMass.earthMu())
    _comparisonValidation.validateMatchingHistories(
        "variable-mass inertia-sweep BSM/MuJoCo",
        bsm["t"], mujoco["t"], simDuration, dt)
    _comparisonValidation.validateHistory(
        "variable-mass inertia-sweep MuJoCo",
        mujoco["t"], simDuration, dt,
        attitude=mujoco["sigma_BN"])
    error = variableMass.relativePrincipalAngle(bsm["sigma_BN"], mujoco["sigma_BN"])
    return float(np.max(error))


def run(showPlots=False, saveJson=False, bobInertias=BOB_INERTIAS,
        timeSteps=TIME_STEPS, simDuration=variableMass.SIM_DURATION,
        resultsDir=None):
    """Run the inertia and step-size control and return its one-axis figure."""
    if not variableMass.couldImportMujoco:
        raise ImportError("Build Basilisk with --mujoco to run this sweep.")

    bobInertias = tuple(bobInertias)
    timeSteps = tuple(timeSteps)
    if not bobInertias or not timeSteps:
        raise ValueError("At least one bob inertia and time step are required.")
    errors = {}
    for dt in timeSteps:
        bsm = _bsmReference(dt, simDuration)
        errors[dt] = [
            _mujocoAttitudeError(bsm, dt, bobInertia, simDuration)
            for bobInertia in bobInertias
        ]

    metrics = {
        "scenario": fileName,
        "simDuration": simDuration,
        "bobInertia": list(bobInertias),
        "timeStep": list(timeSteps),
        "attitudeErrorMax": [errors[dt] for dt in timeSteps],
    }
    if saveJson:
        targetResults = resultsPath if resultsDir is None else resultsDir
        os.makedirs(targetResults, exist_ok=True)
        with open(os.path.join(targetResults, fileName+".json"), "w") as stream:
            json.dump(metrics, stream, indent=2)

    figure, axis = plt.subplots(figsize=(4.8, 2.5), layout="constrained")
    colors = (
        _comparePlots.COLOR_BSM,
        _comparePlots.COLOR_MUJOCO,
        _comparePlots.COLOR_DIFF,
    )
    markers = ("o", "s", "^")
    for index, dt in enumerate(timeSteps):
        axis.loglog(
            bobInertias,
            np.degrees(errors[dt]),
            color=colors[index % len(colors)],
            marker=markers[index % len(markers)],
            label=rf"$\Delta t={dt:g}$ s",
        )
    axis.set_xlabel(r"Initial bob inertia [kg m$^2$]")
    axis.set_ylabel("Attitude difference [deg]")
    axis.legend(loc="best", fontsize=8)
    axis.grid(False, which="both")
    figureList = {fileName: figure}
    _comparePlots.finalizeFigures(figureList)

    if showPlots:
        plt.show()
    return figureList


if __name__ == "__main__":
    figures = run(showPlots=False, saveJson=True)
    os.makedirs(resultsPath, exist_ok=True)
    for name, figure in figures.items():
        figure.savefig(os.path.join(resultsPath, name+".svg"), transparent=True)
