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

r"""Mechanism controls for the velocity-dependent torque artifact.

The propagation sweeps in :ref:`scenarioCompareTorque` show that an inertial
drift velocity changes MuJoCo's attitude even when no physical torque can act.
This driver tests the proposed floating-point cancellation mechanism directly:

#. record the initial MuJoCo generalized bias force for a generic drift;
#. repeat the check along a coordinate axis, where the cancellation is exact;
#. double speed and verify the expected quadratic error scaling; and
#. double mass with inertia fixed and verify the expected linear scaling.

The quiet body has no gravity, applied torque, or initial body rate. Results are
written to ``results/sweepTorqueMechanismChecks.json``.
"""

import json
import os

import numpy as np

import _comparisonValidation
import scenarioCompareTorque as torqueScenario

from Basilisk import hasBuildFeature
from Basilisk.simulation import svIntegrators
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

couldImportMujoco = hasBuildFeature("mujoco")
if couldImportMujoco:
    from Basilisk.simulation import MJJointReactionForces
    from Basilisk.simulation import mujoco


fileName = os.path.basename(os.path.splitext(__file__)[0])
resultsPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")

TIME_STEP = 0.1  # [s]
SIM_DURATION = 600.0  # [s]
RECORD_STEP = 1.0  # [s]
GENERIC_DIRECTION = np.array([2.0, -3.0, 6.0])/7.0
AXIS_DIRECTION = np.array([1.0, 0.0, 0.0])
SPEEDS = (937.5, 1875.0, 3750.0, 7500.0)  # [m/s]
MASSES = (750.0, 1500.0)  # [kg]


def quietMujocoRun(direction, speed, mass, captureBias=False,
                   timeStep=TIME_STEP, simDuration=SIM_DURATION,
                   recordStep=RECORD_STEP):
    """Return maximum attitude drift and, optionally, the initial bias force."""
    if not couldImportMujoco:
        raise ImportError("Build Basilisk with --mujoco to run this control.")

    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("dyn")
    process.addTask(
        simulation.CreateNewTask("dynTask", macros.sec2nano(timeStep))
    )

    scene = mujoco.MJScene(torqueScenario.mujocoModel(mass=mass))
    scene.ModelTag = "quietMujoco"
    scene.extraEoMCall = True
    scene.highOrderAttitudeIntegration = True
    simulation.AddModelToTask("dynTask", scene, 1)

    integrator = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integrator)
    hub = scene.getBody("hub")

    biasRecorder = None
    if captureBias:
        reactionForces = MJJointReactionForces.MJJointReactionForces()
        reactionForces.ModelTag = "jointReactionForces"
        reactionForces.scene = scene
        simulation.AddModelToTask("dynTask", reactionForces)
        biasRecorder = reactionForces.reactionForcesOutMsg.recorder()
        simulation.AddModelToTask("dynTask", biasRecorder)

    stateRecorder = hub.getOrigin().stateOutMsg.recorder(
        macros.sec2nano(recordStep)
    )
    simulation.AddModelToTask("dynTask", stateRecorder)

    simulation.InitializeSimulation()
    hub.setPosition([0.0, 0.0, 0.0])
    hub.setVelocity((speed*np.asarray(direction)).tolist())
    hub.setAttitude([0.0, 0.0, 0.0])
    hub.setAttitudeRate([0.0, 0.0, 0.0])

    simulation.ConfigureStopTime(macros.sec2nano(simDuration))
    simulation.ExecuteSimulation()

    stateTimes = np.asarray(stateRecorder.times())*macros.NANO2SEC
    sigma = np.asarray(stateRecorder.sigma_BN)
    _comparisonValidation.validateHistory(
        "torque mechanism MuJoCo", stateTimes, simDuration, recordStep,
        attitude=sigma)
    attitudeDrift = float(np.max(
        torqueScenario.relativePrincipalAngle(sigma, np.zeros_like(sigma))
    ))
    initialBias = None
    if biasRecorder is not None:
        initialBias = np.asarray(biasRecorder.biasForces[0], dtype=float).tolist()
    return attitudeDrift, initialBias


def _scaledRows(values, resultKey, evaluate):
    """Evaluate a sequence and attach each ratio to the preceding result."""
    rows = []
    previous = None
    for value in values:
        result = float(evaluate(value))
        rows.append({
            resultKey: float(value),
            "mujocoAttitudeMax": result,
            "ratioToPrevious": None if previous is None else result/previous,
        })
        previous = result
    return rows


def run(saveJson=True, speeds=SPEEDS, masses=MASSES, timeStep=TIME_STEP,
        simDuration=SIM_DURATION, recordStep=RECORD_STEP, resultsDir=None):
    """Run the direct-bias, axis-cancellation, velocity, and mass controls."""
    if not couldImportMujoco:
        raise ImportError("Build Basilisk with --mujoco to run this control.")

    speeds = tuple(speeds)
    masses = tuple(masses)
    if not speeds or not masses:
        raise ValueError("At least one speed and mass are required.")

    velocityBias = {}

    def velocityResult(speed):
        attitudeDrift, bias = quietMujocoRun(
            GENERIC_DIRECTION,
            speed,
            torqueScenario.MASS,
            captureBias=speed == speeds[-1],
            timeStep=timeStep,
            simDuration=simDuration,
            recordStep=recordStep,
        )
        if bias is not None:
            velocityBias["generic"] = bias
        return attitudeDrift

    velocityRows = _scaledRows(speeds, "speed", velocityResult)
    axisAttitude, axisBias = quietMujocoRun(
        AXIS_DIRECTION, speeds[-1], torqueScenario.MASS, captureBias=True,
        timeStep=timeStep, simDuration=simDuration, recordStep=recordStep,
    )

    massResults = {torqueScenario.MASS: velocityRows[-1]["mujocoAttitudeMax"]}

    def massResult(mass):
        if mass in massResults:
            return massResults[mass]
        return quietMujocoRun(
            GENERIC_DIRECTION, speeds[-1], mass, captureBias=False,
            timeStep=timeStep, simDuration=simDuration, recordStep=recordStep,
        )[0]

    massRows = _scaledRows(
        masses,
        "mass",
        massResult,
    )

    metrics = {
        "scenario": fileName,
        "configuration": {
            "dt": timeStep,
            "tf": simDuration,
            "recordDt": recordStep,
            "genericDirection": GENERIC_DIRECTION.tolist(),
            "axisDirection": AXIS_DIRECTION.tolist(),
            "speed": speeds[-1],
            "quietBody": "no gravity, applied torque, or initial body rate",
        },
        "genericInitialBiasForces": velocityBias["generic"],
        "axisInitialBiasForces": axisBias,
        "quietAxisAttitudeMax": axisAttitude,
        "quietVelocityScaling": velocityRows,
        "quietMassScaling": massRows,
    }

    if saveJson:
        targetResults = resultsPath if resultsDir is None else resultsDir
        os.makedirs(targetResults, exist_ok=True)
        outFile = os.path.join(targetResults, fileName+".json")
        with open(outFile, "w") as stream:
            json.dump(metrics, stream, indent=2)
        print("Wrote " + outFile)
    return metrics


if __name__ == "__main__":
    run()
