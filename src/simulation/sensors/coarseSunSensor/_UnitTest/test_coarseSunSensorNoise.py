# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

"""Regression tests for coarse sun sensor noise propagation."""

import numpy as np

from Basilisk.architecture import messaging
from Basilisk.simulation import coarseSunSensor
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

DEFAULT_WALK_BOUNDS = -1.0  # [-]


def run_noise_case(sampleCount, propagationFactor=None, walkBounds=DEFAULT_WALK_BOUNDS):
    """Run a stationary CSS case and return its dimensionless measurement errors."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("noiseProcess")
    taskPeriod = 0.1  # [s]
    taskRate = macros.sec2nano(taskPeriod)
    process.addTask(simulation.CreateNewTask("noiseTask", taskRate))

    sunPayload = messaging.SpicePlanetStateMsgPayload()
    sunPayload.PositionVector = [orbitalMotion.AU * 1000.0, 0.0, 0.0]  # [m]
    sunMessage = messaging.SpicePlanetStateMsg().write(sunPayload)

    statePayload = messaging.SCStatesMsgPayload()
    statePayload.r_BN_N = [0.0, 0.0, 0.0]  # [m]
    statePayload.sigma_BN = [0.0, 0.0, 0.0]  # [-]
    stateMessage = messaging.SCStatesMsg().write(statePayload)

    sensor = coarseSunSensor.CoarseSunSensor()
    sensor.RNGSeed = 12345
    sensor.nHat_B = [1.0, 0.0, 0.0]  # [-]
    sensor.senNoiseStd = 0.017  # [-]
    sensor.walkBounds = walkBounds
    sensor.minOutput = -1.0e6  # [-]
    sensor.maxOutput = 1.0e6  # [-]
    if propagationFactor is not None:
        sensor.setAMatrix([[propagationFactor]])
    sensor.sunInMsg.subscribeTo(sunMessage)
    sensor.stateInMsg.subscribeTo(stateMessage)

    recorder = sensor.cssDataOutMsg.recorder()
    simulation.AddModelToTask("noiseTask", sensor)
    simulation.AddModelToTask("noiseTask", recorder)
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime((sampleCount - 1) * taskRate)
    simulation.ExecuteSimulation()

    truthOutput = 1.0  # [-]
    return sensor, np.asarray(recorder.OutputData) - truthOutput


def test_default_noise_is_white():
    """Verify that ``senNoiseStd`` alone produces stationary white noise."""
    sampleCount = 1000
    sensor, errors = run_noise_case(sampleCount)

    noiseStandardDeviation = sensor.senNoiseStd
    meanTolerance = 4.0 * noiseStandardDeviation / np.sqrt(sampleCount)
    lagOneCorrelation = np.corrcoef(errors[:-1], errors[1:])[0, 1]

    assert abs(np.mean(errors)) < meanTolerance
    assert np.isclose(np.std(errors), noiseStandardDeviation, rtol=0.1)
    assert abs(lagOneCorrelation) < 0.15


def test_explicit_bounded_random_walk():
    """Verify that an identity propagation matrix enables a bounded random walk."""
    walkBounds = 0.1  # [-]
    propagationFactor = 1.0  # [-]
    sensor, errors = run_noise_case(400, propagationFactor=propagationFactor, walkBounds=walkBounds)

    lagOneCorrelation = np.corrcoef(errors[:-1], errors[1:])[0, 1]

    assert np.max(np.abs(errors)) <= walkBounds + np.finfo(float).eps
    assert lagOneCorrelation > 0.7
    np.testing.assert_allclose(np.asarray(sensor.getAMatrix()), [[propagationFactor]])


def test_propagation_matrix_python_binding():
    """Verify the Python setter and getter use the existing Eigen vector binding."""
    sensor = coarseSunSensor.CoarseSunSensor()
    propagationMatrix = [[0.25]]  # [-]
    sensor.setAMatrix(propagationMatrix)

    np.testing.assert_allclose(np.asarray(sensor.getAMatrix()), propagationMatrix)


if __name__ == "__main__":
    test_default_noise_is_white()
    test_explicit_bounded_random_walk()
    test_propagation_matrix_python_binding()
