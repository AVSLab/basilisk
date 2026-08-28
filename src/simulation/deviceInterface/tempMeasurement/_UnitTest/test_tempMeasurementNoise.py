#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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

import numpy as np

from Basilisk.architecture import messaging
from Basilisk.simulation import tempMeasurement
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


NOISE_STD = 1.0  # [C]
TRUE_TEMPERATURE = 20.0  # [C]


def run_sensor(sampleCount, seed, propagationFactor=None, walkBound=None):
    """Run a constant-temperature sensor and return its measurement errors."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    taskPeriod = 1.0  # [s]
    process.addTask(simulation.CreateNewTask("testTask", macros.sec2nano(taskPeriod)))

    sensor = tempMeasurement.TempMeasurement()
    sensor.RNGSeed = seed
    sensor.senNoiseStd = NOISE_STD
    if propagationFactor is not None:
        sensor.setAMatrix([[propagationFactor]])
    if walkBound is not None:
        sensor.walkBounds = walkBound

    temperaturePayload = messaging.TemperatureMsgPayload()
    temperaturePayload.temperature = TRUE_TEMPERATURE
    temperatureMessage = messaging.TemperatureMsg().write(temperaturePayload)
    sensor.tempInMsg.subscribeTo(temperatureMessage)

    recorder = sensor.tempOutMsg.recorder()
    simulation.AddModelToTask("testTask", sensor)
    simulation.AddModelToTask("testTask", recorder)
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(float(sampleCount - 1)))
    simulation.ExecuteSimulation()

    return sensor, np.asarray(recorder.temperature) - TRUE_TEMPERATURE


def test_default_noise_is_white():
    """The default propagation must produce the configured white-noise sigma."""
    sampleCount = 1000
    sensor, errors = run_sensor(sampleCount, seed=1234)

    np.testing.assert_allclose(np.asarray(sensor.getAMatrix()), [[0.0]])
    assert sensor.walkBounds <= 0.0
    assert abs(np.mean(errors)) < 5.0 * NOISE_STD / np.sqrt(sampleCount)
    assert abs(np.std(errors, ddof=1) / NOISE_STD - 1.0) < 0.1
    assert abs(np.corrcoef(errors[:-1], errors[1:])[0, 1]) < 0.15


def test_explicit_random_walk_is_bounded():
    """Identity propagation and a positive bound must create a bounded walk."""
    bound = 5.0 * NOISE_STD  # [C]
    sensor, errors = run_sensor(400, seed=1234, propagationFactor=1.0, walkBound=bound)

    assert np.max(np.abs(errors)) <= bound
    assert np.any(np.isclose(np.abs(errors), bound, rtol=0.0, atol=1.0e-12))
    assert np.corrcoef(errors[:-1], errors[1:])[0, 1] > 0.5
    np.testing.assert_allclose(np.asarray(sensor.getAMatrix()), [[1.0]])


def test_rng_seed_controls_noise_sequence():
    """Equal seeds must repeat and different seeds must change the sequence."""
    _, first = run_sensor(8, seed=1234)
    _, repeated = run_sensor(8, seed=1234)
    _, different = run_sensor(8, seed=5678)

    assert np.array_equal(first, repeated)
    assert not np.array_equal(first, different)
