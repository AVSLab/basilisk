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
from Basilisk.simulation import magnetometer
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


NOISE_STD = 1.0e-9  # [T]


def run_sensor(sampleCount, seed, propagationMatrix=None, walkBounds=None):
    """Run a zero-field magnetometer and return its noise samples."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    taskPeriod = 1.0  # [s]
    process.addTask(simulation.CreateNewTask("testTask", macros.sec2nano(taskPeriod)))

    sensor = magnetometer.Magnetometer()
    sensor.RNGSeed = seed
    sensor.senNoiseStd = [NOISE_STD, NOISE_STD, NOISE_STD]
    if propagationMatrix is not None:
        sensor.setAMatrix(propagationMatrix)
    if walkBounds is not None:
        sensor.walkBounds = walkBounds

    stateMessage = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload())
    fieldMessage = messaging.MagneticFieldMsg().write(messaging.MagneticFieldMsgPayload())
    sensor.stateInMsg.subscribeTo(stateMessage)
    sensor.magInMsg.subscribeTo(fieldMessage)

    recorder = sensor.tamDataOutMsg.recorder()
    simulation.AddModelToTask("testTask", sensor)
    simulation.AddModelToTask("testTask", recorder)
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(float(sampleCount - 1)))
    simulation.ExecuteSimulation()

    return sensor, np.asarray(recorder.tam_S)


def test_default_noise_is_white():
    """The default propagation must produce the configured white-noise sigma."""
    sampleCount = 5000
    sensor, samples = run_sensor(sampleCount, seed=1234)

    assert np.array_equal(sensor.getAMatrix(), np.zeros((3, 3)))
    for axis in range(3):
        axisSamples = samples[:, axis]
        assert abs(np.mean(axisSamples)) < 5.0 * NOISE_STD / np.sqrt(sampleCount)
        assert abs(np.std(axisSamples, ddof=1) / NOISE_STD - 1.0) < 0.05
        assert abs(np.corrcoef(axisSamples[:-1], axisSamples[1:])[0, 1]) < 0.05


def test_explicit_random_walk_is_bounded():
    """Identity propagation and positive bounds must create a bounded walk."""
    bound = 5.0 * NOISE_STD  # [T]
    _, samples = run_sensor(
        5000,
        seed=1234,
        propagationMatrix=np.eye(3),
        walkBounds=[bound, bound, bound],
    )

    assert np.max(np.abs(samples)) <= bound
    assert np.any(np.isclose(np.abs(samples), bound, rtol=0.0, atol=1.0e-20))
    assert np.corrcoef(samples[:-1, 0], samples[1:, 0])[0, 1] > 0.5


def test_rng_seed_controls_noise_sequence():
    """Equal seeds must repeat and different seeds must change the sequence."""
    _, first = run_sensor(64, seed=1234)
    _, repeated = run_sensor(64, seed=1234)
    _, different = run_sensor(64, seed=5678)

    assert np.array_equal(first, repeated)
    assert not np.array_equal(first, different)
