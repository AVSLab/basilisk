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
from Basilisk.simulation import starTracker
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


NOISE_STD = 1.0e-3  # [rad]


def quaternion_to_prv(quaternion):
    """Convert a unit quaternion to its principal rotation vector."""
    quaternion = np.asarray(quaternion)
    vectorNorm = np.linalg.norm(quaternion[1:])
    if vectorNorm <= np.finfo(float).eps:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(vectorNorm, quaternion[0])  # [rad]
    return angle * quaternion[1:] / vectorNorm


def run_sensor(sampleCount, seed, propagationMatrix=None, walkBounds=None):
    """Run a stationary star tracker and return attitude-error samples."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    taskPeriod = 1.0  # [s]
    process.addTask(simulation.CreateNewTask("testTask", macros.sec2nano(taskPeriod)))

    sensor = starTracker.StarTracker()
    sensor.RNGSeed = seed
    sensor.PMatrix = np.eye(3) * NOISE_STD
    if propagationMatrix is not None:
        sensor.setAMatrix(propagationMatrix)
    if walkBounds is not None:
        sensor.setWalkBounds(walkBounds)

    stateMessage = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload())
    sensor.scStateInMsg.subscribeTo(stateMessage)

    recorder = sensor.sensorOutMsg.recorder()
    simulation.AddModelToTask("testTask", sensor)
    simulation.AddModelToTask("testTask", recorder)
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(float(sampleCount - 1)))
    simulation.ExecuteSimulation()

    errors = np.asarray([quaternion_to_prv(q) for q in recorder.qInrtl2Case])
    return sensor, errors


def test_default_noise_is_white():
    """The default propagation must produce the configured white-noise sigma."""
    sampleCount = 1000
    sensor, errors = run_sensor(sampleCount, seed=1234)

    np.testing.assert_allclose(np.asarray(sensor.getAMatrix()), np.zeros((3, 3)))
    for axis in range(3):
        axisErrors = errors[:, axis]
        assert abs(np.mean(axisErrors)) < 5.0 * NOISE_STD / np.sqrt(sampleCount)
        assert abs(np.std(axisErrors, ddof=1) / NOISE_STD - 1.0) < 0.1
        assert abs(np.corrcoef(axisErrors[:-1], axisErrors[1:])[0, 1]) < 0.15


def test_explicit_random_walk_is_bounded():
    """Identity propagation and positive bounds must create a bounded walk."""
    bound = 5.0 * NOISE_STD  # [rad]
    sensor, errors = run_sensor(
        400,
        seed=1234,
        propagationMatrix=np.eye(3),
        walkBounds=np.full(3, bound),
    )

    assert np.max(np.abs(errors)) <= bound + 1.0e-12
    assert np.any(np.isclose(np.abs(errors), bound, rtol=0.0, atol=1.0e-12))
    assert np.corrcoef(errors[:-1, 0], errors[1:, 0])[0, 1] > 0.5
    np.testing.assert_allclose(np.asarray(sensor.getAMatrix()), np.eye(3))


def test_rng_seed_controls_noise_sequence():
    """Equal seeds must repeat and different seeds must change the sequence."""
    _, first = run_sensor(8, seed=1234)
    _, repeated = run_sensor(8, seed=1234)
    _, different = run_sensor(8, seed=5678)

    assert np.array_equal(first, repeated)
    assert not np.array_equal(first, different)
