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
from Basilisk.simulation import imuSensor
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


ACCEL_NOISE_STD = 1.0  # [m/s^2]
GYRO_NOISE_STD = 1.0  # [rad/s]


def run_sensor(sampleCount, seed, propagationMatrix=None, errorBound=None):
    """Run a motionless IMU and return accelerometer and gyro noise samples."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    taskPeriod = 1.0  # [s]
    process.addTask(simulation.CreateNewTask("testTask", macros.sec2nano(taskPeriod)))

    sensor = imuSensor.ImuSensor()
    sensor.RNGSeed = seed
    sensor.PMatrixAccel = np.eye(3) * ACCEL_NOISE_STD
    sensor.PMatrixGyro = np.eye(3) * GYRO_NOISE_STD
    if propagationMatrix is not None:
        sensor.setAMatrixAccel(propagationMatrix)
        sensor.setAMatrixGyro(propagationMatrix)
    if errorBound is not None:
        bounds = np.full(3, errorBound)
        sensor.setErrorBoundsAccel(bounds)
        sensor.setErrorBoundsGyro(bounds)

    stateMessage = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload())
    sensor.scStateInMsg.subscribeTo(stateMessage)

    recorder = sensor.sensorOutMsg.recorder()
    simulation.AddModelToTask("testTask", sensor)
    simulation.AddModelToTask("testTask", recorder)
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(float(sampleCount)))
    simulation.ExecuteSimulation()

    return sensor, np.asarray(recorder.AccelPlatform)[1:], np.asarray(recorder.AngVelPlatform)[1:]


def test_default_noise_is_white_and_independent():
    """Default accelerometer and gyro noise must be white and independent."""
    sampleCount = 2500
    sensor, accelNoise, gyroNoise = run_sensor(sampleCount, seed=0)

    np.testing.assert_allclose(np.asarray(sensor.getAMatrixAccel()), np.zeros((3, 3)))
    np.testing.assert_allclose(np.asarray(sensor.getAMatrixGyro()), np.zeros((3, 3)))
    assert not np.array_equal(accelNoise, gyroNoise)
    for samples, expectedSigma in (
        (accelNoise, ACCEL_NOISE_STD),
        (gyroNoise, GYRO_NOISE_STD),
    ):
        for axis in range(3):
            axisSamples = samples[:, axis]
            assert abs(np.mean(axisSamples)) < 5.0 * expectedSigma / np.sqrt(sampleCount)
            assert abs(np.std(axisSamples, ddof=1) / expectedSigma - 1.0) < 0.08
            assert abs(np.corrcoef(axisSamples[:-1], axisSamples[1:])[0, 1]) < 0.08

    for axis in range(3):
        assert abs(np.corrcoef(accelNoise[:, axis], gyroNoise[:, axis])[0, 1]) < 0.08


def test_explicit_random_walk_is_bounded():
    """Identity propagation and positive bounds must create bounded walks."""
    bound = 5.0  # [m/s^2] and [rad/s]
    sensor, accelNoise, gyroNoise = run_sensor(
        800,
        seed=1234,
        propagationMatrix=np.eye(3),
        errorBound=bound,
    )

    for samples in (accelNoise, gyroNoise):
        assert np.max(np.abs(samples)) <= bound
        assert np.any(np.isclose(np.abs(samples), bound, rtol=0.0, atol=1.0e-12))
        assert np.corrcoef(samples[:-1, 0], samples[1:, 0])[0, 1] > 0.5
    np.testing.assert_allclose(np.asarray(sensor.getAMatrixAccel()), np.eye(3))
    np.testing.assert_allclose(np.asarray(sensor.getAMatrixGyro()), np.eye(3))


def test_rng_seed_controls_both_noise_sequences():
    """Equal seeds must repeat and different seeds must change both streams."""
    _, firstAccel, firstGyro = run_sensor(32, seed=1234)
    _, repeatedAccel, repeatedGyro = run_sensor(32, seed=1234)
    _, differentAccel, differentGyro = run_sensor(32, seed=5678)

    assert np.array_equal(firstAccel, repeatedAccel)
    assert np.array_equal(firstGyro, repeatedGyro)
    assert not np.array_equal(firstAccel, differentAccel)
    assert not np.array_equal(firstGyro, differentGyro)
