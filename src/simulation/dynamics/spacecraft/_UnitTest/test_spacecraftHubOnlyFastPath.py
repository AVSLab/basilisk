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

"""
Unit tests for the automatic hub-only spacecraft dynamics fast path.

When no state effectors are attached and point :math:`B` is colocated with the
hub center of mass, ``Spacecraft`` can bypass the general Backsubstitution
matrix assembly and ask ``HubEffector`` to compute direct rigid-body
derivatives. These tests validate the resulting dynamics against an analytic
case and verify that inertia changes refresh the cached factorization.
"""

import numpy as np
import pytest

from Basilisk.simulation import extForceTorque
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


TASK_PERIOD = 0.01  # [s]
STOP_TIME = 0.5  # [s]


def test_spacecraftHubOnlyFastPath_matchesAnalyticForceTorque():
    """Hub-only fast path must reproduce direct rigid-body force/torque motion."""

    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTaskName = "unitTask"
    unitProcessName = "testProcess"
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, macros.sec2nano(TASK_PERIOD)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    mass = 20.0  # [kg]
    inertia = np.diag([4.0, 5.0, 6.0])  # [kg*m^2]
    initialPosition_N = np.array([10.0, -20.0, 30.0])  # [m]
    initialVelocity_N = np.array([1.0, -2.0, 0.5])  # [m/s]
    inertialForce_N = np.array([10.0, -4.0, 6.0])  # [N]
    bodyTorque_B = np.array([0.0, 2.0, 0.0])  # [N*m]

    scObject.hub.mHub = mass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # [m]
    scObject.hub.IHubPntBc_B = inertia.tolist()
    scObject.hub.r_CN_NInit = initialPosition_N.reshape(3, 1).tolist()
    scObject.hub.v_CN_NInit = initialVelocity_N.reshape(3, 1).tolist()
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # [-]
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # [rad/s]

    forceTorqueEffector = extForceTorque.ExtForceTorque()
    forceTorqueEffector.ModelTag = "forceTorque"
    forceTorqueEffector.extForce_N = inertialForce_N.reshape(3, 1).tolist()
    forceTorqueEffector.extTorquePntB_B = bodyTorque_B.reshape(3, 1).tolist()
    scObject.addDynamicEffector(forceTorqueEffector)

    dataLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, scObject)
    unitTestSim.AddModelToTask(unitTaskName, forceTorqueEffector)
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(STOP_TIME))
    unitTestSim.ExecuteSimulation()

    finalTime = dataLog.times()[-1] * macros.NANO2SEC  # [s]
    translationalAcceleration_N = inertialForce_N / mass  # [m/s^2]
    angularAcceleration_B = bodyTorque_B[1] / inertia[1, 1]  # [rad/s^2]

    expectedPosition_N = (initialPosition_N + initialVelocity_N*finalTime
                          + 0.5*translationalAcceleration_N*finalTime*finalTime)  # [m]
    expectedVelocity_N = initialVelocity_N + translationalAcceleration_N*finalTime  # [m/s]
    expectedOmega_BN_B = np.array([0.0, angularAcceleration_B*finalTime, 0.0])  # [rad/s]
    expectedRotationAngle = 0.5*angularAcceleration_B*finalTime*finalTime  # [rad]
    expectedSigma_BN = np.array([0.0, np.tan(expectedRotationAngle/4.0), 0.0])  # [-]

    np.testing.assert_allclose(dataLog.r_BN_N[-1], expectedPosition_N, rtol=1e-13, atol=1e-12)
    np.testing.assert_allclose(dataLog.v_BN_N[-1], expectedVelocity_N, rtol=1e-13, atol=1e-12)
    np.testing.assert_allclose(dataLog.omega_BN_B[-1], expectedOmega_BN_B, rtol=1e-13, atol=1e-12)
    np.testing.assert_allclose(dataLog.sigma_BN[-1], expectedSigma_BN, rtol=1e-10, atol=1e-12)


@pytest.mark.parametrize("update_mode", ["effective", "configuration", "reset"])
def test_hub_only_inertia_cache_refreshes(update_mode):
    """Use current effective inertia after direct updates, configuration changes, and resets."""
    body = spacecraft.Spacecraft()
    initial_inertia = np.diag([4.0, 5.0, 6.0])  # [kg*m^2]
    body.hub.IHubPntBc_B = initial_inertia.tolist()
    body.Reset(0)  # [ns]
    omega_state = body.dynManager.getStateObject(body.hub.nameOfHubOmega)

    # Changing only off-diagonal entries must also invalidate the factorization.
    coupled_inertia = np.array([
        [4.0, 0.2, -0.1],
        [0.2, 5.0, 0.3],
        [-0.1, 0.3, 6.0],
    ])  # [kg*m^2]
    small_change = coupled_inertia.copy()
    small_change[0, 1] += 5.0e-12  # [kg*m^2]
    small_change[1, 0] = small_change[0, 1]
    pivot_change = np.diag([7.0, 5.0, 4.0])  # [kg*m^2]
    angular_rates = np.array([[0.3, -0.4, 0.5], [-0.2, 0.6, -0.1]])  # [rad/s]
    torques = np.array([[0.7, -0.8, 0.9], [-0.3, 0.2, 1.1]])  # [N*m]
    zero_force = [0.0, 0.0, 0.0]  # [N]

    for inertia in (initial_inertia, coupled_inertia, small_change, pivot_change, initial_inertia):
        if update_mode == "effective":
            body.hub.effProps.IEffPntB_B = inertia.tolist()
        else:
            body.hub.IHubPntBc_B = inertia.tolist()
            if update_mode == "reset":
                body.Reset(0)  # [ns]
            else:
                body.hub.updateEffectorMassProps(0.0)  # [s]

        # Reuse unchanged inertia with different right-hand sides between updates.
        for omega, torque in zip(angular_rates, torques):
            omega_state.setState(omega.reshape(3, 1).tolist())
            body.hub.computeHubOnlyDerivatives(zero_force, zero_force, torque.tolist())
            expected = np.linalg.solve(inertia, torque - np.cross(omega, inertia @ omega))
            actual = np.asarray(omega_state.getStateDeriv()).reshape(3)
            np.testing.assert_allclose(actual, expected, rtol=2e-14, atol=2e-15)


if __name__ == "__main__":
    test_spacecraftHubOnlyFastPath_matchesAnalyticForceTorque()
