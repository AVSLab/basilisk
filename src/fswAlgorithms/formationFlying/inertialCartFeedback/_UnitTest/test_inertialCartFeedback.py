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
import pytest

from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import inertialCartFeedback
from Basilisk.utilities import SimulationBaseClass, macros


def computeTruthForce(mu, deputyMass, K, P, rDeputy, vDeputy,
                      rDeputyRef, vDeputyRef, aDeputyRef):
    """Compute expected inertial control force for the test case."""
    deltaR = rDeputy - rDeputyRef
    deltaV = vDeputy - vDeputyRef

    if mu > 0.0:
        gravAccelDeputy = gravAccel(mu, rDeputy)
    else:
        gravAccelDeputy = np.zeros(3)

    return -deputyMass * gravAccelDeputy + deputyMass * aDeputyRef - K.dot(deltaR) - P.dot(deltaV)

def gravAccel(mu, r):
    """Compute gravitational acceleration for the test case."""
    r = np.asarray(r, dtype=float)
    if mu > 0.0:
        if np.linalg.norm(r) > 1e-6:
            return -mu * r / np.linalg.norm(r) ** 3
        else:
            raise ValueError("Position norm too small to evaluate gravity.")
    else:
        return np.zeros(3)


@pytest.mark.parametrize("useMu", [True, False])
@pytest.mark.parametrize("nonNaturalMotion", [True, False])
@pytest.mark.parametrize("accuracy", [1e-10])
def test_inertialCartFeedback(useMu, nonNaturalMotion, accuracy):
    r"""
    **Validation Test Description**

    This unit test validates :ref:`inertialCartFeedback` by checking the inertial force output
    against a Python truth model for:

    1. `mu > 0` (gravity compensation active)
    2. `mu <= 0` (gravity compensation skipped)
    3. `nonNaturalMotion` set to True (non-natural motion is included in control law)
    4. `nonNaturalMotion` set to False (non-natural motion is excluded from control law)

    **Description of Variables Being Tested**

    The following output quantity is tested:
    - ``forceOutMsg.forceRequestInertial``
    """
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    module = inertialCartFeedback.InertialCartFeedback()
    module.ModelTag = "inertialCartFeedback"
    mu = 3.986004418e14 if useMu else 0.0
    module.setMu(mu)
    K = [2.0e-5, 0.0, 0.0,
                 0.0, 3.0e-5, 0.0,
                 0.0, 0.0, 4.0e-5]
    module.setK(K)
    P = [5.0e-2, 0.0, 0.0,
         0.0, 6.0e-2, 0.0,
         0.0, 0.0, 7.0e-2]
    module.setP(P)
    unitTestSim.AddModelToTask(unitTaskName, module)

    deputyData = messaging.NavTransMsgPayload()
    deputyData.r_BN_N = [7000e3 + 40.0, 100e3 - 15.0, -50e3 + 30.0]
    deputyData.v_BN_N = [0.05, 7500.08, 49.92]
    deputyMsg = messaging.NavTransMsg().write(deputyData)

    refData = messaging.TransRefMsgPayload()
    refData.r_RN_N = [7000e3 + 10.0, 100e3 - 20.0, -50e3 + 15.0]
    refData.v_RN_N = [0.03, 7500.0 - 0.01, 50.0 + 0.02]
    aGravRef_N = gravAccel(mu, refData.r_RN_N)
    if nonNaturalMotion:
        # Include non-natural motion feed-forward in reference acceleration
        refData.a_RN_N = aGravRef_N + np.array([0.001, -0.002, 0.003])  # small non-natural acceleration for testing
    else:
        # Exclude non-natural motion feed-forward from reference acceleration
        refData.a_RN_N = aGravRef_N
    refMsg = messaging.TransRefMsg().write(refData)

    deputyConfigData = messaging.VehicleConfigMsgPayload()
    deputyConfigData.massSC = 300.0
    deputyConfigMsg = messaging.VehicleConfigMsg().write(deputyConfigData)

    module.deputyNavInMsg.subscribeTo(deputyMsg)
    module.deputyRefInMsg.subscribeTo(refMsg)
    module.deputyVehicleConfigInMsg.subscribeTo(deputyConfigMsg)

    dataLog = module.forceOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    forceOut = dataLog.forceRequestInertial[0]

    trueForce = computeTruthForce(
        mu=mu,
        deputyMass=deputyConfigData.massSC,
        K=np.array(K).reshape((3, 3)),
        P=np.array(P).reshape((3, 3)),
        rDeputy=np.array(deputyData.r_BN_N),
        vDeputy=np.array(deputyData.v_BN_N),
        rDeputyRef=np.array(refData.r_RN_N),
        vDeputyRef=np.array(refData.v_RN_N),
        aDeputyRef=np.array(refData.a_RN_N)
    )

    np.testing.assert_allclose(forceOut, trueForce, atol=accuracy)


if __name__ == "__main__":
    test_inertialCartFeedback(True, True, 1e-10)
