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


def computeTruthForce(mu, deputyMass, K, P, rDeputy, vDeputy, rDeputyDes, vDeputyDes, forceFF):
    """Compute expected inertial control force for the test case."""
    deltaR = rDeputy - rDeputyDes
    deltaV = vDeputy - vDeputyDes

    if mu > 0.0:
        gravityTerm = -deputyMass * (
            -mu * rDeputy / np.linalg.norm(rDeputy) ** 3
            +mu * rDeputyDes / np.linalg.norm(rDeputyDes) ** 3
        )
    else:
        gravityTerm = np.zeros(3)

    return gravityTerm - deputyMass * K.dot(deltaR) - deputyMass * P.dot(deltaV) + forceFF


@pytest.mark.parametrize("useMu", [True, False])
@pytest.mark.parametrize("useFeedForward", [True, False])
@pytest.mark.parametrize("accuracy", [1e-10])
def test_inertialCartFeedback(useMu, useFeedForward, accuracy):
    r"""
    **Validation Test Description**

    This unit test validates :ref:`inertialCartFeedback` by checking the inertial force output
    against a Python truth model for:

    1. `mu > 0` (gravity compensation active)
    2. `mu <= 0` (gravity compensation skipped)
    3. `forceFeedforwardInMsg` connected (feedforward included in control law)
    4. `forceFeedforwardInMsg` not connected (feedforward excluded from control law)

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

    desiredData = messaging.NavTransMsgPayload()
    desiredData.r_BN_N = [7000e3 + 10.0, 100e3 - 20.0, -50e3 + 15.0]
    desiredData.v_BN_N = [0.03, 7500.0 - 0.01, 50.0 + 0.02]
    desiredMsg = messaging.NavTransMsg().write(desiredData)

    deputyConfigData = messaging.VehicleConfigMsgPayload()
    deputyConfigData.massSC = 300.0
    deputyConfigMsg = messaging.VehicleConfigMsg().write(deputyConfigData)

    ffData = messaging.CmdForceInertialMsgPayload()
    ffData.forceRequestInertial = [0.12, -0.34, 0.56]
    ffMsg = messaging.CmdForceInertialMsg().write(ffData)

    module.deputyTransInMsg.subscribeTo(deputyMsg)
    module.deputyTransDesiredInMsg.subscribeTo(desiredMsg)
    module.deputyVehicleConfigInMsg.subscribeTo(deputyConfigMsg)
    if useFeedForward:
        module.forceFeedforwardInMsg.subscribeTo(ffMsg)

    dataLog = module.forceOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.TotalSim.SingleStepProcesses()

    forceOut = dataLog.forceRequestInertial[0]

    forceFF = np.array(ffData.forceRequestInertial) if useFeedForward else np.zeros(3)
    trueForce = computeTruthForce(
        mu=mu,
        deputyMass=deputyConfigData.massSC,
        K=np.array(K).reshape((3, 3)),
        P=np.array(P).reshape((3, 3)),
        rDeputy=np.array(deputyData.r_BN_N),
        vDeputy=np.array(deputyData.v_BN_N),
        rDeputyDes=np.array(desiredData.r_BN_N),
        vDeputyDes=np.array(desiredData.v_BN_N),
        forceFF=forceFF
    )

    np.testing.assert_allclose(forceOut, trueForce, atol=accuracy)


if __name__ == "__main__":
    test_inertialCartFeedback(True, True, 1e-10)
