#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


import pytest
import os
import inspect
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


# Import all the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.fswAlgorithms import sunPointRoll
from Basilisk.utilities import macros
from Basilisk.architecture import messaging
from Basilisk.architecture import bskLogging


@pytest.mark.parametrize("sigma_BN", [[0.0, 0.0, 0.0],
                                      [0.1, 0.2, 0.3],
                                      [1.1, 2.2, 3.3]])
@pytest.mark.parametrize("hRefHat_B", [[0.0, 1.0, 0.0],
                                       [0.5, 0.5, 0.5],
                                       [0.1, 0.1, -0.9]])
@pytest.mark.parametrize("omegaRoll", [0.0, 0.01, -0.03])
@pytest.mark.parametrize("accuracy", [1e-12])
def test_(show_plots, sigma_BN, hRefHat_B, omegaRoll, accuracy):
    r"""
    **Validation Test Description**

    This unit test script tests the correctness of the reference attitude computed by :ref:`sunPointRoll`.
    The correctness of the output is determined based on whether the reference attitude drives the desired body-frame
    heading :math:`\hat{h}_\text{ref}` onto the relative Sun direction :math:`\hat{r}_{S/B}`.

    **Test Parameters**

    Args:
        sigma_BN (MRP): initial spacecraft attitude
        hRefHat_B (rad): desired body-frame axis to align with Sun
        omegaRoll (rad/s): roll rate about the sunline axis
        accuracy: accuracy of the result.
    """

    rHat_SB_N = np.array([1, -2, 3])                            # Sun direction in inertial coordinates
    rHat_SB_N = rHat_SB_N / np.linalg.norm(rHat_SB_N)
    a1Hat_B = np.array([-9, 8, 7])                            # array axis direction in body frame
    a1Hat_B = a1Hat_B / np.linalg.norm(a1Hat_B)

    hRefHat_B = np.array(hRefHat_B) / np.linalg.norm(hRefHat_B)

    unitTaskName = "unitTask"                                # arbitrary name (don't change)
    unitProcessName = "TestProcess"                          # arbitrary name (don't change)
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(1.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    attGuid = sunPointRoll.SunPointRoll()
    attGuid.a1Hat_B = a1Hat_B
    attGuid.hRefHat_B = hRefHat_B
    attGuid.omegaRoll = omegaRoll
    attGuid.ModelTag = "sunPointRoll"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, attGuid)

    # Initialize the test module configuration data
    # These will eventually become input messages

    # Create input navigation message
    BN = rbk.MRP2C(sigma_BN)
    rS_B = np.matmul(BN, rHat_SB_N)
    NavAttMessageData = messaging.NavAttMsgPayload()     
    NavAttMessageData.sigma_BN = sigma_BN
    NavAttMessageData.vehSunPntBdy = rS_B
    NavAttMsg = messaging.NavAttMsg().write(NavAttMessageData)
    attGuid.attNavInMsg.subscribeTo(NavAttMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = attGuid.attRefOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    dataLogC = attGuid.attRefOutMsgC.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLogC)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    # test the correctness of the C++ output message
    RN = rbk.MRP2C(dataLog.sigma_RN[0])
    omegaRN_N = dataLog.omega_RN_N[0]
    np.testing.assert_allclose(hRefHat_B, np.matmul(RN, rHat_SB_N), rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(omegaRN_N, omegaRoll*rHat_SB_N, rtol=0, atol=accuracy, verbose=True)

    # test the correctness of the C output message
    RN = rbk.MRP2C(dataLogC.sigma_RN[0])
    omegaRN_N = dataLogC.omega_RN_N[0]
    np.testing.assert_allclose(hRefHat_B, np.matmul(RN, rHat_SB_N), rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(omegaRN_N, omegaRoll*rHat_SB_N, rtol=0, atol=accuracy, verbose=True)

    return


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_(
                 False,
                 [0, 0, 0],
                 [0, 1, 0],
                 0.01,
                 1e-12
               )
