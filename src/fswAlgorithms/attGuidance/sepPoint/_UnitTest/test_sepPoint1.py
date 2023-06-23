#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
from Basilisk.fswAlgorithms import sepPoint
from Basilisk.utilities import macros
from Basilisk.architecture import messaging
from Basilisk.architecture import bskLogging


# The 'ang' array spans the interval from 0 to pi.  pi is excluded because
# the code is less accurate around this point;
ang = np.linspace(0, np.pi, 50, endpoint=False)
ang = list(ang)


@pytest.mark.parametrize("alpha", ang)
@pytest.mark.parametrize("alignmentPriority", [1])
@pytest.mark.parametrize("accuracy", [1e-12])
def test_(show_plots, alpha, alignmentPriority, accuracy):
    r"""
    **Validation Test Description**

    This unit test script tests the correctness of the reference attitude computed by :ref:`sepPoint`.
    The correctness of the output is determined based on whether the reference attitude causes the Sun-constrained
    axis :math:`\hat{a}_2` to be within an angle :math:`\beta` from the Sun direction :math:`\hat{r}_{S/B}`.

    **Test Parameters**

    This test generates an array ``ang`` of linearly-spaced points between 0 and :math:`\pi`. Values of
    :math:`\alpha` are drawn from all possible combinations of such linearly spaced values.
    In the test, values of :math:`\alpha` are used to set the angular distance between the vectors
    :math:`{}^\mathcal{N}\hat{r}_{S/B}` and :math:`{}^\mathcal{N}\hat{h}_\text{ref}`.

    Args:
        alpha (rad): angle between :math:`{}^\mathcal{N}\hat{r}_{S/B}` and :math:`{}^\mathcal{N}\hat{h}_\text{ref}`
        alignmentPriority (int): 1 to prioritize body heading alignment and Sun-constrained body-frame axis.

    **Description of Variables Being Tested**

    In this unit test, the module computes a solution that is compliant with the keep-in condition on the
    Sun-constrained body-frame axis. The correctness of the solution is assessed verifying whether the angle between
    such axis and the Sun direction is smaller or equal than the maximum allowed value :math:`\beta`. Within the range
    of admissible solutions, the module also chooses the one that maximizes the incidence of sunlight on the solar
    arrays. See R. Calaon, C. Allard and H. Schaub, "Attitude Reference Generation for Spacecraft
    with Rotating Solar Arrays and Pointing Constraints", in preparation for Journal of Spacecraft and Rockets.

    **General Documentation Comments**

    This module only tests the case :math:`\beta = \pi/2`, which is known to always have a compliant solution. When
    :math:`\beta < \pi/2`, it is not in general possible to guarantee that the solution is compliant with the
    requirements on the Sun-constrained direction, which makes testing difficult. The user can input smaller values for
    :math:`\beta` to verify that it will cause some instantiations of the test to fail. This unit test verifies the
    correctness of the generated reference attitude. It does not test the correctness of the reference angular rates
    and accelerations contained in the ``attRefOutMsg``, because this would require to run the module for multiple
    update calls. To ensure fast execution of the unit test, this is avoided.
    """

    rHat_SB_N = np.array([1, -2, 3])                            # Sun direction in inertial coordinates
    rHat_SB_N = rHat_SB_N / np.linalg.norm(rHat_SB_N)

    a = np.cross(rHat_SB_N, [4, 5, -6])
    a = a / np.linalg.norm(a)

    DCM1 = rbk.PRV2C(a * alpha)

    hHat_N = np.matmul(DCM1, rHat_SB_N)             # required thrust direction in inertial frame, at an angle alpha from rHat_SB_N

    hHat_B  = [0, 0, 1]
    a1Hat_B = [1, 0, 0]
    a2Hat_B = [0, 1, 0]
    beta = np.pi/2

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
    attGuid = sepPoint.SepPoint()
    attGuid.a1Hat_B = a1Hat_B
    attGuid.a2Hat_B = a2Hat_B
    attGuid.beta = beta
    attGuid.alignmentPriority = alignmentPriority
    attGuid.ModelTag = "sepPoint"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, attGuid)

    # Initialize the test module configuration data
    # These will eventually become input messages

    # Create input navigation message
    sigma_BN = np.array([0, 0, 0])
    BN = rbk.MRP2C(sigma_BN)
    rS_B = np.matmul(BN, rHat_SB_N)
    NavAttMessageData = messaging.NavAttMsgPayload()     
    NavAttMessageData.sigma_BN = sigma_BN
    NavAttMessageData.vehSunPntBdy = rS_B
    NavAttMsg = messaging.NavAttMsg().write(NavAttMessageData)
    attGuid.attNavInMsg.subscribeTo(NavAttMsg)

    # Create input bodyHeadingMsg
    bodyHeadingData = messaging.BodyHeadingMsgPayload()
    bodyHeadingData.rHat_XB_B = hHat_B
    bodyHeadingMsg = messaging.BodyHeadingMsg().write(bodyHeadingData)
    attGuid.bodyHeadingInMsg.subscribeTo(bodyHeadingMsg)

    # Create input inertialHeadingMsg
    inertialHeadingData = messaging.InertialHeadingMsgPayload()
    inertialHeadingData.rHat_XN_N = hHat_N
    inertialHeadingMsg = messaging.InertialHeadingMsg().write(inertialHeadingData)
    attGuid.inertialHeadingInMsg.subscribeTo(inertialHeadingMsg)

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

    moduleOutput = dataLog.sigma_RN
    sigma_RN = moduleOutput[0]
    RN = rbk.MRP2C(sigma_RN)
    NR = RN.transpose()
    a1Hat_N = np.matmul(NR, a1Hat_B)
    gamma_sim = np.arcsin( abs( np.clip( np.dot(rHat_SB_N, a1Hat_N), -1, 1 ) ) )

    # set the filtered output truth states
    np.testing.assert_allclose(hHat_B, np.matmul(RN, hHat_N), rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_array_less(np.cos(beta), np.dot(a2Hat_B, np.matmul(RN, rHat_SB_N)) + accuracy, verbose=True)

    return


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_(
                 False,
                 0.62831,    # alpha
                 1,          # priorityFlag
                 1e-12       # accuracy
               )
