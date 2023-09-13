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


def computeGamma(alpha, delta):

    if alpha >= 0 and alpha <= np.pi/2:
        if delta < np.pi/2 - alpha:
            gamma = np.pi/2 - alpha - delta
        elif delta > alpha + np.pi/2:
            gamma = - np.pi/2 - alpha + delta
        else:
            gamma = 0
    else:
        if delta < alpha - np.pi/2:
            gamma = - np.pi/2 + alpha - delta 
        elif delta > 3/2*np.pi - alpha:
            gamma = alpha + delta - 3/2*np.pi
        else:
            gamma = 0

    return gamma


# The 'ang' array spans the interval from 0 to pi.  pi is excluded because
# the code is less accurate around this point;
ang = np.linspace(0, np.pi, 10, endpoint=False)
ang = list(ang)


@pytest.mark.parametrize("alpha", ang)
@pytest.mark.parametrize("delta", ang)
@pytest.mark.parametrize("alignmentPriority", [0])
@pytest.mark.parametrize("accuracy", [1e-12])
def test_(show_plots, alpha, delta, alignmentPriority, accuracy):
    r"""
    **Validation Test Description**

    This unit test script tests the correctness of the reference attitude computed by :ref:`sepPoint`.
    The correctness of the output is determined based on whether the reference attitude causes the solar array drive
    axis :math:`\hat{a}_1` to be at an angle :math:`\gamma` from the Sun direction :math:`\hat{r}_{S/B}`.

    **Test Parameters**

    This test generates an array ``ang`` of linearly-spaced points between 0 and :math:`\pi`. Values of
    :math:`\alpha` and :math:`\delta` are drawn from all possible combinations of such linearly spaced values.
    In the test, values of :math:`\alpha` and :math:`\delta` are used to set the angular distance between the vectors
    :math:`{}^\mathcal{N}\hat{r}_{S/B}` and :math:`{}^\mathcal{N}\hat{h}_\text{ref}`, and :math:`{}^\mathcal{B}\hat{h}_1`
    and :math:`{}^\mathcal{B}\hat{a}_1`.

    Args:
        alpha (rad): angle between :math:`{}^\mathcal{N}\hat{r}_{S/B}` and :math:`{}^\mathcal{N}\hat{h}_\text{ref}`
        delta (rad): angle between :math:`{}^\mathcal{B}\hat{h}_1` and :math:`{}^\mathcal{B}\hat{a}_1`
        alignmentPriority (int): 0 to prioritize body heading alignment and incidence on solar arrays.

    **Description of Variables Being Tested**

    The angle :math:`\gamma` is a function of the angles :math:`\alpha` and :math:`\delta`, where :math:`\alpha` is the
    angle between the Sun direction :math:`{}^\mathcal{N}\hat{r}_{S/B}` and the reference inertial heading
    :math:`{}^\mathcal{N}\hat{h}_\text{ref}`, whereas :math:`\delta` is the angle between the body-frame heading
    :math:`{}^\mathcal{B}\hat{h}_1` and the solar array axis drive :math:`{}^\mathcal{B}\hat{a}_1`.
    The angle :math:`\gamma` is computed from the output reference attitude and compared with the results of a
    python function that computes the correct output based on the geometry of the problem. For a description of how
    such correct result is obtained, see R. Calaon, C. Allard and H. Schaub, "Attitude Reference Generation for Spacecraft
    with Rotating Solar Arrays and Pointing Constraints", in preparation for Journal of Spacecraft and Rockets.

    **General Documentation Comments**

    This unit test verifies the correctness of the generated reference attitude. It does not test the correctness of the
    reference angular rates and accelerations contained in the ``attRefOutMsg``, because this would require to run the
    module for multiple update calls. To ensure fast execution of the unit test, this is avoided.
    """

    gamma_true = computeGamma(alpha, delta)

    rHat_SB_N = np.array([1, -2, 3])                            # Sun direction in inertial coordinates
    rHat_SB_N = rHat_SB_N / np.linalg.norm(rHat_SB_N)
    a1Hat_B = np.array([-9, 8, 7])                            # array axis direction in body frame
    a1Hat_B = a1Hat_B / np.linalg.norm(a1Hat_B)

    a = np.cross(rHat_SB_N, [4, 5, -6])
    a = a / np.linalg.norm(a)
    
    d = np.cross(a1Hat_B, [6, -5, 4])
    d = d / np.linalg.norm(d)
    
    DCM1 = rbk.PRV2C(a * alpha)
    DCM2 = rbk.PRV2C(d * delta)

    hHat_N = np.matmul(DCM1, rHat_SB_N)             # required thrust direction in inertial frame, at an angle alpha from rHat_SB_N
    hHat_B = np.matmul(DCM2, a1Hat_B)               # required thrust direction in inertial frame, at an angle alpha from rHat_SB_N

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
    np.testing.assert_allclose(gamma_sim, gamma_true, rtol=0, atol=accuracy, verbose=True)
    np.testing.assert_allclose(hHat_B, np.matmul(RN, hHat_N), rtol=0, atol=accuracy, verbose=True)

    return


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_(
                 False,
                 0.62831,    # alpha
                 0.79586,    # delta
                 0,          # priorityFlag
                 1e-12       # accuracy
               )
