#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


#
#   Unit Test Script
#   Module Name:        solarArrayRotation
#   Author:             Riccardo Calaon
#   Creation Date:      January 9, 2023
#

import pytest
import os, inspect, random
import numpy as np

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)



# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                   # general support file with common unit test functions
from Basilisk.fswAlgorithms import solarArrayReference           # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.architecture import messaging                      # import the message definitions
from Basilisk.architecture import bskLogging


def computeRotationAngle(sigma_RN, rS_N, a1_R, a2_R, theta0):

    RN = rbk.MRP2C(sigma_RN)
    rS_R = np.matmul(RN, rS_N)

    a2_R_prime = []
    dotP = np.dot(rS_R, a1_R)
    for n in range(3):
        a2_R_prime.append(rS_R[n] - dotP * a1_R[n])
    a2_R_prime = np.array(a2_R_prime)
    a2_R_norm = np.linalg.norm(a2_R_prime)
    if a2_R_norm > 1e-6:
        a2_R_prime = a2_R_prime / a2_R_norm
        theta = np.arccos(min(max(np.dot(a2_R, a2_R_prime),-1),1))
        if np.dot(a1_R, np.cross(a2_R, a2_R_prime)) < 0:
            theta = -theta
    else:
        theta = theta0

    return theta


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.  Note that the order in that you add the parametrize method
# matters for the documentation in that it impacts the order in which the test arguments are shown.
# The first parametrize arguments are shown last in the pytest argument list
@pytest.mark.parametrize("rS_N", [[1, 0, 0],
                                  [0, 0, 1]])
@pytest.mark.parametrize("sigma_BN", [[0.1, 0.2, 0.3],
                                      [0.5, 0.4, 0.3]])
@pytest.mark.parametrize("sigma_RN", [[0.3, 0.2, 0.1],
                                      [0.9, 0.7, 0.8]])
@pytest.mark.parametrize("bodyFrame", [0, 1])                                      
@pytest.mark.parametrize("accuracy", [1e-12])


# update "module" in this function name to reflect the module name
def test_solarArrayRotation(show_plots, rS_N, sigma_BN, sigma_RN, bodyFrame, accuracy):
    r"""
    **Validation Test Description**

    This unit test verifies the correctness of the output reference angle computed by the :ref:`solarArrayRotation`.
    The inputs provided are the inertial Sun direction, current attitude of the hub, and reference frame. Based on
    current attitude, the sun direction vector is mapped into body frame coordinates and passed into the Attitude
    Navigation Message.

    **Test Parameters**

    Args:
        rS_N[3] (double): Sun direction vector, in inertial frame components;
        sigma_BN[3] (double): spacecraft hub attitude with respect to the inertial frame, in MRP;
        sigma_RN[3] (double): reference frame attitude with respect to the inertial frame, in MRP;
        bodyFrame (int): 0 to calculate reference rotation angle w.r.t. reference frame, 1 to calculate it w.r.t the current spacecraft attitude;
        accuracy (float): absolute accuracy value used in the validation tests.

    **Description of Variables Being Tested**

    This unit test checks the correctness of the output attitude reference message 

    - ``spinningBodyRefOutMsg``

    in all its parts. The reference angle ``theta`` is checked versus the value computed by a python function that computes the same angle. 
    The reference angle derivative ``thetaDot`` is checked versus zero, as the module is run for only one Update call.
    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = solarArrayRotationTestFunction(show_plots, rS_N, sigma_BN, sigma_RN, bodyFrame, accuracy)
    assert testResults < 1, testMessage


def solarArrayRotationTestFunction(show_plots, rS_N, sigma_BN, sigma_RN, bodyFrame, accuracy):

    a1_B = np.array([1, 0, 0])
    a2_B = np.array([0, 1, 0])
    BN = rbk.MRP2C(sigma_BN)
    rS_B = np.matmul(BN, rS_N)
    thetaC = 0
    thetaDotC = 0

    testFailCount = 0                        # zero unit test result counter
    testMessages = []                        # create empty array to store test log messages
    unitTaskName = "unitTask"                # arbitrary name (don't change)
    unitProcessName = "TestProcess"          # arbitrary name (don't change)
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    solarArrayConfig = solarArrayReference.solarArrayReferenceConfig()                 # update with current values
    solarArrayWrap = unitTestSim.setModelDataWrap(solarArrayConfig)
    solarArrayWrap.ModelTag = "solarArrayRotation"                                     # update python name of test module

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, solarArrayWrap, solarArrayConfig)

    # Initialize the test module configuration data
    solarArrayConfig.a1_B = a1_B
    solarArrayConfig.a2_B = a2_B
    solarArrayConfig.bodyFrame = bodyFrame

    # Create input attitude navigation message
    NatAttInMsgData = messaging.NavAttMsgPayload()
    NatAttInMsgData.sigma_BN = sigma_BN
    NatAttInMsgData.vehSunPntBdy = rS_B
    NatAttInMsg = messaging.NavAttMsg().write(NatAttInMsgData)
    solarArrayConfig.attNavInMsg.subscribeTo(NatAttInMsg)

    # Create input attitude reference message
    AttRefInMsgData = messaging.AttRefMsgPayload()
    AttRefInMsgData.sigma_RN = sigma_RN
    AttRefInMsg = messaging.AttRefMsg().write(AttRefInMsgData)
    solarArrayConfig.attRefInMsg.subscribeTo(AttRefInMsg)

    # Create input spinning body message
    SpinningBodyInMsgData = messaging.SpinningBodyMsgPayload()
    SpinningBodyInMsgData.theta = thetaC
    SpinningBodyInMsgData.thetaDot = thetaDotC
    SpinningBodyInMsg = messaging.SpinningBodyMsg().write(SpinningBodyInMsgData)
    solarArrayConfig.spinningBodyInMsg.subscribeTo(SpinningBodyInMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = solarArrayConfig.spinningBodyRefOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    if bodyFrame == 0:
        thetaR = computeRotationAngle(sigma_RN, rS_N, a1_B, a2_B, thetaC)
    else:
        thetaR = computeRotationAngle(sigma_BN, rS_N, a1_B, a2_B, thetaC)
    if thetaR-thetaC > np.pi:
        thetaR -= np.pi
    elif thetaR-thetaC < -np.pi:
        thetaR += np.pi
    # compare the module results to the truth values
    if not unitTestSupport.isDoubleEqual(dataLog.theta[0], thetaR, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + solarArrayWrap.ModelTag + "solarArrayRotation module failed unit test on thetaR for sigma_BN = [{},{},{}], sigma_RN = [{},{},{}] and bodyFrame = {} \n".format(sigma_BN[0],sigma_BN[1],sigma_BN[2],sigma_RN[0],sigma_RN[1],sigma_RN[2],bodyFrame))
    if not unitTestSupport.isDoubleEqual(dataLog.thetaDot[0], 0, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + solarArrayWrap.ModelTag + "solarArrayRotation module failed unit test on thetaDotR for sigma_BN = [{},{},{}], sigma_RN = [{},{},{}] and bodyFrame = {} \n".format(sigma_BN[0],sigma_BN[1],sigma_BN[2],sigma_RN[0],sigma_RN[1],sigma_RN[2],bodyFrame))

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_solarArrayRotation(              # update "module" in function name
                 False,
                 np.array([1, 0, 0]),
                 np.array([0.1, 0.2, 0.3]),
                 np.array([0.3, 0.2, 0.1]),
                 0,
                 1e-12        # accuracy
               )
