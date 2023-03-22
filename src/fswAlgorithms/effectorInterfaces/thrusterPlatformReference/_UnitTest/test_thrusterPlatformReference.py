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


#
#   Unit Test Script
#   Module Name:        thrusterPlatformReference
#   Author:             Riccardo Calaon
#   Creation Date:      February 15, 2023
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
from Basilisk.utilities import unitTestSupport 
from Basilisk.fswAlgorithms import thrusterPlatformReference
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.architecture import messaging
from Basilisk.architecture import bskLogging


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.  Note that the order in that you add the parametrize method
# matters for the documentation in that it impacts the order in which the test arguments are shown.
# The first parametrize arguments are shown last in the pytest argument list
@pytest.mark.parametrize("seed", list(np.linspace(1,10,10)))
@pytest.mark.parametrize("delta_CM", [0.1, 0.2, 0.3])
@pytest.mark.parametrize("K", [0,1,5,10])
@pytest.mark.parametrize("accuracy", [1e-10])

# update "module" in this function name to reflect the module name
def test_platformRotation(show_plots, delta_CM, K, seed, accuracy):
    r"""
    **Validation Test Description**

    This unit test script tests the correctness of the tip and tilt reference angles computed by 
    :ref:`thrusterPlatformReference`. The correctness of the output is determined based on whether the thruster 
    is aligned with the system's center of mass, when the momentum dumping control gain :math:`\kappa = 0`.
    Moreover, the other module output messages, ``bodyHeadingOutMsg`` and ``thrusterTorqueOutMsg`` are checked
    versus equivalent python code.

    **Test Parameters**

    This test randomizes the position of the center of mass and runs the test 10 times for any other combination
    of test parameters. 

    Args:
        delta_CM (m): magnitude of the center of mass shift, whose direction is generated randomly
        K (Hz): proportional gain of the momentum dumping control law
        seed (-): seed is varied to randomly change the shift in the center of mass
        accuracy (float): accuracy within which results are considered to match the truth values.

    **Description of Variables Being Tested**

    For :math:`\kappa = 0`, the correctness of the result is assessed based on the norm of the
    cross product between the thrust direction vector :math:`{}^\mathcal{F}\boldsymbol{t}` and the relative position
    of the center of mass with respect to the thruster application point :math:`T`. For :math:`\kappa \neq 0` this 
    test is not performed, as the thruster is not aligned with the center of mass.

    The python code also computes equivalently the thrust direction in body frame coordinates :math:`{}^\mathcal{B}\boldsymbol{t}`
    and the net torque on the system :math:`{}^\mathcal{B}\boldsymbol{L}`, and compares them to the respective output
    messages for all values of :math:`\kappa = 0` tested.

    **General Documentation Comments**

    The offset vectors provided as input parameters ensure that a solution exists, such that the Unit Test can correctly
    assess the alignment of the thruster. This is, in general, not guaranteed.
    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = platformRotationTestFunction(show_plots, delta_CM, K, seed, accuracy)
    assert testResults < 1, testMessage


def platformRotationTestFunction(show_plots, delta_CM, K, seed, accuracy):

    random.seed(seed)

    sigma_MB = np.array([0., 0., 0.])
    r_BM_M = np.array([0.0, 0.1, 1.4])
    r_FM_F = np.array([0.0, 0.0, -0.1])
    r_TF_F = np.array([-0.01, 0.03, 0.02])
    T_F    = np.array([1.0, 1.0, 10.0])

    r_CB_B = np.array([0,0,0]) + np.random.rand(3)
    r_CB_B = r_CB_B / np.linalg.norm(r_CB_B) * delta_CM

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
    platformConfig = thrusterPlatformReference.ThrusterPlatformReferenceConfig()
    platformWrap = unitTestSim.setModelDataWrap(platformConfig)
    platformWrap.ModelTag = "platformReference"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, platformWrap, platformConfig)

    # Initialize the test module configuration data
    platformConfig.sigma_MB = sigma_MB
    platformConfig.r_BM_M = r_BM_M
    platformConfig.r_FM_F = r_FM_F
    platformConfig.r_TF_F = r_TF_F
    platformConfig.T_F    = T_F
    platformConfig.K      = K

    # Create input vehicle configuration msg
    inputVehConfigMsgData = messaging.VehicleConfigMsgPayload()
    inputVehConfigMsgData.CoM_B = r_CB_B
    inputVehConfigMsg = messaging.VehicleConfigMsg().write(inputVehConfigMsgData)
    platformConfig.vehConfigInMsg.subscribeTo(inputVehConfigMsg)

    # Create input RW configuration msg
    inputRWConfigMsgData = messaging.RWArrayConfigMsgPayload()
    inputRWConfigMsgData.GsMatrix_B = [1,0,0,0,1,0,0,0,1]
    inputRWConfigMsgData.JsList = [0.01, 0.01, 0.01]
    inputRWConfigMsgData.numRW = 3
    inputRWConfigMsgData.uMax = [0.001, 0.001, 0.001]
    inputRWConfigMsg = messaging.RWArrayConfigMsg().write(inputRWConfigMsgData)
    platformConfig.rwConfigDataInMsg.subscribeTo(inputRWConfigMsg)

    # Create input RW speeds msg
    inputRWSpeedsMsgData = messaging.RWSpeedMsgPayload()
    inputRWSpeedsMsgData.wheelSpeeds = [100, 100, 100]
    inputRWSpeedsMsg = messaging.RWSpeedMsg().write(inputRWSpeedsMsgData)
    platformConfig.rwSpeedsInMsg.subscribeTo(inputRWSpeedsMsg)

    # Setup logging on the test module output messages so that we get all the writes to it
    ref1Log = platformConfig.hingedRigidBodyRef1OutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, ref1Log)
    ref2Log = platformConfig.hingedRigidBodyRef2OutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, ref2Log)
    bodyHeadingLog = platformConfig.bodyHeadingOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, bodyHeadingLog)
    thrusterTorqueLog = platformConfig.thrusterTorqueOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, thrusterTorqueLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))        # seconds to stop simulation

    # Begin the simulation time run set above
    unitTestSim.ExecuteSimulation()

    theta1 = ref1Log.theta[0]
    theta2 = ref2Log.theta[0]

    FM = [[np.cos(theta2),  np.sin(theta1)*np.sin(theta2), -np.cos(theta1)*np.sin(theta2)],
          [       0      ,          np.cos(theta1)       ,         np.sin(theta1)        ],
          [np.sin(theta2), -np.sin(theta1)*np.cos(theta2),  np.cos(theta1)*np.cos(theta2)]]

    MB = rbk.MRP2C(sigma_MB)

    r_CB_M = np.matmul(MB, r_CB_B)
    r_CM_M = r_CB_M + r_BM_M
    r_CM_F = np.matmul(FM, r_CM_M)
    r_CT_F = r_CM_F - r_FM_F - r_TF_F

    offset = np.linalg.norm(np.cross(r_CT_F,T_F) / np.linalg.norm(np.array(r_CT_F)) / np.linalg.norm(np.array(T_F)))

    # check if the CM offset is zero if control gain K is also 0
    if K == 0:
        if not unitTestSupport.isDoubleEqual(offset, 0.0, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + platformWrap.ModelTag + "thrusterPlatformReference module failed unit test on zero offset \n")

    T_B_hat_sim = bodyHeadingLog.rHat_XB_B[0]               # simulation result
    FB = np.matmul(FM, MB)
    T_B = np.matmul(FB.transpose(), T_F)
    T_B_hat = T_B / np.linalg.norm(T_B)                     # truth value

    # compare the module results to the python computation for body-frame thruster direction
    if not unitTestSupport.isVectorEqual(T_B_hat_sim, T_B_hat, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + platformWrap.ModelTag + "thrusterPlatformReference module failed unit test on body frame thruster direction \n")

    L_B_sim = thrusterTorqueLog.torqueRequestBody[0]        # simulation result
    L_F = np.cross(r_CT_F, T_F)
    L_B = np.matmul(FB.transpose(),L_F)

    # compare the module results to the python computation for body-frame cmd torque
    if not unitTestSupport.isVectorEqual(L_B_sim, L_B, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + platformWrap.ModelTag + "thrusterPlatformReference module failed unit test on thruster torque \n")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_platformRotation(
                 False,                   # show_plots
                 0.1,                     # delta_CM
                 0,                       # k
                 np.random.rand(1)[0],    # seed
                 1e-10                    # accuracy
               )
