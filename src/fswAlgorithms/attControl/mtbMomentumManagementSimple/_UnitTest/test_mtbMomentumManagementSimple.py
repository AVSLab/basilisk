#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        mtbMomentumManagement
#   Author:             Henry Macanas
#   Creation Date:      07 07, 2021
#
import pytest
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.
import numpy as np

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.fswAlgorithms import mtbMomentumManagementSimple  # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.architecture import messaging                     # import the message definitions
from Basilisk.architecture import bskLogging


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_

def test_mtbMomentumManagementSimple():     # update "module" in this function name to reflect the module name
    r"""
    **Validation Test Description**

    This script tests that the module returns expected non-zero and zero 
    outputs.

    **Description of Variables Being Tested**

    The output torque message is being recorded and the following variable is being checked.

    - ``torqueRequestBody``
    """
    # each test method requires a single assert method to be called
    # pass on the testPlotFixture so that the main test function may set the DataStore attributes
    [testResults, testMessage] = mtbMomentumManagementSimpleTestFunction()
    assert testResults < 1, testMessage

def mtbMomentumManagementSimpleTestFunction():
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.01)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = mtbMomentumManagementSimple.mtbMomentumManagementSimpleConfig()  # update with current values
    moduleConfig.Kp = 0.003
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "mtbMomentumManagementSimple"           # update python name of test module
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # wheelConfigData message (array is ordered c11, c22, c33, c44, ...)
    rwConfigParams = messaging.RWArrayConfigMsgPayload()
    beta = 45. * np.pi / 180.
    rwConfigParams.GsMatrix_B = [0., np.cos(beta), np.sin(beta), 0., np.sin(beta), -np.cos(beta), np.cos(beta), -np.sin(beta), 0., -np.cos(beta), -np.sin(beta), 0.]
    rwConfigParams.JsList = [0.002, 0.002, 0.002, 0.002]
    rwConfigParams.numRW = 4
    rwParamsInMsg = messaging.RWArrayConfigMsg().write(rwConfigParams)
    
    # rwSpeeds message
    rwSpeedsInMsgContainer = messaging.RWSpeedMsgPayload()
    rwSpeedsInMsgContainer.wheelSpeeds = [100., 200., 300., 400.]
    rwSpeedsInMsg = messaging.RWSpeedMsg().write(rwSpeedsInMsgContainer)

    # Setup logging on the test module output message so that we get all the writes to it
    resultTauMtbRequestOutMsg = moduleConfig.tauMtbRequestOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, resultTauMtbRequestOutMsg)

    # connect the message interfaces
    moduleConfig.rwParamsInMsg.subscribeTo(rwParamsInMsg)
    moduleConfig.rwSpeedsInMsg.subscribeTo(rwSpeedsInMsg)
    
    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Set the simulation time.
    # NOTE: the total simulation time may be longer than this value. The
    # simulation is stopped at the next logging event on or after the
    # simulation end time.
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.0))        # seconds to stop simulation
    accuracy = 1E-8

    '''
        TEST 1: 
            Check that tauMtbRequestOutMsg is non-zero when the wheel speeds
            are non-zero.
    '''
    unitTestSim.InitializeSimulation()
    unitTestSim.ExecuteSimulation()
    
    testFailCount, testMessages = unitTestSupport.compareVector([0., 0., 0.],
                                                            resultTauMtbRequestOutMsg.torqueRequestBody[0][0:3],
                                                            accuracy,
                                                            "tauMtbRequestOutMsg",
                                                            testFailCount, testMessages, ExpectedResult=0)

    '''
        TEST 2: 
            Check that tauMtbRequestOutMsg is the zero vector when the wheels
            speeds are zero.
    '''
    rwSpeedsInMsgContainer.wheelSpeeds = [0., 0., 0., 0.]
    rwSpeedsInMsg = messaging.RWSpeedMsg().write(rwSpeedsInMsgContainer)
    moduleConfig.rwSpeedsInMsg.subscribeTo(rwSpeedsInMsg)
    
    unitTestSim.InitializeSimulation()
    unitTestSim.ExecuteSimulation()
    
    testFailCount, testMessages = unitTestSupport.compareVector([0., 0., 0.],
                                                            resultTauMtbRequestOutMsg.torqueRequestBody[0][0:3],
                                                            accuracy,
                                                            "tauMtbRequestOutMsg",
                                                            testFailCount, testMessages, ExpectedResult=1)
    
    '''
        TEST 3: 
            Check that tauMtbRequestOutMsg is the zero vector when the wheels
            speeds are non-zero and the feedback gain is zero.
    '''
    rwSpeedsInMsgContainer.wheelSpeeds = [100., 200., 300., 400.]
    rwSpeedsInMsg = messaging.RWSpeedMsg().write(rwSpeedsInMsgContainer)
    moduleConfig.rwSpeedsInMsg.subscribeTo(rwSpeedsInMsg)
    
    moduleConfig.Kp = 0.
    
    unitTestSim.InitializeSimulation()
    unitTestSim.ExecuteSimulation()
    
    testFailCount, testMessages = unitTestSupport.compareVector([0., 0., 0.],
                                                            resultTauMtbRequestOutMsg.torqueRequestBody[0][0:3],
                                                            accuracy,
                                                            "tauMtbRequestOutMsg",
                                                            testFailCount, testMessages, ExpectedResult=1)
    
    
    # reset the module to test this functionality
    moduleWrap.Reset(0)     # this module reset function needs a time input (in NanoSeconds)



    print("Accuracy used: " + str(accuracy))
    if testFailCount == 0:
        print("PASSED: mtbMomentumManagementSimple unit test")
    else:
        print("Failed: mtbMomentumManagementSimple unit test")

    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_mtbMomentumManagementSimple()
