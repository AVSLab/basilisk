''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
#
#   Unit Test Script
#   Module Name:        PRV_Steering
#   Author:             Hanspeter Schaub
#   Creation Date:      December 18, 2015
#
import pytest
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.
import numpy as np


#   Import all of the modules that we are going to call in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import prvSteering
from Basilisk.fswAlgorithms import rateServoFullNonlinear
from Basilisk.architecture import messaging2

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)
# provide a unique test method name, starting with test_
@pytest.mark.parametrize("simCase", [0, 1])
def test_prvSteering(show_plots, simCase):     # update "subModule" in this function name to reflect the module name
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = subModuleTestFunction(show_plots, simCase)
    assert testResults < 1, testMessage


def subModuleTestFunction(show_plots, simCase):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    #   Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    #   Construct algorithm and associated C++ container
    moduleConfig = prvSteering.PrvSteeringConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "prvSteering"

    servoConfig = rateServoFullNonlinear.rateServoFullNonlinearConfig()
    servoWrap = unitTestSim.setModelDataWrap(servoConfig)
    servoWrap.ModelTag = "rate_servo"

    #   Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)
    unitTestSim.AddModelToTask(unitTaskName, servoWrap, servoConfig)

    # configure BSK modules
    moduleConfig.K1 = 0.15
    moduleConfig.K3 = 1.0
    moduleConfig.omega_max = 1.5*macros.D2R
    servoConfig.Ki = 0.01
    servoConfig.P = 150.0
    servoConfig.integralLimit = 2./servoConfig.Ki * 0.1
    servoConfig.knownTorquePntB_B = [0., 0., 0.]


    #   Create input message and size it because the regular creator of that message
    #   is not part of the test.

    #   attGuidOut Message:
    guidCmdData = messaging2.AttGuidMsgPayload()  # Create a structure for the input message

    sigma_BR = []
    if simCase == 0:
        sigma_BR = np.array([0.3, -0.5, 0.7])
    if simCase == 1:
        sigma_BR = np.array([0, 0, 0])
    guidCmdData.sigma_BR = sigma_BR

    omega_BR_B = np.array([0.010, -0.020, 0.015])
    guidCmdData.omega_BR_B = omega_BR_B
    omega_RN_B = np.array([-0.02, -0.01, 0.005])
    guidCmdData.omega_RN_B = omega_RN_B
    domega_RN_B = np.array([0.0002, 0.0003, 0.0001])
    guidCmdData.domega_RN_B = domega_RN_B
    guidInMsg = messaging2.AttGuidMsg().write(guidCmdData)

    # vehicleConfigData Message:
    vehicleConfigOut = messaging2.VehicleConfigMsgPayload()
    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    vehicleConfigOut.ISCPntB_B = I
    vcInMsg = messaging2.VehicleConfigMsg().write(vehicleConfigOut)

    # wheelSpeeds Message
    rwSpeedMessage = messaging2.RWSpeedMsgPayload()
    Omega = [10.0, 25.0, 50.0, 100.0]
    rwSpeedMessage.wheelSpeeds = Omega
    rwSpeedInMsg = messaging2.RWSpeedMsg().write(rwSpeedMessage)

    # wheelConfigData message
    def writeMsgInWheelConfiguration():
        rwConfigParams = messaging2.RWArrayConfigMsgPayload()
        rwConfigParams.GsMatrix_B = [
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        ]
        rwConfigParams.JsList = [0.1, 0.1, 0.1, 0.1]
        rwConfigParams.numRW = 4
        rwParamInMsg = messaging2.RWArrayConfigMsg().write(rwConfigParams)
        return rwParamInMsg

    rwParamInMsg = writeMsgInWheelConfiguration()

    # wheelAvailability message
    def writeMsgInWheelAvailability():
        rwAvailabilityMessage = messaging2.RWAvailabilityMsgPayload()
        avail = [messaging2.AVAILABLE, messaging2.AVAILABLE, messaging2.AVAILABLE, messaging2.AVAILABLE]
        rwAvailabilityMessage.wheelAvailability = avail
        rwAvailInMsg = messaging2.RWAvailabilityMsg().write(rwAvailabilityMessage)
        return rwAvailInMsg

    rwAvailInMsg = writeMsgInWheelAvailability()


    #   Setup logging on the test module output message so that we get all the writes to it
    dataLog = servoConfig.cmdTorqueOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    #   connect input and output messages
    moduleConfig.guidInMsg.subscribeTo(guidInMsg)
    servoConfig.vehConfigInMsg.subscribeTo(vcInMsg)
    servoConfig.guidInMsg.subscribeTo(guidInMsg)
    servoConfig.rwParamsInMsg.subscribeTo(rwParamInMsg)
    servoConfig.rwAvailInMsg.subscribeTo(rwAvailInMsg)
    servoConfig.rwSpeedsInMsg.subscribeTo(rwSpeedInMsg)
    servoConfig.rateSteeringInMsg.subscribeTo(moduleConfig.rateCmdOutMsg)

    #   Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    #   Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    servoWrap.Reset(1)     # this module reset function needs a time input (in NanoSeconds)

    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # set the filtered output truth states
    trueVector = []
    if simCase == 0:
        trueVector = [
                   [-2.9352922876097969, +6.2831737715827778, -4.0554726129822907]
                  ,[-2.9352922876097969, +6.2831737715827778, -4.0554726129822907]
                  ,[-2.9353853745179044, +6.2833455830962901, -4.0556481491012084]
                  ,[-2.9352922876097969, +6.2831737715827778, -4.0554726129822907]
                  ,[-2.9353853745179044, +6.2833455830962901, -4.0556481491012084]
                   ]
    if simCase == 1:
        trueVector = [
                     [-1.39,      3.79,     -1.39]
                    ,[-1.39,      3.79,     -1.39]
                    ,[-1.39005,   3.7901,   -1.390075]
                    ,[-1.39,      3.79,     -1.39]
                    ,[-1.39005,   3.7901,   -1.390075]
                     ]

    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0,len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(dataLog.torqueRequestBody[i], trueVector[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed torqueRequestBody unit test at t="
                                + str(dataLog.times()[i]*macros.NANO2SEC) + "sec\n")







    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    if show_plots:
          plt.show()

    if testFailCount == 0:
        print("PASSED: " + moduleWrap.ModelTag)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]





#
#   This statement below ensures that the unitTestScript can be run as a stand-along python scripts
#   authmatically executes the runUnitTest() method
#
if __name__ == "__main__":
    test_prvSteering(True, 1)
