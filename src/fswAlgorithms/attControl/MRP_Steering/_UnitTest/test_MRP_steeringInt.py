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
import sys, os, inspect
import numpy as np
import pytest







from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import MRP_Steering  # import the module that is to be tested
from Basilisk.fswAlgorithms import rateServoFullNonlinear
from Basilisk.fswAlgorithms import fswMessages
from Basilisk.simulation import simFswInterfaceMessages
from Basilisk.utilities import macros

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def test_mrp_steering_tracking(show_plots):
    [testResults, testMessage] = mrp_steering_tracking(show_plots)
    assert testResults < 1, testMessage


def mrp_steering_tracking(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = MRP_Steering.MRP_SteeringConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "MRP_Steering"

    servoConfig = rateServoFullNonlinear.rateServoFullNonlinearConfig()
    servoWrap = unitTestSim.setModelDataWrap(servoConfig)
    servoWrap.ModelTag = "rate_servo"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)
    unitTestSim.AddModelToTask(unitTaskName, servoWrap, servoConfig)

    # Initialize the test module configuration data
    moduleConfig.inputGuidName = "inputGuidName"
    moduleConfig.outputDataName = "rate_steering"
    servoConfig.inputGuidName = moduleConfig.inputGuidName
    servoConfig.vehConfigInMsgName = "vehicleConfigName"
    servoConfig.rwParamsInMsgName = "rwa_config_data_parsed"
    servoConfig.rwAvailInMsgName = "rw_availability"
    servoConfig.inputRWSpeedsName = "reactionwheel_speeds"
    servoConfig.inputRateSteeringName = moduleConfig.outputDataName
    servoConfig.outputDataName = "outputName"

    moduleConfig.K1 = 0.15
    moduleConfig.K3 = 1.0
    moduleConfig.omega_max = 1.5 * macros.D2R
    servoConfig.Ki = 0.01
    servoConfig.P = 150.0
    servoConfig.integralLimit = 2. / servoConfig.Ki * 0.1
    servoConfig.knownTorquePntB_B = [0., 0., 0.]

    #   Create input message and size it because the regular creator of that message
    #   is not part of the test.
    #   attGuidOut Message:
    guidCmdData = fswMessages.AttGuidFswMsg()  # Create a structure for the input message
    inputMessageSize = guidCmdData.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName, moduleConfig.inputGuidName,
                                          inputMessageSize, 2)# number of buffers (leave at 2 as default, don't make zero)

    sigma_BR = np.array([0.3, -0.5, 0.7])
    guidCmdData.sigma_BR = sigma_BR
    omega_BR_B = np.array([0.010, -0.020, 0.015])
    guidCmdData.omega_BR_B = omega_BR_B
    omega_RN_B = np.array([-0.02, -0.01, 0.005])
    guidCmdData.omega_RN_B = omega_RN_B
    domega_RN_B = np.array([0.0002, 0.0003, 0.0001])
    guidCmdData.domega_RN_B = domega_RN_B
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputGuidName, inputMessageSize,
                                          0, guidCmdData)

    # vehicleConfigData Message:
    vehicleConfigOut = fswMessages.VehicleConfigFswMsg()
    inputMessageSize = vehicleConfigOut.getStructSize()                           # 18 doubles + 1 32bit integer
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName, servoConfig.vehConfigInMsgName,
                                          inputMessageSize, 2)            # number of buffers (leave at 2 as default, don't make zero)
    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    vehicleConfigOut.ISCPntB_B = I
    unitTestSim.TotalSim.WriteMessageData(servoConfig.vehConfigInMsgName,
                                          inputMessageSize,
                                          0, vehicleConfigOut)

    # wheelSpeeds Message
    rwSpeedMessage = simFswInterfaceMessages.RWSpeedIntMsg()
    inputMessageSize = rwSpeedMessage.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          servoConfig.inputRWSpeedsName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    Omega = [10.0, 25.0, 50.0, 100.0]
    rwSpeedMessage.wheelSpeeds = Omega
    unitTestSim.TotalSim.WriteMessageData(servoConfig.inputRWSpeedsName,
                                          inputMessageSize,
                                          0,
                                          rwSpeedMessage)

    # wheelConfigData message
    def writeMsgInWheelConfiguration():
        rwConfigParams = fswMessages.RWArrayConfigFswMsg()
        inputMessageSize = rwConfigParams.getStructSize()
        unitTestSim.TotalSim.CreateNewMessage(unitProcessName, servoConfig.rwParamsInMsgName,
                                              inputMessageSize, 2) # number of buffers (leave at 2 as default)
        rwConfigParams.GsMatrix_B = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.5773502691896258, 0.5773502691896258, 0.5773502691896258
        ]
        rwConfigParams.JsList = [0.1, 0.1, 0.1, 0.1]
        rwConfigParams.numRW = 4
        unitTestSim.TotalSim.WriteMessageData(servoConfig.rwParamsInMsgName, inputMessageSize,
                                              0, rwConfigParams)
    if len(servoConfig.rwParamsInMsgName)>0:
        writeMsgInWheelConfiguration()

    # wheelAvailability message
    def writeMsgInWheelAvailability():
        rwAvailabilityMessage = rateServoFullNonlinear.RWAvailabilityFswMsg()
        inputMessageSize = rwAvailabilityMessage.getStructSize()
        unitTestSim.TotalSim.CreateNewMessage(unitProcessName, servoConfig.rwAvailInMsgName,
                                              inputMessageSize, 2) # number of buffers (leave at 2 as default)
        avail = [rateServoFullNonlinear.AVAILABLE, rateServoFullNonlinear.AVAILABLE, rateServoFullNonlinear.AVAILABLE, rateServoFullNonlinear.AVAILABLE]
        rwAvailabilityMessage.wheelAvailability = avail
        unitTestSim.TotalSim.WriteMessageData(servoConfig.rwAvailInMsgName, inputMessageSize,
                                              0, rwAvailabilityMessage)

    if len(servoConfig.rwAvailInMsgName)>0:
        writeMsgInWheelAvailability()

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(servoConfig.outputDataName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    servoWrap.Reset(1)  # this module reset function needs a time input (in NanoSeconds)

    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "torqueRequestBody"
    moduleOutput = unitTestSim.pullMessageLogData(servoConfig.outputDataName + '.' + moduleOutputName,
                                                  range(3))

    # set the filtered output truth states
    trueVector = [
        [-4.447838135953905, 8.4091608989577455, -4.8609788769036752]
        , [-4.447838135953905, 8.4091608989577455, -4.8609788769036752]
        , [-4.4480000793994314, 8.4093848702156535, -4.8611816778282186]
        , [-4.447838135953905, 8.4091608989577455, -4.8609788769036752]
        , [-4.4480000793994314, 8.4093848702156535, -4.8611816778282186]
    ]

    # compare the module results to the truth values
    accuracy = 1e-12
    for i in range(0, len(trueVector)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i], trueVector[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " + moduleOutputName +
                                " unit test at t=" + str(moduleOutput[i, 0] * macros.NANO2SEC) +
                                "sec \n")

    print moduleOutput
    print trueVector

    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    if show_plots:
        plt.show()

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    test_mrp_steering_tracking(False)
