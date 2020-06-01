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
from Basilisk.fswAlgorithms import rateServoFullNonlinear  # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.fswAlgorithms import fswMessages
from Basilisk.simulation import simFswInterfaceMessages

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_


@pytest.mark.parametrize("rwNum", [4, 0])
@pytest.mark.parametrize("intGain", [0.01, -1])
@pytest.mark.parametrize("omegap_BastR_B", [(1.87766650e-04, -3.91233583e-05, 3.56369489e-05), (0, 0, 0)])
@pytest.mark.parametrize("omega_BastR_B",  [(-2.23886891e-02, 2.47942516e-02, -2.55601849e-02), (0, 0, 0)])
@pytest.mark.parametrize("integralLimit", [0, 20])
@pytest.mark.parametrize("useRwAvailability", ["NO", "ON", "OFF"])


def test_rate_servo_full_nonlinear(show_plots, rwNum, intGain, omegap_BastR_B, omega_BastR_B, integralLimit,
                                   useRwAvailability):
    """Module Unit Test"""

    [testResults, testMessage] = rate_servo_full_nonlinear(show_plots, rwNum, intGain, omegap_BastR_B, omega_BastR_B,
                                                           integralLimit, useRwAvailability)

    assert testResults < 1, testMessage


def rate_servo_full_nonlinear(show_plots,rwNum, intGain, omegap_BastR_B, omega_BastR_B, integralLimit,
                              useRwAvailability):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    #__tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = rateServoFullNonlinear.rateServoFullNonlinearConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "rate_servo"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    # Initialize the test module configuration data
    moduleConfig.inputGuidName = "inputGuidName"
    moduleConfig.vehConfigInMsgName = "vehicleConfigName"
    moduleConfig.rwParamsInMsgName = "rwa_config_data_parsed"
    moduleConfig.rwAvailInMsgName = "rw_availability"
    moduleConfig.inputRWSpeedsName = "reactionwheel_speeds"
    moduleConfig.inputRateSteeringName = "rate_steering"
    moduleConfig.outputDataName = "outputName"

    moduleConfig.Ki = intGain
    moduleConfig.P = 150.0
    moduleConfig.integralLimit = integralLimit
    moduleConfig.knownTorquePntB_B = (1,1,1)

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
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName, moduleConfig.vehConfigInMsgName,
                                          inputMessageSize, 2)            # number of buffers (leave at 2 as default, don't make zero)
    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    vehicleConfigOut.ISCPntB_B = I
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.vehConfigInMsgName,
                                          inputMessageSize,
                                          0, vehicleConfigOut)

    # wheelSpeeds Message
    rwSpeedMessage = simFswInterfaceMessages.RWSpeedIntMsg()
    inputMessageSize = rwSpeedMessage.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputRWSpeedsName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    Omega = [10.0, 25.0, 50.0, 100.0]
    rwSpeedMessage.wheelSpeeds = Omega
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputRWSpeedsName,
                                          inputMessageSize,
                                          0,
                                          rwSpeedMessage)

    # wheelConfigData message
    jsList = []
    GsMatrix_B = []
    def writeMsgInWheelConfiguration():
        rwConfigParams = fswMessages.RWArrayConfigFswMsg()
        inputMessageSize = rwConfigParams.getStructSize()
        unitTestSim.TotalSim.CreateNewMessage(unitProcessName, moduleConfig.rwParamsInMsgName,
                                              inputMessageSize, 2)  # number of buffers (leave at 2 as default)
        rwConfigParams.GsMatrix_B = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.5773502691896258, 0.5773502691896258, 0.5773502691896258
        ]
        rwConfigParams.JsList = [0.1, 0.1, 0.1, 0.1]
        rwConfigParams.numRW = rwNum
        unitTestSim.TotalSim.WriteMessageData(moduleConfig.rwParamsInMsgName, inputMessageSize,
                                              0, rwConfigParams)
        jsList = rwConfigParams.JsList
        GsMatrix_B = rwConfigParams.GsMatrix_B
        return jsList, GsMatrix_B


    if len(moduleConfig.rwParamsInMsgName) > 0:
        jsList, GsMatrix_B = writeMsgInWheelConfiguration()

    # wheelAvailability message
    rwAvailabilityMessage = fswMessages.RWAvailabilityFswMsg()
    if useRwAvailability != "NO":
        moduleConfig.rwAvailInMsgName = "rw_availability"
        if useRwAvailability == "ON":
            rwAvailabilityMessage.wheelAvailability  = [rateServoFullNonlinear.AVAILABLE, rateServoFullNonlinear.AVAILABLE,
                                                        rateServoFullNonlinear.AVAILABLE, rateServoFullNonlinear.AVAILABLE]
        elif useRwAvailability == "OFF":
            rwAvailabilityMessage.wheelAvailability  = [rateServoFullNonlinear.UNAVAILABLE, rateServoFullNonlinear.UNAVAILABLE,
                                                        rateServoFullNonlinear.UNAVAILABLE, rateServoFullNonlinear.UNAVAILABLE]
        else:
            print("WARNING: unknown rw availability status")

        unitTestSupport.setMessage(unitTestSim.TotalSim,
                                   unitProcessName,
                                   moduleConfig.rwAvailInMsgName,
                                   rwAvailabilityMessage)
    else:
        # set default availability
        rwAvailabilityMessage.wheelAvailability = [rateServoFullNonlinear.AVAILABLE, rateServoFullNonlinear.AVAILABLE,
                                                   rateServoFullNonlinear.AVAILABLE, rateServoFullNonlinear.AVAILABLE]


    # rateSteering message
    rateSteeringMsg = fswMessages.RateCmdFswMsg()
    inputMessageSize = rateSteeringMsg.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.inputRateSteeringName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)

    rateSteeringMsg.omega_BastR_B = omega_BastR_B
    rateSteeringMsg.omegap_BastR_B = omegap_BastR_B
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.inputRateSteeringName,
                                          inputMessageSize,
                                          0,
                                          rateSteeringMsg)

    # Setup logging on the test module output message so that we get all the writes to it
    unitTestSim.TotalSim.logThisMessage(moduleConfig.outputDataName, testProcessRate)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    # Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    moduleWrap.Reset(1)  # this module reset function needs a time input (in NanoSeconds)

    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # This pulls the actual data log from the simulation run.
    # Note that range(3) will provide [0, 1, 2]  Those are the elements you get from the vector (all of them)
    moduleOutputName = "torqueRequestBody"
    moduleOutput = unitTestSim.pullMessageLogData(moduleConfig.outputDataName + '.' + moduleOutputName,
                                                  list(range(3)))

    # set the filtered output truth states
    LrTrue = findTrueTorques(moduleConfig, guidCmdData, rwSpeedMessage, vehicleConfigOut, jsList,
                             rwNum, GsMatrix_B, rwAvailabilityMessage,rateSteeringMsg)


    # compare the module results to the truth values
    accuracy = 1e-8
    for i in range(0, len(LrTrue)):
        # check a vector values
        if not unitTestSupport.isArrayEqual(moduleOutput[i], LrTrue[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed " + moduleOutputName +
                                " unit test at t=" + str(moduleOutput[i, 0] * macros.NANO2SEC) +
                                "sec \n")

    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    if show_plots:
        plt.show()

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + moduleWrap.ModelTag)

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]






def findTrueTorques(moduleConfig,guidCmdData,rwSpeedMessage,vehicleConfigOut,jsList,numRW,GsMatrix_B,rwAvailMsg,rateSteeringMsg ):
    Lr = []

    #Read in variables
    L = np.asarray(moduleConfig.knownTorquePntB_B)
    steps = [0, 0, .5, 0, .5]
    omega_BR_B = np.asarray(guidCmdData.omega_BR_B)
    omega_RN_B = np.asarray(guidCmdData.omega_RN_B)
    omega_BN_B = omega_BR_B + omega_RN_B #find body rate
    domega_RN_B = np.asarray(guidCmdData.domega_RN_B)
    omega_BastR_B = np.asarray(rateSteeringMsg.omega_BastR_B)
    omegap_BastR_B = np.asarray(rateSteeringMsg.omegap_BastR_B) #body-frame derivative of omega_BastR_B
    omega_BastN_B = omega_BastR_B+omega_RN_B
    omega_BBast_B = omega_BN_B - omega_BastN_B

    Isc = np.asarray(vehicleConfigOut.ISCPntB_B)
    Isc = np.reshape(Isc, (3, 3))
    Ki = moduleConfig.Ki
    P = moduleConfig.P
    jsVec = jsList
    GsMatrix_B_array = np.asarray(GsMatrix_B)
    GsMatrix_B_array = np.reshape(GsMatrix_B_array[0:numRW * 3], (numRW, 3))

    #Compute toruqes
    for i in range(len(steps)):
        dt = steps[i]
        if dt == 0:
            zVec = np.asarray([0, 0, 0])

        #evaluate integral term
        if Ki > 0 and abs(moduleConfig.integralLimit) > 0: #if integral feedback is on
            zVec = dt * omega_BBast_B + zVec  # z = integral(del_omega)
            # Make sure each component is less than the integral limit
            for i in range(3):
                if zVec[i] > moduleConfig.integralLimit:
                        zVec[i] = zVec[i]/abs(zVec[i])*moduleConfig.integralLimit

        else: #integral gain turned off/negative setting
            zVec = np.asarray([0, 0, 0])

        #compute torque Lr
        Lr0 = Ki * zVec  # +K*sigmaBR
        Lr1 = Lr0 + P * omega_BBast_B  # +P*deltaOmega

        GsHs = np.array([0,0,0])

        if numRW > 0:
            for i in range(numRW):
                if rwAvailMsg.wheelAvailability[i] == 0:  # Make RW availability check
                    GsHs = GsHs + np.dot(GsMatrix_B_array[i, :], jsVec[i]*(np.dot(omega_BN_B, GsMatrix_B_array[i, :]) + rwSpeedMessage.wheelSpeeds[i]))
                    # J_s*(dot(omegaBN_B,Gs_vec)+Omega_wheel)

        Lr2 = Lr1 - np.cross(omega_BastN_B, (Isc.dot(omega_BN_B)+GsHs))  #  - omega_BastN x ([I]omega + [Gs]h_s)

        Lr3 = Lr2 - Isc.dot(omegap_BastR_B + domega_RN_B - np.cross(omega_BN_B, omega_RN_B))
        # - [I](d(omega_B^ast/R)/dt + d(omega_r)/dt - omega x omega_r)
        Lr4 = Lr3 + L
        Lr4 = -Lr4
        Lr.append(np.ndarray.tolist(Lr4))
    return Lr



if __name__ == "__main__":
    test_rate_servo_full_nonlinear(False, #show plots T/F
                                   4,           # Number of RW
                                   0.01,        # Integral Gain
                                   (0,0,0),     # omegap_BastR_B
                                   (0,0,0),     # omega_BastR_B
                                   20,          # integraLimit
                                   "ON")        # useRwAvailability
