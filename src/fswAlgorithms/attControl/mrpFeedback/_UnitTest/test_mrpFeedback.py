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
#   Module Name:        MRP_Feedback
#   Author:             Hanspeter Schaub
#   Creation Date:      December 18, 2015
#
import pytest
import sys, os, inspect
# import packages as needed e.g. 'numpy', 'ctypes, 'math' etc.
import numpy as np








#   Import all of the modules that we are going to call in this simulation

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport
from Basilisk.fswAlgorithms import mrpFeedback
from Basilisk.architecture import messaging2

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
#@pytest.mark.xfail()
# provide a unique test method name, starting with test_


@pytest.mark.parametrize("intGain", [0.01, -1])
@pytest.mark.parametrize("rwNum", [4, 0])
@pytest.mark.parametrize("integralLimit", [0, 20])
@pytest.mark.parametrize("useRwAvailability", ["NO", "ON", "OFF"])

def test_MRP_Feedback(show_plots, intGain, rwNum, integralLimit, useRwAvailability):
    r"""
        **Validation Test Description**

        The unit test for this module tests a set of gains :math:`K`, :math:`K_i`, :math:`P` on a rigid body
        with no external torques, and with a fixed input reference attitude message. The torque requested
        by the controller is evaluated against python computed torques at 0s, 0.5s, 1s, 1.5s and 2s to
        within a tolerance of :math:`10^{-8}`. After 1s the simulation is stopped and the ``Reset()``
        function is called to check that integral feedback related variables are properly reset.
        The following permutations are run:

        - The test is run for a case with error integration feedback (:math:`k_i`=0.01) and one case
          where :math:`k_i` is set to a negative value, resulting in a case with no integrator.
        - The RW array number is configured either to 4 or 0
        - The integral limit term is set to either 0 or 20
        - The RW availability message is tested in 3 manners.  Either the availability  message is not
          written where all wheels should default to being available.  If the availability message is
          written, then the RWs are either zero to available or not available.
        - The control parameter :math:`\delta\omega_{0}` is set to either a zero or non-zero vector

        All permutations of these test cases are expected to pass.


        **Test Parameters**

        Args:
            intGain (float): value of the integral gain :math:`K_i`
            rwNum (int): number of RW devices to simulate
            integralLimit (float): value of the integral limit
            useRwAvailability (string): Flag to not use RW availabillity (``NO``), use the availability
               message and turn on the RW devices (``ON``) and use the message and turn off the devices (``OFF``)
    """

    # each test method requires a single assert method to be called

    [testResults, testMessage] = run(show_plots,intGain, rwNum, integralLimit, useRwAvailability)

    assert testResults < 1, testMessage


def run(show_plots, intGain, rwNum, integralLimit, useRwAvailability):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
                                                        # this creates a fresh and consistent simulation environment for each test run

    #   Create test thread
    testProcessRate = macros.sec2nano(0.5)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))


    #   Construct algorithm and associated C++ container
    moduleConfig = mrpFeedback.mrpFeedbackConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "MRP_Feedback"

    #   Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    #   Initialize the test module configuration data
    moduleConfig.K = 0.15
    moduleConfig.Ki = intGain
    moduleConfig.P = 150.0
    moduleConfig.integralLimit = integralLimit
    moduleConfig.knownTorquePntB_B = [1., 1., 1.]

    # create input messages
    #   AttGuidFswMsg Message:
    guidCmdData = messaging2.AttGuidMsgPayload()
    sigma_BR = [0.3, -0.5, 0.7]
    guidCmdData.sigma_BR = sigma_BR
    omega_BR_B = [0.010, -0.020, 0.015]
    guidCmdData.omega_BR_B = omega_BR_B
    omega_RN_B = [-0.02, -0.01, 0.005]
    guidCmdData.omega_RN_B = omega_RN_B
    domega_RN_B = [0.0002, 0.0003, 0.0001]
    guidCmdData.domega_RN_B = domega_RN_B
    guidInMsg = messaging2.AttGuidMsg().write(guidCmdData)

    # vehicleConfigData Message:
    vehicleConfig = messaging2.VehicleConfigMsgPayload()
    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    vehicleConfig.ISCPntB_B = I
    vcInMsg = messaging2.VehicleConfigMsg().write(vehicleConfig)

    # wheelSpeeds Message
    rwSpeedMessage = messaging2.RWSpeedMsgPayload()
    Omega = [10.0, 25.0, 50.0, 100.0]  # rad/sec
    rwSpeedMessage.wheelSpeeds = Omega
    rwSpeedInMsg = messaging2.RWSpeedMsg().write(rwSpeedMessage)

    # wheelConfigData message
    jsList = []
    GsMatrix_B = []
    def writeMsgInWheelConfiguration():
        rwConfigParams = messaging2.RWArrayConfigMsgPayload()

        rwConfigParams.GsMatrix_B = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            0.577350269190, 0.577350269190, 0.577350269190
        ]
        rwConfigParams.JsList = [0.1, 0.1, 0.1, 0.1]
        rwConfigParams.numRW = rwNum
        msg = messaging2.RWArrayConfigMsg().write(rwConfigParams)
        return rwConfigParams.JsList, rwConfigParams.GsMatrix_B, msg

    if rwNum > 0:
        jsList, GsMatrix_B, rwParamInMsg = writeMsgInWheelConfiguration()

    # wheelAvailability message
    rwAvailabilityMessage = messaging2.RWAvailabilityMsgPayload()
    if useRwAvailability != "NO":
        if useRwAvailability == "ON":
            rwAvailabilityMessage.wheelAvailability = [messaging2.AVAILABLE, messaging2.AVAILABLE,
                                                       messaging2.AVAILABLE, messaging2.AVAILABLE]
        elif useRwAvailability == "OFF":
            rwAvailabilityMessage.wheelAvailability = [messaging2.UNAVAILABLE, messaging2.UNAVAILABLE,
                                                       messaging2.UNAVAILABLE, messaging2.UNAVAILABLE]
        else:
            print("WARNING: unknown rw availability status")
        rwAvailInMsg = messaging2.RWAvailabilityMsg().write(rwAvailabilityMessage)
    else:
        # set default availability
        rwAvailabilityMessage.wheelAvailability = [messaging2.AVAILABLE, messaging2.AVAILABLE,
                                                   messaging2.AVAILABLE, messaging2.AVAILABLE]

    LrTrue = findTrueTorques(moduleConfig, guidCmdData, rwSpeedMessage, vehicleConfig, jsList,
                             rwNum, GsMatrix_B, rwAvailabilityMessage)

    #   Setup logging on the test module output message so that we get all the writes to it
    dataLog = moduleConfig.cmdTorqueOutMsg.log()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # connect messages
    moduleConfig.guidInMsg.subscribeTo(guidInMsg)
    moduleConfig.vehConfigInMsg.subscribeTo(vcInMsg)
    if rwNum > 0:
        moduleConfig.rwParamsInMsg.subscribeTo(rwParamInMsg)
        moduleConfig.rwSpeedsInMsg.subscribeTo(rwSpeedInMsg)
    if useRwAvailability != "NO":
        moduleConfig.rwAvailInMsg.subscribeTo(rwAvailInMsg)

    #   Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    #   Step the simulation to 3*process rate so 4 total steps including zero
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    moduleWrap.Reset(1)     # this module reset function needs a time input (in NanoSeconds)

    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0))        # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # compare the module results to the truth values
    accuracy = 1e-8
    for i in range(0, len(LrTrue)):
        # check vector values
        if not unitTestSupport.isArrayEqual(dataLog.torqueRequestBody[i], LrTrue[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: " + moduleWrap.ModelTag + " Module failed mrpFeedback unit test at t="
                                + str(dataLog.times()[i]*macros.NANO2SEC) + "sec\n")

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + moduleWrap.ModelTag)
    else:
        print("Failed: " + moduleWrap.ModelTag)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


def findTrueTorques(moduleConfig,guidCmdData,rwSpeedMessage,vehicleConfigOut,jsList,numRW,GsMatrix_B,rwAvailMsg):
    Lr = []

    #Read in variables
    L = np.asarray(moduleConfig.knownTorquePntB_B)
    steps = [0, 0, .5, 0, .5]
    omega_BR_B = np.asarray(guidCmdData.omega_BR_B)
    omega_RN_B = np.asarray(guidCmdData.omega_RN_B)
    omega_BN_B = omega_BR_B + omega_RN_B #find body rate
    domega_RN_B = np.asarray(guidCmdData.domega_RN_B)
    sigma_BR = np.asarray(guidCmdData.sigma_BR)
    Isc = np.asarray(vehicleConfigOut.ISCPntB_B)
    Isc = np.reshape(Isc, (3, 3))
    Ki = moduleConfig.Ki
    K = moduleConfig.K
    P = moduleConfig.P
    jsVec = jsList
    GsMatrix_B_array = np.asarray(GsMatrix_B)
    GsMatrix_B_array = np.reshape(GsMatrix_B_array[0:numRW * 3], (numRW, 3))
    sigmaInt = np.asarray([0, 0, 0])

    #Compute toruqes
    for i in range(len(steps)):
        dt = steps[i]
        if dt == 0:
            sigmaInt = np.asarray([0, 0, 0])

        #evaluate integral term
        if Ki > 0: #if integral feedback is on
            sigmaInt = K * dt * sigma_BR + sigmaInt
            for n in range(3):
                if abs(sigmaInt[n]) > moduleConfig.integralLimit:
                    sigmaInt[n] *= moduleConfig.integralLimit/sigmaInt[n] #check elementwise if integral term is greater than limit; preserve direction (+/-)

            zVec = sigmaInt + Isc.dot(omega_BR_B)
        else: #integral gain turned off/negative setting
            zVec = np.asarray([0, 0, 0])

        #compute torque Lr
        Lr0 = K * sigma_BR  # +K*sigmaBR
        Lr1 = Lr0 + P * omega_BR_B  # +P*deltaOmega
        Lr2 = Lr1 + P * Ki * zVec  # +P*Ki*z
        GsHs = np.array([0,0,0])

        if numRW>0:
            for i in range(numRW):
                if rwAvailMsg.wheelAvailability[i] == 0:  #Make RW availability check
                    GsHs = GsHs + np.dot(GsMatrix_B_array[i, :], jsVec[i]*(np.dot(omega_BN_B, GsMatrix_B_array[i, :])+rwSpeedMessage.wheelSpeeds[i]))
                    #J_s*(dot(omegaBN_B,Gs_vec)+Omega_wheel)

        Lr3 = Lr2 - np.cross((omega_RN_B+Ki*zVec), (Isc.dot(omega_BN_B)+GsHs)) # -[v3Tilde(omega_r+Ki*z)]([I]omega + [Gs]h_s)

        Lr4 = Lr3 + Isc.dot(-domega_RN_B + np.cross(omega_BN_B, omega_RN_B)) #+[I](-d(omega_r)/dt + omega x omega_r)
        Lr5 = Lr4 + L
        Lr5 = -Lr5
        Lr.append(np.ndarray.tolist(Lr5))
    return Lr




#   This statement below ensures that the unitTestScript can be run as a stand-along python scripts
#   automatically executes the test_MRP_Feedback() method
#
if __name__ == "__main__":
    test_MRP_Feedback(False,    # showplots
                      0.01,     # intGain
                      0,        # rwNum
                      0.0,      # integralLimit
                      "NO"      # useRwAvailability ("NO", "ON", "OFF")
                      )
