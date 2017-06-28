''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
import math

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('FswAlgorithms')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

import SimulationBaseClass
import alg_contain
import SunLineEKF_test_utilities as FilterPlots
import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
import sunlineEKF  # import the module that is to be tested
import cssComm
import vehicleConfigData
import macros
import sim_model
import ctypes


def setupFilterData(filterObject):
    filterObject.navStateOutMsgName = "sunline_state_estimate"
    filterObject.filtDataOutMsgName = "sunline_filter_data"
    filterObject.cssDataInMsgName = "css_sensors_data"
    filterObject.massPropsInMsgName = "adcs_config_data"
    filterObject.cssConfInMsgName = "css_config_data"

    filterObject.sensorUseThresh = 0.
    filterObject.states = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0]
    filterObject.x = [1.0, 0.0, 1.0, 0.0, 0.1, 0.0]
    filterObject.covar = [1., 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 1., 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 1., 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.1]

    filterObject.qProcVal = 0.1**2
    filterObject.qObsVal = 0.17 ** 2
    filterObject.eKFSwitch = 20.

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def test_all_sunline_ekf(show_plots):
    [testResults, testMessage] = sunline_individual_test(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = testStatePropStatic(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = testStatePropVariable(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = testStateUpdateSunLine(show_plots)
    assert testResults < 1, testMessage

def sunline_individual_test(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    ###################################################################################
    ## Testing dynamics matrix computation
    ###################################################################################

    inputStates = [2,1,0.75,0.1,0.4,0.05]

    expDynMat = np.zeros([6,6])
    expDynMat[0:3, 0:3] = -(np.outer(inputStates[0:3],inputStates[3:6])/np.linalg.norm(inputStates[0:3])**2. +
                         np.dot(inputStates[3:6],inputStates[0:3])*(np.linalg.norm(inputStates[0:3])**2.*np.eye(3)- 2*np.outer(inputStates[0:3],inputStates[0:3]))/np.linalg.norm(inputStates[0:3])**4.)
    expDynMat[0:3, 3:6] = np.eye(3) - np.outer(inputStates[0:3],inputStates[0:3])/np.linalg.norm(inputStates[0:3])**2

    dynMat = sunlineEKF.new_doubleArray(6*6)
    for i in range(36):
        sunlineEKF.doubleArray_setitem(dynMat, i, 0.0)
    sunlineEKF.sunlineDynMatrix(inputStates, dynMat)

    DynOut = []
    for i in range(36):
        DynOut.append(sunlineEKF.doubleArray_getitem(dynMat, i))

    DynOut = np.array(DynOut).reshape(6, 6)
    errorNorm = np.linalg.norm(expDynMat - DynOut)
    if(errorNorm > 1.0E-12):
        print errorNorm
        testFailCount += 1
        testMessages.append("Dynamics Matrix generation Failure \n")

    ###################################################################################
    ## STM and State Test
    ###################################################################################

    inputStates = [2,1,0.75, 1.5, 0.5, 0.5]
    dt =0.5
    stateTransition = sunlineEKF.new_doubleArray(36)
    states = sunlineEKF.new_doubleArray(6)
    for i in range(6):
        sunlineEKF.doubleArray_setitem(states, i, inputStates[i])
        for j in range(6):
            if i==j:
                sunlineEKF.doubleArray_setitem(stateTransition, 6*i+j, 1.0)
            else:
                sunlineEKF.doubleArray_setitem(stateTransition, 6*i+j, 0.0)

    sunlineEKF.sunlineStateSTMProp(expDynMat.flatten().tolist(), dt, states, stateTransition)

    PropStateOut = []
    PropSTMOut = []
    for i in range(6):
        PropStateOut.append(sunlineEKF.doubleArray_getitem(states, i))
    for i in range(36):
        PropSTMOut.append(sunlineEKF.doubleArray_getitem(stateTransition, i))

    STMout = np.array(PropSTMOut).reshape([6,6])
    StatesOut = np.array(PropStateOut)

    expectedSTM = dt*np.dot(expDynMat, np.eye(6)) + np.eye(6)
    expectedStates = np.zeros(6)
    inputStatesArray = np.array(inputStates)
    expectedStates[3:6] = np.array(inputStatesArray[3:6])
    expectedStates[0:3] = np.array(inputStatesArray[0:3] + dt*(inputStatesArray[3:6] - np.dot(inputStatesArray[3:6], inputStatesArray[0:3])*inputStatesArray[0:3]/np.linalg.norm(inputStatesArray[0:3])**2.))
    errorNormSTM = np.linalg.norm(expectedSTM - STMout)
    errorNormStates = np.linalg.norm(expectedStates - StatesOut)

    if(errorNormSTM > 1.0E-12):
        print errorNormSTM
        testFailCount += 1
        testMessages.append("STM Propagation Failure \n")

    if(errorNormStates > 1.0E-12):
        print errorNormStates
        testFailCount += 1
        testMessages.append("State Propagation Failure \n")

    ###################################################################################
    ## Test the H and yMeas matrix generation as well as the observation count
    ###################################################################################

    numCSS = 4
    cssCos = [np.cos(np.deg2rad(10.)), np.cos(np.deg2rad(25.)), np.cos(np.deg2rad(5.)), np.cos(np.deg2rad(90.))]
    sensorTresh = np.cos(np.deg2rad(50.))
    cssNormals = [1.,0.,0.,0.,1.,0., 0.,0.,1., 1./np.sqrt(2), 1./np.sqrt(2),0.]

    measMat = sunlineEKF.new_doubleArray(8*6)
    obs = sunlineEKF.new_doubleArray(8)
    yMeas = sunlineEKF.new_doubleArray(8)
    numObs = sunlineEKF.new_intArray(1)

    for i in range(8*6):
        sunlineEKF.doubleArray_setitem(measMat, i, 0.)
    for i in range(8):
        sunlineEKF.doubleArray_setitem(obs, i, 0.0)
        sunlineEKF.doubleArray_setitem(yMeas, i, 0.0)

    sunlineEKF.sunlineHMatrixYMeas(inputStates, numCSS, cssCos, sensorTresh, cssNormals, obs, yMeas, numObs, measMat)

    obsOut = []
    yMeasOut = []
    numObsOut = []
    HOut = []
    for i in range(8*6):
        HOut.append(sunlineEKF.doubleArray_getitem(measMat, i))
    for i in range(8):
        yMeasOut.append(sunlineEKF.doubleArray_getitem(yMeas, i))
        obsOut.append(sunlineEKF.doubleArray_getitem(obs, i))
    numObsOut.append(sunlineEKF.intArray_getitem(numObs, 0))

    #Fill in expected values for test
    expectedH = np.zeros([8,6])
    expectedY = np.zeros(8)
    for j in range(3):
        expectedH[j,0:3] = np.eye(3)[j,:]
        expectedY[j] =np.array(cssCos[j]) - np.dot(np.array(inputStates)[0:3], np.array(cssNormals)[j*3:(j+1)*3])
    expectedObs = np.array([np.cos(np.deg2rad(10.)), np.cos(np.deg2rad(25.)), np.cos(np.deg2rad(5.)),0.,0.,0.,0.,0.])
    expectedNumObs = 3

    HOut = np.array(HOut).reshape([8, 6])
    errorNorm = np.zeros(4)
    errorNorm[0] = np.linalg.norm(HOut - expectedH)
    errorNorm[1] = np.linalg.norm(yMeasOut - expectedY)
    errorNorm[2] = np.linalg.norm(obsOut - expectedObs)
    errorNorm[3] = np.linalg.norm(numObsOut[0] - expectedNumObs)
    for i in range(4):
        if(errorNorm[i] > 1.0E-12):
            testFailCount += 1
            testMessages.append("H and yMeas update failure \n")

    ###################################################################################
    ## Test the Kalman Gain
    ###################################################################################

    numObs = 3
    h = [1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
         0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
    covar = [1., 0., 0., 1., 0., 0.,
        0., 1., 0., 0., 1., 0.,
        0., 0., 1., 0., 0., 1.,
        1., 0., 0., 1., 0., 0.,
        0., 1., 0., 0., 1., 0.,
        0., 0., 1., 0., 0., 1.]
    noise= 0.01

    Kalman = sunlineEKF.new_doubleArray(6 * 8)

    for i in range(8 * 6):
        sunlineEKF.doubleArray_setitem(Kalman, i, 0.)

    sunlineEKF.sunlineKalmanGain(covar, h, noise, numObs, Kalman)

    KalmanOut = []
    for i in range(8 * 6):
        KalmanOut.append(sunlineEKF.doubleArray_getitem(Kalman, i))

    # Fill in expected values for test
    Hmat = np.array(h).reshape([8,6])
    Pk = np.array(covar).reshape([6,6])
    R = noise*np.eye(3)
    expectedK = np.dot(np.dot(Pk, Hmat[0:numObs,:].T), np.linalg.inv(np.dot(np.dot(Hmat[0:numObs,:], Pk), Hmat[0:numObs,:].T) + R[0:numObs,0:numObs]))

    KalmanOut = np.array(KalmanOut)[0:6*numObs].reshape([6, 3])
    errorNorm = np.linalg.norm(KalmanOut[:,0:numObs] - expectedK)


    if (errorNorm > 1.0E-12):
        print errorNorm
        testFailCount += 1
        testMessages.append("Kalman Gain update failure \n")

    ###################################################################################
    ## Test the EKF update
    ###################################################################################

    KGain = [1.,2.,3., 0., 1., 2., 1., 0., 1., 0., 1., 0., 3., 0., 1., 0., 2., 0.]
    for i in range(6*8-6*3):
        KGain.append(0.)
    inputStates = [2,1,0.75,0.1,0.4,0.05]
    xbar = [0.1, 0.2, 0.01, 0.005, 0.009, 0.001]
    numObs = 3
    h = [1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
         0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
    covar = [1., 0., 0., 1., 0., 0.,
             0., 1., 0., 0., 1., 0.,
             0., 0., 1., 0., 0., 1.,
             1., 0., 0., 1., 0., 0.,
             0., 1., 0., 0., 1., 0.,
             0., 0., 1., 0., 0., 1.]
    noise = 0.01
    inputY = np.zeros(3)
    for j in range(3):
        inputY[j] = np.array(cssCos[j]) - np.dot(np.array(inputStates)[0:3], np.array(cssNormals)[j * 3:(j + 1) * 3])
    inputY = inputY.tolist()

    stateError = sunlineEKF.new_doubleArray(6)
    covarMat = sunlineEKF.new_doubleArray(6*6)
    inputs = sunlineEKF.new_doubleArray(6)


    for i in range(6):
        sunlineEKF.doubleArray_setitem(stateError, i, 0.)
        sunlineEKF.doubleArray_setitem(inputs, i, inputStates[i])
        for j in range(6):
            sunlineEKF.doubleArray_setitem(covarMat,i+j,0.)

    sunlineEKF.sunlineEKFUpdate(KGain, covar, noise, numObs, inputY, h, inputs, stateError, covarMat)

    stateOut = []
    covarOut = []
    errorOut = []
    for i in range(6):
        stateOut.append(sunlineEKF.doubleArray_getitem(inputs, i))
        errorOut.append(sunlineEKF.doubleArray_getitem(stateError, i))
    for j in range(36):
        covarOut.append(sunlineEKF.doubleArray_getitem(covarMat, j))

    # Fill in expected values for test
    KK = np.array(KGain)[0:6*3].reshape([6,3])
    expectedStates = np.array(inputStates) + np.dot(KK, np.array(inputY))
    H = np.array(h).reshape([8,6])[0:3,:]
    Pk = np.array(covar).reshape([6, 6])
    R = noise * np.eye(3)
    expectedP = np.dot(np.dot(np.eye(6) - np.dot(KK, H), Pk), np.transpose(np.eye(6) - np.dot(KK, H))) + np.dot(KK, np.dot(R,KK.T))

    errorNorm = np.zeros(2)
    errorNorm[0] = np.linalg.norm(np.array(stateOut) - expectedStates)
    errorNorm[1] = np.linalg.norm(expectedP - np.array(covarOut).reshape([6,6]))
    for i in range(2):
        if(errorNorm[i] > 1.0E-12):
            testFailCount += 1
            testMessages.append("EKF update failure \n")

    ###################################################################################
    ## Test the CKF update
    ###################################################################################

    KGain = [1., 2., 3., 0., 1., 2., 1., 0., 1., 0., 1., 0., 3., 0., 1., 0., 2., 0.]
    for i in range(6 * 8 - 6 * 3):
        KGain.append(0.)
    inputStates = [2, 1, 0.75, 0.1, 0.4, 0.05]
    xbar = [0.1, 0.2, 0.01, 0.005, 0.009, 0.001]
    numObs = 3
    h = [1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
         0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
    covar = [1., 0., 0., 1., 0., 0.,
             0., 1., 0., 0., 1., 0.,
             0., 0., 1., 0., 0., 1.,
             1., 0., 0., 1., 0., 0.,
             0., 1., 0., 0., 1., 0.,
             0., 0., 1., 0., 0., 1.]
    noise =0.01
    inputY = np.zeros(3)
    for j in range(3):
        inputY[j] = np.array(cssCos[j]) - np.dot(np.array(inputStates)[0:3],
                                                 np.array(cssNormals)[j * 3:(j + 1) * 3])
    inputY = inputY.tolist()

    stateError = sunlineEKF.new_doubleArray(6)
    covarMat = sunlineEKF.new_doubleArray(6 * 6)

    for i in range(6):
        sunlineEKF.doubleArray_setitem(stateError, i, xbar[i])
        for j in range(6):
            sunlineEKF.doubleArray_setitem(covarMat, i + j, 0.)

    sunlineEKF.sunlineCKFUpdate(xbar, KGain, covar, noise, numObs, inputY, h, stateError, covarMat)

    covarOut = []
    errorOut = []
    for i in range(6):
        errorOut.append(sunlineEKF.doubleArray_getitem(stateError, i))
    for j in range(36):
        covarOut.append(sunlineEKF.doubleArray_getitem(covarMat, j))

    # Fill in expected values for test
    KK = np.array(KGain)[0:6 * 3].reshape([6, 3])
    H = np.array(h).reshape([8, 6])[0:3, :]
    expectedStateError = np.array(xbar) + np.dot(KK, (np.array(inputY) - np.dot(H, np.array(xbar))))
    Pk = np.array(covar).reshape([6, 6])
    expectedP = np.dot(np.dot(np.eye(6) - np.dot(KK, H), Pk), np.transpose(np.eye(6) - np.dot(KK, H))) + np.dot(KK,
                                                                                                                np.dot(
                                                                                                                    R,
                                                                                                                    KK.T))

    errorNorm = np.zeros(2)
    errorNorm[0] = np.linalg.norm(np.array(errorOut) - expectedStateError)
    errorNorm[1] = np.linalg.norm(expectedP - np.array(covarOut).reshape([6, 6]))
    for i in range(2):
        if (errorNorm[i] > 1.0E-12):
            testFailCount += 1
            testMessages.append("CKF update failure \n")

    ###################################################################################
    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    if show_plots:
        plt.show()

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + " EKF individual tests"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

####################################################################################
# Test for the time and update with static states (zero d_dot)
####################################################################################
def testStatePropStatic(show_plots):
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
    moduleConfig = sunlineEKF.sunlineEKFConfig()
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        sunlineEKF.Update_sunlineEKF,
                                        sunlineEKF.SelfInit_sunlineEKF,
                                        sunlineEKF.CrossInit_sunlineEKF,
                                        sunlineEKF.Reset_sunlineEKF)
    moduleWrap.ModelTag = "SunlineEKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    setupFilterData(moduleConfig)
    unitTestSim.AddVariableForLogging('SunlineEKF.covar', testProcessRate * 10, 0, 35)
    unitTestSim.AddVariableForLogging('SunlineEKF.states', testProcessRate * 10, 0, 5)
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(8000.0))
    unitTestSim.ExecuteSimulation()

    covarLog = unitTestSim.GetLogVariableData('SunlineEKF.covar')
    stateLog = unitTestSim.GetLogVariableData('SunlineEKF.states')


    for i in range(6):
        if (abs(stateLog[-1, i + 1] - stateLog[0, i + 1]) > 1.0E-10):
            print abs(stateLog[-1, i + 1] - stateLog[0, i + 1])
            testFailCount += 1
            testMessages.append("State propagation failure \n")

    unitTestSim.terminateSimulation()

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " static state propagation"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


####################################################################################
# Test for the time and update with changing states (non-zero d_dot)
####################################################################################
def testStatePropVariable(show_plots):
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
    moduleConfig = sunlineEKF.sunlineEKFConfig()
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        sunlineEKF.Update_sunlineEKF,
                                        sunlineEKF.SelfInit_sunlineEKF,
                                        sunlineEKF.CrossInit_sunlineEKF,
                                        sunlineEKF.Reset_sunlineEKF)
    moduleWrap.ModelTag = "SunlineEKF"



    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    setupFilterData(moduleConfig)

    InitialState = moduleConfig.states
    Initialx = moduleConfig.x
    InitialCovar = moduleConfig.covar

    moduleConfig.states = InitialState
    unitTestSim.AddVariableForLogging('SunlineEKF.covar', testProcessRate, 0, 35)
    unitTestSim.AddVariableForLogging('SunlineEKF.stateTransition', testProcessRate, 0, 35)
    unitTestSim.AddVariableForLogging('SunlineEKF.states', testProcessRate , 0, 5)
    unitTestSim.AddVariableForLogging('SunlineEKF.x', testProcessRate , 0, 5)
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1000.0))
    unitTestSim.ExecuteSimulation()


    covarLog = unitTestSim.GetLogVariableData('SunlineEKF.covar')
    stateLog = unitTestSim.GetLogVariableData('SunlineEKF.states')
    stateErrorLog = unitTestSim.GetLogVariableData('SunlineEKF.x')
    stmLog = unitTestSim.GetLogVariableData('SunlineEKF.stateTransition')


    dt = 0.5
    expectedStateArray = np.zeros([2001,7])
    expectedStateArray[0,1:7] = np.array(InitialState)

    for i in range(1,2001):
        expectedStateArray[i,0] = dt*i*1E9
        expectedStateArray[i,1:4] = expectedStateArray[i-1,1:4] + dt*(expectedStateArray[i-1,4:7] - (np.dot(expectedStateArray[i-1,4:7],expectedStateArray[i-1,1:4]))*expectedStateArray[i-1,1:4]/np.linalg.norm(expectedStateArray[i-1,1:4])**2.)
        expectedStateArray[i, 4:7] = expectedStateArray[i-1,4:7]

    expDynMat = np.zeros([2001,6,6])
    for i in range(0,2001):
        expDynMat[i, 0:3, 0:3] = -(np.outer(expectedStateArray[i,1:4],expectedStateArray[i,4:7])/np.linalg.norm(expectedStateArray[i,1:4])**2. +
                             np.dot(expectedStateArray[i,4:7], expectedStateArray[i,1:4])*(np.linalg.norm(expectedStateArray[i,1:4])**2.*np.eye(3)- 2*np.outer(expectedStateArray[i,1:4],expectedStateArray[i,1:4]))/np.linalg.norm(expectedStateArray[i,1:4])**4.)
        expDynMat[i, 0:3, 3:6] = np.eye(3) - np.outer(expectedStateArray[i,1:4],expectedStateArray[i,1:4])/np.linalg.norm(expectedStateArray[i,1:4])**2


    expectedSTM = np.zeros([2001,6,6])
    expectedSTM[0,:,:] = np.eye(6)
    for i in range(1,2001):
        expectedSTM[i,:,:] = dt * np.dot(expDynMat[i-1,:,:], np.eye(6)) + np.eye(6)

    expectedXBar = np.zeros([2001,7])
    expectedXBar[0,1:7] = np.array(Initialx)
    for i in range(1,2001):
        expectedXBar[i,0] = dt*i*1E9
        expectedXBar[i, 1:7] = np.dot(expectedSTM[i, :, :], expectedXBar[i - 1, 1:7])

    expectedCovar = np.zeros([2001,37])
    expectedCovar[0,1:37] = np.array(InitialCovar)
    Gamma = np.zeros([6, 3])
    Gamma[0:3, 0:3] = dt ** 2. / 2. * np.eye(3)
    Gamma[3:6, 0:3] = dt * np.eye(3)
    ProcNoiseCovar = np.dot(Gamma, np.dot(moduleConfig.qProcVal*np.eye(3),Gamma.T))
    for i in range(1,2001):
        expectedCovar[i,0] =  dt*i*1E9
        expectedCovar[i,1:37] = (np.dot(expectedSTM[i,:,:], np.dot(np.reshape(expectedCovar[i-1,1:37],[6,6]), np.transpose(expectedSTM[i,:,:])))+ ProcNoiseCovar).flatten()

    if show_plots:
        FilterPlots.StatesVsExpected(stateLog, expectedStateArray)
        FilterPlots.StatesPlotCompare(stateErrorLog, expectedXBar, covarLog, expectedCovar)

    for j in range(1,2001):
        for i in range(6):
            if (abs(stateLog[j, i + 1] - expectedStateArray[j, i + 1]) > 1.0E-10):
                testFailCount += 1
                testMessages.append("General state propagation failure: State Prop \n")
            if (abs(stateErrorLog[j, i + 1] - expectedXBar[j, i + 1]) > 1.0E-10):
                testFailCount += 1
                testMessages.append("General state propagation failure: State Error Prop \n")

        for i in range(36):
            if (abs(covarLog[j, i + 1] - expectedCovar[j, i + 1]) > 1.0E-8):
                print abs(covarLog[j, i + 1] - expectedCovar[j, i + 1])
                testFailCount += 1
                testMessages.append("General state propagation failure: Covariance Prop \n")
            if (abs(stmLog[j, i + 1] - expectedSTM[j,:].flatten()[i]) > 1.0E-10):
                print abs(stateLog[j, i + 1] - expectedStateArray[j, i + 1])
                testFailCount += 1
                testMessages.append("General state propagation failure: STM Prop \n")

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " general state propagation"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


####################################################################################
# Test for the full filter with time and measurement update
####################################################################################
def testStateUpdateSunLine(show_plots):
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
    moduleConfig = sunlineEKF.sunlineEKFConfig()
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        sunlineEKF.Update_sunlineEKF,
                                        sunlineEKF.SelfInit_sunlineEKF,
                                        sunlineEKF.CrossInit_sunlineEKF,
                                        sunlineEKF.Reset_sunlineEKF)
    moduleWrap.ModelTag = "SunlineEKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    setupFilterData(moduleConfig)

    cssConstelation = vehicleConfigData.CSSConstConfig()

    CSSOrientationList = [
        [0.70710678118654746, -0.5, 0.5],
        [0.70710678118654746, -0.5, -0.5],
        [0.70710678118654746, 0.5, -0.5],
        [0.70710678118654746, 0.5, 0.5],
        [-0.70710678118654746, 0, 0.70710678118654757],
        [-0.70710678118654746, 0.70710678118654757, 0.0],
        [-0.70710678118654746, 0, -0.70710678118654757],
        [-0.70710678118654746, -0.70710678118654757, 0.0],
    ]
    totalCSSList = []
    i = 0
    # Initializing a 2D double array is hard with SWIG.  That's why there is this
    # layer between the above list and the actual C variables.
    for CSSHat in CSSOrientationList:
        newCSS = vehicleConfigData.CSSConfigurationElement()
        newCSS.nHat_S = CSSHat
        totalCSSList.append(newCSS)
    cssConstelation.nCSS = len(CSSOrientationList)
    cssConstelation.cssVals = totalCSSList
    msgSize = cssConstelation.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage("TestProcess", "css_config_data",
                                          msgSize, 2, "CSSConstellation")
    unitTestSim.TotalSim.WriteMessageData("css_config_data", msgSize, 0, cssConstelation)

    vehicleConfigOut = sunlineEKF.VehicleConfigFswMsg()
    inputMessageSize = vehicleConfigOut.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.massPropsInMsgName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    vehicleConfigOut.ISCPntB_B = I
    BS = [1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0]
    vehicleConfigOut.dcm_BS = BS
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.massPropsInMsgName,
                                          inputMessageSize,
                                          0,
                                          vehicleConfigOut)

    testVector = np.array([-0.7, 0.7, 0.0])
    inputData = cssComm.CSSArraySensorIntMsg()
    dotList = []
    for element in CSSOrientationList:
        dotProd = np.dot(np.array(element), testVector)
        dotList.append(dotProd)
    inputData.CosValue = dotList
    inputMessageSize = inputData.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.cssDataInMsgName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)

    stateTarget = testVector.tolist()
    stateTarget.extend([0.0, 0.0, 0.0])
    moduleConfig.states = [0.7, 0.7, 0.0, 0.0, 0.0, 0.0]
    moduleConfig.x = (stateTarget - np.array([0.7, 0.7, 0.0, 0.0, 0.0, 0.0])).tolist()
    # print (stateTarget - np.array([0.7, 0.7, 0.0, 0.0, 0.0, 0.0])).tolist()
    unitTestSim.AddVariableForLogging('SunlineEKF.covar', testProcessRate , 0, 35, 'double')
    unitTestSim.AddVariableForLogging('SunlineEKF.states', testProcessRate , 0, 5, 'double')
    unitTestSim.AddVariableForLogging('SunlineEKF.x', testProcessRate , 0, 5, 'double')

    unitTestSim.InitializeSimulation()

    for i in range(200):
        if i > 20:
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.cssDataInMsgName,
                                                  inputMessageSize,
                                                  unitTestSim.TotalSim.CurrentNanos,
                                                  inputData)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i + 1) * 0.5))
        unitTestSim.ExecuteSimulation()

    covarLog = unitTestSim.GetLogVariableData('SunlineEKF.covar')
    stateLog = unitTestSim.GetLogVariableData('SunlineEKF.states')
    stateErrorLog = unitTestSim.GetLogVariableData('SunlineEKF.x')

    for i in range(6):
        if (covarLog[-1, i * 6 + 1 + i] > covarLog[0, i * 6 + 1 + i] / 100):
            testFailCount += 1
            testMessages.append("Covariance update failure")
        if (abs(stateErrorLog[-1, i + 1] - stateTarget[i]) > 1.0E-10):
            print abs(stateLog[-1, i + 1] - stateTarget[i])
            testFailCount += 1
            testMessages.append("State update failure")

    testVector = np.array([-0.8, -0.9, 0.0])
    inputData = cssComm.CSSArraySensorIntMsg()
    dotList = []
    for element in CSSOrientationList:
        dotProd = np.dot(np.array(element), testVector)
        dotList.append(dotProd)
    inputData.CosValue = dotList

    for i in range(200):
        if i > 20:
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.cssDataInMsgName,
                                                  inputMessageSize,
                                                  unitTestSim.TotalSim.CurrentNanos,
                                                  inputData)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i + 201) * 0.5))
        unitTestSim.ExecuteSimulation()

    covarLog = unitTestSim.GetLogVariableData('SunlineEKF.covar')
    stateLog = unitTestSim.GetLogVariableData('SunlineEKF.states')
    stateErrorLog = unitTestSim.GetLogVariableData('SunlineEKF.x')
    stateTarget = testVector.tolist()
    stateTarget.extend([0.0, 0.0, 0.0])


    measMat = sunlineEKF.new_doubleArray(8*6)
    obs = sunlineEKF.new_doubleArray(8)
    yMeas = sunlineEKF.new_doubleArray(8)
    numObs = sunlineEKF.new_intArray(1)

    ####################################################################################
    # Compute H and y in order to check post-fit residuals
    ####################################################################################
    threshold = moduleConfig.sensorUseThresh
    CSSnormals = []
    for j in range(8):
        CSSnormals+=CSSOrientationList[j]

    ytest = np.zeros([len(stateLog[:, 0]), 9])
    Htest = np.zeros([len(stateLog[:, 0]), 49])
    PostFitRes = np.zeros([len(stateLog[:,0]), 9])

    for i in range(1,len(stateLog[:,0])):
        ytest[i,0] = stateLog[i,0]
        Htest[i,0] = stateLog[i,0]
        PostFitRes[i, 0] = stateLog[i, 0]

        sunlineEKF.sunlineHMatrixYMeas(stateLog[i-1,1:7].tolist(), 8, dotList, threshold, CSSnormals, obs, yMeas, numObs, measMat)
        obsOut = []
        yMeasOut = []
        numObsOut = []
        HOut = []
        for i in range(8*6):
            HOut.append(sunlineEKF.doubleArray_getitem(measMat, i))
        for i in range(8):
            yMeasOut.append(sunlineEKF.doubleArray_getitem(yMeas, i))

        ytest[i,1:9] = np.array(yMeasOut)
        Htest[i,1:49] = np.array(HOut)
        PostFitRes[i,1:9] = ytest[i,1:9] - np.dot( Htest[i,1:49].reshape([8,6]), stateErrorLog[i,1:7])

    for i in range(6):
        if (covarLog[-1, i * 6 + 1 + i] > covarLog[0, i * 6 + 1 + i] /100.):
            testFailCount += 1
            testMessages.append("Covariance update failure")
        if (abs(stateLog[-1, i + 1] - stateTarget[i]) > 1.0E-10):
            print abs(stateLog[-1, i + 1] - stateTarget[i])
            print i
            testFailCount += 1
            testMessages.append("State update failure")

    target1 = np.array([-0.7, 0.7, 0.0, 0., 0., 0.])
    target2 = np.array([-0.8, -0.9, 0.0, 0., 0., 0.])

    # show_plots =True
    if show_plots:
        FilterPlots.PostFitResiduals(PostFitRes, moduleConfig.qObsVal)
        FilterPlots.StatesVsTargets(target1, target2, stateLog)
        FilterPlots.StatesPlot(stateErrorLog, covarLog)

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " state update"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]



if __name__ == "__main__":
    test_all_sunline_ekf(False)

