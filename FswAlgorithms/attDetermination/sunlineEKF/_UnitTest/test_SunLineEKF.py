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

    filterObject.states = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    filterObject.covar = [0.4, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.4, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.4, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.04, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.04, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.04]
    R = np.identity(6)
    R[0:3, 0:3] = R[0:3, 0:3]*0.017*0.017
    R[3:6, 3:6] = R[3:6, 3:6]*0.0017*0.0017
    filterObject.measNoise = R

    Q = np.identity(3)
    Q[0:3, 0:3] = Q[0:3, 0:3]*0.017*0.017
    filterObject.procNoise = Q
    filterObject.qObsVal = 0.017*0.017

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def test_all_sunline_ekf(show_plots):
    [testResults, testMessage] = sunline_individual_test(show_plots)
    assert testResults < 1, testMessage
    # [testResults, testMessage] = testStatePropSunLine(show_plots)
    # assert testResults < 1, testMessage
    # [testResults, testMessage] = testStateUpdateSunLine(show_plots)
    # assert testResults < 1, testMessage

def sunline_individual_test(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    ## Testing dynamics matrix computation
    inputStates = [2,1,0.75,0.1,0.4,0.05]

    expDynMat = np.zeros([6,6])
    expDynMat[0:3, 0:3] = -(np.outer(inputStates[0:3],inputStates[3:6])/np.linalg.norm(inputStates[0:3])**2. +
                         np.dot(inputStates[3:6],inputStates[0:3])*(np.linalg.norm(inputStates[0:3])**2.*np.eye(3)- 2*np.outer(inputStates[0:3],inputStates[0:3]))/np.linalg.norm(inputStates[0:3])**4.)
    expDynMat[0:3, 3:6] = np.eye(3) - np.outer(inputStates[0:3],inputStates[0:3])/np.linalg.norm(inputStates[0:3])**2

    dynMat = sunlineEKF.new_doubleArray(6*6)
    for i in range(36):
        sunlineEKF.doubleArray_setitem(dynMat, i, 0)
    sunlineEKF.sunlineDynMatrix(inputStates, dynMat)

    DynOut = []
    for i in range(36):
        DynOut.append(sunlineEKF.doubleArray_getitem(dynMat, i))

    DynOut = np.array(DynOut).reshape(6, 6)
    errorNorm = np.linalg.norm(expDynMat - DynOut)
    if(errorNorm > 1.0E-12):
        print errorNorm
        testFailCount += 1
        testMessages.append("Dynamics Matrix generation Failure")


    ## STM and State Test
    inputStates = [2,1,0.75,0.1,0.4,0.05]
    dt =0.1
    stateTransition = sunlineEKF.new_doubleArray(6*6)
    states = sunlineEKF.new_doubleArray(6)
    for i in range(6):
        sunlineEKF.doubleArray_setitem(states, i, inputStates[i])
        for j in range(6):
            if i==j:
                sunlineEKF.doubleArray_setitem(stateTransition, 6*i+j, 1.0)
            else:
                sunlineEKF.doubleArray_setitem(stateTransition, 6*i+j, 0.0)

    sunlineEKF.sunlineStateSTMProp(expDynMat, 0.1, states, stateTransition)

    PropStateOut = []
    PropSTMOut = []
    for i in range(6):
        PropStateOut.append(sunlineEKF.doubleArray_getitem(states, i))
    for i in range(6*6):
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
        print STMout
        print expectedSTM
        testFailCount += 1
        testMessages.append("STM Propagation Failure")

    #print np.dot(np.array(inputStates[3:6]),np.array(inputStates[0:3]))/np.linalg.norm(inputStates[0:3])*np.array(inputStates[0:3])/np.linalg.norm(inputStates[0:3])

    if(errorNormStates > 1.0E-12):
        print errorNormStates
        testFailCount += 1
        testMessages.append("State Propagation Failure")

    numCSS = 8
    cssCos = [np.cos(np.deg2rad(10.)), np.cos(np.deg2rad(25.)), np.cos(np.deg2rad(5.)), np.cos(np.deg2rad(90.))]
    sensorTresh = np.cos(np.deg2rad(50.))
    cssNormals = [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.],[1./np.sqrt(2), 1./np.sqrt(2),0.]]

    measMat = sunlineEKF.new_doubleArray(8*6)
    obs = sunlineEKF.new_doubleArray(8)
    yMeas = sunlineEKF.new_doubleArray(8)
    numObs = sunlineEKF.new_intArray(1)

    expectedH = np.zeros([8,6])
    expectedY = np.zeros(8)
    for j in range(3):
        expectedH[0,0:3] = np.eye(3)[j,:]
        expectedY[j] =np.array(cssCos[j]) - np.dot(np.array(inputStates)[0:3], np.array(cssNormals)[j,:])
    expectedObs = np.array([np.cos(np.deg2rad(10.)), np.cos(np.deg2rad(25.)), np.cos(np.deg2rad(5.)),0.,0.,0.,0.,0.])
    expectedNumObs = 3.

    for i in range(8*6):
        sunlineEKF.doubleArray_setitem(measMat, i, 0.)
    for i in range(8):
        sunlineEKF.doubleArray_setitem(obs, i, 0.0)
        sunlineEKF.doubleArray_setitem(yMeas, i, 0.0)

    sunlineEKF.sunlineHMatrixYMeas(inputStates, numCSS, cssCos, sensorTresh, cssNormals, obs, measMat, yMeas, numObs)

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

    print np.shape(expectedH)

    HOut = np.array(HOut).reshape([8, 6])
    errorNorm = np.zeros(4)
    errorNorm[0] = np.linalg.norm(HOut - expectedH)
    errorNorm[1] = np.linalg.norm(yMeasOut - expectedY)
    errorNorm[2] = np.linalg.norm(obsOut - expectedObs)
    errorNorm[3] = np.linalg.norm(numObsOut[0] - 3)
    for i in range(4):
        if(errorNorm[i] > 1.0E-12):
            print i
            print errorNorm[i]
            print numObsOut[0]
            testFailCount += 1
            testMessages.append("H and yMeas update failure")


    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    if show_plots:
        plt.show()

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + " EKF utilities"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

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
    i=0
    #Initializing a 2D double array is hard with SWIG.  That's why there is this
    #layer between the above list and the actual C variables.
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
    moduleConfig.states = [0.7, 0.7, 0.0]
    unitTestSim.AddVariableForLogging('SunlineEKF.covar', testProcessRate*10, 0, 35, 'double')
    unitTestSim.AddVariableForLogging('SunlineEKF.states', testProcessRate*10, 0, 5, 'double')

    unitTestSim.InitializeSimulation()

    for i in range(20000):
        if i > 20:
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.cssDataInMsgName,
                                      inputMessageSize,
                                      unitTestSim.TotalSim.CurrentNanos,
                                      inputData)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i+1)*0.5))
        unitTestSim.ExecuteSimulation()

    covarLog = unitTestSim.GetLogVariableData('SunlineEKF.covar')
    stateLog = unitTestSim.GetLogVariableData('SunlineEKF.states')

    for i in range(6):
        if(covarLog[-1, i*6+1+i] > covarLog[0, i*6+1+i]/100):
            testFailCount += 1
            testMessages.append("Covariance update failure")
        if(abs(stateLog[-1, i+1] - stateTarget[i]) > 1.0E-10):
            print abs(stateLog[-1, i+1] - stateTarget[i])
            testFailCount += 1
            testMessages.append("State update failure")

    testVector = np.array([-0.8, -0.9, 0.0])
    inputData = cssComm.CSSArraySensorIntMsg()
    dotList = []
    for element in CSSOrientationList:
        dotProd = np.dot(np.array(element), testVector)
        dotList.append(dotProd)
    inputData.CosValue = dotList

    for i in range(20000):
        if i > 20:
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.cssDataInMsgName,
                                      inputMessageSize,
                                      unitTestSim.TotalSim.CurrentNanos,
                                      inputData)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i+20001)*0.5))
        unitTestSim.ExecuteSimulation()


    covarLog = unitTestSim.GetLogVariableData('SunlineEKF.covar')
    stateLog = unitTestSim.GetLogVariableData('SunlineEKF.states')
    stateTarget = testVector.tolist()
    stateTarget.extend([0.0, 0.0, 0.0])
    for i in range(6):
        if(covarLog[-1, i*6+1+i] > covarLog[0, i*6+1+i]/100):
            testFailCount += 1
            testMessages.append("Covariance update failure")
        if(abs(stateLog[-1, i+1] - stateTarget[i]) > 1.0E-10):
            print abs(stateLog[-1, i+1] - stateTarget[i])
            testFailCount += 1
            testMessages.append("State update failure")
    plt.figure()
    for i in range(moduleConfig.numStates):
        plt.plot(stateLog[:,0]*1.0E-9, stateLog[:,i+1])

    plt.figure()
    for i in range(moduleConfig.numStates):
        plt.plot(covarLog[:,0]*1.0E-9, covarLog[:,i*moduleConfig.numStates+i+1])

    if(show_plots):
        plt.show()
    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " state update"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]
def testStatePropSunLine(show_plots):

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
    unitTestSim.AddVariableForLogging('SunlineEKF.covar', testProcessRate*10, 0, 35)
    unitTestSim.AddVariableForLogging('SunlineEKF.states', testProcessRate*10, 0, 5)
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(8000.0))
    unitTestSim.ExecuteSimulation()

    covarLog = unitTestSim.GetLogVariableData('SunlineEKF.covar')
    stateLog = unitTestSim.GetLogVariableData('SunlineEKF.states')


    for i in range(6):
        if(abs(stateLog[-1, i+1] - stateLog[0, i+1]) > 1.0E-10):
            print abs(stateLog[-1, i+1] - stateLog[0, i+1])
            testFailCount += 1
            testMessages.append("State propagation failure")



    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " state propagation"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    test_all_sunline_ekf(False)
