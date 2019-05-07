''' '''
'''
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
import numpy
import pytest
import math

from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport
from Basilisk.simulation import coarse_sun_sensor
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import sunlineSuKF, cssComm, fswMessages  # import the module that is to be tested

import SunLineSuKF_test_utilities as FilterPlots

def setupFilterData(filterObject, initialized):
    filterObject.navStateOutMsgName = "sunline_state_estimate"
    filterObject.filtDataOutMsgName = "sunline_filter_data"
    filterObject.cssDataInMsgName = "css_sensors_data"
    filterObject.cssConfigInMsgName = "css_config_data"

    filterObject.alpha = 0.02
    filterObject.beta = 2.0
    filterObject.kappa = 0.0

    if initialized:
        filterObject.stateInit = [0.0, 0.0, 1.0, 0.0, 0.0, 1.]
        filterObject.filterInitialized = 1
    else:
        filterObject.filterInitialized = 0

    filterObject.covarInit = [1., 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 1., 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 1., 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.02, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.02, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 1E-4]
    qNoiseIn = numpy.identity(6)
    qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3]*0.001*0.001
    qNoiseIn[3:5, 3:5] = qNoiseIn[3:5, 3:5]*0.001*0.001
    qNoiseIn[5, 5] = qNoiseIn[5, 5]*0.0000002*0.0000002
    filterObject.qNoise = qNoiseIn.reshape(36).tolist()
    filterObject.qObsVal = 0.002
    filterObject.sensorUseThresh = 0.0


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
@pytest.mark.parametrize("kellyOn", [
    (False),
    (True)
])


def test_all_sunline_kf(show_plots, kellyOn):
    [testResults, testMessage] = StatePropSunLine(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = StateUpdateSunLine(show_plots, kellyOn)
    assert testResults < 1, testMessage
    [testResults, testMessage] = test_FaultScenarios()
    assert testResults < 1, testMessage


def StateUpdateSunLine(show_plots, kellyOn):
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
    moduleConfig = sunlineSuKF.SunlineSuKFConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "sunlineSuKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)
    
    setupFilterData(moduleConfig, False)
    cssConstelation = fswMessages.CSSConfigFswMsg()
    
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
    for CSSHat in CSSOrientationList:
        newCSS = fswMessages.CSSUnitConfigFswMsg()
        newCSS.CBias = 1.0
        newCSS.nHat_B = CSSHat
        totalCSSList.append(newCSS)
    cssConstelation.nCSS = len(CSSOrientationList)
    cssConstelation.cssVals = totalCSSList
    unitTestSupport.setMessage(unitTestSim.TotalSim,
                               unitProcessName,
                               moduleConfig.cssConfigInMsgName,
                               cssConstelation)
    unitTestSim.TotalSim.logThisMessage('sunline_filter_data', testProcessRate)

    # Add the kelly curve coefficients
    if kellyOn:
        kellList = []
        for j in range(len(CSSOrientationList)):
            kellyData = sunlineSuKF.SunlineSuKFCFit()
            kellyData.cssKellFact = 0.05
            kellyData.cssKellPow = 2.
            kellyData.cssRelScale = 1.
            kellList.append(kellyData)
        moduleConfig.kellFits = kellList

    testVector = numpy.array([-0.7, 0.7, 0.0])
    testVector/=numpy.linalg.norm(testVector)
    inputData = cssComm.CSSArraySensorIntMsg()
    dotList = []
    for element in CSSOrientationList:
        dotProd = numpy.dot(numpy.array(element), testVector)/(numpy.linalg.norm(element)*numpy.linalg.norm(testVector))
        dotList.append(dotProd)

    inputData.CosValue = dotList
    inputMessageSize = inputData.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                      moduleConfig.cssDataInMsgName,
                                      inputMessageSize,
                                      2)  # number of buffers (leave at 2 as default, don't make zero)

    stateTarget = testVector.tolist()
    stateTarget.extend([0.0, 0.0, 1.])
    # moduleConfig.stateInit = [0.7, 0.7, 0.0, 0.01, 0.001, 1.]

    numStates = len(moduleConfig.stateInit)
    unitTestSim.InitializeSimulation()
    if kellyOn:
        time = 1000
    else:
        time =  500
    for i in range(time):
        unitTestSim.TotalSim.WriteMessageData(moduleConfig.cssDataInMsgName,
                                  inputMessageSize,
                                  unitTestSim.TotalSim.CurrentNanos,
                                  inputData)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i+1)*0.5))
        unitTestSim.ExecuteSimulation()

    stateLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".state", range(numStates))
    postFitLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".postFitRes", range(8))
    covarLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".covar", range(numStates*numStates))

    accuracy = 1.0E-3
    if kellyOn:
        accuracy = 1.0E-2 # 1% Error test for the kelly curves given errors
    for i in range(numStates):
        if(covarLog[-1, i*numStates+1+i] > covarLog[0, i*numStates+1+i]):
            testFailCount += 1
            testMessages.append("Covariance update failure first part")
    if(numpy.arccos(numpy.dot(stateLog[-1, 1:4], stateTarget[0:3])/(numpy.linalg.norm(stateLog[-1, 1:4])*numpy.linalg.norm(stateTarget[0:3]))) > accuracy):
        print numpy.arccos(numpy.dot(stateLog[-1, 1:4], stateTarget[0:3])/(numpy.linalg.norm(stateLog[-1, 1:4])*numpy.linalg.norm(stateTarget[0:3])))
        testFailCount += 1
        testMessages.append("Pointing update failure")
    if(numpy.linalg.norm(stateLog[-1, 4:7] - stateTarget[3:6]) > accuracy):
        print numpy.linalg.norm(stateLog[-1,  4:7] - stateTarget[3:6])
        testFailCount += 1
        testMessages.append("Rate update failure")
    if(abs(stateLog[-1, 6] - stateTarget[5]) > accuracy):
        print abs(stateLog[-1, 6] - stateTarget[5])
        testFailCount += 1
        testMessages.append("Sun Intensity update failure")

    testVector = numpy.array([-0.7, 0.75, 0.0])
    testVector /= numpy.linalg.norm(testVector)
    inputData = cssComm.CSSArraySensorIntMsg()
    dotList = []
    for element in CSSOrientationList:
        dotProd = numpy.dot(numpy.array(element), testVector)
        dotList.append(dotProd)
    inputData.CosValue = dotList
        
    for i in range(time):
        if i > 20:
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.cssDataInMsgName,
                                      inputMessageSize,
                                      unitTestSim.TotalSim.CurrentNanos,
                                      inputData)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i+time+1)*0.5))
        unitTestSim.ExecuteSimulation()

    stateLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".state", range(numStates))
    postFitLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".postFitRes", range(8))
    covarLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".covar", range(numStates*numStates))

    stateTarget = testVector.tolist()
    stateTarget.extend([0.0, 0.0, 1.0])

    for i in range(numStates):
        if(covarLog[-1, i*numStates+1+i] > covarLog[0, i*numStates+1+i]):
            print covarLog[-1, i*numStates+1+i] - covarLog[0, i*numStates+1+i]
            testFailCount += 1
            testMessages.append("Covariance update failure")
    if(numpy.arccos(numpy.dot(stateLog[-1, 1:4], stateTarget[0:3])/(numpy.linalg.norm(stateLog[-1, 1:4])*numpy.linalg.norm(stateTarget[0:3]))) > accuracy):
        print numpy.arccos(numpy.dot(stateLog[-1, 1:4], stateTarget[0:3])/(numpy.linalg.norm(stateLog[-1, 1:4])*numpy.linalg.norm(stateTarget[0:3])))
        testFailCount += 1
        testMessages.append("Pointing update failure")
    if(numpy.linalg.norm(stateLog[-1, 4:7] - stateTarget[3:6]) > accuracy):
        print numpy.linalg.norm(stateLog[-1,  4:7] - stateTarget[3:6])
        testFailCount += 1
        testMessages.append("Rate update failure")
    if(abs(stateLog[-1, 6] - stateTarget[5]) > accuracy):
        print abs(stateLog[-1, 6] - stateTarget[5])
        testFailCount += 1
        testMessages.append("Sun Intensity update failure")

    FilterPlots.StateCovarPlot(stateLog, covarLog, show_plots)
    FilterPlots.PostFitResiduals(postFitLog, moduleConfig.qObsVal, show_plots)

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " state update"
    else:
        print testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def StatePropSunLine(show_plots):
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
    moduleConfig = sunlineSuKF.SunlineSuKFConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "sunlineSuKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)
    
    setupFilterData(moduleConfig, True)
    numStates = 6
    unitTestSim.TotalSim.logThisMessage('sunline_filter_data', testProcessRate)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(8000.0))
    unitTestSim.ExecuteSimulation()
    
    stateLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".state", range(numStates))
    postFitLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".postFitRes", range(8))
    covarLog = unitTestSim.pullMessageLogData('sunline_filter_data' + ".covar", range(numStates*numStates))

    FilterPlots.StateCovarPlot(stateLog, covarLog, show_plots)
    FilterPlots.PostFitResiduals(postFitLog, moduleConfig.qObsVal, show_plots)

    for i in range(numStates):
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

def test_FaultScenarios():
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

    # Clean methods for Measurement and Time Updates
    unitTestSim.TotalSim.terminateSimulation()
    moduleConfigClean1 = sunlineSuKF.SunlineSuKFConfig()
    moduleConfigClean1.numStates = 6
    moduleConfigClean1.state = [0., 0., 0., 0., 0., 0.]
    moduleConfigClean1.statePrev = [0., 0., 0., 0., 0., 0.]
    moduleConfigClean1.sBar = [0., 0., 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.,
                               0., 0., 0., 0., 0., 0.]
    moduleConfigClean1.sBarPrev = [1., 0., 0., 0., 0., 0.,
                                   0., 1., 0., 0., 0., 0.,
                                   0., 0., 1., 0., 0., 0.,
                                   0., 0., 0., 1., 0., 0.,
                                   0., 0., 0., 0., 1., 0.,
                                   0., 0., 0., 0., 0., 1.]
    moduleConfigClean1.covar = [0., 0., 0., 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.,
                                0., 0., 0., 0., 0., 0.]
    moduleConfigClean1.covarPrev = [2., 0., 0., 0., 0., 0.,
                                    0., 2., 0., 0., 0., 0.,
                                    0., 0., 2., 0., 0., 0.,
                                    0., 0., 0., 2., 0., 0.,
                                    0., 0., 0., 0., 2., 0.,
                                    0., 0., 0., 0., 0., 2.]

    sunlineSuKF.sunlineSuKFCleanUpdate(moduleConfigClean1)

    if numpy.linalg.norm(numpy.array(moduleConfigClean1.covarPrev) - numpy.array(moduleConfigClean1.covar)) > 1E10:
        testFailCount += 1
        testMessages.append("sunlineSuKFClean Covar failed")
    if numpy.linalg.norm(numpy.array(moduleConfigClean1.statePrev) - numpy.array(moduleConfigClean1.state)) > 1E10:
        testFailCount += 1
        testMessages.append("sunlineSuKFClean States failed")
    if numpy.linalg.norm(numpy.array(moduleConfigClean1.sBar) - numpy.array(moduleConfigClean1.sBarPrev)) > 1E10:
        testFailCount += 1
        testMessages.append("sunlineSuKFClean sBar failed")

    moduleConfigClean1.navStateOutMsgName = "sunline_state_estimate"
    moduleConfigClean1.filtDataOutMsgName = "sunline_filter_data"
    moduleConfigClean1.cssDataInMsgName = "css_sensors_data"
    moduleConfigClean1.cssConfigInMsgName = "css_config_data"

    moduleConfigClean1.alpha = 0.02
    moduleConfigClean1.beta = 2.0
    moduleConfigClean1.kappa = 0.0

    moduleConfigClean1.wC = [-1] * (moduleConfigClean1.numStates * 2 + 1)
    moduleConfigClean1.wM = [-1] * (moduleConfigClean1.numStates * 2 + 1)
    retTime = sunlineSuKF.sunlineSuKFTimeUpdate(moduleConfigClean1, 1)
    retMease = sunlineSuKF.sunlineSuKFMeasUpdate(moduleConfigClean1, 1)
    if retTime == 0:
        testFailCount += 1
        testMessages.append("Failed to catch bad Update and clean in Time update")
    if retMease == 0:
        testFailCount += 1
        testMessages.append("Failed to catch bad Update and clean in Meas update")

    moduleConfigClean1.wC = [1] * (moduleConfigClean1.numStates * 2 + 1)
    moduleConfigClean1.wM = [1] * (moduleConfigClean1.numStates * 2 + 1)
    qNoiseIn = numpy.identity(6)
    qNoiseIn[0:3, 0:3] = -qNoiseIn[0:3, 0:3] * 0.0017 * 0.0017
    qNoiseIn[3:6, 3:6] = -qNoiseIn[3:6, 3:6] * 0.00017 * 0.00017
    moduleConfigClean1.qNoise = qNoiseIn.reshape(36).tolist()
    retTime = sunlineSuKF.sunlineSuKFTimeUpdate(moduleConfigClean1, 1)
    retMease = sunlineSuKF.sunlineSuKFMeasUpdate(moduleConfigClean1, 1)

    if retTime == 0:
        testFailCount += 1
        testMessages.append("Failed to catch bad Update and clean in Time update")
    if retMease == 0:
        testFailCount += 1
        testMessages.append("Failed to catch bad Update and clean in Meas update")

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: fault detection test"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    # test_all_sunline_kf(True)
    # StateUpdateSunLine(True, True)
    test_FaultScenarios()
