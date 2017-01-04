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
import numpy
import pytest
import math

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('ADCSAlgorithms')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

import SimulationBaseClass
import alg_contain
import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
import inertialUKF  # import the module that is to be tested
import stComm
import vehicleConfigData
import macros
import sim_model
import ctypes


def setupFilterData(filterObject):
    filterObject.navStateOutMsgName = "inertial_state_estimate"
    filterObject.filtDataOutMsgName = "inertial_filter_data"
    filterObject.stDataInMsgName = "star_tracker_data"
    filterObject.massPropsInMsgName = "adcs_config_data"
    filterObject.rwSpeedsInMsgName = "reactionwheel_output_states"
    filterObject.rwParamsInMsgName = "rwa_config_data_parsed"

    filterObject.alpha = 0.02
    filterObject.beta = 2.0
    filterObject.kappa = 0.0
    filterObject.switchMag = 1.2

    filterObject.state = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    filterObject.covar = [0.04, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.04, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.04, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.004, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.004, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.004]
    qNoiseIn = numpy.identity(6)
    qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3]*0.0017*0.0017
    qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6]*0.00017*0.00017
    filterObject.qNoise = qNoiseIn.reshape(36).tolist()
    filterObject.qObs = [0.000017*0.000017, 0.0, 0.0,
                         0.0, 0.000017*0.000017, 0.0,
                         0.0, 0.0, 0.000017*0.000017]

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def test_all_sunline_kf(show_plots):
    [testResults, testMessage] = test_StatePropInertialAttitude(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = test_StatePropRateInertialAttitude(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = testStateUpdateInertialAttitude(show_plots)
    assert testResults < 1, testMessage

def testStateUpdateInertialAttitude(show_plots):
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
    moduleConfig = inertialUKF.InertialUKFConfig()
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        inertialUKF.Update_inertialUKF,
                                        inertialUKF.SelfInit_inertialUKF,
                                        inertialUKF.CrossInit_inertialUKF,
                                        inertialUKF.Reset_inertialUKF)
    moduleWrap.ModelTag = "InertialUKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)
    
    setupFilterData(moduleConfig)

    vehicleConfigOut = vehicleConfigData.vehicleConfigData()
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
                                                
    stMessage = stComm.STOutputData()
    stMessage.MRP_BdyInrtl = [0.3, 0.4, 0.5]

    inputMessageSize = stMessage.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                      moduleConfig.stDataInMsgName,
                                      inputMessageSize,
                                      2)  # number of buffers (leave at 2 as default, don't make zero)

#    stateTarget = testVector.tolist()
#    stateTarget.extend([0.0, 0.0, 0.0])
#    moduleConfig.state = [0.7, 0.7, 0.0]
    unitTestSim.AddVariableForLogging('InertialUKF.covar', testProcessRate*10, 0, 35, 'double')
    unitTestSim.AddVariableForLogging('InertialUKF.state', testProcessRate*10, 0, 5, 'double')

    unitTestSim.InitializeSimulation()

    for i in range(20000):
        if i > 20:
            stMessage.timeTag = i*0.5
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.stDataInMsgName,
                                      inputMessageSize,
                                      unitTestSim.TotalSim.CurrentNanos,
                                      stMessage)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i+1)*0.5))
        unitTestSim.ExecuteSimulation()

    covarLog = unitTestSim.GetLogVariableData('InertialUKF.covar')
    stateLog = unitTestSim.GetLogVariableData('InertialUKF.state')

    for i in range(3):
        if(covarLog[-1, i*6+1+i] > covarLog[0, i*6+1+i]):
            testFailCount += 1
            testMessages.append("Covariance update failure")
        if(abs(stateLog[-1, i+1] - stMessage.MRP_BdyInrtl[i]) > 1.0E-3):
            print abs(stateLog[-1, i+1] - stMessage.MRP_BdyInrtl[i])
            testFailCount += 1
            testMessages.append("State update failure")

    stMessage.MRP_BdyInrtl = [1.2, 0.0, 0.0]


    for i in range(20000):
        if i > 20:
            stMessage.timeTag = (i+20000)*0.5
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.stDataInMsgName,
                                      inputMessageSize,
                                      unitTestSim.TotalSim.CurrentNanos,
                                      stMessage)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i+20000+1)*0.5))
        unitTestSim.ExecuteSimulation()


    covarLog = unitTestSim.GetLogVariableData('InertialUKF.covar')
    stateLog = unitTestSim.GetLogVariableData('InertialUKF.state')
    for i in range(3):
        if(covarLog[-1, i*6+1+i] > covarLog[0, i*6+1+i]):
            testFailCount += 1
            testMessages.append("Covariance update failure")
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
def test_StatePropInertialAttitude(show_plots):

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
    moduleConfig = inertialUKF.InertialUKFConfig()
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        inertialUKF.Update_inertialUKF,
                                        inertialUKF.SelfInit_inertialUKF,
                                        inertialUKF.CrossInit_inertialUKF,
                                        inertialUKF.Reset_inertialUKF)
    moduleWrap.ModelTag = "InertialUKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)
    
    setupFilterData(moduleConfig)
    vehicleConfigOut = vehicleConfigData.vehicleConfigData()
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
    unitTestSim.AddVariableForLogging('InertialUKF.covar', testProcessRate*10, 0, 35)
    unitTestSim.AddVariableForLogging('InertialUKF.state', testProcessRate*10, 0, 5)
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(8000.0))
    unitTestSim.ExecuteSimulation()
    
    covarLog = unitTestSim.GetLogVariableData('InertialUKF.covar')
    stateLog = unitTestSim.GetLogVariableData('InertialUKF.state')

    
    for i in range(6):
        if(abs(stateLog[-1, i+1] - stateLog[0, i+1]) > 1.0E-10):
            print abs(stateLog[-1, i+1] - stateLog[0, i+1])
            testFailCount += 1
            testMessages.append("State propagation failure")

    for i in range(6):
        if(covarLog[-1, i*6+i+1] <= covarLog[0, i*6+i+1]):
            testFailCount += 1
            testMessages.append("State covariance failure")
        
    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " state propagation"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def test_StatePropRateInertialAttitude(show_plots):

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
    moduleConfig = inertialUKF.InertialUKFConfig()
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        inertialUKF.Update_inertialUKF,
                                        inertialUKF.SelfInit_inertialUKF,
                                        inertialUKF.CrossInit_inertialUKF,
                                        inertialUKF.Reset_inertialUKF)
    moduleWrap.ModelTag = "InertialUKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)
    
    setupFilterData(moduleConfig)
    vehicleConfigOut = vehicleConfigData.vehicleConfigData()
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
    moduleConfig.state = [0.0, 0.0, 0.0, math.pi/1800.0, 0.0, 0.0]
    unitTestSim.AddVariableForLogging('InertialUKF.covar', testProcessRate*10, 0, 35)
    unitTestSim.AddVariableForLogging('InertialUKF.state', testProcessRate*10, 0, 5)
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(3600.0))
    unitTestSim.ExecuteSimulation()
    
    covarLog = unitTestSim.GetLogVariableData('InertialUKF.covar')
    stateLog = unitTestSim.GetLogVariableData('InertialUKF.state')

    
    for i in range(6):
        if(abs(stateLog[-1, i+1] - stateLog[0, i+1]) > 1.0E-3):
            print abs(stateLog[-1, i+1] - stateLog[0, i+1])
            testFailCount += 1
            testMessages.append("State propagation failure")

    for i in range(6):
        if(covarLog[-1, i*6+i+1] <= covarLog[0, i*6+i+1]):
            testFailCount += 1
            testMessages.append("State covariance failure")
        
    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " state rate propagation"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    test_all_sunline_kf(False)
