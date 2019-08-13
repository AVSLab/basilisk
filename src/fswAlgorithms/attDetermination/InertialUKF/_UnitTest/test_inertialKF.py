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
import numpy
import math, inspect, os

from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import alg_contain
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import inertialUKF  # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.simulation import sim_model

from Basilisk.utilities import simIncludeRW
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import spacecraftPlus

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
textSnippetPassed = r'\textcolor{ForestGreen}{' + "PASSED" + '}'
textSnippetFailed = r'\textcolor{Red}{' + "Failed" + '}'

def setupFilterData(filterObject):
    filterObject.navStateOutMsgName = "inertial_state_estimate"
    filterObject.filtDataOutMsgName = "inertial_filter_data"
    filterObject.massPropsInMsgName = "adcs_config_data"
    filterObject.rwSpeedsInMsgName = "reactionwheel_output_states"
    filterObject.rwParamsInMsgName = "rwa_config_data_parsed"
    filterObject.gyrBuffInMsgName = "gyro_buffer_data"

    filterObject.alpha = 0.02
    filterObject.beta = 2.0
    filterObject.kappa = 0.0
    filterObject.switchMag = 1.2

    ST1Data = inertialUKF.STMessage()
    ST1Data.stInMsgName = "star_tracker_1_data"

    ST1Data.noise = [0.00017 * 0.00017, 0.0, 0.0,
                         0.0, 0.00017 * 0.00017, 0.0,
                         0.0, 0.0, 0.00017 * 0.00017]

    ST2Data = inertialUKF.STMessage()
    ST2Data.stInMsgName = "star_tracker_2_data"

    ST2Data.noise = [0.00017 * 0.00017, 0.0, 0.0,
                         0.0, 0.00017 * 0.00017, 0.0,
                         0.0, 0.0, 0.00017 * 0.00017]
    STList = [ST1Data, ST2Data]
    filterObject.STDatasStruct.STMessages = STList
    filterObject.STDatasStruct.numST = len(STList)

    filterObject.stateInit = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    filterObject.covarInit = [0.04, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.04, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.04, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.004, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.004, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.004]
    qNoiseIn = numpy.identity(6)
    qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3]*0.0017*0.0017
    qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6]*0.00017*0.00017
    filterObject.qNoise = qNoiseIn.reshape(36).tolist()

    lpDataUse = inertialUKF.LowPassFilterData()
    lpDataUse.hStep = 0.5
    lpDataUse.omegCutoff = 15.0/(2.0*math.pi)
    filterObject.gyroFilt = [lpDataUse, lpDataUse, lpDataUse]

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def all_inertial_kfTest(show_plots):
    [testResults, testMessage] = test_StatePropInertialAttitude(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = test_StatePropRateInertialAttitude(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = test_StateUpdateInertialAttitude(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = test_StateUpdateRWInertialAttitude(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = test_FilterMethods()
    assert  testResults <1, testMessage
    [testResults, testMessage] = test_FaultScenarios()
    assert testResults < 1, testMessage

def test_FilterMethods():
    testFailCount = 0
    testMessages = []

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    # Create test thread
    testProcessRate = macros.sec2nano(1.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    accuracy = 1E-10
    # Construct algorithm and associated C++ container
    moduleConfig = inertialUKF.InertialUKFConfig()
    moduleWrap = alg_contain.AlgContain(moduleConfig,
                                        inertialUKF.Update_inertialUKF,
                                        inertialUKF.SelfInit_inertialUKF,
                                        inertialUKF.CrossInit_inertialUKF,
                                        inertialUKF.Reset_inertialUKF)
    moduleWrap.ModelTag = "inertialUKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    st1 = inertialUKF.STAttFswMsg()
    st1.timeTag = macros.sec2nano(1.25)
    st1.MRP_BdyInrtl = [0.1, 0.2, 0.3]
    st2 = inertialUKF.STAttFswMsg()
    st2.timeTag = macros.sec2nano(1.0)
    st1.MRP_BdyInrtl = [0.2, 0.2, 0.3]
    st3 = inertialUKF.STAttFswMsg()
    st3.timeTag = macros.sec2nano(0.75)
    st1.MRP_BdyInrtl = [0.3, 0.2, 0.3]

    ST1Data = inertialUKF.STMessage()
    ST1Data.stInMsgName = "star_tracker_1_data"
    ST2Data = inertialUKF.STMessage()
    ST2Data.stInMsgName = "star_tracker_2_data"
    ST3Data = inertialUKF.STMessage()
    ST3Data.stInMsgName = "star_tracker_3_data"

    STList = [ST1Data, ST2Data, ST3Data]

    state = inertialUKF.new_doubleArray(6)
    stateInput = numpy.array([1., 0., 0., 0.1, 0.1, 0.1])
    for i in range(len(stateInput)):
        inertialUKF.doubleArray_setitem(state, i, stateInput[i])

    wheelAccel = numpy.array([-5, 5]) / 1. * numpy.array([1., 1])
    angAccel = -0.5 * (wheelAccel[0] + wheelAccel[1]) * numpy.array([1., 0., 0])
    expectedRate = numpy.array(stateInput[3:]) + angAccel

    inertialUKF.inertialStateProp(moduleConfig, state, 0.5)
    stateOut = []
    for j in range(6):
        stateOut.append(inertialUKF.doubleArray_getitem(state, j))

    if numpy.linalg.norm(expectedRate - numpy.array(stateOut)[3:]) > accuracy:
        testFailCount += 1
        testMessages.append("Failed to caputre wheel acceleration in inertialStateProp")

    setupFilterData(moduleConfig)
    vehicleConfigOut = inertialUKF.VehicleConfigFswMsg()
    inputMessageSize = vehicleConfigOut.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.massPropsInMsgName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)

    I = [1000., 0., 0.,
     0., 800., 0.,
     0., 0., 800.]
    vehicleConfigOut.ISCPntB_B = I
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.massPropsInMsgName,
                                                inputMessageSize,
                                                0,
                                                vehicleConfigOut)
    moduleConfig.STDatasStruct.STMessages = STList
    moduleConfig.STDatasStruct.numST = len(STList)
    unitTestSim.AddVariableForLogging('inertialUKF.stSensorOrder', testProcessRate , 0, 3, 'double')
    for i in range(1,4):
        inputMessageSize = eval("st"+str(i)).getStructSize()
        unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                              "star_tracker_" + str(i) + "_data",
                                              inputMessageSize,
                                              2)  # number of buffers (leave at 2 as default, don't make zero)
        unitTestSim.TotalSim.WriteMessageData("star_tracker_" + str(i) + "_data",
                                                    inputMessageSize,
                                                    0,
                                                    eval("st"+str(i)))
    # Star Tracker Read Message and Order method
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(1E9)
    unitTestSim.ExecuteSimulation()

    stOrdered = unitTestSim.GetLogVariableData('inertialUKF.stSensorOrder')
    if numpy.linalg.norm(numpy.array(stOrdered[0]) - numpy.array([0., 2, 1, 0, 0])) > accuracy:
        testFailCount+=1
        testMessages.append("ST order test failed")

    unitTestSupport.writeTeXSnippet("toleranceValue00", str(accuracy), path)
    if testFailCount == 0:
        unitTestSupport.writeTeXSnippet("passFail00", textSnippetPassed, path)
    else:
        unitTestSupport.writeTeXSnippet("passFail00", textSnippetFailed, path)

    return [testFailCount, ''.join(testMessages)]

def test_StateUpdateInertialAttitude(show_plots):
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
    moduleConfig.maxTimeJump = 10

    vehicleConfigOut = inertialUKF.VehicleConfigFswMsg()
    inputMessageSize = vehicleConfigOut.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.massPropsInMsgName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)

    I = [1000., 0., 0.,
     0., 800., 0.,
     0., 0., 800.]
    vehicleConfigOut.ISCPntB_B = I
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.massPropsInMsgName,
                                                inputMessageSize,
                                                0,
                                                vehicleConfigOut)

    stMessage1 = inertialUKF.STAttFswMsg()
    stMessage1.MRP_BdyInrtl = [0.3, 0.4, 0.5]

    inputMessageSize = stMessage1.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.STDatasStruct.STMessages[0].stInMsgName,
                                      inputMessageSize,
                                      2)  # number of buffers (leave at 2 as default, don't make zero)

    stMessage2 = inertialUKF.STAttFswMsg()
    stMessage2.MRP_BdyInrtl = [0.3, 0.4, 0.5]
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.STDatasStruct.STMessages[1].stInMsgName,
                                      inputMessageSize,
                                      2)
#    stateTarget = testVector.tolist()
#    stateTarget.extend([0.0, 0.0, 0.0])
#    moduleConfig.state = [0.7, 0.7, 0.0]
    unitTestSim.AddVariableForLogging('InertialUKF.covar', testProcessRate*10, 0, 35, 'double')
    unitTestSim.AddVariableForLogging('InertialUKF.state', testProcessRate*10, 0, 5, 'double')

    unitTestSim.InitializeSimulation()

    for i in range(20000):
        if i > 21:
            stMessage1.timeTag = int(i*0.5*1E9)
            stMessage2.timeTag = int(i*0.5*1E9)

            unitTestSim.TotalSim.WriteMessageData(moduleConfig.STDatasStruct.STMessages[0].stInMsgName,
                                      inputMessageSize,
                                      unitTestSim.TotalSim.CurrentNanos,
                                      stMessage1)
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.STDatasStruct.STMessages[1].stInMsgName,
                                      inputMessageSize,
                                      unitTestSim.TotalSim.CurrentNanos,
                                      stMessage2)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i+1)*0.5))
        unitTestSim.ExecuteSimulation()

    covarLog = unitTestSim.GetLogVariableData('InertialUKF.covar')
    stateLog = unitTestSim.GetLogVariableData('InertialUKF.state')
    accuracy = 1.0E-5
    unitTestSupport.writeTeXSnippet("toleranceValue11", str(accuracy), path)
    for i in range(3):
        if(covarLog[-1, i*6+1+i] > covarLog[0, i*6+1+i]):
            testFailCount += 1
            testMessages.append("Covariance update failure")
            unitTestSupport.writeTeXSnippet('passFail11', textSnippetFailed, path)
        else:
            unitTestSupport.writeTeXSnippet('passFail11', textSnippetPassed, path)
        if(abs(stateLog[-1, i+1] - stMessage1.MRP_BdyInrtl[i]) > accuracy):
            print(abs(stateLog[-1, i+1] - stMessage1.MRP_BdyInrtl[i]))
            testFailCount += 1
            testMessages.append("State update failure")
            unitTestSupport.writeTeXSnippet('passFail11', textSnippetFailed, path)
        else:
            unitTestSupport.writeTeXSnippet('passFail11', textSnippetPassed, path)

    stMessage1.MRP_BdyInrtl = [1.2, 0.0, 0.0]
    stMessage2.MRP_BdyInrtl = [1.2, 0.0, 0.0]


    for i in range(20000):
        if i > 20:
            stMessage1.timeTag = int((i+20000)*0.25*1E9)
            stMessage2.timeTag = int((i+20000)*0.5*1E9)
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.STDatasStruct.STMessages[0].stInMsgName,
                                      inputMessageSize,
                                      unitTestSim.TotalSim.CurrentNanos,
                                      stMessage1)
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.STDatasStruct.STMessages[1].stInMsgName,
                                      inputMessageSize,
                                      unitTestSim.TotalSim.CurrentNanos,
                                      stMessage2)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i+20000+1)*0.5))
        unitTestSim.ExecuteSimulation()


    covarLog = unitTestSim.GetLogVariableData('InertialUKF.covar')
    stateLog = unitTestSim.GetLogVariableData('InertialUKF.state')
    for i in range(3):
        if(covarLog[-1, i*6+1+i] > covarLog[0, i*6+1+i]):
            testFailCount += 1
            testMessages.append("Covariance update large failure")
            unitTestSupport.writeTeXSnippet('passFail11', textSnippetFailed, path)
        else:
            unitTestSupport.writeTeXSnippet('passFail11', textSnippetPassed, path)
    plt.figure()
    for i in range(moduleConfig.numStates):
        plt.plot(stateLog[:,0]*1.0E-9, stateLog[:,i+1], label='State_' +str(i))
        plt.legend()
        plt.ylim([-1, 1])

    unitTestSupport.writeFigureLaTeX('Test11', 'Test 1 State convergence', plt, 'width=0.9\\textwidth, keepaspectratio', path)
    plt.figure()
    for i in range(moduleConfig.numStates):
        plt.plot(covarLog[:,0]*1.0E-9, covarLog[:,i*moduleConfig.numStates+i+1], label='Covar_' +str(i))
        plt.legend()
        plt.ylim([0, 2.E-7])

    unitTestSupport.writeFigureLaTeX('Test12', 'Test 1 Covariance convergence', plt, 'width=0.9\\textwidth, keepaspectratio', path)
    if(show_plots):
        plt.show()
        plt.close('all')

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + moduleWrap.ModelTag + " state update")

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
    vehicleConfigOut = inertialUKF.VehicleConfigFswMsg()
    inputMessageSize = vehicleConfigOut.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.massPropsInMsgName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    I = [1000., 0., 0.,
     0., 800., 0.,
     0., 0., 800.]
    vehicleConfigOut.ISCPntB_B = I
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

    accuracy = 1.0E-10
    unitTestSupport.writeTeXSnippet("toleranceValue22", str(accuracy), path)
    for i in range(6):
        if(abs(stateLog[-1, i+1] - stateLog[0, i+1]) > accuracy):
            print(abs(stateLog[-1, i+1] - stateLog[0, i+1]))
            testFailCount += 1
            testMessages.append("State propagation failure")
            unitTestSupport.writeTeXSnippet('passFail22', textSnippetFailed, path)
        else:
            unitTestSupport.writeTeXSnippet('passFail22', textSnippetPassed, path)

    for i in range(6):
       if(covarLog[-1, i*6+i+1] <= covarLog[0, i*6+i+1]):
           testFailCount += 1
           testMessages.append("State covariance failure")

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + moduleWrap.ModelTag + " state propagation")

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def test_StateUpdateRWInertialAttitude(show_plots):
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

    vehicleConfigOut = inertialUKF.VehicleConfigFswMsg()
    inputMessageSize = vehicleConfigOut.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.massPropsInMsgName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)

    I = [1000., 0., 0.,
         0., 800., 0.,
         0., 0., 800.]
    vehicleConfigOut.ISCPntB_B = I
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.massPropsInMsgName,
                                          inputMessageSize,
                                          0,
                                          vehicleConfigOut)

    rwArrayConfigOut = inertialUKF.RWArrayConfigFswMsg()
    msgSize = rwArrayConfigOut.getStructSize()
    rwArrayConfigOut.numRW = 3
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.rwParamsInMsgName,
                                          msgSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)

    unitTestSim.TotalSim.WriteMessageData(moduleConfig.rwParamsInMsgName,
                                          msgSize,
                                          0,
                                          rwArrayConfigOut)

    rwSpeedIntMsg = inertialUKF.RWSpeedIntMsg()
    msgSize = rwSpeedIntMsg.getStructSize()
    rwSpeedIntMsg.wheelSpeeds = [0.1, 0.01, 0.1]
    rwSpeedIntMsg.wheelThetas = [0.,0.,0.]
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.rwSpeedsInMsgName,
                                          msgSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)

    unitTestSim.TotalSim.WriteMessageData(moduleConfig.rwSpeedsInMsgName,
                                          msgSize,
                                          0,
                                          rwSpeedIntMsg)

    stMessage1 = inertialUKF.STAttFswMsg()
    stMessage1.MRP_BdyInrtl = [0.3, 0.4, 0.5]

    inputMessageSize = stMessage1.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.STDatasStruct.STMessages[0].stInMsgName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)

    stMessage2 = inertialUKF.STAttFswMsg()
    stMessage2.MRP_BdyInrtl = [0.3, 0.4, 0.5]
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.STDatasStruct.STMessages[1].stInMsgName,
                                          inputMessageSize,
                                          2)
    #    stateTarget = testVector.tolist()
    #    stateTarget.extend([0.0, 0.0, 0.0])
    #    moduleConfig.state = [0.7, 0.7, 0.0]
    unitTestSim.AddVariableForLogging('InertialUKF.covar', testProcessRate * 10, 0, 35, 'double')
    unitTestSim.AddVariableForLogging('InertialUKF.state', testProcessRate * 10, 0, 5, 'double')

    unitTestSim.InitializeSimulation()

    for i in range(20000):
        if i > 20:
            stMessage1.timeTag = int(i * 0.5 * 1E9)
            stMessage2.timeTag = int(i * 0.5 * 1E9)

            unitTestSim.TotalSim.WriteMessageData(moduleConfig.STDatasStruct.STMessages[0].stInMsgName,
                                                  inputMessageSize,
                                                  unitTestSim.TotalSim.CurrentNanos,
                                                  stMessage1)
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.STDatasStruct.STMessages[1].stInMsgName,
                                                  inputMessageSize,
                                                  unitTestSim.TotalSim.CurrentNanos,
                                                  stMessage2)
        if i==10000:
            rwSpeedIntMsg.wheelSpeeds = [0.5, 0.1, 0.05]
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.rwSpeedsInMsgName,
                                                  msgSize,
                                                  0,
                                                  rwSpeedIntMsg)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i + 1) * 0.5))
        unitTestSim.ExecuteSimulation()

    covarLog = unitTestSim.GetLogVariableData('InertialUKF.covar')
    stateLog = unitTestSim.GetLogVariableData('InertialUKF.state')
    accuracy = 1.0E-5
    unitTestSupport.writeTeXSnippet("toleranceValue33", str(accuracy), path)
    for i in range(3):
        if (covarLog[-1, i * 6 + 1 + i] > covarLog[0, i * 6 + 1 + i]):
            testFailCount += 1
            testMessages.append("Covariance update with RW failure")
        if (abs(stateLog[-1, i + 1] - stMessage1.MRP_BdyInrtl[i]) > accuracy):
            print(abs(stateLog[-1, i + 1] - stMessage1.MRP_BdyInrtl[i]))
            testFailCount += 1
            testMessages.append("State update with RW failure")
            unitTestSupport.writeTeXSnippet('passFail33', textSnippetFailed, path)
        else:
            unitTestSupport.writeTeXSnippet('passFail33', textSnippetPassed, path)

    stMessage1.MRP_BdyInrtl = [1.2, 0.0, 0.0]
    stMessage2.MRP_BdyInrtl = [1.2, 0.0, 0.0]

    for i in range(20000):
        if i > 20:
            stMessage1.timeTag = int((i + 20000) * 0.25 * 1E9)
            stMessage2.timeTag = int((i + 20000) * 0.5 * 1E9)
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.STDatasStruct.STMessages[0].stInMsgName,
                                                  inputMessageSize,
                                                  unitTestSim.TotalSim.CurrentNanos,
                                                  stMessage1)
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.STDatasStruct.STMessages[1].stInMsgName,
                                                  inputMessageSize,
                                                  unitTestSim.TotalSim.CurrentNanos,
                                                  stMessage2)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i + 20000 + 1) * 0.5))
        unitTestSim.ExecuteSimulation()

    covarLog = unitTestSim.GetLogVariableData('InertialUKF.covar')
    stateLog = unitTestSim.GetLogVariableData('InertialUKF.state')
    for i in range(3):
        if (covarLog[-1, i * 6 + 1 + i] > covarLog[0, i * 6 + 1 + i]):
            testFailCount += 1
            testMessages.append("Covariance update large failure")
            unitTestSupport.writeTeXSnippet('passFail33', textSnippetFailed, path)
        else:
            unitTestSupport.writeTeXSnippet('passFail33', textSnippetPassed, path)
    plt.figure()
    for i in range(moduleConfig.numStates):
        plt.plot(stateLog[:, 0] * 1.0E-9, stateLog[:, i + 1], label='State_' +str(i))
        plt.legend()
        plt.ylim([-1, 1])

    unitTestSupport.writeFigureLaTeX('Test31', 'Test 3 State convergence', plt, 'width=0.7\\textwidth, keepaspectratio', path)
    plt.figure()
    for i in range(moduleConfig.numStates):
        plt.plot(covarLog[:, 0] * 1.0E-9, covarLog[:, i * moduleConfig.numStates + i + 1], label='Covar_' +str(i))
        plt.legend()
        plt.ylim([0., 2E-7])

    unitTestSupport.writeFigureLaTeX('Test32', 'Test 3 Covariance convergence', plt, 'width=0.7\\textwidth, keepaspectratio', path)
    if (show_plots):
        plt.show()
        plt.close('all')

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + moduleWrap.ModelTag + " state update with RW")

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

    moduleConfig.navStateOutMsgName = "inertial_state_estimate"
    moduleConfig.filtDataOutMsgName = "inertial_filter_data"
    moduleConfig.massPropsInMsgName = "adcs_config_data"
    moduleConfig.rwSpeedsInMsgName = "reactionwheel_output_states"
    moduleConfig.rwParamsInMsgName = "rwa_config_data_parsed"
    moduleConfig.gyrBuffInMsgName = "gyro_buffer_data"

    moduleConfig.alpha = 0.02
    moduleConfig.beta = 2.0
    moduleConfig.kappa = 0.0
    moduleConfig.switchMag = 1.2

    moduleConfig.stateInit = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    moduleConfig.covarInit = [0.04, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.04, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.04, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.004, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.004, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.004]
    qNoiseIn = numpy.identity(6)
    qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3] * 0.0017 * 0.0017
    qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6] * 0.00017 * 0.00017
    moduleConfig.qNoise = qNoiseIn.reshape(36).tolist()

    ST1Data = inertialUKF.STMessage()
    ST1Data.stInMsgName = "star_tracker_1_data"
    ST1Data.noise = [0.00017 * 0.00017, 0.0, 0.0,
                         0.0, 0.00017 * 0.00017, 0.0,
                         0.0, 0.0, 0.00017 * 0.00017]
    STList = [ST1Data]
    moduleConfig.STDatasStruct.STMessages = STList
    moduleConfig.STDatasStruct.numST = len(STList)

    lpDataUse = inertialUKF.LowPassFilterData()
    lpDataUse.hStep = 0.5
    lpDataUse.omegCutoff = 15.0 / (2.0 * math.pi)
    moduleConfig.gyroFilt = [lpDataUse, lpDataUse, lpDataUse]

    vehicleConfigOut = inertialUKF.VehicleConfigFswMsg()
    inputMessageSize = vehicleConfigOut.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.massPropsInMsgName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    I = [1000., 0., 0.,
     0., 800., 0.,
     0., 0., 800.]
    vehicleConfigOut.ISCPntB_B = I
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.massPropsInMsgName,
                                                inputMessageSize,
                                                0,
                                                vehicleConfigOut)
    stateInit = [0.0, 0.0, 0.0, math.pi/18.0, 0.0, 0.0]
    moduleConfig.stateInit = stateInit
    unitTestSim.AddVariableForLogging('InertialUKF.covar', testProcessRate*10, 0, 35)
    unitTestSim.AddVariableForLogging('InertialUKF.sigma_BNOut', testProcessRate*10, 0, 2)
    unitTestSim.AddVariableForLogging('InertialUKF.omega_BN_BOut', testProcessRate*10, 0, 2)
    unitTestSim.InitializeSimulation()
    stMessage1 = inertialUKF.STAttFswMsg()
    stMessage1.MRP_BdyInrtl = [0., 0., 0.]

    inputMessageSize = stMessage1.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.STDatasStruct.STMessages[0].stInMsgName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    stMessage1.timeTag = int(1* 1E9)
    unitTestSim.TotalSim.WriteMessageData(moduleConfig.STDatasStruct.STMessages[0].stInMsgName,
                                          inputMessageSize,
                                          int(1 * 1E9),
                                          stMessage1)
    gyroBufferData = inertialUKF.AccDataFswMsg()
    for i in range(3600*2+1):
        gyroBufferData.accPkts[i%inertialUKF.MAX_ACC_BUF_PKT].measTime = (int(i*0.5*1E9))
        gyroBufferData.accPkts[i%inertialUKF.MAX_ACC_BUF_PKT].gyro_B = \
            [math.pi/18.0, 0.0, 0.0]
        unitTestSim.TotalSim.WriteMessageData(moduleConfig.gyrBuffInMsgName,
                                              gyroBufferData.getStructSize(),
                                              (int(i*0.5*1E9)),
                                              gyroBufferData)

        unitTestSim.ConfigureStopTime(macros.sec2nano((i+1)*0.5))
        unitTestSim.ExecuteSimulation()

    covarLog = unitTestSim.GetLogVariableData('InertialUKF.covar')
    sigmaLog = unitTestSim.GetLogVariableData('InertialUKF.sigma_BNOut')
    omegaLog = unitTestSim.GetLogVariableData('InertialUKF.omega_BN_BOut')
    accuracy = 1.0E-3
    unitTestSupport.writeTeXSnippet("toleranceValue44", str(accuracy), path)
    for i in range(3):
        if(abs(omegaLog[-1, i+1] - stateInit[i+3]) > accuracy):
            print(abs(omegaLog[-1, i+1] - stateInit[i+3]))
            testFailCount += 1
            testMessages.append("State omega propagation failure")
            unitTestSupport.writeTeXSnippet('passFail44', textSnippetFailed, path)
        else:
            unitTestSupport.writeTeXSnippet('passFail44', textSnippetPassed, path)

    for i in range(6):
       if(covarLog[-1, i*6+i+1] <= covarLog[0, i*6+i+1]):
           testFailCount += 1
           testMessages.append("State covariance failure")

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: " + moduleWrap.ModelTag + " state rate propagation")
    else:
        print("Failed: " + testMessages[0])

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


def test_FaultScenarios(show_plots):
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
    moduleConfigClean1 = inertialUKF.InertialUKFConfig()
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

    inertialUKF.inertialUKFCleanUpdate(moduleConfigClean1)

    if numpy.linalg.norm(numpy.array(moduleConfigClean1.covarPrev) - numpy.array(moduleConfigClean1.covar)) > 1E10:
        testFailCount += 1
        testMessages.append("inertialUKFClean Covar failed")
    if numpy.linalg.norm(numpy.array(moduleConfigClean1.statePrev) - numpy.array(moduleConfigClean1.state)) > 1E10:
        testFailCount += 1
        testMessages.append("inertialUKFClean States failed")
    if numpy.linalg.norm(numpy.array(moduleConfigClean1.sBar) - numpy.array(moduleConfigClean1.sBarPrev)) > 1E10:
        testFailCount += 1
        testMessages.append("inertialUKFClean sBar failed")

    # inertialStateProp rate test with time step difference
    moduleConfigClean1.rwConfigParams.numRW = 2
    moduleConfigClean1.rwSpeeds.wheelSpeeds = [10, 5]
    moduleConfigClean1.rwSpeedPrev.wheelSpeeds = [15, 10]
    moduleConfigClean1.rwConfigParams.JsList = [1., 1.]
    moduleConfigClean1.rwConfigParams.GsMatrix_B = [1., 0., 0., 1., 0., 0.]
    moduleConfigClean1.speedDt = 1.
    #moduleConfigClean1.IInv = [1., 0., 0., 0., 1., 0., 0., 0., 1.]

    # Bad Time and Measurement Update
    st1 = inertialUKF.STAttFswMsg()
    st1.timeTag = macros.sec2nano(1.)
    st1.MRP_BdyInrtl = [0.1, 0.2, 0.3]

    ST1Data = inertialUKF.STMessage()
    ST1Data.stInMsgName = "star_tracker_1_data"
    ST1Data.noise = [1., 0., 0.,
                     0., 1., 0.,
                     0., 0., 1.]

    STList = [ST1Data]

    moduleConfigClean1.navStateOutMsgName = "inertial_state_estimate"
    moduleConfigClean1.filtDataOutMsgName = "inertial_filter_data"
    moduleConfigClean1.massPropsInMsgName = "adcs_config_data"
    moduleConfigClean1.rwSpeedsInMsgName = "reactionwheel_output_states"
    moduleConfigClean1.rwParamsInMsgName = "rwa_config_data_parsed"
    moduleConfigClean1.gyrBuffInMsgName = "gyro_buffer_data"

    moduleConfigClean1.alpha = 0.02
    moduleConfigClean1.beta = 2.0
    moduleConfigClean1.kappa = 0.0
    moduleConfigClean1.switchMag = 1.2

    moduleConfigClean1.countHalfSPs = moduleConfigClean1.numStates
    moduleConfigClean1.STDatasStruct.STMessages = STList
    moduleConfigClean1.STDatasStruct.numST = len(STList)
    moduleConfigClean1.wC = [-1] * (moduleConfigClean1.numStates * 2 + 1)
    moduleConfigClean1.wM = [-1] * (moduleConfigClean1.numStates * 2 + 1)
    retTime = inertialUKF.inertialUKFTimeUpdate(moduleConfigClean1, 1)
    retMease = inertialUKF.inertialUKFMeasUpdate(moduleConfigClean1, 1)
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
    retTime = inertialUKF.inertialUKFTimeUpdate(moduleConfigClean1, 1)
    retMease = inertialUKF.inertialUKFMeasUpdate(moduleConfigClean1, 1)

    if retTime == 0:
        testFailCount += 1
        testMessages.append("Failed to catch bad Update and clean in Time update")
    if retMease == 0:
        testFailCount += 1
        testMessages.append("Failed to catch bad Update and clean in Meas update")

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED: state rate propagation")

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    test_FilterMethods()
