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
import math

from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import alg_contain
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import inertialUKF  # import the module that is to be tested
from Basilisk.utilities import macros
from Basilisk.simulation import sim_model


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
        if i > 20:
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

    for i in range(3):
        if(covarLog[-1, i*6+1+i] > covarLog[0, i*6+1+i]):
            testFailCount += 1
            testMessages.append("Covariance update failure")
        if(abs(stateLog[-1, i+1] - stMessage1.MRP_BdyInrtl[i]) > 1.0E-5):
            print abs(stateLog[-1, i+1] - stMessage1.MRP_BdyInrtl[i])
            testFailCount += 1
            testMessages.append("State update failure")

    stMessage1.MRP_BdyInrtl = [1.2, 0.0, 0.0]
    stMessage2.MRP_BdyInrtl = [1.2, 0.0, 0.0]


    for i in range(20000):
        if i > 20:
            stMessage1.timeTag = int((i+20000)*0.5*1E9)
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
    plt.figure()
    for i in range(moduleConfig.numStates):
        plt.plot(stateLog[:,0]*1.0E-9, stateLog[:,i+1])

    plt.figure()
    for i in range(moduleConfig.numStates):
        plt.plot(covarLog[:,0]*1.0E-9, covarLog[:,i*moduleConfig.numStates+i+1])

    if(show_plots):
        plt.show()
        plt.close('all')

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

    
    for i in range(6):
        if(abs(stateLog[-1, i+1] - stateLog[0, i+1]) > 1.0E-10):
            print abs(stateLog[-1, i+1] - stateLog[0, i+1])
            testFailCount += 1
            testMessages.append("State propagation failure")

    #for i in range(6):
    #    if(covarLog[-1, i*6+i+1] <= covarLog[0, i*6+i+1]):
    #        testFailCount += 1
    #        testMessages.append("State covariance failure")
        
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
    stateInit = [0.0, 0.0, 0.0, math.pi/1800.0, 0.0, 0.0]
    moduleConfig.state = stateInit
    unitTestSim.AddVariableForLogging('InertialUKF.covar', testProcessRate*10, 0, 35)
    unitTestSim.AddVariableForLogging('InertialUKF.sigma_BNOut', testProcessRate*10, 0, 2)
    unitTestSim.AddVariableForLogging('InertialUKF.omega_BN_BOut', testProcessRate*10, 0, 2)
    unitTestSim.InitializeSimulation()
    gyroBufferData = inertialUKF.AccDataFswMsg()
    for i in range(3600*2+1):
        gyroBufferData.accPkts[i%inertialUKF.MAX_ACC_BUF_PKT].measTime = (int(i*0.5*1E9))
        gyroBufferData.accPkts[i%inertialUKF.MAX_ACC_BUF_PKT].gyro_B = \
            [math.pi/1800.0, 0.0, 0.0]
        unitTestSim.TotalSim.WriteMessageData(moduleConfig.gyrBuffInMsgName,
                                              gyroBufferData.getStructSize(),
                                              (int(i*0.5*1E9)),
                                              gyroBufferData)
        
        unitTestSim.ConfigureStopTime(macros.sec2nano((i+1)*0.5))
        unitTestSim.ExecuteSimulation()
    
    covarLog = unitTestSim.GetLogVariableData('InertialUKF.covar')
    sigmaLog = unitTestSim.GetLogVariableData('InertialUKF.sigma_BNOut')
    omegaLog = unitTestSim.GetLogVariableData('InertialUKF.omega_BN_BOut')

    for i in range(3):
        if(abs(sigmaLog[-1, i+1] - sigmaLog[0, i+1]) > 1.0E-3):
            print abs(sigmaLog[-1, i+1] - sigmaLog[0, i+1])
            testFailCount += 1
            testMessages.append("State sigma propagation failure")
        if(abs(omegaLog[-1, i+1] - stateInit[i+3]) > 1.0E-3):
            print abs(omegaLog[-1, i+1] - stateInit[i+3])
            testFailCount += 1
            testMessages.append("State omega propagation failure")

#    for i in range(6):
#        if(covarLog[-1, i*6+i+1] <= covarLog[0, i*6+i+1]):
#            testFailCount += 1
#            testMessages.append("State covariance failure")

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " state rate propagation"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    all_inertial_kfTest(False)
