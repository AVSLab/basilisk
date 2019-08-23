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
import numpy as np
import pytest
import math

from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, unitTestSupport
from Basilisk.fswAlgorithms import pixelLineBiasUKF  # import the module that is to be tested

import relativeODuKF_test_utilities as FilterPlots
import numpy as np

def rk4(f, t, x0):
    x = np.zeros([len(t),len(x0)+1])
    h = (t[len(t)-1] - t[0])/len(t)
    x[0,0] = t[0]
    x[0,1:] = x0
    for i in range(len(t)-1):
        h = t[i+1] - t[i]
        x[i,0] = t[i]
        k1 = h * f(t[i], x[i,1:])
        k2 = h * f(t[i] + 0.5 * h, x[i,1:] + 0.5 * k1)
        k3 = h * f(t[i] + 0.5 * h, x[i,1:] + 0.5 * k2)
        k4 = h * f(t[i] + h, x[i,1:] + k3)
        x[i+1,1:] = x[i,1:] + (k1 + 2.*k2 + 2.*k3 + k4) / 6.
        x[i+1,0] = t[i+1]
    return x

def twoBodyGrav(t, x, mu = 42828.314*1E9):
    dxdt = np.zeros(np.shape(x))
    dxdt[0:3] = x[3:6]
    dxdt[3:6] = -mu/np.linalg.norm(x[0:3])**3.*x[0:3]
    return dxdt


def setupFilterData(filterObject):
    filterObject.navStateOutMsgName = "relod_state_estimate"
    filterObject.filtDataOutMsgName = "relod_filter_data"
    filterObject.circlesInMsgName = "circles_data"
    filterObject.cameraConfigMsgName = "camera_config_data"
    filterObject.attInMsgName = "simple_att_nav_output"

    filterObject.planetIdInit = 2
    filterObject.alpha = 0.02
    filterObject.beta = 2.0
    filterObject.kappa = 0.0
    filterObject.gamma = 0.9

    mu = 42828.314*1E9 #m^3/s^2
    elementsInit = orbitalMotion.ClassicElements()
    elementsInit.a = 4000*1E3 #m
    elementsInit.e = 0.2
    elementsInit.i = 10
    elementsInit.Omega = 0.001
    elementsInit.omega = 0.01
    elementsInit.f = 0.1
    r, v = orbitalMotion.elem2rv(mu, elementsInit)
    bias = [1,1,-2]

    filterObject.stateInit = r.tolist() + v.tolist() + bias
    filterObject.covarInit = [1000.*1E6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 1000.*1E6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 1000.*1E6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 5*1E6, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 5*1E6, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 5*1E6, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0,]

    qNoiseIn = np.identity(9)
    qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3]*0.00001*0.00001*1E-6
    qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6]*0.0001*0.0001*1E-6
    qNoiseIn[6:9, 6:9] = qNoiseIn[3:6, 3:6]*0.0001*0.0001
    filterObject.qNoise = qNoiseIn.reshape(9*9).tolist()

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

def test_methods_kf(show_plots):
    [testResults, testMessage] = relOD_method_test(show_plots)
    assert testResults < 1, testMessage
def test_propagation_kf(show_plots):
    [testResults, testMessage] = StatePropRelOD(show_plots)
    assert testResults < 1, testMessage
def test_measurements_kf(show_plots):
    [testResults, testMessage] = StateUpdateRelOD(show_plots)
    assert testResults < 1, testMessage


def relOD_method_test(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    state = [250, 32000, 1000, 5, 3, 2, 1, 1, 1]
    dt = 10
    mu = 42828.314
    # Measurement Model Test
    data = pixelLineBiasUKF.PixelLineBiasUKFConfig()
    msg = pixelLineBiasUKF.CirclesOpNavMsg()
    msg.circlesCenters = [100, 200]
    msg.circlesRadii = [100]
    data.cirlcesInMsg = msg
    data.planetId = 2
    data.countHalfSPs = len(state)

    # Dynamics Model Test
    data.planetId = 2

    stateIn = pixelLineBiasUKF.new_doubleArray(len(state))
    for i in range(len(state)):
        pixelLineBiasUKF.doubleArray_setitem(stateIn, i, state[i])

    pixelLineBiasUKF.relODStateProp(data, stateIn, dt)

    propedState = []
    for i in range(len(state)):
        propedState.append(pixelLineBiasUKF.doubleArray_getitem(stateIn, i))
    expected = rk4(twoBodyGrav, [0, dt], np.array(state)*1E3)
    expected[:,1:]*=1E-3
    if np.linalg.norm((np.array(propedState) - expected[-1,1:])/(expected[-1,1:])) > 1.0E-15:
        testFailCount += 1
        testMessages.append("State Prop Failure")
    return [testFailCount, ''.join(testMessages)]


def StateUpdateRelOD(show_plots):
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
    dt = 1
    t1 = 250
    multT1 = 8
    state = [250, 32000, 1000, 5, 3, 2, 1, 1, 1]

    testProcessRate = macros.sec2nano(dt)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = pixelLineBiasUKF.PixelLineBiasUKFConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "relodSuKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    setupFilterData(moduleConfig)
    unitTestSim.TotalSim.logThisMessage('relod_filter_data', testProcessRate)

    # Create the input messages.
    inputCamera = pixelLineBiasUKF.CameraConfigMsg()
    inputCircles = pixelLineBiasUKF.CirclesOpNavMsg()
    inputAtt = pixelLineBiasUKF.NavAttIntMsg()

    # Set camera
    inputCamera.focalLength = 1.
    inputCamera.sensorSize = [10, 10]  # In mm
    inputCamera.resolution = [512, 512]
    inputCamera.sigma_CB = [1., 0.3, 0.1]
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.cameraConfigMsgName, inputCamera)

    # Set attitude
    inputAtt.sigma_BN = [0.6, 1., 0.1]
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.attInMsgName, inputAtt)


    time = np.linspace(0,multT1*t1,multT1*t1/dt+1)
    dydt = np.zeros(len(state))
    energy = np.zeros(len(time))
    expected=np.zeros([len(time), len(state)+1])
    expected[0,1:] = moduleConfig.stateInit
    mu = 42828.314*1E9
    energy[0] = -mu/(2*orbitalMotion.rv2elem(mu, expected[0,1:4], expected[0,4:7]).a)

    kick = np.array([0.,0.,0.,-0.01, 0.01, 0.02, 0.,0.,0.]) * 10 *1E3

    expected[0:t1,:] = rk4(twoBodyGrav, time[0:t1], moduleConfig.stateInit)
    expected[t1:multT1*t1+1,:] = rk4(twoBodyGrav, time[t1:len(time)], expected[t1-1,1:] + kick)
    for i in range(1, len(time)):
        energy[i] = - mu / (2 * orbitalMotion.rv2elem(mu, expected[i, 1:4], expected[i, 4:7]).a)

    # Set circles
    inputCircles.circlesCenters = [152, 251]
    inputCircles.circlesRadii = [75]
    inputCircles.uncertainty = [0.5, 0., 0., 0., 0.5, 0., 0., 0., 1.]
    inputCircles.timeTag = 12345
    inputCircles.valid = 1
    inputCircles.planetIds = [2]
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.circlesInMsgName, inputCircles)

    inputCircles.circlesCenters = expected[0,1:4]

    unitTestSim.InitializeSimulation()
    for i in range(t1):
        if i > 0 and i % 50 == 0:
            inputCircles.timeTag = macros.sec2nano(i * dt)
            inputCircles.circlesCenters = expected[i,1:4] + np.random.normal(0, 5*1E-2, 3)
            inputCircles.circlesRadii = [1]
            inputCircles.valid = 1
            inputCircles.uncertainty = [5.*1E-2, 0.,0.,
                                 0., 5.*1E-2, 0.,
                                 0., 0., 5.*1E-2]
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.circlesInMsgName,
                                                  inputCircles.getStructSize(),
                                                  unitTestSim.TotalSim.CurrentNanos,
                                                  inputCircles)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        unitTestSim.ExecuteSimulation()

    covarLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".covar", range(len(state) * len(state)))

    for i in range(len(state)):
        if (covarLog[t1, i * len(state) + 1 + i] > covarLog[0, i * len(state) + 1 + i] / 100):
            testFailCount += 1
            testMessages.append("Covariance update failure at " + str(t1))

    for i in range(t1, multT1*t1):
        if i % 50 == 0:
            inputCircles.timeTag = macros.sec2nano(i * dt)
            inputCircles.circlesCenters = expected[i,1:4] + np.random.normal(0, 5*1E-2, 3)
            inputCircles.circlesRadii = [1]
            inputCircles.valid = 1
            inputCircles.uncertainty = [5.*1E-2, 0.,0.,
                                 0., 5.*1E-2, 0.,
                                 0., 0., 5.*1E-2]
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.circlesInMsgName,
                                                  inputCircles.getStructSize(),
                                                  unitTestSim.TotalSim.CurrentNanos,
                                                  inputCircles)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i + 1)*dt))
        unitTestSim.ExecuteSimulation()

    stateLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".state", range(len(state)))
    stateErrorLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".stateError", range(len(state)))
    postFitLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".postFitRes", range(3))
    covarLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".covar", range(len(state) * len(state)))

    diff = np.copy(stateLog)
    diff[:,1:]-=expected[:,1:]
    FilterPlots.EnergyPlot(time, energy, 'Update', show_plots)
    FilterPlots.StateCovarPlot(stateLog, covarLog, 'Update', show_plots)
    FilterPlots.StatePlot(diff, 'Update', show_plots)
    FilterPlots.plot_TwoOrbits(expected[:,0:4], stateLog[:,0:4])
    FilterPlots.PostFitResiduals(postFitLog, np.sqrt(5*1E-2*1E6), 'Update', show_plots)

    for i in range(len(state)):
        if (covarLog[t1*multT1, i * len(state) + 1 + i] > covarLog[0, i * len(state) + 1 + i] / 100):
            testFailCount += 1
            testMessages.append("Covariance update failure at " + str(t1*multT1))

    if (np.linalg.norm(diff[-1, 1:]/expected[-1,1:]) > 1.0E-1):
        testFailCount += 1
        testMessages.append("State propagation failure")

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " state update"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


def StatePropRelOD(show_plots):
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
    state = [250, 32000, 1000, 5, 3, 2, 1, 1, 1]
    dt = 1
    testProcessRate = macros.sec2nano(dt)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = pixelLineBiasUKF.PixelLineBiasUKFConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "relodSuKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)
    setupFilterData(moduleConfig)

    # Create the input messages.
    inputCamera = pixelLineBiasUKF.CameraConfigMsg()
    inputAtt = pixelLineBiasUKF.NavAttIntMsg()

    # Set camera
    inputCamera.focalLength = 1.
    inputCamera.sensorSize = [10, 10]  # In mm
    inputCamera.resolution = [512, 512]
    inputCamera.sigma_CB = [1., 0.3, 0.1]
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.cameraConfigMsgName, inputCamera)

    # Set attitude
    inputAtt.sigma_BN = [0.6, 1., 0.1]
    unitTestSupport.setMessage(unitTestSim.TotalSim, unitProcessName, moduleConfig.attInMsgName, inputAtt)

    unitTestSim.TotalSim.logThisMessage('relod_filter_data', testProcessRate)

    timeSim = 60
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.min2nano(timeSim))
    unitTestSim.ExecuteSimulation()

    time = np.linspace(0,timeSim*60,timeSim*60/dt+1)
    dydt = np.zeros(len(state))
    energy = np.zeros(len(time))
    expected=np.zeros([len(time), len(state)+1])
    expected[0,1:] = moduleConfig.stateInit
    mu = 42828.314*1E9
    energy[0] = -mu/(2*orbitalMotion.rv2elem(mu, expected[0,1:4], expected[0,4:7]).a)
    expected = rk4(twoBodyGrav, time, moduleConfig.stateInit)
    for i in range(1, len(time)):
        energy[i] = - mu / (2 * orbitalMotion.rv2elem(mu, expected[i, 1:4], expected[i, 4:7]).a)

    stateLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".state", range(len(state)))
    covarLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".covar", range(len(state) * len(state)))

    diff = np.copy(stateLog)
    diff[:,1:]-=expected[:,1:]
    FilterPlots.plot_TwoOrbits(expected[:,0:4], stateLog[:,0:4])
    FilterPlots.EnergyPlot(time, energy, 'Prop', show_plots)
    FilterPlots.StateCovarPlot(stateLog, covarLog, 'Prop', show_plots)
    FilterPlots.StatePlot(diff, 'Prop', show_plots)

    if (np.linalg.norm(diff[-1,1:]/expected[-1,1:]) > 1.0E-10):
        testFailCount += 1
        testMessages.append("State propagation failure")

    if (energy[0] - energy[-1])/energy[0] > 1.0E-10:
        testFailCount += 1
        testMessages.append("State propagation failure")

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " state propagation"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    relOD_method_test(True)
    # StatePropRelOD(True)
    # StateUpdateRelOD(True)
