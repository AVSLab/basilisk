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

from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion
import matplotlib.pyplot as plt
from Basilisk.fswAlgorithms import relativeODuKF, fswMessages  # import the module that is to be tested

import relativeODuKF_test_utilities as FilterPlots


import math
import numpy as np

def rk4(f, t, x0):
    x = np.zeros([len(t),len(x0)+1])
    h = (t[len(t)-1] - t[0])/len(t)
    x[0,1:] = x0
    for i in range(len(t)-1):
        x[i,0] = t[i]
        k1 = h * f(t[i], x[i,1:])
        k2 = h * f(t[i] + 0.5 * h, x[i,1:] + 0.5 * k1)
        k3 = h * f(t[i] + 0.5 * h, x[i,1:] + 0.5 * k2)
        k4 = h * f(t[i] + h, x[i,1:] + k3)
        x[i+1,1:] = x[i,1:] + (k1 + 2.*k2 + 2.*k3 + k4) / 6.
    return x

def twoBodyGrav(t, x, mu = 42828.314):
    dxdt = np.zeros(np.shape(x))
    dxdt[0:3] = x[3:]
    dxdt[3:] = -mu/np.linalg.norm(x[0:3])**3.*x[0:3]
    return dxdt


def setupFilterData(filterObject):
    filterObject.navStateOutMsgName = "relod_state_estimate"
    filterObject.filtDataOutMsgName = "relod_filter_data"
    filterObject.opNavInMsgName = "reold_opnav_meas"

    filterObject.planetIdInit = 2
    filterObject.alpha = 0.02
    filterObject.beta = 2.0
    filterObject.kappa = 0.0

    mu = 42828.314
    elementsInit = orbitalMotion.ClassicElements()
    elementsInit.a = 4000
    elementsInit.e = 0.2
    elementsInit.i = 10
    elementsInit.Omega = 0.001
    elementsInit.omega = 0.01
    elementsInit.f = 0.1
    r, v = orbitalMotion.elem2rv(mu, elementsInit)

    filterObject.stateInit = r.tolist() + v.tolist()
    filterObject.covarInit = [10000, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 10000., 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 10000., 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 5., 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 5., 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 5.]

    qNoiseIn = np.identity(6)
    qNoiseIn[0:3, 0:3] = qNoiseIn[0:3, 0:3]*0.00001*0.00001
    qNoiseIn[3:6, 3:6] = qNoiseIn[3:6, 3:6]*0.0001*0.0001
    filterObject.qNoise = qNoiseIn.reshape(36).tolist()

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

def test_utilitites_kf(show_plots):
    [testResults, testMessage] = relOD_utilities_test(show_plots)
    assert testResults < 1, testMessage
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

    state = [250, 32000, 1000, 5, 3, 2]
    dt = 10
    mu = 42828.314
    # Measurement Model Test
    data = relativeODuKF.RelODuKFConfig()
    msg = relativeODuKF.OpnavFswMsg()
    msg.r_N = [300, 200, 100]
    data.planetId = 2
    data.opNavInMsg = msg
    data.countHalfSPs = 6

    Covar = np.eye(6)
    SPexp = np.zeros([6, 2*6+1])
    SPexp[:,0] = np.array(state)
    for i in range(1, 6+1):
        SPexp[:,i] = np.array(state) + Covar[:,i-1]
        SPexp[:, i+6] = np.array(state) - Covar[:,i-1]


    data.SP =  np.transpose(SPexp).flatten().tolist()
    relativeODuKF.relODuKFMeasModel(data)

    measurements = data.yMeas

    if np.linalg.norm(np.array(measurements) - np.transpose(SPexp[0:3,:]).flatten()) > 1.0E-15:
        testFailCount += 1
        testMessages.append("Measurement Model Failure")

    # Dynamics Model Test
    data.planetId = 2

    stateIn = relativeODuKF.new_doubleArray(6)
    for i in range(len(state)):
        relativeODuKF.doubleArray_setitem(stateIn, i, state[i])

    relativeODuKF.relODStateProp(data, stateIn, dt)

    propedState = []
    for i in range(6):
        propedState.append(relativeODuKF.doubleArray_getitem(stateIn, i))

    dydt = np.zeros(6)
    dydt[0:3] = state[3:6]
    dydt[3:6] = -mu/np.linalg.norm(state[0:3])**3.*np.array(state[0:3])

    expected = np.array(state) + dt*dydt

    if np.linalg.norm(np.array(propedState) - expected) > 1.0E-15:
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
    t1 = 500

    testProcessRate = macros.sec2nano(dt)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = relativeODuKF.RelODuKFConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "relodSuKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    setupFilterData(moduleConfig)
    unitTestSim.TotalSim.logThisMessage('relod_filter_data', testProcessRate)

    time = np.linspace(0,2*t1,2*t1/dt+1)
    dydt = np.zeros(6)
    energy = np.zeros(len(time))
    expected=np.zeros([len(time), 7])
    expected[0,1:] = moduleConfig.stateInit
    mu = 42828.314
    energy[0] = -mu/(2*orbitalMotion.rv2elem(mu, expected[0,1:4], expected[0,4:]).a)


    for i in range(1, len(time)):
        dydt[0:3] = expected[i-1,4:]
        if time[i] == t1:
            kick = np.array([-0.01, 0.01, 0.02])*10
            dydt[3:6] = - mu / np.linalg.norm(expected[i-1,1:4])**3.*np.array(expected[i-1,1:4]) + kick
        else:
            dydt[3:6] = - mu / np.linalg.norm(expected[i - 1, 1:4]) ** 3. * np.array(expected[i - 1, 1:4])
        expected[i,0] = time[i]*1E9
        expected[i,1:] = expected[i-1,1:] + (time[i]-time[i-1])*dydt
        energy[i] = - mu / (2 * orbitalMotion.rv2elem(mu, expected[i, 1:4], expected[i, 4:]).a)

    inputData = relativeODuKF.OpnavFswMsg()
    inputMessageSize = inputData.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          moduleConfig.opNavInMsgName,
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)

    inputData.planetID = 2
    inputData.r_B = expected[0,1:4]

    unitTestSim.InitializeSimulation()
    for i in range(t1):
        if i > 0 and i % 50 == 0:
            inputData.timeTag = macros.sec2nano(i * dt)
            inputData.r_N = expected[i,1:4] #+ np.random.normal(0, 10, 3)
            inputData.covar_N = [10., 0.,0.,
                                 0., 10., 0.,
                                 0., 0., 10.]
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.opNavInMsgName,
                                                  inputMessageSize,
                                                  unitTestSim.TotalSim.CurrentNanos,
                                                  inputData)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        unitTestSim.ExecuteSimulation()

    stateLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".state", range(6))
    postFitLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".postFitRes", range(3))
    covarLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".covar", range(6 * 6))

    for i in range(5):
        # check covariance immediately after measurement is taken,
        # ensure order of magnitude less than initial covariance.
        if (covarLog[t1, i * 5 + 1 + i] > covarLog[0, i * 5 + 1 + i] / 10):
            testFailCount += 1
            testMessages.append("Covariance update failure")
        if (abs(stateLog[-1, i + 1] - expected[t1, i+1]) > 1.0E-1):
            testFailCount += 1
            testMessages.append("State update failure")

    for i in range(t1, 2*t1):
        if i % 50 == 0:
            inputData.timeTag = macros.sec2nano(i * dt + 1)
            inputData.r_N = expected[i,1:4] #+  np.random.normal(0, 10, 3)
            inputData.covar_N = [10., 0.,0.,
                                 0., 10., 0.,
                                 0., 0., 10.]
            unitTestSim.TotalSim.WriteMessageData(moduleConfig.opNavInMsgName,
                                                  inputMessageSize,
                                                  unitTestSim.TotalSim.CurrentNanos,
                                                  inputData)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i + 1) *dt))
        unitTestSim.ExecuteSimulation()

    stateLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".state", range(6))
    stateErrorLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".stateError", range(6))
    postFitLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".postFitRes", range(3))
    covarLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".covar", range(6 * 6))

    diff = np.copy(expected)
    diff[:,1:]-=stateLog[:,1:]
    FilterPlots.EnergyPlot(time, energy, 'Update', show_plots)
    FilterPlots.StateCovarPlot(stateLog, covarLog, 'Update', show_plots)
    FilterPlots.StatePlot(diff, 'Update', show_plots)
    FilterPlots.PostFitResiduals(postFitLog, 100, 'Update', show_plots)

    for i in range(5):
        if (covarLog[t1*2, i * 5 + 1 + i] > covarLog[0, i * 5 + 1 + i] / 10):
            testFailCount += 1
            testMessages.append("Covariance update failure")
        if (abs(stateLog[-1, i + 1] - expected[-1,i+1]) > 1.0E-1):
            testFailCount += 1
            testMessages.append("State update failure")

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
    dt = 5
    testProcessRate = macros.sec2nano(dt)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = relativeODuKF.RelODuKFConfig()
    moduleWrap = unitTestSim.setModelDataWrap(moduleConfig)
    moduleWrap.ModelTag = "relodSuKF"

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleWrap, moduleConfig)

    setupFilterData(moduleConfig)
    unitTestSim.TotalSim.logThisMessage('relod_filter_data', testProcessRate)

    timeSim = 60
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.min2nano(timeSim))
    unitTestSim.ExecuteSimulation()

    time = np.linspace(0,timeSim*60,timeSim*60/dt+1)
    dydt = np.zeros(6)
    energy = np.zeros(len(time))
    expected=np.zeros([len(time), 7])
    expected[0,1:] = moduleConfig.stateInit
    mu = 42828.314
    energy[0] = -mu/(2*orbitalMotion.rv2elem(mu, expected[0,1:4], expected[0,4:]).a)
    energyRK4 = np.copy(energy)

    expRK4 = rk4(twoBodyGrav, time, moduleConfig.stateInit)
    for i in range(1, len(time)):
        dydt[0:3] = expected[i-1,4:]
        dydt[3:6] = - mu / np.linalg.norm(expected[i-1,1:4])**3.*np.array(expected[i-1,1:4])
        expected[i,0] = time[i]*1E9
        expected[i,1:] = expected[i-1,1:] + (time[i]-time[i-1])*dydt
        energy[i] = - mu / (2 * orbitalMotion.rv2elem(mu, expected[i, 1:4], expected[i, 4:]).a)
        energyRK4[i] = - mu / (2 * orbitalMotion.rv2elem(mu, expRK4[i, 1:4], expRK4[i, 4:]).a)

    stateLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".state", range(6))
    postFitLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".postFitRes", range(3))
    covarLog = unitTestSim.pullMessageLogData('relod_filter_data' + ".covar", range(6 * 6))

    diff = np.copy(expected)
    diff[:,1:]-=stateLog[:,1:]
    FilterPlots.EnergyPlot(time, energyRK4, 'Prop', show_plots)
    FilterPlots.EnergyPlot(time, energy, 'Prop', show_plots)
    FilterPlots.StateCovarPlot(stateLog, covarLog, 'Prop', show_plots)
    FilterPlots.StatePlot(diff, 'Prop', show_plots)
    FilterPlots.PostFitResiduals(postFitLog, 100, 'Prop', show_plots)

    if (np.linalg.norm(diff[-1,1:]) > 1.0E-10):
        testFailCount += 1
        testMessages.append("State propagation failure")

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + moduleWrap.ModelTag + " state propagation"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


def relOD_utilities_test(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    # Initialize the test module configuration data
    AMatrix = [0.488894, 0.888396, 0.325191, 0.319207,
               1.03469, -1.14707, -0.754928, 0.312859,
               0.726885, -1.06887, 1.3703, -0.86488,
               -0.303441, -0.809499, -1.71152, -0.0300513,
               0.293871, -2.94428, -0.102242, -0.164879,
               -0.787283, 1.43838, -0.241447, 0.627707]

    RVector = relativeODuKF.new_doubleArray(len(AMatrix))
    AVector = relativeODuKF.new_doubleArray(len(AMatrix))
    for i in range(len(AMatrix)):
        relativeODuKF.doubleArray_setitem(AVector, i, AMatrix[i])
        relativeODuKF.doubleArray_setitem(RVector, i, 0.0)

        relativeODuKF.ukfQRDJustR(AVector, 6, 4, RVector)
    RMatrix = []
    for i in range(4 * 4):
        RMatrix.append(relativeODuKF.doubleArray_getitem(RVector, i))
    RBaseNumpy = np.array(RMatrix).reshape(4, 4)
    AMatNumpy = np.array(AMatrix).reshape(6, 4)
    q, r = np.linalg.qr(AMatNumpy)
    for i in range(r.shape[0]):
        if r[i, i] < 0.0:
            r[i, :] *= -1.0
    if np.linalg.norm(r - RBaseNumpy) > 1.0E-15:
        testFailCount += 1
        testMessages.append("QR Decomposition accuracy failure")

    AMatrix = [1.09327, 1.10927, -0.863653, 1.32288,
               -1.21412, -1.1135, -0.00684933, -2.43508,
               -0.769666, 0.371379, -0.225584, -1.76492,
               -1.08906, 0.0325575, 0.552527, -1.6256,
               1.54421, 0.0859311, -1.49159, 1.59683]

    RVector = relativeODuKF.new_doubleArray(len(AMatrix))
    AVector = relativeODuKF.new_doubleArray(len(AMatrix))
    for i in range(len(AMatrix)):
        relativeODuKF.doubleArray_setitem(AVector, i, AMatrix[i])
        relativeODuKF.doubleArray_setitem(RVector, i, 0.0)

    relativeODuKF.ukfQRDJustR(AVector, 5, 4, RVector)
    RMatrix = []
    for i in range(4 * 4):
        RMatrix.append(relativeODuKF.doubleArray_getitem(RVector, i))
    RBaseNumpy = np.array(RMatrix).reshape(4, 4)
    AMatNumpy = np.array(AMatrix).reshape(5, 4)
    q, r = np.linalg.qr(AMatNumpy)
    for i in range(r.shape[0]):
        if r[i, i] < 0.0:
            r[i, :] *= -1.0
    if np.linalg.norm(r - RBaseNumpy) > 1.0E-14:
        testFailCount += 1
        testMessages.append("QR Decomposition accuracy failure")

    AMatrix = [0.2236, 0,
               0, 0.2236,
               -0.2236, 0,
               0, -0.2236,
               0.0170, 0,
               0, 0.0170]

    RVector = relativeODuKF.new_doubleArray(len(AMatrix))
    AVector = relativeODuKF.new_doubleArray(len(AMatrix))
    for i in range(len(AMatrix)):
        relativeODuKF.doubleArray_setitem(AVector, i, AMatrix[i])
        relativeODuKF.doubleArray_setitem(RVector, i, 0.0)

    relativeODuKF.ukfQRDJustR(AVector, 6, 2, RVector)
    RMatrix = []
    for i in range(2 * 2):
        RMatrix.append(relativeODuKF.doubleArray_getitem(RVector, i))
    RBaseNumpy = np.array(RMatrix).reshape(2, 2)
    AMatNumpy = np.array(AMatrix).reshape(6, 2)
    q, r = np.linalg.qr(AMatNumpy)
    for i in range(r.shape[0]):
        if r[i, i] < 0.0:
            r[i, :] *= -1.0

    if np.linalg.norm(r - RBaseNumpy) > 1.0E-15:
        testFailCount += 1
        testMessages.append("QR Decomposition accuracy failure")

    LUSourceMat = [8, 1, 6, 3, 5, 7, 4, 9, 2]
    LUSVector = relativeODuKF.new_doubleArray(len(LUSourceMat))
    LVector = relativeODuKF.new_doubleArray(len(LUSourceMat))
    UVector = relativeODuKF.new_doubleArray(len(LUSourceMat))
    intSwapVector = relativeODuKF.new_intArray(3)

    for i in range(len(LUSourceMat)):
        relativeODuKF.doubleArray_setitem(LUSVector, i, LUSourceMat[i])
        relativeODuKF.doubleArray_setitem(UVector, i, 0.0)
        relativeODuKF.doubleArray_setitem(LVector, i, 0.0)

    exCount = relativeODuKF.ukfLUD(LUSVector, 3, 3, LVector, intSwapVector)
    # relativeODuKF.ukfUInv(LUSVector, 3, 3, UVector)
    LMatrix = []
    UMatrix = []
    # UMatrix = []
    for i in range(3):
        currRow = relativeODuKF.intArray_getitem(intSwapVector, i)
        for j in range(3):
            if (j < i):
                LMatrix.append(relativeODuKF.doubleArray_getitem(LVector, i * 3 + j))
                UMatrix.append(0.0)
            elif (j > i):
                LMatrix.append(0.0)
                UMatrix.append(relativeODuKF.doubleArray_getitem(LVector, i * 3 + j))
            else:
                LMatrix.append(1.0)
                UMatrix.append(relativeODuKF.doubleArray_getitem(LVector, i * 3 + j))
    # UMatrix.append(relativeODuKF.doubleArray_getitem(UVector, i))

    LMatrix = np.array(LMatrix).reshape(3, 3)
    UMatrix = np.array(UMatrix).reshape(3, 3)
    outMat = np.dot(LMatrix, UMatrix)
    outMatSwap = np.zeros((3, 3))
    for i in range(3):
        currRow = relativeODuKF.intArray_getitem(intSwapVector, i)
        outMatSwap[i, :] = outMat[currRow, :]
        outMat[currRow, :] = outMat[i, :]
    LuSourceArray = np.array(LUSourceMat).reshape(3, 3)

    if (np.linalg.norm(outMatSwap - LuSourceArray) > 1.0E-14):
        testFailCount += 1
        testMessages.append("LU Decomposition accuracy failure")

    EqnSourceMat = [2.0, 1.0, 3.0, 2.0, 6.0, 8.0, 6.0, 8.0, 18.0]
    BVector = [1.0, 3.0, 5.0]
    EqnVector = relativeODuKF.new_doubleArray(len(EqnSourceMat))
    EqnBVector = relativeODuKF.new_doubleArray(len(LUSourceMat) / 3)
    EqnOutVector = relativeODuKF.new_doubleArray(len(LUSourceMat) / 3)

    for i in range(len(EqnSourceMat)):
        relativeODuKF.doubleArray_setitem(EqnVector, i, EqnSourceMat[i])
        relativeODuKF.doubleArray_setitem(EqnBVector, i / 3, BVector[i / 3])
        relativeODuKF.intArray_setitem(intSwapVector, i / 3, 0)
        relativeODuKF.doubleArray_setitem(LVector, i, 0.0)

    exCount = relativeODuKF.ukfLUD(EqnVector, 3, 3, LVector, intSwapVector)

    relativeODuKF.ukfLUBckSlv(LVector, 3, 3, intSwapVector, EqnBVector, EqnOutVector)

    expectedSol = [3.0 / 10.0, 4.0 / 10.0, 0.0]
    errorVal = 0.0
    for i in range(3):
        errorVal += abs(relativeODuKF.doubleArray_getitem(EqnOutVector, i) - expectedSol[i])

    if (errorVal > 1.0E-14):
        testFailCount += 1
        testMessages.append("LU Back-Solve accuracy failure")

    InvSourceMat = [8, 1, 6, 3, 5, 7, 4, 9, 2]
    SourceVector = relativeODuKF.new_doubleArray(len(InvSourceMat))
    InvVector = relativeODuKF.new_doubleArray(len(InvSourceMat))
    for i in range(len(InvSourceMat)):
        relativeODuKF.doubleArray_setitem(SourceVector, i, InvSourceMat[i])
        relativeODuKF.doubleArray_setitem(InvVector, i, 0.0)
    nRow = int(math.sqrt(len(InvSourceMat)))
    relativeODuKF.ukfMatInv(SourceVector, nRow, nRow, InvVector)

    InvOut = []
    for i in range(len(InvSourceMat)):
        InvOut.append(relativeODuKF.doubleArray_getitem(InvVector, i))

    InvOut = np.array(InvOut).reshape(nRow, nRow)
    expectIdent = np.dot(InvOut, np.array(InvSourceMat).reshape(3, 3))
    errorNorm = np.linalg.norm(expectIdent - np.identity(3))
    if (errorNorm > 1.0E-14):
        testFailCount += 1
        testMessages.append("LU Matrix Inverse accuracy failure")

    cholTestMat = [1.0, 0.0, 0.0, 0.0, 10.0, 5.0, 0.0, 5.0, 10.0]
    SourceVector = relativeODuKF.new_doubleArray(len(cholTestMat))
    CholVector = relativeODuKF.new_doubleArray(len(cholTestMat))
    for i in range(len(cholTestMat)):
        relativeODuKF.doubleArray_setitem(SourceVector, i, cholTestMat[i])
        relativeODuKF.doubleArray_setitem(CholVector, i, 0.0)
    nRow = int(math.sqrt(len(cholTestMat)))
    relativeODuKF.ukfCholDecomp(SourceVector, nRow, nRow, CholVector)
    cholOut = []
    for i in range(len(cholTestMat)):
        cholOut.append(relativeODuKF.doubleArray_getitem(CholVector, i))

    cholOut = np.array(cholOut).reshape(nRow, nRow)
    cholComp = np.linalg.cholesky(np.array(cholTestMat).reshape(nRow, nRow))
    errorNorm = np.linalg.norm(cholOut - cholComp)
    if (errorNorm > 1.0E-14):
        testFailCount += 1
        testMessages.append("Cholesky Matrix Decomposition accuracy failure")

    InvSourceMat = [2.1950926119414667, 0.0, 0.0, 0.0,
                    1.0974804773131115, 1.9010439702743847, 0.0, 0.0,
                    0.0, 1.2672359635912551, 1.7923572711881284, 0.0,
                    1.0974804773131113, -0.63357997864171967, 1.7920348101787789, 0.033997451205364251]

    SourceVector = relativeODuKF.new_doubleArray(len(InvSourceMat))
    InvVector = relativeODuKF.new_doubleArray(len(InvSourceMat))
    for i in range(len(InvSourceMat)):
        relativeODuKF.doubleArray_setitem(SourceVector, i, InvSourceMat[i])
        relativeODuKF.doubleArray_setitem(InvVector, i, 0.0)
    nRow = int(math.sqrt(len(InvSourceMat)))
    relativeODuKF.ukfLInv(SourceVector, nRow, nRow, InvVector)

    InvOut = []
    for i in range(len(InvSourceMat)):
        InvOut.append(relativeODuKF.doubleArray_getitem(InvVector, i))

    InvOut = np.array(InvOut).reshape(nRow, nRow)
    expectIdent = np.dot(InvOut, np.array(InvSourceMat).reshape(nRow, nRow))
    errorNorm = np.linalg.norm(expectIdent - np.identity(nRow))
    if (errorNorm > 1.0E-12):
        print errorNorm
        testFailCount += 1
        testMessages.append("L Matrix Inverse accuracy failure")

    InvSourceMat = np.transpose(np.array(InvSourceMat).reshape(nRow, nRow)).reshape(nRow * nRow).tolist()
    SourceVector = relativeODuKF.new_doubleArray(len(InvSourceMat))
    InvVector = relativeODuKF.new_doubleArray(len(InvSourceMat))
    for i in range(len(InvSourceMat)):
        relativeODuKF.doubleArray_setitem(SourceVector, i, InvSourceMat[i])
        relativeODuKF.doubleArray_setitem(InvVector, i, 0.0)
    nRow = int(math.sqrt(len(InvSourceMat)))
    relativeODuKF.ukfUInv(SourceVector, nRow, nRow, InvVector)

    InvOut = []
    for i in range(len(InvSourceMat)):
        InvOut.append(relativeODuKF.doubleArray_getitem(InvVector, i))

    InvOut = np.array(InvOut).reshape(nRow, nRow)
    expectIdent = np.dot(InvOut, np.array(InvSourceMat).reshape(nRow, nRow))
    errorNorm = np.linalg.norm(expectIdent - np.identity(nRow))
    if (errorNorm > 1.0E-12):
        print errorNorm
        testFailCount += 1
        testMessages.append("U Matrix Inverse accuracy failure")

    # If the argument provided at commandline "--show_plots" evaluates as true,
    # plot all figures
    if show_plots:
        plt.show()

    # print out success message if no error were found
    if testFailCount == 0:
        print "PASSED: " + " UKF utilities"

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    # test_all_relOD_kf(True)
    StatePropRelOD(True)
