
# ISC License
#
# Copyright (c) 2023, Laboratory  for Atmospheric and Space Physics, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.



import positionODuKF_test_utilities as FilterPlots
import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import positionODuKF  # import the module that is to be tested
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion


def addTimeColumn(time, data):
    return np.transpose(np.vstack([[time], np.transpose(data)]))

def rk4(f, t, x0, arg = None):
    if arg is not None:
        functionArg = arg
    x = np.zeros([len(t),len(x0)+1])
    h = (t[len(t)-1] - t[0])/len(t)
    x[0,0] = t[0]
    x[0,1:] = x0
    for i in range(len(t)-1):
        h = t[i+1] - t[i]
        x[i,0] = t[i]
        k1 = h * f(t[i], x[i,1:], functionArg)
        k2 = h * f(t[i] + 0.5 * h, x[i,1:] + 0.5 * k1, functionArg)
        k3 = h * f(t[i] + 0.5 * h, x[i,1:] + 0.5 * k2, functionArg)
        k4 = h * f(t[i] + h, x[i,1:] + k3, functionArg)
        x[i+1,1:] = x[i,1:] + (k1 + 2.*k2 + 2.*k3 + k4) / 6.
        x[i+1,0] = t[i+1]
    return x

def twoBodyGrav(t, x, mu = 42828.314*1E9):
    dxdt = np.zeros(np.shape(x))
    dxdt[0:3] = x[3:]
    dxdt[3:] = -mu/np.linalg.norm(x[0:3])**3.*x[0:3]
    return dxdt


def setupFilterData(filterObject):

    filterObject.alpha = 0.02
    filterObject.beta = 2.0

    filterObject.muCentral = 42828.314*1E9
    elementsInit = orbitalMotion.ClassicElements()
    elementsInit.a = 4000*1E3 #m
    elementsInit.e = 0.2
    elementsInit.i = 10
    elementsInit.Omega = 0.001
    elementsInit.omega = 0.01
    elementsInit.f = 0.1
    r, v = orbitalMotion.elem2rv(filterObject.muCentral, elementsInit)
    states = r.tolist() + v.tolist()

    filterObject.stateInitial = [[s] for s in states]
    filterObject.covarInitial = [[1000.*1E6, 0.0, 0.0, 0.0, 0.0, 0.0],
                                 [0.0, 1000.*1E6, 0.0, 0.0, 0.0, 0.0],
                                 [0.0, 0.0, 1000.*1E6, 0.0, 0.0, 0.0],
                                 [0.0, 0.0, 0.0, 0.1*1E6, 0.0, 0.0],
                                 [0.0, 0.0, 0.0, 0.0, 0.1*1E6, 0.0],
                                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.1*1E6]]

    sigmaPos = (1E2)**2
    sigmaVel = (1E0)**2
    filterObject.processNoise = [[sigmaPos, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, sigmaPos, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, sigmaPos, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, sigmaVel, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, sigmaVel, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, sigmaVel]]
    filterObject.measNoiseScaling = 100*100


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

def test_propagation_kf(show_plots):
    """State and covariance propagation test"""
    statePropPositionOD(show_plots, 10.0)
def test_measurements_kf(show_plots):
    """Measurement model test"""
    stateUpdatePositionOD(show_plots)

def stateUpdatePositionOD(show_plots):
    __tracebackhide__ = True

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    dt = 1.0
    t1 = 250
    multT1 = 20
    measNoiseSD = 100 # m

    testProcessRate = macros.sec2nano(dt)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = positionODuKF.PositionODuKF()

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleConfig)
    setupFilterData(moduleConfig)
    moduleConfig.measNoiseSD = measNoiseSD

    time = np.linspace(0, int(multT1*t1), int(multT1*t1//dt)+1)
    energy = np.zeros(len(time))
    expected=np.zeros([len(time), 7])
    expected[0,1:] = np.array(moduleConfig.stateInitial).reshape([6,])
    energy[0] = -moduleConfig.muCentral/(2*orbitalMotion.rv2elem(moduleConfig.muCentral, expected[0,1:4], expected[0,4:]).a)

    kick = np.array([0., 0., 0., -0.01, 0.01, 0.02]) * 10 * 1E3

    expected[0:t1,:] = rk4(twoBodyGrav, time[0:t1], expected[0,1:], arg = moduleConfig.muCentral)
    expected[t1:multT1*t1+1, :] = rk4(twoBodyGrav, time[t1:len(time)], expected[t1-1, 1:] + kick, arg = moduleConfig.muCentral)
    for i in range(1, len(time)):
        energy[i] = - moduleConfig.muCentral / (2 * orbitalMotion.rv2elem(moduleConfig.muCentral, expected[i, 1:4], expected[i, 4:]).a)

    inputData = messaging.CameraLocalizationMsgPayload()
    cameraPosMsg = messaging.CameraLocalizationMsg()
    moduleConfig.cameraPosMsg.subscribeTo(cameraPosMsg)
    inputData.cameraPos_N = expected[0, 1:4]

    dataLog = moduleConfig.opNavFilterMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)
    unitTestSim.InitializeSimulation()
    for i in range(t1):
        if i > 0 and i % 10 == 0:
            inputData.timeTag = macros.sec2nano(i * dt)
            inputData.cameraPos_N = (expected[i,1:4] + np.random.normal(0, measNoiseSD, 3))
            inputData.valid = True
            cameraPosMsg.write(inputData, unitTestSim.TotalSim.CurrentNanos)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        unitTestSim.ExecuteSimulation()

    covarLog = addTimeColumn(dataLog.times(), dataLog.covar)

    # Propgate after the kick
    for i in range(t1, 2*t1):
        unitTestSim.ConfigureStopTime(macros.sec2nano((i + 1)*dt))
        unitTestSim.ExecuteSimulation()

    # Start ingesting measurements again after the kick
    for i in range(t1*2, multT1*t1):
        if i % 50 == 0:
            inputData.timeTag = macros.sec2nano(i * dt)
            inputData.cameraPos_N = (expected[i,1:4] + np.random.normal(0, measNoiseSD, 3))
            inputData.valid = True
            cameraPosMsg.write(inputData, unitTestSim.TotalSim.CurrentNanos)
        unitTestSim.ConfigureStopTime(macros.sec2nano((i + 1)*dt))
        unitTestSim.ExecuteSimulation()

    stateLog = addTimeColumn(dataLog.times(), dataLog.state)
    postFitLog = addTimeColumn(dataLog.times(), dataLog.postFitRes)
    covarLog = addTimeColumn(dataLog.times(), dataLog.covar)

    expected[:,0] *= 1E9
    diff = np.copy(stateLog)
    diff[:,1:] -= expected[:,1:]
    diffRel = np.copy(diff)
    diffRel[:,1:] /= expected[:,1:]
    FilterPlots.EnergyPlot(time, energy, 'Update', show_plots)
    FilterPlots.StateCovarPlot(stateLog, covarLog, 'Update', show_plots)
    FilterPlots.StatePlot(diff, 'Update', show_plots)
    FilterPlots.plot_TwoOrbits(expected[:,0:4], stateLog[:,0:4], show_plots)
    FilterPlots.PostFitResiduals(postFitLog, measNoiseSD, 'Update', show_plots)

    np.testing.assert_array_less(np.linalg.norm(covarLog[t1, 1:]),
                                 np.linalg.norm(covarLog[0, 1:]),
                                 err_msg=('Covariance must decrease during first measurement arc'),
                                 verbose=True)

    np.testing.assert_allclose(np.linalg.norm(stateLog[:,1:4]),
                               np.linalg.norm(expected[:,1:4]),
                               rtol = 0.01,
                               err_msg=('Position output must match expected values'),
                               verbose=True)
    np.testing.assert_allclose(np.linalg.norm(stateLog[:,4:]),
                               np.linalg.norm(expected[:,4:]),
                               rtol = 0.01,
                               err_msg=('Velocity output must match expected values'),
                               verbose=True)
    np.testing.assert_array_less(np.linalg.norm(covarLog[t1*multT1, 1:]),
                                 np.linalg.norm(covarLog[0, 1:])/10,
                                 err_msg=('Covariance must decrease during second measurement arc'),
                                 verbose=True)
    return


def statePropPositionOD(show_plots, dt):
    __tracebackhide__ = True

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(dt)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    moduleConfig = positionODuKF.PositionODuKF()

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, moduleConfig)

    setupFilterData(moduleConfig)
    moduleConfig.measNoiseScaling = 1

    dataLog = moduleConfig.opNavFilterMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    cameraPosMsg = messaging.CameraLocalizationMsg()
    moduleConfig.cameraPosMsg.subscribeTo(cameraPosMsg)

    timeSim = 60
    time = np.linspace(0, int(timeSim*60), int(timeSim*60//dt)+1)
    dydt = np.zeros(6)
    energy = np.zeros(len(time))
    expected = np.zeros([len(time), 7])
    expected[0,1:] = np.array(moduleConfig.stateInitial).reshape([6,])
    energy[0] = -moduleConfig.muCentral/(2*orbitalMotion.rv2elem(moduleConfig.muCentral, expected[0,1:4], expected[0,4:]).a)
    expected = rk4(twoBodyGrav, time, expected[0,1:], arg = moduleConfig.muCentral)
    for i in range(1, len(time)):
        energy[i] = - moduleConfig.muCentral / (2 * orbitalMotion.rv2elem(moduleConfig.muCentral, expected[i, 1:4], expected[i, 4:]).a)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.min2nano(timeSim))
    unitTestSim.ExecuteSimulation()

    stateLog = addTimeColumn(dataLog.times(), dataLog.state)
    covarLog = addTimeColumn(dataLog.times(), dataLog.covar)

    expected[:,0] *= 1E9
    diff = np.copy(stateLog)
    diff[:,1:] -= expected[:,1:]
    FilterPlots.plot_TwoOrbits(expected[:,0:4], stateLog[:,0:4], show_plots)
    FilterPlots.EnergyPlot(time, energy, 'Prop', show_plots)
    FilterPlots.StateCovarPlot(stateLog, covarLog, 'Prop', show_plots)
    FilterPlots.StatePlot(diff, 'Prop', show_plots)

    np.testing.assert_allclose(stateLog,
                               expected,
                               atol = 1E-10,
                               err_msg=('State error'),
                               verbose=True)
    np.testing.assert_array_less(np.linalg.norm(5*covarLog[0, 1:]),
                                 np.linalg.norm(covarLog[-1, 1:]),
                                 err_msg=('Covariance doesn\'t increase'),
                                 verbose=True)
    np.testing.assert_allclose(energy,
                               energy[0],
                               rtol = 1E-5,
                               err_msg=('Energy doesn\'t conserve'),
                               verbose=True)
    return


if __name__ == "__main__":
    stateUpdatePositionOD(True)
