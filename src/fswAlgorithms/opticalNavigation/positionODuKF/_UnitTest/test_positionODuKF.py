
# ISC License
#
# Copyright (c) 2024, University of Colorado at Boulder
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



import positionODuKF_test_utilities as filter_plots
import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import positionODuKF  # import the module that is to be tested
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion


def add_time_column(time, data):
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

def two_body_grav(t, x, mu = 42828.314*1E9):
    dxdt = np.zeros(np.shape(x))
    dxdt[0:3] = x[3:]
    dxdt[3:] = -mu/np.linalg.norm(x[0:3])**3.*x[0:3]
    return dxdt


def setup_filter_data(filter_object):

    filter_object.alphaParameter = 0.02
    filter_object.betaParameter = 2.0

    filter_object.muCentral = 42828.314*1E9
    elements_init = orbitalMotion.ClassicElements()
    elements_init.a = 4000*1E3 #m
    elements_init.e = 0.2
    elements_init.i = 10
    elements_init.Omega = 0.001
    elements_init.omega = 0.01
    elements_init.f = 0.1
    r, v = orbitalMotion.elem2rv(filter_object.muCentral, elements_init)
    states = r.tolist() + v.tolist()

    filter_object.stateInitial = [[s] for s in states]
    filter_object.covarInitial = [[1000.*1E6, 0.0, 0.0, 0.0, 0.0, 0.0],
                                 [0.0, 1000.*1E6, 0.0, 0.0, 0.0, 0.0],
                                 [0.0, 0.0, 1000.*1E6, 0.0, 0.0, 0.0],
                                 [0.0, 0.0, 0.0, 0.1*1E6, 0.0, 0.0],
                                 [0.0, 0.0, 0.0, 0.0, 0.1*1E6, 0.0],
                                 [0.0, 0.0, 0.0, 0.0, 0.0, 0.1*1E6]]

    sigmaPos = (1E2)**2
    sigmaVel = (1E0)**2
    filter_object.processNoise = [[sigmaPos, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, sigmaPos, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, sigmaPos, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, sigmaVel, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, sigmaVel, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, sigmaVel]]
    filter_object.measNoiseScaling = 100*100


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

def test_propagation_kf(show_plots):
    """State and covariance propagation test"""
    state_propagation_test(show_plots, 10.0)
def test_measurements_kf(show_plots):
    """Measurement model test"""
    state_update_test(show_plots)

def state_update_test(show_plots):
    __tracebackhide__ = True

    unit_task_name = "unit_task"
    unit_process_name = "TestProcess"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    dt = 1.0
    t1 = 250
    time_multiple = 20
    meas_noise_std_dev = 100 # m

    test_process_rate = macros.sec2nano(dt)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    module_config = positionODuKF.PositionODuKF()

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module_config)
    setup_filter_data(module_config)
    module_config.measNoiseSD = meas_noise_std_dev

    time = np.linspace(0, int(time_multiple*t1), int(time_multiple*t1//dt)+1)
    energy = np.zeros(len(time))
    expected=np.zeros([len(time), 7])
    expected[0,1:] = np.array(module_config.stateInitial).reshape([6,])
    energy[0] = -module_config.muCentral/(2*orbitalMotion.rv2elem(module_config.muCentral, expected[0,1:4], expected[0,4:]).a)

    kick = np.array([0., 0., 0., -0.01, 0.01, 0.02]) * 10 * 1E3

    expected[0:t1,:] = rk4(two_body_grav, time[0:t1], expected[0,1:], arg = module_config.muCentral)
    expected[t1:time_multiple*t1+1, :] = rk4(two_body_grav, time[t1:len(time)], expected[t1-1, 1:] + kick, arg = module_config.muCentral)
    for i in range(1, len(time)):
        energy[i] = - module_config.muCentral / (2 * orbitalMotion.rv2elem(module_config.muCentral, expected[i, 1:4], expected[i, 4:]).a)

    input_data = messaging.CameraLocalizationMsgPayload()
    camera_position_msg = messaging.CameraLocalizationMsg()
    module_config.cameraPosMsg.subscribeTo(camera_position_msg)
    input_data.cameraPos_N = expected[0, 1:4]

    filter_data_log = module_config.opNavFilterMsg.recorder()
    residual_data_log = module_config.opNavResidualMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, filter_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, residual_data_log)
    unit_test_sim.InitializeSimulation()
    for i in range(t1):
        if i > 0 and i % 10 == 0:
            input_data.timeTag = macros.sec2nano(i * dt)
            input_data.cameraPos_N = (expected[i,1:4] + np.random.normal(0, meas_noise_std_dev, 3))
            input_data.valid = True
            camera_position_msg.write(input_data, unit_test_sim.TotalSim.getCurrentNanos())
        unit_test_sim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        unit_test_sim.ExecuteSimulation()

    num_states = filter_data_log.numberOfStates[0]
    covar_log = add_time_column(filter_data_log.times(), filter_data_log.covar[:, :num_states**2])

    # Propgate after the kick
    for i in range(t1, 2*t1):
        unit_test_sim.ConfigureStopTime(macros.sec2nano((i + 1)*dt))
        unit_test_sim.ExecuteSimulation()

    # Start ingesting measurements again after the kick
    for i in range(t1*2, time_multiple*t1):
        if i % 50 == 0:
            input_data.timeTag = macros.sec2nano(i * dt)
            input_data.cameraPos_N = (expected[i,1:4] + np.random.normal(0, meas_noise_std_dev, 3))
            input_data.valid = True
            camera_position_msg.write(input_data, unit_test_sim.TotalSim.getCurrentNanos())
        unit_test_sim.ConfigureStopTime(macros.sec2nano((i + 1)*dt))
        unit_test_sim.ExecuteSimulation()

    state_log = add_time_column(filter_data_log.times(), filter_data_log.state[:, :num_states])
    covar_log = add_time_column(filter_data_log.times(), filter_data_log.covar[:, :num_states**2])
    number_obs = residual_data_log.numberOfObservations
    size_obs = residual_data_log.sizeOfObservations
    post_fit_log_sparse = add_time_column(residual_data_log.times(), residual_data_log.postFits)
    post_fit_log = np.zeros([len(residual_data_log.times()), np.max(size_obs)+1])
    post_fit_log[:, 0] = post_fit_log_sparse[:, 0]

    for i in range(len(number_obs)):
        if number_obs[i] > 0:
            post_fit_log[i, 1:size_obs[i]+1] = post_fit_log_sparse[i, 1:size_obs[i]+1]

    expected[:,0] *= 1E9
    diff = np.copy(state_log)
    diff[:,1:] -= expected[:,1:]
    diff_rel = np.copy(diff)
    diff_rel[:,1:] /= expected[:,1:]
    filter_plots.energy(time, energy, 'Update', show_plots)
    filter_plots.state_covar(state_log, covar_log, 'Update', show_plots)
    filter_plots.states(diff, 'Update', show_plots)
    filter_plots.two_orbits(expected[:,0:4], state_log[:,0:4], show_plots)
    filter_plots.post_fit_residuals(post_fit_log, meas_noise_std_dev, 'Update', show_plots)

    np.testing.assert_array_less(np.linalg.norm(covar_log[t1, 1:]),
                                 np.linalg.norm(covar_log[0, 1:]),
                                 err_msg=('Covariance must decrease during first measurement arc'),
                                 verbose=True)

    np.testing.assert_allclose(np.linalg.norm(state_log[:,1:4]),
                               np.linalg.norm(expected[:,1:4]),
                               rtol = 0.01,
                               err_msg=('Position output must match expected values'),
                               verbose=True)
    np.testing.assert_allclose(np.linalg.norm(state_log[:,4:]),
                               np.linalg.norm(expected[:,4:]),
                               rtol = 0.01,
                               err_msg=('Velocity output must match expected values'),
                               verbose=True)
    np.testing.assert_array_less(np.linalg.norm(covar_log[t1*time_multiple, 1:]),
                                 np.linalg.norm(covar_log[0, 1:])/10,
                                 err_msg=('Covariance must decrease during second measurement arc'),
                                 verbose=True)
    return


def state_propagation_test(show_plots, dt):
    __tracebackhide__ = True

    unit_task_name = "unit_task"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(dt)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    module_config = positionODuKF.PositionODuKF()

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module_config)

    setup_filter_data(module_config)
    module_config.measNoiseScaling = 1

    filter_data_log = module_config.opNavFilterMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, filter_data_log)

    camera_position_msg = messaging.CameraLocalizationMsg()
    module_config.cameraPosMsg.subscribeTo(camera_position_msg)

    sim_tim = 60
    time = np.linspace(0, int(sim_tim*60), int(sim_tim*60//dt)+1)
    dydt = np.zeros(6)
    energy = np.zeros(len(time))
    expected = np.zeros([len(time), 7])
    expected[0,1:] = np.array(module_config.stateInitial).reshape([6,])
    energy[0] = -module_config.muCentral/(2*orbitalMotion.rv2elem(module_config.muCentral, expected[0,1:4], expected[0,4:]).a)
    expected = rk4(two_body_grav, time, expected[0,1:], arg = module_config.muCentral)
    for i in range(1, len(time)):
        energy[i] = - module_config.muCentral / (2 * orbitalMotion.rv2elem(module_config.muCentral, expected[i, 1:4], expected[i, 4:]).a)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.min2nano(sim_tim))
    unit_test_sim.ExecuteSimulation()

    num_states = filter_data_log.numberOfStates[0]
    state_log = add_time_column(filter_data_log.times(), filter_data_log.state[:, :num_states])
    covar_log = add_time_column(filter_data_log.times(), filter_data_log.covar[:, :num_states**2])

    expected[:,0] *= 1E9
    diff = np.copy(state_log)
    diff[:,1:] -= expected[:,1:]
    filter_plots.two_orbits(expected[:,0:4], state_log[:,0:4], show_plots)
    filter_plots.energy(time, energy, 'Prop', show_plots)
    filter_plots.state_covar(state_log, covar_log, 'Prop', show_plots)
    filter_plots.states(diff, 'Prop', show_plots)

    np.testing.assert_allclose(state_log,
                               expected,
                               atol = 1E-10,
                               err_msg=('State error'),
                               verbose=True)
    np.testing.assert_array_less(np.linalg.norm(5*covar_log[0, 1:]),
                                 np.linalg.norm(covar_log[-1, 1:]),
                                 err_msg=('Covariance doesn\'t increase'),
                                 verbose=True)
    np.testing.assert_allclose(energy,
                               energy[0],
                               rtol = 1E-5,
                               err_msg=('Energy doesn\'t conserve'),
                               verbose=True)
    return


if __name__ == "__main__":
    state_update_test(True)
