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


import flybyODuKF_test_utilities as filter_plots
import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import flybyODuKF
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion


def add_time_column(time, data):
    return np.transpose(np.vstack([[time], np.transpose(data)]))


def rk4(f, t, x0):
    x = np.zeros([len(t), len(x0) + 1])
    h = (t[len(t) - 1] - t[0]) / len(t)
    x[0, 0] = t[0]
    x[0, 1:] = x0
    for i in range(len(t) - 1):
        h = t[i + 1] - t[i]
        x[i, 0] = t[i]
        k1 = h * f(t[i], x[i, 1:])
        k2 = h * f(t[i] + 0.5 * h, x[i, 1:] + 0.5 * k1)
        k3 = h * f(t[i] + 0.5 * h, x[i, 1:] + 0.5 * k2)
        k4 = h * f(t[i] + h, x[i, 1:] + k3)
        x[i + 1, 1:] = x[i, 1:] + (k1 + 2. * k2 + 2. * k3 + k4) / 6.
        x[i + 1, 0] = t[i + 1]
    return x


def two_body_gravity(t, x, mu=42828.314 * 1E9):
    dxdt = np.zeros(np.shape(x))
    dxdt[0:3] = x[3:]
    dxdt[3:] = -mu / np.linalg.norm(x[0:3]) ** 3. * x[0:3]
    return dxdt


def setup_filter_data(filter_object):
    filter_object.setAlpha(0.02)
    filter_object.setBeta(2.0)

    filter_object.setUnitConversionFromSItoState(1E-3)  # filter in km and km/s
    filter_object.setCentralBodyGravitationParameter(42828.314 * 1E9)
    orbital_elements = orbitalMotion.ClassicElements()
    orbital_elements.a = 4000 * 1E3  # m
    orbital_elements.e = 0.2
    orbital_elements.i = 10
    orbital_elements.Omega = 0.001
    orbital_elements.omega = 0.01
    orbital_elements.f = 0.1
    r, v = orbitalMotion.elem2rv(filter_object.getCentralBodyGravitationParameter(), orbital_elements)
    states = r.tolist() + v.tolist()

    filter_object.setInitialState([[s] for s in states])
    filter_object.setInitialCovariance([[1000. * 1E6, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 1000. * 1E6, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 1000. * 1E6, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.1 * 1E6, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.1 * 1E6, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.1 * 1E6]])

    sigmaPos = (1E-8) ** 2
    sigmaVel = (1E-10) ** 2
    filter_object.setProcessNoise([[sigmaPos, 0.0, 0.0, 0.0, 0.0, 0.0],
                                   [0.0, sigmaPos, 0.0, 0.0, 0.0, 0.0],
                                   [0.0, 0.0, sigmaPos, 0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0, sigmaVel, 0.0, 0.0],
                                   [0.0, 0.0, 0.0, 0.0, sigmaVel, 0.0],
                                   [0.0, 0.0, 0.0, 0.0, 0.0, sigmaVel]])


def test_propagation_kf(show_plots):
    """Module Unit Test"""
    state_propagation_flyby(show_plots, 10.0)


def test_measurements_kf(show_plots):
    """Module Unit Test"""
    state_update_flyby(show_plots)


def state_update_flyby(show_plots):
    unit_task_name = "unit_task"
    unit_process_name = "test_process"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    dt = 1.0
    t1 = 250
    multT1 = 8

    test_process_rate = macros.sec2nano(dt)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    flyby_module = flybyODuKF.FlybyODuKF()

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, flyby_module)

    setup_filter_data(flyby_module)

    filter_data_log = flyby_module.opNavFilterMsg.recorder()
    residual_data_log = flyby_module.opNavResidualMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, filter_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, residual_data_log)

    time = np.linspace(0, int(multT1 * t1), int(multT1 * t1 // dt) + 1)
    energy = np.zeros(len(time))
    expected = np.zeros([len(time), 7])
    expected[0, 1:] = np.array(flyby_module.getInitialState()).reshape([6, ])
    energy[0] = -flyby_module.getCentralBodyGravitationParameter() / (2 * orbitalMotion.rv2elem(
        flyby_module.getCentralBodyGravitationParameter(), expected[0, 1:4], expected[0, 4:]).a)

    kick = np.array([0., 0., 0., -0.01, 0.01, 0.02]) * 10 * 1E3

    expected[0:t1, :] = rk4(two_body_gravity, time[0:t1], expected[0, 1:])
    expected[t1:multT1 * t1 + 1, :] = rk4(two_body_gravity, time[t1:len(time)], expected[t1 - 1, 1:] + kick)
    for i in range(1, len(time)):
        energy[i] = - flyby_module.getCentralBodyGravitationParameter() / (2 * orbitalMotion.rv2elem(
            flyby_module.getCentralBodyGravitationParameter(), expected[i, 1:4], expected[i, 4:]).a)

    input_data = messaging.OpNavUnitVecMsgPayload()
    opnav_input_msg = messaging.OpNavUnitVecMsg()
    flyby_module.opNavHeadingMsg.subscribeTo(opnav_input_msg)
    input_data.rhat_BN_B = expected[0, 1:4] / np.linalg.norm(expected[0, 1:4])

    unit_test_sim.InitializeSimulation()
    for i in range(t1):
        if i > 0 and i % 10 == 0:
            input_data.timeTag = i * dt
            input_data.rhat_BN_N = (expected[i, 1:4] + np.random.normal(0, 100, 3))
            input_data.rhat_BN_N /= np.linalg.norm(input_data.rhat_BN_N)
            input_data.valid = True
            input_data.covar_N = [5. * 1E-5, 0., 0.,
                                  0., 5. * 1E-5, 0.,
                                  0., 0., 5. * 1E-5]
            opnav_input_msg.write(input_data, unit_test_sim.TotalSim.getCurrentNanos())
        unit_test_sim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        unit_test_sim.ExecuteSimulation()

    num_states = filter_data_log.numberOfStates[0]
    covariance_data_log = add_time_column(filter_data_log.times(), filter_data_log.covar[:, :num_states**2])

    np.testing.assert_array_less(np.diag(covariance_data_log[t1, 1:].reshape([6, 6]))[1:],
                                 np.diag(covariance_data_log[0, 1:].reshape([6, 6]))[1:],
                                 err_msg='middle covariance error',
                                 verbose=True)

    for i in range(t1, multT1 * t1):
        if i % 10 == 0:
            input_data.timeTag = i * dt
            input_data.rhat_BN_N = (expected[i, 1:4] + np.random.normal(0, 100, 3))
            input_data.rhat_BN_N /= np.linalg.norm(input_data.rhat_BN_N)
            input_data.valid = True
            input_data.covar_N = [5. * 1E-5, 0., 0.,
                                  0., 5. * 1E-5, 0.,
                                  0., 0., 5. * 1E-5]
            opnav_input_msg.write(input_data, unit_test_sim.TotalSim.getCurrentNanos())
        unit_test_sim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        unit_test_sim.ExecuteSimulation()

    state_data_log = add_time_column(filter_data_log.times(), filter_data_log.state[:, :num_states])
    covariance_data_log = add_time_column(filter_data_log.times(), filter_data_log.covar[:, :num_states**2])
    number_obs = residual_data_log.numberOfObservations
    size_obs = residual_data_log.sizeOfObservations
    post_fit_log_sparse = add_time_column(residual_data_log.times(), residual_data_log.postFits)
    post_fit_log = np.zeros([len(residual_data_log.times()), np.max(size_obs)+1])
    post_fit_log[:, 0] = post_fit_log_sparse[:, 0]
    pre_fit_log_sparse = add_time_column(residual_data_log.times(), residual_data_log.preFits)
    pre_fit_log = np.zeros([len(residual_data_log.times()), np.max(size_obs)+1])
    pre_fit_log[:, 0] = pre_fit_log_sparse[:, 0]

    for i in range(len(number_obs)):
        if number_obs[i] > 0:
            post_fit_log[i, 1:size_obs[i]+1] = post_fit_log_sparse[i, 1:size_obs[i]+1]
            pre_fit_log[i, 1:size_obs[i]+1] = pre_fit_log_sparse[i, 1:size_obs[i]+1]

    diff = np.copy(state_data_log)
    diff[:, 1:] -= expected[:, 1:]
    filter_plots.energy(time, energy, 'Update', show_plots)
    filter_plots.state_covar(state_data_log, covariance_data_log, 'Update', show_plots)
    filter_plots.states(diff, 'Update', show_plots)
    filter_plots.two_orbits(expected[:, 0:4], state_data_log[:, 0:4], show_plots)
    filter_plots.post_fit_residuals(post_fit_log, np.sqrt(1E-6), 'Update PreFit', show_plots)
    filter_plots.post_fit_residuals(pre_fit_log, np.sqrt(1E-6), 'Update PostFit', show_plots)

    np.testing.assert_array_less(np.diag(covariance_data_log[t1 * multT1, 1:].reshape([6, 6]))[1:],
                                 np.diag(covariance_data_log[0, 1:].reshape([6, 6]))[1:],
                                 err_msg='final covariance error',
                                 verbose=True)
    np.testing.assert_allclose(state_data_log[-1, 1:],
                               expected[-1, 1:],
                               rtol=1E-1,
                               err_msg='state propagation error',
                               verbose=True)

    return


def state_propagation_flyby(show_plots, dt):
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(dt)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    flyby_module = flybyODuKF.FlybyODuKF()

    # Add test module to runtime call list
    setup_filter_data(flyby_module)
    unit_test_sim.AddModelToTask(unit_task_name, flyby_module)

    filter_data_log = flyby_module.opNavFilterMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, filter_data_log)

    opnav_input_msg = messaging.OpNavUnitVecMsg()
    flyby_module.opNavHeadingMsg.subscribeTo(opnav_input_msg)

    sim_time = 60
    time = np.linspace(0, int(sim_time * 60), int(sim_time * 60 // dt) + 1)
    dydt = np.zeros(6)
    energy = np.zeros(len(time))
    expected = np.zeros([len(time), 7])
    expected[0, 1:] = np.array(flyby_module.getInitialState()).reshape([6, ])
    energy[0] = -flyby_module.getCentralBodyGravitationParameter() / (2 * orbitalMotion.rv2elem(
        flyby_module.getCentralBodyGravitationParameter(), expected[0, 1:4], expected[0, 4:]).a)
    expected = rk4(two_body_gravity, time, expected[0, 1:])
    for i in range(1, len(time)):
        energy[i] = - flyby_module.getCentralBodyGravitationParameter() / (2 * orbitalMotion.rv2elem(
            flyby_module.getCentralBodyGravitationParameter(), expected[i, 1:4], expected[i, 4:]).a)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.min2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    num_states = filter_data_log.numberOfStates[0]
    state_data_log = add_time_column(filter_data_log.times(), filter_data_log.state[:, :num_states])
    covariance_data_log = add_time_column(filter_data_log.times(), filter_data_log.covar[:, :num_states**2])

    diff = np.copy(state_data_log)
    diff[:, 1:] -= expected[:, 1:]
    filter_plots.two_orbits(expected[:, 0:4], state_data_log[:, 0:4], show_plots)
    filter_plots.energy(time, energy, 'Prop', show_plots)
    filter_plots.state_covar(state_data_log, covariance_data_log, 'Prop', show_plots)
    filter_plots.states(diff, 'Prop', show_plots)

    np.testing.assert_array_less(5 * np.linalg.norm(covariance_data_log[0, 1:]),
                                 np.linalg.norm(covariance_data_log[-1, 1:]),
                                 err_msg='covariance must increase without measurements',
                                 verbose=True)
    np.testing.assert_allclose(state_data_log[:, 1:],
                               expected[:, 1:],
                               rtol=1E-10,
                               err_msg='state propagation error',
                               verbose=True)
    np.testing.assert_allclose(energy[0],
                               energy,
                               rtol=1E-2,
                               atol=1E-10,
                               err_msg='energy not conserved',
                               verbose=True)
    return


if __name__ == "__main__":
    state_update_flyby(False)
