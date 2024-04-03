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
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import flybyODuKF  # import the module that is to be tested
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion


def add_time_column(time, data):
    return np.transpose(np.vstack([[time], np.transpose(data)]))


def runge_kutta_4(f, t, x0):
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
    filter_object.alphaParameter = 0.02
    filter_object.betaParameter = 2.0

    filter_object.muCentral = 42828.314 * 1E9
    elements_init = orbitalMotion.ClassicElements()
    elements_init.a = 4000 * 1E3  # m
    elements_init.e = 0.2
    elements_init.i = 10
    elements_init.Omega = 0.001
    elements_init.omega = 0.01
    elements_init.f = 0.1
    r, v = orbitalMotion.elem2rv(filter_object.muCentral, elements_init)
    states = r.tolist() + v.tolist()

    filter_object.stateInitial = [[s] for s in states]
    filter_object.covarInitial = [[1000. * 1E6, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  [0.0, 1000. * 1E6, 0.0, 0.0, 0.0, 0.0],
                                  [0.0, 0.0, 1000. * 1E6, 0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, 0.1 * 1E6, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, 0.1 * 1E6, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, 0.0, 0.1 * 1E6]]

    sigma_pos = (1E-8) ** 2
    sigma_vel = (1E-10) ** 2
    filter_object.processNoise = [[sigma_pos, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  [0.0, sigma_pos, 0.0, 0.0, 0.0, 0.0],
                                  [0.0, 0.0, sigma_pos, 0.0, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, sigma_vel, 0.0, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, sigma_vel, 0.0],
                                  [0.0, 0.0, 0.0, 0.0, 0.0, sigma_vel]]
    filter_object.measNoiseScaling = 1


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

def test_propagation_kf(show_plots):
    """Module Unit Test"""
    [test_results, test_message] = state_prop_test(show_plots, 10.0)
    assert test_results < 1, test_message


def test_measurements_kf(show_plots):
    """Module Unit Test"""
    [test_results, test_message] = state_update_test(show_plots)
    assert test_results < 1, test_message


def state_update_test(show_plots):
    test_fail_count = 0
    test_messages = []

    unit_task_name = "unitTask"
    unit_process_name = "TestProcess"

    # Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    dt = 1.0
    t1 = 250
    time_multiplier = 8

    test_process_rate = macros.sec2nano(dt)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    module_config = flybyODuKF.FlybyODuKF()

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module_config)

    setup_filter_data(module_config)
    module_config.measNoiseScaling = 1

    filter_data_log = module_config.opNavFilterMsg.recorder()
    residual_data_log = module_config.opNavResidualMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, filter_data_log)
    unit_test_sim.AddModelToTask(unit_task_name, residual_data_log)

    time = np.linspace(0, int(time_multiplier * t1), int(time_multiplier * t1 // dt) + 1)
    energy = np.zeros(len(time))
    expected = np.zeros([len(time), 7])
    expected[0, 1:] = np.array(module_config.stateInitial).reshape([6, ])
    energy[0] = -module_config.muCentral / (
                2 * orbitalMotion.rv2elem(module_config.muCentral, expected[0, 1:4], expected[0, 4:]).a)

    kick = np.array([0., 0., 0., -0.01, 0.01, 0.02]) * 10 * 1E3

    expected[0:t1, :] = runge_kutta_4(two_body_gravity, time[0:t1], expected[0, 1:])
    expected[t1:time_multiplier * t1 + 1, :] = runge_kutta_4(two_body_gravity, time[t1:len(time)],
                                                             expected[t1 - 1, 1:] + kick)
    for i in range(1, len(time)):
        energy[i] = - module_config.muCentral / (
                    2 * orbitalMotion.rv2elem(module_config.muCentral, expected[i, 1:4], expected[i, 4:]).a)

    input_data = messaging.OpNavUnitVecMsgPayload()
    opnav_input_msg = messaging.OpNavUnitVecMsg()
    module_config.opNavHeadingMsg.subscribeTo(opnav_input_msg)
    input_data.rhat_BN_B = expected[0, 1:4] / np.linalg.norm(expected[0, 1:4])

    unit_test_sim.InitializeSimulation()
    for i in range(t1):
        if i > 0 and i % 10 == 0:
            input_data.timeTag = macros.sec2nano(i * dt)
            input_data.rhat_BN_N = (expected[i, 1:4] + np.random.normal(0, 100, 3))
            input_data.rhat_BN_N /= np.linalg.norm(input_data.rhat_BN_N)
            input_data.valid = True
            input_data.covar_N = [5. * 1E-5, 0., 0.,
                                 0., 5. * 1E-5, 0.,
                                 0., 0., 5. * 1E-5]
            opnav_input_msg.write(input_data, unit_test_sim.TotalSim.CurrentNanos)
        unit_test_sim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        unit_test_sim.ExecuteSimulation()

    covar_log = add_time_column(filter_data_log.times(), filter_data_log.covar)

    for i in range(3, 6):
        if (covar_log[t1, i * 6 + 1 + i] > covar_log[0, i * 6 + 1 + i]):
            test_fail_count += 1
            test_messages.append("Covariance update failure at " + str(t1))

    for i in range(t1, time_multiplier * t1):
        if i % 50 == 0:
            input_data.timeTag = macros.sec2nano(i * dt)
            input_data.rhat_BN_N = (expected[i, 1:4] + np.random.normal(0, 100, 3))
            input_data.rhat_BN_N /= np.linalg.norm(input_data.rhat_BN_N)
            input_data.valid = True
            input_data.covar_N = [5. * 1E-5, 0., 0.,
                                 0., 5. * 1E-5, 0.,
                                 0., 0., 5. * 1E-5]
            opnav_input_msg.write(input_data, unit_test_sim.TotalSim.CurrentNanos)
        unit_test_sim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        unit_test_sim.ExecuteSimulation()

    state_log = add_time_column(filter_data_log.times(), filter_data_log.state)
    covar_log = add_time_column(filter_data_log.times(), filter_data_log.covar)
    postFitLog = add_time_column(residual_data_log.times(), residual_data_log.postFits)

    diff = np.copy(state_log)
    diff[:, 1:] -= expected[:, 1:]
    filter_plots.energy(time, energy, 'Update', show_plots)
    filter_plots.state_covar(state_log, covar_log, 'Update', show_plots)
    filter_plots.states(diff, 'Update', show_plots)
    filter_plots.two_orbits(expected[:, 0:4], state_log[:, 0:4], show_plots)
    filter_plots.post_fit_residuals(postFitLog, np.sqrt(1E-6), 'Update', show_plots)

    for i in range(3, 6):
        if covar_log[t1 * time_multiplier, i * 6 + 1 + i] > covar_log[0, i * 6 + 1 + i] / 10:
            test_fail_count += 1
            test_messages.append("Covariance update failure at " + str(t1 * time_multiplier))

    if (np.linalg.norm(diff[-1, 1:] / expected[-1, 1:]) > 1.0E-1):
        test_fail_count += 1
        test_messages.append("State propagation failure")

    # print out success message if no error were found
    if test_fail_count == 0:
        print("PASSED: flybyUKF state update")
    else:
        print(test_messages)

    # return fail count and join into a single string all messages in the list
    # test_message
    return [test_fail_count, ''.join(test_messages)]


def state_prop_test(show_plots, dt):

    test_fail_count = 0  # zero unit test result counter
    test_messages = []  # create empty list to store test log messages

    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(dt)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    module_config = flybyODuKF.FlybyODuKF()

    # Add test module to runtime call list
    unit_test_sim.AddModelToTask(unit_task_name, module_config)

    setup_filter_data(module_config)
    module_config.measNoiseScaling = 1

    filter_data_log = module_config.opNavFilterMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, filter_data_log)

    opnav_input_msg = messaging.OpNavUnitVecMsg()
    module_config.opNavHeadingMsg.subscribeTo(opnav_input_msg)

    timeSim = 60
    time = np.linspace(0, int(timeSim * 60), int(timeSim * 60 // dt) + 1)
    dydt = np.zeros(6)
    energy = np.zeros(len(time))
    expected = np.zeros([len(time), 7])
    expected[0, 1:] = np.array(module_config.stateInitial).reshape([6, ])
    energy[0] = -module_config.muCentral / (
                2 * orbitalMotion.rv2elem(module_config.muCentral, expected[0, 1:4], expected[0, 4:]).a)
    expected = runge_kutta_4(two_body_gravity, time, expected[0, 1:])
    for i in range(1, len(time)):
        energy[i] = - module_config.muCentral / (
                    2 * orbitalMotion.rv2elem(module_config.muCentral, expected[i, 1:4], expected[i, 4:]).a)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.min2nano(timeSim))
    unit_test_sim.ExecuteSimulation()

    state_log = add_time_column(filter_data_log.times(), filter_data_log.state)
    covar_log = add_time_column(filter_data_log.times(), filter_data_log.covar)

    diff = np.copy(state_log)
    diff[:, 1:] -= expected[:, 1:]
    filter_plots.two_orbits(expected[:, 0:4], state_log[:, 0:4], show_plots)
    filter_plots.energy(time, energy, 'Prop', show_plots)
    filter_plots.state_covar(state_log, covar_log, 'Prop', show_plots)
    filter_plots.states(diff, 'Prop', show_plots)

    if np.linalg.norm(diff[-1, 1:] / expected[-1, 1:]) > 1.0E-10:
        test_fail_count += 1
        test_messages.append("State propagation failure")

    if np.linalg.norm(covar_log[-1, 1:]) < 5 * np.linalg.norm(covar_log[0, 1:]):
        test_fail_count += 1
        test_messages.append("Covariance update failure, it does not grow without measurements")

    if (energy[0] - energy[-1]) / energy[0] > 1.0E-10:
        test_fail_count += 1
        test_messages.append("State propagation failure")

    # print out success message if no error were found
    if test_fail_count == 0:
        print("PASSED: flybyUKF state propagation")

    # return fail count and join into a single string all messages in the list
    # test_message
    return [test_fail_count, ''.join(test_messages)]


if __name__ == "__main__":
    state_update_test(False)
