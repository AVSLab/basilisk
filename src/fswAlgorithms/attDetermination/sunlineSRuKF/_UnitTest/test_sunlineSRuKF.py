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


import sunlineSRuKF_test_utilities as filter_plots
import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import sunlineSRuKF
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion
from Basilisk.utilities import RigidBodyKinematics as rbk


def add_time_column(time, data):
    return np.transpose(np.vstack([[time], np.transpose(data)]))


def rk4(f, t, x0, normalizeState=False, mrpShadow=False):
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
        if normalizeState:
            x[i + 1, 1:4] = x[i + 1, 1:4] / np.linalg.norm(x[i + 1, 1:4])
        if mrpShadow:
            s = np.linalg.norm(x[i + 1, 1:4])**2
            if s > 1:
                x[i + 1, 1:4] = - (x[i + 1, 1:4]) / s
        x[i + 1, 0] = t[i + 1]
    return x


def sunline_dynamics(t, x):
    dxdt = np.zeros(np.shape(x))
    dxdt[0:3] = np.cross(x[:3], x[3:])
    dxdt[3:] = np.zeros(3)
    return dxdt


def mrp_integration(t, x):
    dxdt = np.zeros(np.shape(x))
    B = rbk.BmatMRP(x[0:3])
    dxdt[:3] = 0.25 * np.matmul(B, x[3:])
    dxdt[3:] = np.zeros(3)
    return dxdt


def setup_filter_data(filter_object):
    filter_object.setAlpha(0.02)
    filter_object.setBeta(2.0)

    states = [0.0, 0.0, 1.0, 0.02, -0.005, 0.01]

    filter_object.setInitialState([[s] for s in states])
    filter_object.setInitialCovariance([[0.0001, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0001, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0001, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0001, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.0001, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]])
    filter_object.setCssMeasurementNoiseStd(0.01)
    filter_object.setGyroMeasurementNoiseStd(0.001)
    sigmaSun = (1E-12) ** 2
    sigmaRate = (1E-14) ** 2
    filter_object.setProcessNoise([[sigmaSun, 0.0, 0.0, 0.0, 0.0, 0.0],
                                   [0.0, sigmaSun, 0.0, 0.0, 0.0, 0.0],
                                   [0.0, 0.0, sigmaSun, 0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0, sigmaRate, 0.0, 0.0],
                                   [0.0, 0.0, 0.0, 0.0, sigmaRate, 0.0],
                                   [0.0, 0.0, 0.0, 0.0, 0.0, sigmaRate]])

def setup_css_config_msg(CSSOrientationList, cssConfigDataInMsg):
    numCSS = len(CSSOrientationList)

    # set the CSS unit vectors
    cssConfigData = messaging.CSSConfigMsgPayload()
    totalCSSList = []
    for CSSHat in CSSOrientationList:
        CSSConfigElement = messaging.CSSUnitConfigMsgPayload()
        CSSConfigElement.CBias = 1.0
        CSSConfigElement.nHat_B = CSSHat
        totalCSSList.append(CSSConfigElement)
    cssConfigData.nCSS = numCSS
    cssConfigData.cssVals = totalCSSList
    cssConfigDataInMsg.write(cssConfigData)


def test_propagation_kf(show_plots):
    state_propagation_flyby(show_plots)


@pytest.mark.parametrize("initial_error", [False, True])
def test_measurements_kf(show_plots, initial_error):
    state_update_flyby(initial_error, show_plots)


def state_propagation_flyby(show_plots=False):
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(1.0)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    sunHeadingFilter = sunlineSRuKF.SunlineSRuKF()

    # Add test module to runtime call list
    setup_filter_data(sunHeadingFilter)
    unit_test_sim.AddModelToTask(unit_task_name, sunHeadingFilter)

    sun_heading_data_log = sunHeadingFilter.filterOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, sun_heading_data_log)

    simpleNavMsgData = messaging.NavAttMsgPayload()
    initState = sunHeadingFilter.getInitialState()
    simpleNavMsgData.timeTag = 0
    simpleNavMsgData.omega_BN_B = [initState[3][0], initState[4][0], initState[5][0]]
    simpleNavMsg = messaging.NavAttMsg().write(simpleNavMsgData)
    sunHeadingFilter.navAttInMsg.subscribeTo(simpleNavMsg)

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

    cssConfigMsg = messaging.CSSConfigMsg()
    setup_css_config_msg(CSSOrientationList, cssConfigMsg)
    sunHeadingFilter.cssConfigInMsg.subscribeTo(cssConfigMsg)

    cssDataMsg = messaging.CSSArraySensorMsgPayload()
    for i in range(8):
        cssDataMsg.CosValue[i] = 0.0
    cssMsg = messaging.CSSArraySensorMsg().write(cssDataMsg)
    sunHeadingFilter.cssDataInMsg.subscribeTo(cssMsg)

    sim_time = 5
    time = np.linspace(0, 5, 6)
    expected = np.zeros([len(time), 7])
    expected[0, 1:] = np.array(sunHeadingFilter.getInitialState()).reshape([6, ])
    expected = rk4(sunline_dynamics, time, expected[0, 1:], normalizeState=True)

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    num_states = 6
    state_data_log = add_time_column(sun_heading_data_log.times(), sun_heading_data_log.state[:, :num_states])
    covariance_data_log = add_time_column(sun_heading_data_log.times(), sun_heading_data_log.covar[:, :num_states**2])

    np.testing.assert_array_less(np.linalg.norm(covariance_data_log[0, 1:]),
                                 np.linalg.norm(covariance_data_log[-1, 1:]),
                                 err_msg='covariance must increase without measurements',
                                 verbose=True)
    np.testing.assert_allclose(state_data_log[:, 1:],
                               expected[:, 1:],
                               rtol=1E-10,
                               err_msg='state propagation error',
                               verbose=True)


def state_update_flyby(initial_error, show_plots=False):
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(1.0)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    sunHeadingFilter = sunlineSRuKF.SunlineSRuKF()

    # Add test module to runtime call list
    setup_filter_data(sunHeadingFilter)
    if initial_error:
        states = [1.0, 0.0, 0.0, -0.02, 0.005, -0.01]
        sunHeadingFilter.setInitialState([[s] for s in states])
    unit_test_sim.AddModelToTask(unit_task_name, sunHeadingFilter)

    sun_heading_data_log = sunHeadingFilter.filterOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, sun_heading_data_log)

    css_residual_data_log = sunHeadingFilter.filterCssResOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, css_residual_data_log)

    gyro_residual_data_log = sunHeadingFilter.filterGyroResOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, gyro_residual_data_log)

    nav_att_data_log = sunHeadingFilter.navAttOutMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, nav_att_data_log)

    simpleNavMsgData = messaging.NavAttMsgPayload()
    initState = sunHeadingFilter.getInitialState()
    simpleNavMsgData.timeTag = 0
    simpleNavMsgData.omega_BN_B = [initState[3][0], initState[4][0], initState[5][0]]
    simpleNavMsg = messaging.NavAttMsg()
    sunHeadingFilter.navAttInMsg.subscribeTo(simpleNavMsg)

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

    cssConfigMsg = messaging.CSSConfigMsg()
    setup_css_config_msg(CSSOrientationList, cssConfigMsg)
    sunHeadingFilter.cssConfigInMsg.subscribeTo(cssConfigMsg)

    sim_time = 1000
    time = np.linspace(0, sim_time, sim_time+1)
    expected = np.zeros([len(time), 7])
    expected[0, 1:] = np.array([0.0, 0.0, 1.0, 0.02, -0.005, 0.01])
    expected = rk4(sunline_dynamics, time, expected[0, 1:], normalizeState=True)

    bodyFrame = np.zeros([len(time), 7])
    bodyFrame[0, 1:] = np.array([0.0, 0.0, 0.0, expected[0, 4], expected[0, 5], expected[0, 6]])
    bodyFrame = rk4(mrp_integration, time, bodyFrame[0, 1:], mrpShadow=True)

    cssDataMsg = messaging.CSSArraySensorMsgPayload()
    cssMsg = messaging.CSSArraySensorMsg()
    sunHeadingFilter.cssDataInMsg.subscribeTo(cssMsg)

    cssSigma = sunHeadingFilter.getCssMeasurementNoiseStd()
    gyroSigma = sunHeadingFilter.getGyroMeasurementNoiseStd()

    unit_test_sim.InitializeSimulation()
    for i in range(0, len(time)-1):
        BN = rbk.MRP2C(bodyFrame[i, 1:4])
        cosList = []
        for j in range(len(CSSOrientationList)):
            cosList.append((np.dot(CSSOrientationList[j], np.matmul(BN, [0, 0, 1]))
                            + np.random.normal(0, cssSigma, 1))[0])
        cssDataMsg.CosValue = np.array(cosList)
        cssDataMsg.timeTag = time[i]
        omega = expected[0, 4:] + np.random.normal(0, gyroSigma, 3)
        simpleNavMsgData.timeTag = time[i]
        simpleNavMsgData.omega_BN_B = omega
        if i % 2 == 0:
            cssMsg.write(cssDataMsg)
            simpleNavMsg.write(simpleNavMsgData)
        unit_test_sim.ConfigureStopTime(macros.sec2nano(time[i+1]))
        unit_test_sim.ExecuteSimulation()

    num_states = 6
    state_data_log = add_time_column(sun_heading_data_log.times(), sun_heading_data_log.state[:, :num_states])
    covariance_data_log = add_time_column(sun_heading_data_log.times(), sun_heading_data_log.covar[:, :num_states**2])

    covariance = []
    for i in range(num_states):
        covariance.append([])
        for j in range(num_states):
            covariance[-1].append(covariance_data_log[i][1+j*(num_states+1)])

    css_number_obs = css_residual_data_log.numberOfObservations
    css_size_obs = css_residual_data_log.sizeOfObservations
    css_post_fit_log_sparse = add_time_column(css_residual_data_log.times(), css_residual_data_log.postFits)
    css_post_fit_log = np.zeros([len(css_residual_data_log.times()), np.max(css_size_obs)+1])
    css_post_fit_log[:, 0] = css_post_fit_log_sparse[:, 0]
    css_pre_fit_log_sparse = add_time_column(css_residual_data_log.times(), css_residual_data_log.preFits)
    css_pre_fit_log = np.zeros([len(css_residual_data_log.times()), np.max(css_size_obs)+1])
    css_pre_fit_log[:, 0] = css_pre_fit_log_sparse[:, 0]

    for i in range(len(css_number_obs)):
        if css_number_obs[i] > 0:
            css_post_fit_log[i, 1:css_size_obs[i]+1] = css_post_fit_log_sparse[i, 1:css_size_obs[i]+1]
            css_pre_fit_log[i, 1:css_size_obs[i]+1] = css_pre_fit_log_sparse[i, 1:css_size_obs[i]+1]

    gyro_number_obs = gyro_residual_data_log.numberOfObservations
    gyro_size_obs = gyro_residual_data_log.sizeOfObservations
    gyro_post_fit_log_sparse = add_time_column(gyro_residual_data_log.times(), gyro_residual_data_log.postFits)
    gyro_post_fit_log = np.zeros([len(gyro_residual_data_log.times()), np.max(gyro_size_obs)+1])
    gyro_post_fit_log[:, 0] = gyro_post_fit_log_sparse[:, 0]
    gyro_pre_fit_log_sparse = add_time_column(gyro_residual_data_log.times(), gyro_residual_data_log.preFits)
    gyro_pre_fit_log = np.zeros([len(gyro_residual_data_log.times()), np.max(gyro_size_obs)+1])
    gyro_pre_fit_log[:, 0] = gyro_pre_fit_log_sparse[:, 0]

    for i in range(len(gyro_number_obs)):
        if gyro_number_obs[i] > 0:
            gyro_post_fit_log[i, 1:gyro_size_obs[i]+1] = gyro_post_fit_log_sparse[i, 1:gyro_size_obs[i]+1]
            gyro_pre_fit_log[i, 1:gyro_size_obs[i]+1] = gyro_pre_fit_log_sparse[i, 1:gyro_size_obs[i]+1]

    half_time = len(time) // 2
    # testing that Sun Heading vector estimate is correct within 5 sigma
    np.testing.assert_allclose(state_data_log[half_time:, 1:4],
                               expected[half_time:, 1:4],
                                rtol=1E-9,
                                atol=5*cssSigma,
                                err_msg='state propagation error',
                                verbose=True)
    # testing that rate estimate is correct within 5 sigma
    np.testing.assert_allclose(state_data_log[half_time:, 4:],
                                expected[half_time:, 4:],
                               rtol=1E-9,
                               atol=5*gyroSigma,
                               err_msg='state propagation error',
                               verbose=True)
    # testing that covariance is shrinking
    np.testing.assert_array_less(np.diag(covariance_data_log[half_time, 1:].reshape([6, 6])),
                                np.diag(covariance_data_log[0, 1:].reshape([6, 6])),
                                err_msg='covariance error',
                                verbose=True)

    diff = np.copy(state_data_log)
    diff[:, 1:] -= expected[:, 1:]
    if show_plots:
        plt_1 = filter_plots.state_covar(state_data_log, covariance_data_log, 'Update').show()
        plt_2 = filter_plots.states(diff, 'Update').show()
        plt_3 = filter_plots.post_fit_residuals(css_post_fit_log, cssSigma, 'Update CSS PreFit').show()
        plt_4 = filter_plots.post_fit_residuals(css_pre_fit_log, cssSigma, 'Update CSS PostFit').show()
        plt_5 = filter_plots.post_fit_residuals(gyro_post_fit_log, gyroSigma, 'Update Gyro PreFit').show()
        plt_6 = filter_plots.post_fit_residuals(gyro_pre_fit_log, gyroSigma, 'Update Gyro PostFit').show()


if __name__ == "__main__":
    state_update_flyby(False, False)
