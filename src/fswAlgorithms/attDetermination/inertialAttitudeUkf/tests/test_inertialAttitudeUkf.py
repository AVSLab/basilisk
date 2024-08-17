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

import uKF_test_utilities as filter_plots
import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import inertialAttitudeUkf
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion
from Basilisk.utilities import RigidBodyKinematics as rbk

starOnly = inertialAttitudeUkf.AttitudeFilterMethod_StarOnly
gyroWhenDazzled = inertialAttitudeUkf.AttitudeFilterMethod_GyroWhenDazzled
allMeasurements = inertialAttitudeUkf.AttitudeFilterMethod_AllMeasurements

def add_time_column(time, data):
    return np.transpose(np.vstack([[time], np.transpose(data)]))

def rk4(f, t, x0, Inertia=np.eye(3), mrpShadow=True):
    x = np.zeros([len(t), len(x0) + 1])
    h = (t[len(t) - 1] - t[0]) / len(t)
    x[0, 0] = t[0]
    x[0, 1:] = x0
    for i in range(len(t) - 1):
        h = t[i + 1] - t[i]
        x[i, 0] = t[i]
        k1 = h * f(t[i], x[i, 1:], Inertia)
        k2 = h * f(t[i] + 0.5 * h, x[i, 1:] + 0.5 * k1, Inertia)
        k3 = h * f(t[i] + 0.5 * h, x[i, 1:] + 0.5 * k2, Inertia)
        k4 = h * f(t[i] + h, x[i, 1:] + k3, Inertia)
        x[i + 1, 1:] = x[i, 1:] + (k1 + 2. * k2 + 2. * k3 + k4) / 6.
        if mrpShadow:
            s = np.linalg.norm(x[i + 1, 1:4])**2
            if s > 1:
                x[i + 1, 1:4] = - (x[i + 1, 1:4]) / s
        x[i + 1, 0] = t[i + 1]
    return x

def attitude_dynamics(t, x, I):
    dxdt = np.zeros(np.shape(x))
    mrp = x[:3]
    omega = x[3:]
    B = rbk.BmatMRP(mrp)
    dxdt[:3] = 0.25 * np.matmul(B, omega)
    dxdt[3:] = - np.dot(np.linalg.inv(I), np.cross(omega, np.dot(I, omega)))
    return dxdt

def mrp_integration(t, x, I=np.eye(3)):
    dxdt = np.zeros(np.shape(x))
    B = rbk.BmatMRP(x[0:3])
    dxdt[:3] = 0.25 * np.matmul(B, x[3:])
    dxdt[3:] = np.zeros(3)
    return dxdt


def setup_filter_data(filter_object):
    filter_object.setAlpha(0.02)
    filter_object.setBeta(2.0)

    filter_object.setInitialPosition([0.05, 0.5, 0.1])
    filter_object.setInitialVelocity([0.2, -0.05, 0.1])
    filter_object.setInitialCovariance([[1, 0.0, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 1, 0.0, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 1, 0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.01, 0.0, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.01, 0.0],
                                        [0.0, 0.0, 0.0, 0.0, 0.0, 0.01]])
    filter_object.setGyroNoise([[1e-3,0,0],[0.,1e-3,0],[0,0,1e-3]])
    sigma_mrp = (1E-2) ** 2
    sigma_rate = (1E-2) ** 2
    filter_object.setProcessNoise([[sigma_mrp, 0.0, 0.0, 0.0, 0.0, 0.0],
                                   [0.0, sigma_mrp, 0.0, 0.0, 0.0, 0.0],
                                   [0.0, 0.0, sigma_mrp, 0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0, sigma_rate, 0.0, 0.0],
                                   [0.0, 0.0, 0.0, 0.0, sigma_rate, 0.0],
                                   [0.0, 0.0, 0.0, 0.0, 0.0, sigma_rate]])

    filter_object.setLowPassFilter(0.5, 15/(2*np.pi))

    return

@pytest.mark.parametrize("show_plots", [False])
def test_propagation_kf(show_plots):
    """Module Unit Test"""
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    test_process_rate = macros.sec2nano(1.0)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    # Construct algorithm and associated C++ container
    allMeasurements = inertialAttitudeUkf.AttitudeFilterMethod_AllMeasurements
    intertialAttitudeFilter = inertialAttitudeUkf.InertialAttitudeUkf(allMeasurements)
    unit_test_sim.AddModelToTask(unit_task_name, intertialAttitudeFilter)

    # Add test module to runtime call list
    setup_filter_data(intertialAttitudeFilter)

    rw_orientation_list = [
        0.70710678118654746, -0.5, 0.5,
        0.70710678118654746, -0.5, -0.5,
        0.70710678118654746, 0.5, -0.5,
        0.70710678118654746, 0.5, 0.5
    ]

    rw_inertia_list = [5, 10, 5, 10]

    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]

    vehicle_config_data = messaging.VehicleConfigMsgPayload()
    vehicle_config_data.ISCPntB_B = I

    vehicle_config = messaging.VehicleConfigMsg().write(vehicle_config_data)
    intertialAttitudeFilter.vehicleConfigMsg.subscribeTo(vehicle_config)

    rw_data_msg = messaging.RWArrayConfigMsgPayload()
    rw_data_msg.numRW = 4
    rw_data_msg.GsMatrix_B = rw_orientation_list
    rw_data_msg.JsList = rw_inertia_list
    rw_msg = messaging.RWArrayConfigMsg().write(rw_data_msg)
    intertialAttitudeFilter.rwArrayConfigMsg.subscribeTo(rw_msg)

    rw_speeds_data = messaging.RWSpeedMsgPayload()
    for i in range(rw_data_msg.numRW):
        rw_speeds_data.wheelSpeeds[i] = i%2*200
    rw_speeds = messaging.RWSpeedMsg().write(rw_speeds_data)
    intertialAttitudeFilter.rwSpeedMsg.subscribeTo(rw_speeds)

    intertialAttitudeFilter.setInitialPosition([0.05, 0.5, 0.1])
    intertialAttitudeFilter.setInitialVelocity([-0.02, 0.005, -0.01])
    initState = [0.05, 0.5, 0.1, -0.02, 0.005, -0.01]

    st_1_data = messaging.STAttMsgPayload()
    st_1_data.MRP_BdyInrtl = initState[:3]
    st_1_data.timeTag = 0

    star_tracker1 = inertialAttitudeUkf.StarTrackerMessage()
    st_1_msg = messaging.STAttMsg().write(st_1_data)
    star_tracker1.starTrackerMsg.subscribeTo(st_1_msg)
    star_tracker1.measurementNoise = [[1e-3, 0, 0], [0,1e-3,0], [0,0,1e-3]]
    intertialAttitudeFilter.addStarTrackerInput(star_tracker1)

    st_2_data = messaging.STAttMsgPayload()
    st_2_data.MRP_BdyInrtl = initState[:3]
    st_2_data.timeTag = 0

    star_tracker2 = inertialAttitudeUkf.StarTrackerMessage()
    st_2_msg = messaging.STAttMsg().write(st_2_data)
    star_tracker2.starTrackerMsg.subscribeTo(st_2_msg)
    star_tracker2.measurementNoise = [[1e-3, 0, 0], [0,1e-3,0], [0,0,1e-3]]
    intertialAttitudeFilter.addStarTrackerInput(star_tracker2)

    accel_data = messaging.AccDataMsgPayload()
    accel_measurement = messaging.AccDataMsg().write(accel_data)
    intertialAttitudeFilter.accelDataMsg.subscribeTo(accel_measurement)

    attitude_data_log = intertialAttitudeFilter.inertialFilterOutputMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, attitude_data_log)

    sim_time = 100
    time = np.linspace(0, sim_time, sim_time+1)
    initial_condition = np.zeros(6)
    initial_condition[:3] = np.array(intertialAttitudeFilter.getInitialPosition()).reshape(3)
    initial_condition[3:] = np.array(intertialAttitudeFilter.getInitialVelocity()).reshape(3)
    expected = rk4(attitude_dynamics, time, initial_condition, np.array(I).reshape([3,3]))

    unit_test_sim.InitializeSimulation()
    unit_test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    unit_test_sim.ExecuteSimulation()

    num_states = 6
    state_data_log = add_time_column(attitude_data_log.times(), attitude_data_log.state[:, :num_states])
    covariance_data_log = add_time_column(attitude_data_log.times(), attitude_data_log.covar[:, :num_states**2])

    diff = np.copy(state_data_log)
    diff[:, 1:] -= expected[:, 1:]
    filter_plots.state_covar(state_data_log, covariance_data_log, 'Update', show_plots)
    filter_plots.states(diff, 'Update', show_plots)

    np.testing.assert_array_less(np.linalg.norm(covariance_data_log[0, 1:]),
                                 np.linalg.norm(covariance_data_log[-1, 1:]),
                                 err_msg='covariance must increase without measurements',
                                 verbose=True)
    np.testing.assert_allclose(state_data_log[:, 1:],
                               expected[:, 1:],
                               rtol=1E-10,
                               err_msg='state propagation error',
                               verbose=True)
    return

@pytest.mark.parametrize("show_plots", [False])
@pytest.mark.parametrize("initial_error", [False, True])
@pytest.mark.parametrize("method", [starOnly, gyroWhenDazzled, allMeasurements])
def test_measurements_kf(show_plots, initial_error, method):
    """Module Unit Test"""
    unit_task_name = "unitTask"  # arbitrary name (don't change)
    unit_process_name = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unit_test_sim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    step_size = 1
    test_process_rate = macros.sec2nano(step_size)  # update process rate update time
    test_process = unit_test_sim.CreateNewProcess(unit_process_name)
    test_process.addTask(unit_test_sim.CreateNewTask(unit_task_name, test_process_rate))

    intertialAttitudeFilter = inertialAttitudeUkf.InertialAttitudeUkf(method)
    unit_test_sim.AddModelToTask(unit_task_name, intertialAttitudeFilter)

    # Add test module to runtime call list
    setup_filter_data(intertialAttitudeFilter)

    rw_orientation_list = [
        0.70710678118654746, -0.5, 0.5,
        0.70710678118654746, -0.5, -0.5,
        0.70710678118654746, 0.5, -0.5,
        0.70710678118654746, 0.5, 0.5
    ]

    rw_inertia_list = [5, 10, 5, 10]

    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]

    vehicle_config_data = messaging.VehicleConfigMsgPayload()
    vehicle_config_data.ISCPntB_B = I

    vehicle_config = messaging.VehicleConfigMsg().write(vehicle_config_data)
    intertialAttitudeFilter.vehicleConfigMsg.subscribeTo(vehicle_config)

    rw_data_msg = messaging.RWArrayConfigMsgPayload()
    rw_data_msg.numRW = 4
    rw_data_msg.GsMatrix_B = rw_orientation_list
    rw_data_msg.JsList = rw_inertia_list
    rw_msg = messaging.RWArrayConfigMsg().write(rw_data_msg)
    intertialAttitudeFilter.rwArrayConfigMsg.subscribeTo(rw_msg)

    rw_speeds_data = messaging.RWSpeedMsgPayload()
    for i in range(rw_data_msg.numRW):
        rw_speeds_data.wheelSpeeds[i] = i%2*200
    rw_speeds = messaging.RWSpeedMsg().write(rw_speeds_data)
    intertialAttitudeFilter.rwSpeedMsg.subscribeTo(rw_speeds)

    initial_condition = np.zeros(6)
    initial_condition[:3] = np.array(intertialAttitudeFilter.getInitialPosition()).reshape(3)
    initial_condition[3:] = np.array(intertialAttitudeFilter.getInitialVelocity()).reshape(3)

    if initial_error:
        intertialAttitudeFilter.setInitialPosition([0, 0, 0])
        intertialAttitudeFilter.setInitialVelocity([0, 0, 0])

    st_1_data = messaging.STAttMsgPayload()
    st_1_data.MRP_BdyInrtl = initial_condition[:3]
    st_1_data.timeTag = 0

    star_tracker1 = inertialAttitudeUkf.StarTrackerMessage()
    st_1_msg = messaging.STAttMsg().write(st_1_data)
    star_tracker1.starTrackerMsg.subscribeTo(st_1_msg)
    star_tracker1.measurementNoise = [[1e-4, 0, 0], [0,1e-4,0], [0,0,1e-4]]
    intertialAttitudeFilter.addStarTrackerInput(star_tracker1)

    st_2_data = messaging.STAttMsgPayload()
    st_2_data.MRP_BdyInrtl = initial_condition[:3]
    st_2_data.timeTag = 0

    star_tracker2 = inertialAttitudeUkf.StarTrackerMessage()
    st_2_msg = messaging.STAttMsg().write(st_2_data)
    star_tracker2.starTrackerMsg.subscribeTo(st_2_msg)
    star_tracker2.measurementNoise = [[1e-4, 0, 0], [0,1e-4,0], [0,0,1e-4]]
    intertialAttitudeFilter.addStarTrackerInput(star_tracker2)

    accel_data = messaging.AccDataMsgPayload()
    accel_measurement = messaging.AccDataMsg().write(accel_data)
    intertialAttitudeFilter.accelDataMsg.subscribeTo(accel_measurement)

    filter_data_log = intertialAttitudeFilter.inertialFilterOutputMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, filter_data_log)

    attitude_data_log = intertialAttitudeFilter.inertialFilterOutputMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, attitude_data_log)

    st_residual_data_log = intertialAttitudeFilter.starTrackerResidualMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, st_residual_data_log)

    gyro_residual_data_log = intertialAttitudeFilter.gyroResidualMsg.recorder()
    unit_test_sim.AddModelToTask(unit_task_name, gyro_residual_data_log)

    sim_time = 200
    max_buffer_len = 120
    substeps = 50
    time = np.linspace(0, sim_time, substeps*sim_time+1)
    expected = np.zeros([len(time), 7])
    expected[:int(len(time)/2), :] = rk4(attitude_dynamics, time[:int(len(time)/2)], initial_condition,
                                         np.array(I).reshape([3,3]))
    kick = np.array([0.1, 0.01, -0.5, 1E-1, 1E-2, 1E-1])
    initial_condition = expected[int(len(time)/2), 1:] + kick
    expected[int(len(time)/2):, :] = rk4(attitude_dynamics, time[int(len(time)/2):], initial_condition,
                                         np.array(I).reshape([3,3]))

    st_sigma_2 = np.diag(intertialAttitudeFilter.getStarTrackerNoise(1)).mean()
    gyroSigma = np.diag(intertialAttitudeFilter.getGyroNoise()).mean()

    unit_test_sim.InitializeSimulation()
    for i in range(len(time)-1 - substeps):
        for k in range(rw_data_msg.numRW):
            rw_speeds_data.wheelSpeeds[k] = k%2*200
        rw_speeds.write(rw_speeds_data, int((i+1)*1E9))
        if (i > 10*substeps and i< substeps*sim_time/4) or i > substeps*sim_time/2:
            if (time[i]-0.2).is_integer():
                st_1_data.timeTag = int(time[i] * 1E9)
                st_1_data.valid = True
                st_1_data.MRP_BdyInrtl = expected[i, 1:4] + np.random.normal(0, 1e-4, 3)
                st_1_msg.write(st_1_data, int(time[i]*1E9))

            if (time[i]-0.8).is_integer():
                st_2_data.timeTag = int(time[i]*1e9)
                st_2_data.valid = True
                st_2_data.MRP_BdyInrtl = expected[i, 1:4] + np.random.normal(0, 1e-4, 3)
                st_2_msg.write(st_2_data, int(time[i]*1e9))

        if i > substeps*sim_time/4 and i < len(time):
            if (time[i]-0.5).is_integer():
                accel_data = messaging.AccDataMsgPayload()
                accel_data.valid = True
                for k in range(max_buffer_len):
                    accel_data.accPkts[(k)%max_buffer_len].measTime = int(time[i]*1e9)
                    accel_data.accPkts[(k)%max_buffer_len].gyro_B = (expected[i, 4:7]
                                                                     + np.random.normal(0, 1e-4, 3))
                accel_measurement.write(accel_data, int(time[i]*1e9))

        unit_test_sim.ConfigureStopTime(macros.sec2nano((time[i+1])))
        unit_test_sim.ExecuteSimulation()

    num_states = 6
    state_data_log = add_time_column(filter_data_log.times(), filter_data_log.state[:, :num_states])
    covariance_data_log = add_time_column(filter_data_log.times(), filter_data_log.covar[:, :num_states**2])

    covariance = []
    for i in range(num_states):
        covariance.append([])
        for j in range(num_states):
            covariance[-1].append(covariance_data_log[i][1+j*(num_states+1)])

    st_number_obs = st_residual_data_log.numberOfObservations
    st_size_obs = st_residual_data_log.sizeOfObservations
    st_post_fit_log_sparse = add_time_column(st_residual_data_log.times(), st_residual_data_log.postFits)
    st_post_fit_log = np.zeros([len(st_residual_data_log.times()), np.max(st_size_obs)+1])
    st_post_fit_log[:, 0] = st_post_fit_log_sparse[:, 0]
    st_pre_fit_log_sparse = add_time_column(st_residual_data_log.times(), st_residual_data_log.preFits)
    st_pre_fit_log = np.zeros([len(st_residual_data_log.times()), np.max(st_size_obs)+1])
    st_pre_fit_log[:, 0] = st_pre_fit_log_sparse[:, 0]

    for i in range(len(st_number_obs)):
        if st_number_obs[i] > 0:
            st_post_fit_log[i, 1:st_size_obs[i]+1] = st_post_fit_log_sparse[i, 1:st_size_obs[i]+1]
            st_pre_fit_log[i, 1:st_size_obs[i]+1] = st_pre_fit_log_sparse[i, 1:st_size_obs[i]+1]

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

    quarter_time = int(3* len(time) / 4 / substeps)
    diff = np.copy(state_data_log)
    for i in range(sim_time):
        diff[i, 1:] -= expected[i*substeps, 1:]

    # testing that Sun Heading vector estimate is correct within 5 sigma
    np.testing.assert_allclose(np.linalg.norm(diff[quarter_time:, 1:], axis=1),
                               0,
                                rtol=1e-4,
                                atol=1e-2,
                                err_msg='state propagation error',
                                verbose=True)
    # testing that covariance is shrinking
    np.testing.assert_array_less(np.diag(covariance_data_log[quarter_time, 1:].reshape([6, 6])),
                                np.diag(covariance_data_log[0, 1:].reshape([6, 6])),
                                err_msg='covariance error',
                                verbose=True)

    filter_plots.state_covar(diff, covariance_data_log, 'Update', show_plots)
    filter_plots.states(diff, 'Update', show_plots)
    filter_plots.post_fit_residuals(st_pre_fit_log, st_sigma_2, 'Update ST PostFit', show_plots)
    if np.max(gyro_size_obs)>0:
        filter_plots.post_fit_residuals(gyro_pre_fit_log, gyroSigma, 'Update Gyro PostFit', show_plots)

if __name__ == "__main__":
    test_measurements_kf(True, True, allMeasurements)
