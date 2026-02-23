# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


#
#   Unit Test Script
#   Module Name:        generalSingleBodyStateEffector
#   Author:             Leah Kiner
#   Creation Date:      February 19, 2026
#

import inspect
import os
import matplotlib.pyplot as plt
import numpy
import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import spacecraft, generalSingleBodyStateEffector, gravityEffector
from Basilisk.architecture import messaging


def test_generalSingleBodyStateEffector(show_plots):
    task_name = "unitTask"
    process_name = "TestProcess"
    test_sim = SimulationBaseClass.SimBaseClass()
    test_time_step_sec = 0.01
    test_process_rate = macros.sec2nano(test_time_step_sec)
    test_process = test_sim.CreateNewProcess(process_name)
    test_process.addTask(test_sim.CreateNewTask(task_name, test_process_rate))

    # Create the spacecraft module
    sc_object = spacecraft.Spacecraft()
    sc_object.ModelTag = "spacecraftBody"
    sc_object.hub.mHub = 750.0
    sc_object.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    sc_object.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    sc_object.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    sc_object.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    sc_object.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    sc_object.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]
    test_sim.AddModelToTask(task_name, sc_object)

    # Create the general effector
    general_body = generalSingleBodyStateEffector.GeneralSingleBodyStateEffector()
    general_body.ModelTag = "generalBody"
    general_body.setMass(50.0)
    general_body.setIPntGc_G(np.array([[50.0, 0.0, 0.0],
                                       [0.0, 30.0, 0.0],
                                       [0.0, 0.0, 40.0]]))
    general_body.setR_GcG_G(np.array([0.0, 0.0, 0.0]))

    # Add first rotational DOF to the general body
    rotHat_G1 = np.array([1.0, 0.0, 0.0])
    dcm_G10B = np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0]])
    theta1Init = 5.0 * macros.D2R
    thetaDot1Init = 0.0
    spring_constant_k = 100.0
    damper_constant_c = 0.0
    general_body.addRotationalDOF(rotHat_G1,
                                  dcm_G10B,
                                  theta1Init,
                                  thetaDot1Init,
                                  spring_constant_k,
                                  damper_constant_c)

    # Add second rotational DOF to the general body
    rotHat_G2 = np.array([0.0, 1.0, 0.0])
    dcm_G20B = np.array([[1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0],
                         [0.0, 0.0, 1.0]])
    theta2Init = 10.0 * macros.D2R
    thetaDot2Init = 0.0
    general_body.addRotationalDOF(rotHat_G2,
                                  dcm_G20B,
                                  theta2Init,
                                  thetaDot2Init,
                                  spring_constant_k,
                                  damper_constant_c)

    sc_object.addStateEffector(general_body)
    test_sim.AddModelToTask(task_name, general_body)

    # Set up data logging
    sc_state_data_log = sc_object.scStateOutMsg.recorder()
    general_body_inertial_state_data_log = general_body.generalSingleBodyConfigLogOutMsg.recorder()
    test_sim.AddModelToTask(task_name, sc_state_data_log)
    test_sim.AddModelToTask(task_name, general_body_inertial_state_data_log)

    general_body_theta_states_data_log = []
    for outMsg in general_body.spinningBodyOutMsgs:
        general_body_theta_states_data_log.append(outMsg.recorder())
        test_sim.AddModelToTask(task_name, general_body_theta_states_data_log[-1])

    # Rum the simulation
    sim_time = 10.0
    test_sim.InitializeSimulation()
    test_sim.ConfigureStopTime(macros.sec2nano(sim_time))
    test_sim.ExecuteSimulation()

    # Extract logged data
    timespan = sc_state_data_log.times() * macros.NANO2SEC
    sigma_BN = sc_state_data_log.sigma_BN
    sigma_GN = general_body_inertial_state_data_log.sigma_BN
    theta = []
    theta_dot = []

    for thetaData in general_body_theta_states_data_log:
        theta.append(thetaData.theta * macros.R2D)
        theta_dot.append(thetaData.thetaDot * macros.R2D)

    # Plot hub attitude
    plt.figure(1)
    plt.clf()
    plt.plot(timespan, sigma_BN[:, 0], label=r'$\sigma_{1}$')
    plt.plot(timespan, sigma_BN[:, 1], label=r'$\sigma_{2}$')
    plt.plot(timespan, sigma_BN[:, 2], label=r'$\sigma_{3}$')
    plt.title(r'$\sigma_{\mathcal{B}/\mathcal{N}}$ Hub Inertial MRP Attitude', fontsize=14)
    plt.ylabel('', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(loc='center right', prop={'size': 12})
    plt.grid(True)

    # Plot general body attitude
    plt.figure(2)
    plt.clf()
    plt.plot(timespan, sigma_GN[:, 0], label=r'$\sigma_{1}$')
    plt.plot(timespan, sigma_GN[:, 1], label=r'$\sigma_{2}$')
    plt.plot(timespan, sigma_GN[:, 2], label=r'$\sigma_{3}$')
    plt.title(r'$\sigma_{\mathcal{G}/\mathcal{N}}$ General Body Inertial MRP Attitude', fontsize=14)
    plt.ylabel('', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(loc='center right', prop={'size': 12})
    plt.grid(True)

    # Plot general body theta
    plt.figure(3)
    plt.clf()
    for idx, angle in enumerate(theta):
        plt.plot(timespan, angle, label=r'$\theta_' + str(idx + 1) + '$')
    plt.title(r'General Body Angle', fontsize=14)
    plt.ylabel('Angle (deg)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(loc='center right', prop={'size': 12})
    plt.grid(True)

    # Plot general body thetaDot
    plt.figure(4)
    plt.clf()
    for idx, angle_rate in enumerate(theta_dot):
        plt.plot(timespan, angle_rate, label=r'$\dot{\theta}_' + str(idx + 1) + '$')
    plt.title(r'General Body Angle Rate', fontsize=14)
    plt.ylabel('Angle Rate (deg/s)', fontsize=14)
    plt.xlabel('Time (min)', fontsize=14)
    plt.legend(loc='center right', prop={'size': 12})
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")


if __name__ == "__main__":
    test_generalSingleBodyStateEffector(True)
