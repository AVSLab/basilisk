#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


#
#   Unit Test Script
#   Module Name:        PrescribedRotation1DOF
#   Author:             Leah Kiner
#   Creation Date:      Nov 14, 2022
#   Last Updated:       Jan 26, 2024

import inspect
import os

import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.simulation import prescribedRotation1DOF
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


@pytest.mark.parametrize("thetaInit", [0, 2*np.pi/3])  # [rad]
@pytest.mark.parametrize("thetaRef", [0, 2*np.pi/3])  # [rad]
@pytest.mark.parametrize("thetaDDotMax", [0.008, 0.1])  # [rad/s^2]
@pytest.mark.parametrize("accuracy", [1e-12])
def test_prescribedRotation(show_plots, thetaInit, thetaRef, thetaDDotMax, accuracy):
    r"""
    **Validation Test Description**

    This unit test ensures that the profiled 1 DOF attitude maneuver for a secondary rigid body connected
    to the spacecraft hub is properly computed for a series of initial and reference PRV angles and maximum
    angular accelerations. The final prescribed attitude and angular velocity magnitude are compared with
    the reference values.

    **Test Parameters**

    Args:
        thetaInit (float): [rad] Initial PRV angle of the F frame with respect to the M frame
        thetaRef (float): [rad] Reference PRV angle of the F frame with respect to the M frame
        thetaDDotMax (float): [rad/s^2] Maximum angular acceleration for the attitude maneuver
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    This unit test ensures that the profiled 1 DOF rotational attitude maneuver is properly computed for a series of
    initial and reference PRV angles and maximum angular accelerations. The final prescribed angle ``theta_FM_Final``
    and angular velocity magnitude ``thetaDot_Final`` are compared with the reference values ``theta_Ref`` and
    zero, respectively.
    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testProcessRate = macros.sec2nano(0.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the prescribedRotation1DOF module to be tested
    PrescribedRotation1DOF = prescribedRotation1DOF.PrescribedRotation1DOF()
    PrescribedRotation1DOF.ModelTag = "PrescribedRotation1DOF"

    unitTestSim.AddModelToTask(unitTaskName, PrescribedRotation1DOF)

    # Initialize the PrescribedRotation1DOF test module configuration data
    rotAxisM = np.array([1.0, 0.0, 0.0])
    prvInit_FM = thetaInit * rotAxisM
    PrescribedRotation1DOF.rotAxis_M = rotAxisM
    PrescribedRotation1DOF.thetaDDotMax = thetaDDotMax
    PrescribedRotation1DOF.omega_FM_F = np.array([0.0, 0.0, 0.0])
    PrescribedRotation1DOF.omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])
    PrescribedRotation1DOF.sigma_FM = rbk.PRV2MRP(prvInit_FM)

    # Create the PrescribedRotation1DOF input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = thetaRef
    HingedRigidBodyMessageData.thetaDot = 0.0
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData)
    PrescribedRotation1DOF.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)

    # Log the test module output message for data comparison
    dataLog = PrescribedRotation1DOF.prescribedRotationOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    simTime = np.sqrt(((0.5 * np.abs(thetaRef - thetaInit)) * 8) / thetaDDotMax) + 1  # [s]
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))
    unitTestSim.InitializeSimulation()
    unitTestSim.ExecuteSimulation()

    # Extract logged data
    timespan = dataLog.times() * macros.NANO2SEC  # [s]
    omega_FM_F = dataLog.omega_FM_F * macros.R2D  # [deg/s]
    sigma_FM = dataLog.sigma_FM

    # Convert the logged sigma_FM MRPs to a scalar theta angle array
    theta = []
    for i in range(len(timespan)):
        theta.append(macros.R2D * 4 * np.arctan(np.linalg.norm(sigma_FM[i, :])))  # [deg]

    # Plot theta
    thetaRef_plotting = np.ones(len(timespan)) * macros.R2D * thetaRef  # [deg]
    thetaInit_plotting = np.ones(len(timespan)) * macros.R2D * thetaInit  # [deg]
    plt.figure()
    plt.clf()
    plt.plot(timespan, theta, label=r"$\theta$")
    plt.plot(timespan, thetaRef_plotting, '--', label=r'$\theta_{Ref}$')
    plt.plot(timespan, thetaInit_plotting, '--', label=r'$\theta_{0}$')
    plt.title(r'$\theta_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory', fontsize=18)
    plt.ylabel('(deg)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='center right', prop={'size': 16})
    plt.grid(True)

    # Plot omega_FM_F
    plt.figure()
    plt.clf()
    plt.plot(timespan, omega_FM_F[:, 0], label=r'$\omega_{1}$')
    plt.plot(timespan, omega_FM_F[:, 1], label=r'$\omega_{2}$')
    plt.plot(timespan, omega_FM_F[:, 2], label=r'$\omega_{3}$')
    plt.title(r'${}^\mathcal{F} \omega_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory', fontsize=18)
    plt.ylabel('(deg/s)', fontsize=16)
    plt.xlabel('Time (s)', fontsize=16)
    plt.legend(loc='upper right', prop={'size': 16})
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")

    thetaDotFinal = np.linalg.norm(omega_FM_F[-1, :])  # [deg/s]
    thetaFinal = 4 * np.arctan(np.linalg.norm(sigma_FM[-1, :]))  # [deg]

    np.testing.assert_allclose(thetaDotFinal,
                               0.0,
                               atol=accuracy,
                               err_msg="thetaDofRef and thetaDot_Final do not match",
                               verbose=True)

    np.testing.assert_allclose(thetaFinal,
                               thetaRef,
                               atol=accuracy,
                               err_msg="thetaRef and theta_Final do not match",
                               verbose=True)


if __name__ == "__main__":
    test_prescribedRotation(
        True,
        np.pi/6,  # [rad] thetaInit
        2*np.pi/3,  # [rad] thetaRef
        0.008,  # [rad/s^2] thetaDDotMax
        1e-12  # accuracy
        )
