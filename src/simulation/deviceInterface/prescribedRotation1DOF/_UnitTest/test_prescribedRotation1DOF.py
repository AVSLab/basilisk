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
#   Last Updated:       March 28, 2024

import inspect
import os

import matplotlib.pyplot as plt
import numpy as np
import pytest

from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.simulation import prescribedRotation1DOF
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

@pytest.mark.parametrize("coastOptionBangDuration", [0.0, 2.0])  # [s]
@pytest.mark.parametrize("smoothingDuration", [0.0, 2.0])  # [s]
@pytest.mark.parametrize("thetaInit", [0.0, macros.D2R * -5.0])  # [rad]
@pytest.mark.parametrize("thetaRef1", [0.0, macros.D2R * -10.0])  # [rad]
@pytest.mark.parametrize("thetaRef2", [macros.D2R * 5.0])  # [rad]
@pytest.mark.parametrize("thetaDDotMax", [macros.D2R * 0.05, macros.D2R * 0.1])  # [rad/s^2]
@pytest.mark.parametrize("accuracy", [1e-8])
def test_prescribedRotation1DOF(show_plots,
                                coastOptionBangDuration,
                                smoothingDuration,
                                thetaInit,
                                thetaRef1,
                                thetaRef2,
                                thetaDDotMax,
                                accuracy):
    r"""
    **Validation Test Description**

    The unit test for this module ensures that the profiled 1 DOF rotation for a secondary rigid body relative to
    the spacecraft hub is properly computed for several different simulation configurations. The unit test profiles
    two successive rotations to ensure the module is correctly configured. The initial spinning body angle relative
    to the spacecraft hub is varied, along with the two final reference angles and the maximum angular acceleration
    for the rotation.

    This unit test also tests four different methods of profiling the rotation. Two profilers prescribe a pure
    bang-bang or bang-coast-bang angular acceleration profile for the rotation. The bang-bang option results in
    the fastest possible rotation; while the bang-coast-bang option includes a coast period with zero acceleration
    between the acceleration segments. The other two profilers apply smoothing to the bang-bang and bang-coast-bang
    acceleration profiles so that the spinning body hub-relative rates start and end at zero.

    **Test Parameters**

    Args:
        show_plots (bool): Variable for choosing whether plots should be displayed
        coastOptionBangDuration: (float): [s] Time the acceleration is applied during the bang segments
        (Variable must be nonzero to select the bang-coast-bang option)
        smoothingDuration (float) [s] Time the acceleration is smoothed to the given maximum acceleration value
        (Variable must be nonzero to toggle the smoothed profiler options)
        thetaInit (float): [rad] Initial spinning body angle relative to the mount frame
        thetaRef1 (float): [rad] First spinning body reference angle relative to the mount frame
        thetaRef2 (float): [rad] Second spinning body reference angle relative to the mount frame
        thetaDDotMax (float): [rad/s^2] Maximum angular acceleration for the rotation
        accuracy (float): Absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    To verify the module functionality, the final angle at the end of each rotation is checked to match the specified
    reference angle (``thetaRef1`` and ``thetaRef2``). Additionally, for the smoothed profiler options,
    the numerical derivative of the profiled angles and their rates is determined across the entire simulation in this
    test script. These numerical derivatives are checked with the profiled angular accelerations and angle rates to
    ensure the profiled acceleration is correctly integrated in the module to obtain the angles and their rates.
    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testTimeStepSec = 0.1  # [s]
    testProcessRate = macros.sec2nano(testTimeStepSec)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the prescribedRotation1DOF module to be tested
    rotAxis_M = np.array([1.0, 0.0, 0.0])  # Spinning body rotation axis
    prescribedRot1DOF = prescribedRotation1DOF.PrescribedRotation1DOF()
    prescribedRot1DOF.ModelTag = "prescribedRotation1DOF"
    prescribedRot1DOF.setCoastOptionBangDuration(coastOptionBangDuration)
    prescribedRot1DOF.setRotHat_M(rotAxis_M)
    prescribedRot1DOF.setSmoothingDuration(smoothingDuration)
    prescribedRot1DOF.setThetaDDotMax(thetaDDotMax)
    prescribedRot1DOF.setThetaInit(thetaInit)
    unitTestSim.AddModelToTask(unitTaskName, prescribedRot1DOF)

    # Create the reference angle input message for the first rotation
    hingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    hingedRigidBodyMessageData.theta = thetaRef1  # [rad]
    hingedRigidBodyMessageData.thetaDot = 0.0  # [rad/s]
    hingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(hingedRigidBodyMessageData)
    prescribedRot1DOF.spinningBodyInMsg.subscribeTo(hingedRigidBodyMessage)

    # Log module data for module unit test validation
    prescribedRotStatesDataLog = prescribedRot1DOF.prescribedRotationOutMsg.recorder()
    scalarAngleDataLog = prescribedRot1DOF.spinningBodyOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, prescribedRotStatesDataLog)
    unitTestSim.AddModelToTask(unitTaskName, scalarAngleDataLog)

    # Execute the first spinning body rotation
    simTime = 5 * 60  # [s]
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime))
    unitTestSim.ExecuteSimulation()

    # Create the reference angle input message for the second rotation
    hingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    hingedRigidBodyMessageData.theta = thetaRef2  # [rad]
    hingedRigidBodyMessageData.thetaDot = 0.0  # [rad/s]
    hingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(hingedRigidBodyMessageData)
    prescribedRot1DOF.spinningBodyInMsg.subscribeTo(hingedRigidBodyMessage)

    # Execute the second spinning body rotation
    unitTestSim.ConfigureStopTime(macros.sec2nano(2 * simTime))
    unitTestSim.ExecuteSimulation()

    # Extract logged data
    timespan = macros.NANO2SEC * scalarAngleDataLog.times()  # [s]
    omega_FM_F = macros.R2D * prescribedRotStatesDataLog.omega_FM_F  # [deg/s]
    omegaPrime_FM_F = macros.R2D * prescribedRotStatesDataLog.omegaPrime_FM_F  # [deg/s^2]
    sigma_FM = prescribedRotStatesDataLog.sigma_FM
    theta = macros.R2D * scalarAngleDataLog.theta  # [deg]
    thetaDot = macros.R2D * scalarAngleDataLog.thetaDot  # [deg/s]
    thetaDDot = omegaPrime_FM_F.dot(rotAxis_M)  # [deg/s^2]

    # Unit test validation 1: Check that the profiler converges to the required final angles
    tf_1_index = int(round(simTime / testTimeStepSec)) + 1
    thetaFinal1 = theta[tf_1_index]
    thetaFinal2 = theta[-1]
    thetaFinalList = [thetaFinal1, thetaFinal2]  # [deg]
    thetaRefList = [macros.R2D * thetaRef1, macros.R2D * thetaRef2]  # [deg]
    np.testing.assert_allclose(thetaRefList,
                               thetaFinalList,
                               atol=accuracy,
                               verbose=True)

    # Unit test validation 2: Numerically check that the profiled accelerations, angle rates, and angles are correct
    if (smoothingDuration > 0.0):
        thetaDDotNumerical = []
        thetaDotNumerical = []
        for i in range(len(timespan) - 1):
            # First order forward difference
            thetaDDotNumerical.append((thetaDot[i+1] - thetaDot[i]) / testTimeStepSec)
            thetaDotNumerical.append((theta[i+1] - theta[i]) / testTimeStepSec)

        np.testing.assert_allclose(thetaDDot[0:-1],
                                   thetaDDotNumerical,
                                   atol=1e-2,
                                   verbose=True)
        np.testing.assert_allclose(thetaDot[0:-1],
                                   thetaDotNumerical,
                                   atol=1e-2,
                                   verbose=True)
        if show_plots:
            # Plot the difference between the numerical and profiled results
            plt.figure()
            plt.clf()
            plt.plot(timespan[0:-1], thetaDDotNumerical - thetaDDot[0:-1], label=r'$\ddot{\theta}$')
            plt.plot(timespan[0:-1], thetaDotNumerical - thetaDot[0:-1], label=r"$\dot{\theta}$")
            plt.title(r'Profiled vs Numerical Difference', fontsize=14)
            plt.legend(loc='upper right', prop={'size': 12})
            plt.grid(True)

    if show_plots:
        # 1. Plot the scalar spinning body rotational states
        # 1A. Plot theta
        thetaInitPlotting = np.ones(len(timespan)) * macros.R2D * thetaInit  # [deg]
        thetaRef1Plotting = np.ones(len(timespan)) * macros.R2D * thetaRef1  # [deg]
        thetaRef2Plotting = np.ones(len(timespan)) * macros.R2D * thetaRef2  # [deg]
        plt.figure()
        plt.clf()
        plt.plot(timespan, theta, label=r"$\theta$")
        plt.plot(timespan, thetaInitPlotting, '--', label=r'$\theta_{0}$')
        plt.plot(timespan, thetaRef1Plotting, '--', label=r'$\theta_{Ref_1}$')
        plt.plot(timespan, thetaRef2Plotting, '--', label=r'$\theta_{Ref_2}$')
        plt.title(r'Profiled Angle $\theta_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
        plt.ylabel('(deg)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper right', prop={'size': 12})
        plt.grid(True)

        # 1B. Plot thetaDot
        plt.figure()
        plt.clf()
        plt.plot(timespan, thetaDot, label=r"$\dot{\theta}$")
        plt.title(r'Profiled Angle Rate $\dot{\theta}_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
        plt.ylabel('(deg/s)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper right', prop={'size': 12})
        plt.grid(True)

        # 1C. Plot thetaDDot
        plt.figure()
        plt.clf()
        plt.plot(timespan, thetaDDot, label=r"$\ddot{\theta}$")
        plt.title(r'Profiled Angular Acceleration $\ddot{\theta}_{\mathcal{F}/\mathcal{M}}$ ', fontsize=14)
        plt.ylabel('(deg/s$^2$)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper right', prop={'size': 12})
        plt.grid(True)

        # 2. Plot the spinning body prescribed rotational states
        # 2A. Plot PRV angle from sigma_FM
        phi_FM = []
        for i in range(len(timespan)):
            phi_FM.append(macros.R2D * 4 * np.arctan(np.linalg.norm(sigma_FM[i, :])))  # [deg]

        plt.figure()
        plt.clf()
        plt.plot(timespan, phi_FM, label=r"$\Phi$")
        plt.title(r'Profiled PRV Angle $\Phi_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
        plt.ylabel('(deg)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='center right', prop={'size': 14})
        plt.grid(True)

        # 2B. Plot omega_FM_F
        plt.figure()
        plt.clf()
        plt.plot(timespan, omega_FM_F[:, 0], label=r'$\omega_{1}$')
        plt.plot(timespan, omega_FM_F[:, 1], label=r'$\omega_{2}$')
        plt.plot(timespan, omega_FM_F[:, 2], label=r'$\omega_{3}$')
        plt.title(r'Profiled Angular Velocity ${}^\mathcal{F} \omega_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
        plt.ylabel('(deg/s)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper right', prop={'size': 14})
        plt.grid(True)

        # 2C. Plot omegaPrime_FM_F
        plt.figure()
        plt.clf()
        plt.plot(timespan, omegaPrime_FM_F[:, 0], label=r'1')
        plt.plot(timespan, omegaPrime_FM_F[:, 1], label=r'2')
        plt.plot(timespan, omegaPrime_FM_F[:, 2], label=r'3')
        plt.title(r'Profiled Angular Acceleration ${}^\mathcal{F} \omega$Prime$_{\mathcal{F}/\mathcal{M}}$',
                  fontsize=14)
        plt.ylabel('(deg/s$^2$)', fontsize=14)
        plt.xlabel('Time (s)', fontsize=14)
        plt.legend(loc='upper right', prop={'size': 14})
        plt.grid(True)

        plt.show()
    plt.close("all")

if __name__ == "__main__":
    test_prescribedRotation1DOF(
        True,  # show_plots
        2.0,  # [s] coastOptionBangDuration
        2.0,  # [s] smoothingDuration
        macros.D2R * -5.0,  # [rad] thetaInit
        macros.D2R * -10.0,  # [rad] thetaRef1
        macros.D2R * 5.0,  # [rad] thetaRef2
        macros.D2R * 0.1,  # [rad/s^2] thetaDDotMax
        1e-8  # accuracy
    )
