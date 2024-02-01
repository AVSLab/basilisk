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


@pytest.mark.parametrize("coastOptionRampDuration", [0.0, 2.0, 5.0])  # [s]
@pytest.mark.parametrize("thetaInit", [0.0, macros.D2R * -25.0])  # [rad]
@pytest.mark.parametrize("thetaRef1", [0.0, macros.D2R * -15.0])  # [rad]
@pytest.mark.parametrize("thetaRef2", [macros.D2R * -25.0, macros.D2R * 35.0])  # [rad]
@pytest.mark.parametrize("thetaDDotMax", [macros.D2R * 0.4, macros.D2R * 1.0])  # [rad/s^2]
@pytest.mark.parametrize("accuracy", [1e-4])
def test_prescribedRotation1DOF(show_plots,
                            coastOptionRampDuration,
                            thetaInit,
                            thetaRef1,
                            thetaRef2,
                            thetaDDotMax,
                            accuracy):
    r"""
    **Validation Test Description**

    This unit test ensures that a profiled 1 DOF rotation for a secondary rigid body connected to a spacecraft hub
    is properly computed for several different simulation configurations. This unit test profiles two successive
    rotations for the spinning body to ensure the module is correctly configured. The initial spinning body angle
    relative to the spacecraft hub is varied, along with the two final reference angles and the maximum angular
    acceleration for the rotation. This unit test also tests both methods of profiling the rotation, where either
    a pure bang-bang acceleration profile can be selected for the rotation, or a coast option can be selected
    where the accelerations are only applied for a specified ramp time and a coast segment with zero acceleration
    is applied between the two acceleration periods. To validate the module, the final spinning body angles at the
    end of each rotation are checked to match the specified reference angles.

    **Test Parameters**

    Args:
        coastOptionRampDuration (double): [s] Ramp duration used for the coast option
        thetaInit (float): [rad] Initial spinning body angle relative to the mount frame
        thetaRef1 (float): [rad] First spinning body reference angle relative to the mount frame
        thetaRef2 (float): [rad] Second spinning body reference angle relative to the mount frame
        thetaDDotMax (float): [rad/s^2] Maximum angular acceleration for the rotation
        accuracy (float): Absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    This unit test checks that the final spinning body angles at the end of each rotation converge to the specified
    reference values ``thetaRef1`` and ``thetaRef2``.
    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testTimeStepSec = 0.01  # [s]
    testProcessRate = macros.sec2nano(testTimeStepSec)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the prescribedRotation1DOF module to be tested
    PrescribedRotation1DOF = prescribedRotation1DOF.PrescribedRotation1DOF()
    PrescribedRotation1DOF.ModelTag = "PrescribedRotation1DOF"

    unitTestSim.AddModelToTask(unitTaskName, PrescribedRotation1DOF)

    # Initialize the PrescribedRotation1DOF test module configuration data
    rotAxisM = np.array([1.0, 0.0, 0.0])
    PrescribedRotation1DOF.setCoastOptionRampDuration(coastOptionRampDuration)
    PrescribedRotation1DOF.setRotAxis_M(rotAxisM)
    PrescribedRotation1DOF.setThetaDDotMax(thetaDDotMax)
    PrescribedRotation1DOF.setThetaInit(thetaInit)

    # Create the prescribedRotation1DOF input reference angle message for the first spinning body rotation
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = thetaRef1
    HingedRigidBodyMessageData.thetaDot = 0.0
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData)
    PrescribedRotation1DOF.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)

    # Log module data for module unit test validation
    prescribedRotStatesDataLog = PrescribedRotation1DOF.prescribedRotationOutMsg.recorder()
    scalarAngleDataLog = PrescribedRotation1DOF.spinningBodyOutMsg.recorder()
    thetaDDotLog = PrescribedRotation1DOF.logger("thetaDDot", testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, prescribedRotStatesDataLog)
    unitTestSim.AddModelToTask(unitTaskName, scalarAngleDataLog)
    unitTestSim.AddModelToTask(unitTaskName, thetaDDotLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Determine the required simulation time for the first rotation
    thetaDotInit = 0.0  # [rad/s]
    tCoast_1 = 0.0  # [s]
    if (coastOptionRampDuration > 0.0):
        # Determine the angle and angle rate at the end of the ramp segment/start of the coast segment
        if (thetaInit < thetaRef1):
            theta_tr_1 = ((0.5 * thetaDDotMax * coastOptionRampDuration * coastOptionRampDuration)
                          + (thetaDotInit * coastOptionRampDuration) + thetaInit)  # [rad]
            thetaDot_tr_1 = thetaDDotMax * coastOptionRampDuration + thetaDotInit  # [rad/s]
        else:
            theta_tr_1 = - ((0.5 * thetaDDotMax * coastOptionRampDuration * coastOptionRampDuration)
                            + (thetaDotInit * coastOptionRampDuration)) + thetaInit  # [rad]
            thetaDot_tr_1 = - thetaDDotMax * coastOptionRampDuration + thetaDotInit  # [rad/s]

        # Determine the angle traveled during the coast period
        deltaThetaCoast_1 = thetaRef1 - thetaInit - 2 * (theta_tr_1 - thetaInit)  # [rad]

        # Determine the time duration of the coast segment
        tCoast_1 = np.abs(deltaThetaCoast_1) / np.abs(thetaDot_tr_1)  # [s]
        rotation1ReqTime = (2 * coastOptionRampDuration) + tCoast_1  # [s]
    else:
        rotation1ReqTime = np.sqrt(((0.5 * np.abs(thetaRef1 - thetaInit)) * 8) / thetaDDotMax)  # [s]

    rotation1ExtraTime = 5  # [s]
    unitTestSim.ConfigureStopTime(macros.sec2nano(rotation1ReqTime + rotation1ExtraTime))

    # Execute the first spinning body rotation
    unitTestSim.ExecuteSimulation()

    # Create the hingedRigidBody reference angle input message for the second rotation
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = thetaRef2  # [rad]
    HingedRigidBodyMessageData.thetaDot = 0.0  # [rad/s]
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData)
    PrescribedRotation1DOF.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)

    # Determine the required simulation time for the second rotation
    tCoast_2 = 0.0  # [s]
    if (coastOptionRampDuration > 0.0):
        # Determine the angle and angle rate at the end of the ramp segment/start of the coast segment
        if (thetaRef1 < thetaRef2):
            theta_tr_2 = ((0.5 * thetaDDotMax * coastOptionRampDuration * coastOptionRampDuration)
                          + (thetaDotInit * coastOptionRampDuration) + thetaRef1)  # [rad]
            thetaDot_tr_2 = thetaDDotMax * coastOptionRampDuration + thetaDotInit  # [rad/s]
        else:
            theta_tr_2 = - ((0.5 * thetaDDotMax * coastOptionRampDuration * coastOptionRampDuration)
                            + (thetaDotInit * coastOptionRampDuration)) + thetaRef1  # [rad]
            thetaDot_tr_2 = - thetaDDotMax * coastOptionRampDuration + thetaDotInit  # [rad/s]

        # Determine the angle traveled during the coast period
        deltaThetaCoast_2 = thetaRef2 - thetaRef1 - 2 * (theta_tr_2 - thetaRef1)  # [rad]

        # Determine the time duration of the coast segment
        tCoast_2 = np.abs(deltaThetaCoast_2) / np.abs(thetaDot_tr_2)  # [s]
        rotation2ReqTime = (2 * coastOptionRampDuration) + tCoast_2  # [s]
    else:
        rotation2ReqTime = np.sqrt(((0.5 * np.abs(thetaRef2 - thetaRef1)) * 8) / thetaDDotMax)  # [s]

    rotation2ExtraTime = 5.0  # [s]
    unitTestSim.ConfigureStopTime(macros.sec2nano(rotation1ReqTime
                                                  + rotation1ExtraTime
                                                  + rotation2ReqTime
                                                  + rotation2ExtraTime))

    # Execute the second spinning body rotation
    unitTestSim.ExecuteSimulation()

    # Extract logged data
    timespan = macros.NANO2SEC * scalarAngleDataLog.times()  # [s]
    theta = macros.R2D * scalarAngleDataLog.theta  # [deg]
    thetaDot = macros.R2D * scalarAngleDataLog.thetaDot  # [deg/s]
    thetaDDot = macros.R2D * thetaDDotLog.thetaDDot  # [deg/s^2]
    sigma_FM = prescribedRotStatesDataLog.sigma_FM
    omega_FM_F = macros.R2D * prescribedRotStatesDataLog.omega_FM_F  # [deg/s]
    omegaPrime_FM_F = macros.R2D * prescribedRotStatesDataLog.omegaPrime_FM_F  # [deg/s^2]

    # Unit test validation
    # Store the truth data used to validate the module in two lists
    if (coastOptionRampDuration > 0.0):
        # Compute tf for the first rotation, and tInit tf for the second rotation
        tf_1 = 2 * coastOptionRampDuration + tCoast_1  # [s]
        tInit_2 = rotation1ReqTime + rotation1ExtraTime  # [s]
        tf_2 = tInit_2 + (2 * coastOptionRampDuration) + tCoast_2  # [s]
    
        # Compute the timespan indices for each check
        tf_1_index = int(round(tf_1 / testTimeStepSec)) + 1
        tInit_2_index = int(round(tInit_2 / testTimeStepSec)) + 1
        tf_2_index = int(round(tf_2 / testTimeStepSec)) + 1
    
        # Store the timespan indices in a list
        timeCheckIndicesList = [tf_1_index,
                                tInit_2_index,
                                tf_2_index]
    
        # Store the angles to check in a list
        thetaCheckList = [macros.R2D * thetaRef1,
                          macros.R2D * thetaRef1,
                          macros.R2D * thetaRef2]
    
    else:
        # Compute tf for the first rotation, and tInit tf for the second rotation
        tf_1 = rotation1ReqTime  # [s]
        tInit_2 = rotation1ReqTime + rotation1ExtraTime  # [s]
        tf_2 = tInit_2 + rotation2ReqTime  # [s]
    
        # Compute the timespan indices for each check
        tf_1_index = int(round(tf_1 / testTimeStepSec)) + 1
        tInit_2_index = int(round(tInit_2 / testTimeStepSec)) + 1
        tf_2_index = int(round(tf_2 / testTimeStepSec)) + 1
    
        # Store the timespan indices in a list
        timeCheckIndicesList = [tf_1_index,
                                tInit_2_index,
                                tf_2_index]
    
        # Store the angles to check in a list
        thetaCheckList = [macros.R2D * thetaRef1,
                          macros.R2D * thetaRef1,
                          macros.R2D * thetaRef2]

    # Use the two truth data lists to compare with the module-extracted data
    np.testing.assert_allclose(theta[timeCheckIndicesList],
                               thetaCheckList,
                               atol=accuracy,
                               verbose=True)

    # 1. Plot the scalar spinning body rotational states
    # 1A. Plot theta
    thetaInit_plotting = np.ones(len(timespan)) * macros.R2D * thetaInit
    thetaRef1_plotting = np.ones(len(timespan)) * macros.R2D * thetaRef1
    thetaRef2_plotting = np.ones(len(timespan)) * macros.R2D * thetaRef2
    plt.figure()
    plt.clf()
    plt.plot(timespan, theta, label=r"$\theta$")
    plt.plot(timespan, thetaInit_plotting, '--', label=r'$\theta_{0}$')
    plt.plot(timespan, thetaRef1_plotting, '--', label=r'$\theta_{Ref_1}$')
    plt.plot(timespan, thetaRef2_plotting, '--', label=r'$\theta_{Ref_2}$')
    plt.title(r'Spinning Body Angle $\theta_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # 1B. Plot thetaDot
    plt.figure()
    plt.clf()
    plt.plot(timespan, thetaDot, label=r"$\dot{\theta}$")
    plt.title(r'Spinning Body Angle Rate $\dot{\theta}_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # 1C. Plot thetaDDot
    plt.figure()
    plt.clf()
    plt.plot(timespan, thetaDDot, label=r"$\ddot{\theta}$")
    plt.title(r'Spinning Body Angular Acceleration $\ddot{\theta}_{\mathcal{F}/\mathcal{M}}$ ', fontsize=14)
    plt.ylabel('(deg/s$^2$)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # 2. Plot the spinning body prescribed rotational states
    # 2A. Plot PRV angle from sigma_FM
    phi_FM = []
    for i in range(len(timespan)):
        phi_FM.append(macros.R2D * 4 * np.arctan(np.linalg.norm(sigma_FM[i, :])))

    plt.figure()
    plt.clf()
    plt.plot(timespan, phi_FM, label=r"$\Phi$")
    plt.title(r'Spinning Body PRV Angle $\Phi_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
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
    plt.title(r'Spinning Body Angular Velocity ${}^\mathcal{F} \omega_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
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
    plt.title(r'Spinning Body Angular Acceleration ${}^\mathcal{F} \omega$Prime$_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
    plt.ylabel('(deg/s$^2$)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 14})
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")


if __name__ == "__main__":
    test_prescribedRotation1DOF(
        True,  # show_plots
        5.0,  # [s] coastOptionRampDuration
        macros.D2R * -25.0,  # [rad] thetaInit
        macros.D2R * 15.0,  # [rad] thetaRef1
        macros.D2R * 55.0,  # [rad] thetaRef2
        macros.D2R * 0.4,  # [rad/s^2] thetaDDotMax
        1e-4  # accuracy
    )
