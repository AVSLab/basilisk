#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        prescribedRot1DOF
#   Author:             Leah Kiner
#   Creation Date:      Nov 14, 2022
#

import pytest
import inspect
import matplotlib.pyplot as plt
import numpy as np
import os
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import prescribedRot1DOF  # import the module that is to be tested
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


# Vary the initial angle, reference angle, and maximum angular acceleration for pytest
@pytest.mark.parametrize("thetaInit", [0])
@pytest.mark.parametrize("thetaRef", [10])
@pytest.mark.parametrize("thetaDDotMax", [0.008, 0.1])
@pytest.mark.parametrize("accuracy", [1e-12])
def test_prescribedRot1DOFTestFunction(show_plots, thetaInit, thetaRef, thetaDDotMax, accuracy):
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
    ``thetaDot_Ref``, respectively.
    """
    [testResults, testMessage] = prescribedRot1DOFTestFunction(show_plots, thetaInit, thetaRef, thetaDDotMax, accuracy)

    assert testResults < 1, testMessage


def prescribedRot1DOFTestFunction(show_plots, thetaInit, thetaRef, thetaDDotMax, accuracy):
    numSteps = 5
    stepAngle = 1.0

    """Call this routine directly to run the unit test."""
    testFailCount = 0                                        # Zero the unit test result counter
    testMessages = []                                        # Create an empty array to store the test log messages
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testProcessRate = macros.sec2nano(0.1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the prescribedRot1DOF module to be tested
    PrescribedRot1DOF = prescribedRot1DOF.prescribedRot1DOF()
    PrescribedRot1DOF.ModelTag = "prescribedRot1DOF"

    # Add the prescribedRot1DOF test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, PrescribedRot1DOF)

    # Initialize the prescribedRot1DOF test module configuration data
    rotAxisM = np.array([1.0, 0.0, 0.0])
    prvInit_FM = thetaInit * rotAxisM

    PrescribedRot1DOFConfig.r_FM_M = np.array([1.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.rPrime_FM_M = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.rotAxis_M = rotAxisM
    PrescribedRot1DOFConfig.omega_FM_F = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.sigma_FM = rbk.PRV2MRP(prvInit_FM)
    stepTime = 2 * np.sqrt(stepAngle / thetaDDotMax)
    PrescribedRot1DOFConfig.stepTime = stepTime
    PrescribedRot1DOFConfig.stepAngle = stepAngle
    PrescribedRot1DOFConfig.thetaDotRef = 0.0

    # Create the prescribedRot1DOF input message
    MotorStepCountMessageData = messaging.MotorStepCountMsgPayload()
    MotorStepCountMessageData.numSteps = numSteps
    MotorStepCountMessage = messaging.MotorStepCountMsg().write(MotorStepCountMessageData)
    PrescribedRot1DOFConfig.motorStepCountInMsg.subscribeTo(MotorStepCountMessage)

    # Log the test module output message for data comparison
    prescribedDataLog = PrescribedRot1DOFConfig.prescribedMotionOutMsg.recorder()
    stepperMotorDataLog = PrescribedRot1DOFConfig.stepperMotorOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, prescribedDataLog)
    unitTestSim.AddModelToTask(unitTaskName, stepperMotorDataLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Sim chunk 1
    simTimeA = 0.05
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTimeA))
    unitTestSim.ExecuteSimulation()

    # Sim chunk 2
    simTimeB = numSteps * np.sqrt(((0.5 * np.abs(stepAngle)) * 8) / thetaDDotMax) + 50
    MotorStepCountMessageData = messaging.MotorStepCountMsgPayload()
    MotorStepCountMessageData.numSteps = 0
    MotorStepCountMessage = messaging.MotorStepCountMsg().write(MotorStepCountMessageData, macros.sec2nano(simTimeA))
    PrescribedRot1DOFConfig.motorStepCountInMsg.subscribeTo(MotorStepCountMessage)
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTimeA + simTimeB))
    unitTestSim.ExecuteSimulation()

    # Sim chunk 3
    simTimeC = 0.1
    MotorStepCountMessageData = messaging.MotorStepCountMsgPayload()
    MotorStepCountMessageData.numSteps = -5
    MotorStepCountMessage = messaging.MotorStepCountMsg().write(MotorStepCountMessageData, macros.sec2nano(simTimeA + simTimeB))
    PrescribedRot1DOFConfig.motorStepCountInMsg.subscribeTo(MotorStepCountMessage)
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTimeA + simTimeB + simTimeC))
    unitTestSim.ExecuteSimulation()

    # Sim chunk 3
    simTimeD = 5 * np.sqrt(((0.5 * np.abs(stepAngle)) * 8) / thetaDDotMax) + 50
    MotorStepCountMessageData = messaging.MotorStepCountMsgPayload()
    MotorStepCountMessageData.numSteps = 0
    MotorStepCountMessage = messaging.MotorStepCountMsg().write(MotorStepCountMessageData, macros.sec2nano(simTimeA + simTimeB + simTimeC))
    PrescribedRot1DOFConfig.motorStepCountInMsg.subscribeTo(MotorStepCountMessage)
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTimeA + simTimeB + simTimeC + simTimeD))
    unitTestSim.ExecuteSimulation()

    # Sim chunk 4
    simTimeE = 0.1
    MotorStepCountMessageData = messaging.MotorStepCountMsgPayload()
    MotorStepCountMessageData.numSteps = 7
    MotorStepCountMessage = messaging.MotorStepCountMsg().write(MotorStepCountMessageData, macros.sec2nano(simTimeA + simTimeB + simTimeC + simTimeD))
    PrescribedRot1DOFConfig.motorStepCountInMsg.subscribeTo(MotorStepCountMessage)
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTimeA + simTimeB + simTimeC + simTimeD + simTimeE))
    unitTestSim.ExecuteSimulation()
    # print(simTimeA + simTimeB + simTimeC)

    # Sim chunk 5
    simTimeF = 7 * np.sqrt(((0.5 * np.abs(stepAngle)) * 8) / thetaDDotMax) - 75
    MotorStepCountMessageData = messaging.MotorStepCountMsgPayload()
    MotorStepCountMessageData.numSteps = 0
    MotorStepCountMessage = messaging.MotorStepCountMsg().write(MotorStepCountMessageData, macros.sec2nano(simTimeA + simTimeB + simTimeC + simTimeD + simTimeE))
    PrescribedRot1DOFConfig.motorStepCountInMsg.subscribeTo(MotorStepCountMessage)
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTimeA + simTimeB + simTimeC + simTimeD + simTimeE + simTimeF))
    unitTestSim.ExecuteSimulation()

    # Sim chunk 6
    simTimeG = 0.1
    MotorStepCountMessageData = messaging.MotorStepCountMsgPayload()
    MotorStepCountMessageData.numSteps = -2
    MotorStepCountMessage = messaging.MotorStepCountMsg().write(MotorStepCountMessageData, macros.sec2nano(simTimeA + simTimeB + simTimeC + simTimeD + simTimeE + simTimeF))
    PrescribedRot1DOFConfig.motorStepCountInMsg.subscribeTo(MotorStepCountMessage)
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTimeA + simTimeB + simTimeC + simTimeD + simTimeE + simTimeF + simTimeG))
    unitTestSim.ExecuteSimulation()
    # print(simTimeA + simTimeB + simTimeC + simTimeD + simTimeE)

    # Sim chunk 6
    simTimeH = 2 * np.sqrt(((0.5 * np.abs(stepAngle)) * 8) / thetaDDotMax) + 50
    MotorStepCountMessageData = messaging.MotorStepCountMsgPayload()
    MotorStepCountMessageData.numSteps = 0
    MotorStepCountMessage = messaging.MotorStepCountMsg().write(MotorStepCountMessageData, macros.sec2nano(simTimeA + simTimeB + simTimeC + simTimeD + simTimeE + simTimeF + simTimeG))
    PrescribedRot1DOFConfig.motorStepCountInMsg.subscribeTo(MotorStepCountMessage)
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTimeA + simTimeB + simTimeC + simTimeD + simTimeE + simTimeF + simTimeG + simTimeH))
    unitTestSim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = prescribedDataLog.times()
    theta = stepperMotorDataLog.theta
    thetaDot = stepperMotorDataLog.thetaDot
    thetaDDot = stepperMotorDataLog.thetaDDot
    motorStepCount = stepperMotorDataLog.stepCount
    motorCommandedSteps = stepperMotorDataLog.numSteps
    sigma_FM = prescribedDataLog.sigma_FM
    omega_FM_F = prescribedDataLog.omega_FM_F
    omegaPrime_FM_F = prescribedDataLog.omegaPrime_FM_F

    # Convert the logged sigma_FM MRPs to a scalar theta_FM array
    n = len(timespan)
    phi_FM = []
    for i in range(n):
        phi_FM.append((180 / np.pi) * 4 * np.arctan(np.linalg.norm(sigma_FM[i, :])))

    # Plot motor theta
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, theta, label=r"$\theta$")
    plt.title(r'Stepper Motor Angle $\theta_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot motor thetaDot
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, thetaDot, label=r"$\dot{\theta}$")
    plt.title(r'Stepper Motor Angle Rate $\dot{\theta}_{\mathcal{F}/\mathcal{M}}$', fontsize=14)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot motor thetaDDot
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, thetaDDot, label=r"$\ddot{\theta}$")
    plt.title(r'Stepper Motor Angular Acceleration $\ddot{\theta}_{\mathcal{F}/\mathcal{M}}$ ', fontsize=14)
    plt.ylabel('(deg/s$^2$)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot steps commanded and motor steps taken
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, motorStepCount)
    plt.plot(timespan * macros.NANO2SEC, motorCommandedSteps, '--', label='Commanded')
    plt.title(r'Motor Step History', fontsize=14)
    plt.ylabel('Steps', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot phi_FM
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, phi_FM, label=r"$\Phi$")
    plt.title(r'$\Phi_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory', fontsize=14)
    plt.ylabel('(deg)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot omega_FM_F
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, omega_FM_F[:, 0], label=r'$\omega_{1}$')
    plt.plot(timespan * macros.NANO2SEC, omega_FM_F[:, 1], label=r'$\omega_{2}$')
    plt.plot(timespan * macros.NANO2SEC, omega_FM_F[:, 2], label=r'$\omega_{3}$')
    plt.title(r'${}^\mathcal{F} \omega_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory', fontsize=14)
    plt.ylabel('(deg/s)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    # Plot omegaPrime_FM_F
    plt.figure()
    plt.clf()
    plt.plot(timespan * macros.NANO2SEC, omegaPrime_FM_F[:, 0], label=r'1')
    plt.plot(timespan * macros.NANO2SEC, omegaPrime_FM_F[:, 1], label=r'2')
    plt.plot(timespan * macros.NANO2SEC, omegaPrime_FM_F[:, 2], label=r'3')
    plt.title(r'${}^\mathcal{F} \omega Prime_{\mathcal{F}/\mathcal{M}}$ Profiled Trajectory', fontsize=14)
    plt.ylabel('(deg/s$^2$)', fontsize=14)
    plt.xlabel('Time (s)', fontsize=14)
    plt.legend(loc='upper right', prop={'size': 12})
    plt.grid(True)

    if show_plots:
        plt.show()
    plt.close("all")

    # Check to ensure the initial angle rate converged to the reference angle rate
    if not unitTestSupport.isDoubleEqual(thetaDot_Final, thetaDotRef, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + PrescribedRot1DOF.ModelTag + "thetaDot_Final and thetaDotRef do not match")

    # Check to ensure the initial angle converged to the reference angle
    if not unitTestSupport.isDoubleEqual(theta_FM_Final, thetaRef, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + PrescribedRot1DOF.ModelTag + "theta_FM_Final and thetaRef do not match")
    return [testFailCount, ''.join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    prescribedRot1DOFTestFunction(
                 True,
                 0,     # thetaInit
                 10,     # thetaRef
                 0.008,       # thetaDDotMax
                 1e-12        # accuracy
               )
