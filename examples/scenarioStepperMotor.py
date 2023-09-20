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
from Basilisk.fswAlgorithms import stepperMotor

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
@pytest.mark.parametrize("accuracy", [1e-12])
def test_stepperMotor(show_plots, thetaInit, thetaRef, accuracy):
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
    [testResults, testMessage] = stepperMotorTestFunction(show_plots, thetaInit, thetaRef, accuracy)

    assert testResults < 1, testMessage


def stepperMotorTestFunction(show_plots, thetaInit, thetaRef, accuracy):
    numSteps = thetaRef - thetaInit
    testFailCount = 0                                        # Zero the unit test result counter
    testMessages = []                                        # Create an empty array to store the test log messages
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testProc = unitTestSim.CreateNewProcess(unitProcessName)

    testProc.addTask(unitTestSim.CreateNewTask("fswTask1", macros.sec2nano(0.1)))
    testProc.addTask(unitTestSim.CreateNewTask("fswTask2", macros.sec2nano(0.01)))

    # Create an instance of the stepperMotor module to be tested
    stepperMotorConfig = stepperMotor.StepperMotorConfig()
    StepperMotorWrap = unitTestSim.setModelDataWrap(stepperMotorConfig)
    StepperMotorWrap.ModelTag = "stepperMotor"
    stepAngle = 1.0
    stepperMotorConfig.stepAngle = stepAngle
    motorRPM = 0.5
    stepTime = 60 / ( motorRPM * (360 / stepAngle) )
    print("Step Time:")
    print(stepTime)
    stepperMotorConfig.stepTime = stepTime

    # Add the stepperMotor test module to runtime call list
    unitTestSim.AddModelToTask("fswTask1", StepperMotorWrap, stepperMotorConfig)

    # Create the stepperMotor input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = thetaRef
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData)
    stepperMotorConfig.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)

    # Create an instance of the prescribedRot1DOF module to be tested
    PrescribedRot1DOFConfig = prescribedRot1DOF.PrescribedRot1DOFConfig()
    PrescribedWrap = unitTestSim.setModelDataWrap(PrescribedRot1DOFConfig)
    PrescribedWrap.ModelTag = "prescribedRot1DOF"

    # Add the prescribedRot1DOF test module to runtime call list
    unitTestSim.AddModelToTask("fswTask2", PrescribedWrap, PrescribedRot1DOFConfig)

    # Initialize the prescribedRot1DOF test module configuration data
    rotAxisM = np.array([1.0, 0.0, 0.0])
    prvInit_FM = thetaInit * rotAxisM
    PrescribedRot1DOFConfig.r_FM_M = np.array([1.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.rPrime_FM_M = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.rotAxis_M = rotAxisM
    # PrescribedRot1DOFConfig.thetaDDotMax = thetaDDotMax
    PrescribedRot1DOFConfig.omega_FM_F = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.omegaPrime_FM_F = np.array([0.0, 0.0, 0.0])
    PrescribedRot1DOFConfig.sigma_FM = rbk.PRV2MRP(prvInit_FM)
    PrescribedRot1DOFConfig.stepAngle = stepAngle
    PrescribedRot1DOFConfig.thetaDotRef = 0.0
    PrescribedRot1DOFConfig.stepTime = stepTime
    PrescribedRot1DOFConfig.motorStepCountInMsg.subscribeTo(stepperMotorConfig.motorStepCountOutMsg)

    # Log the test module output message for data comparison
    prescribedDataLog = PrescribedRot1DOFConfig.prescribedMotionOutMsg.recorder()
    dataLog = stepperMotorConfig.motorStepCountOutMsg.recorder()
    stepperMotorDataLog = PrescribedRot1DOFConfig.stepperMotorOutMsg.recorder()
    unitTestSim.AddModelToTask("fswTask2", prescribedDataLog)
    unitTestSim.AddModelToTask("fswTask2", stepperMotorDataLog)
    unitTestSim.AddModelToTask("fswTask1", dataLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Set the simulation time
    simTime1 = (numSteps * stepTime) + 5
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime1))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    thetaRef2 = 0.0 # Should take -5 steps
    thetaRef3 = 7.0 # Should start taking 7 steps before interrupted
    thetaRef4 = 0.0 # Should take -2 steps

    # Sim chunk 2: Negative steps
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = thetaRef2
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData, unitTestSim.TotalSim.CurrentNanos)
    stepperMotorConfig.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)
    simTime2 = (5 * stepTime) + 5
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime1 + simTime2))
    unitTestSim.ExecuteSimulation()

    # Sim chunk 3: Positive steps (interrupted half-way)
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = thetaRef3
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData, unitTestSim.TotalSim.CurrentNanos)
    stepperMotorConfig.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)
    simTime3 = (7 * stepTime) - (3 * stepTime)
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime1 + simTime2 + simTime3))
    unitTestSim.ExecuteSimulation()

    # Sim chunk 3: Negative steps (interrupted message)
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = thetaRef4
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData, unitTestSim.TotalSim.CurrentNanos)
    stepperMotorConfig.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)
    simTime4 = (2 * stepTime) + 10
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime1 + simTime2 + simTime3 + simTime4))
    unitTestSim.ExecuteSimulation()


    # Extract the logged data for plotting and data comparison
    stepsOutput = dataLog.numSteps
    print("Motor Module Step Commands:")
    print(stepsOutput)
    timespan = prescribedDataLog.times()
    omega_FM_F = prescribedDataLog.omega_FM_F
    sigma_FM = prescribedDataLog.sigma_FM
    theta = stepperMotorDataLog.theta
    thetaDot = stepperMotorDataLog.thetaDot
    thetaDDot = stepperMotorDataLog.thetaDDot
    motorStepCount = stepperMotorDataLog.stepCount
    motorCommandedSteps = stepperMotorDataLog.numSteps
    omegaPrime_FM_F = prescribedDataLog.omegaPrime_FM_F

    # Convert the logged sigma_FM MRPs to a scalar theta_FM array
    n = len(timespan)
    phi_FM = []
    for i in range(n):
        phi_FM.append((180 / np.pi) * 4 * np.arctan(np.linalg.norm(sigma_FM[i, :])))

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


# This statement below ensures that the unitTestScript can be run as a
# stand-along python script

if __name__ == "__main__":
    stepperMotorTestFunction(
                 True,
                 0,     # thetaInit
                 1,     # thetaRef
                 1e-12        # accuracy
               )