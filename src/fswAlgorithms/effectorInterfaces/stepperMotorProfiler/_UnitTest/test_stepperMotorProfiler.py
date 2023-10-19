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
#   Module Name:        stepperMotorProfiler
#   Author:             Leah Kiner
#   Creation Date:      Sept 29, 2023
#

import inspect
import os
import matplotlib.pyplot as plt
import numpy as np
import pytest
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import stepperMotorProfiler
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

@pytest.mark.parametrize("initialMotorAngle", [0.0 * (np.pi / 180), 10.0 * (np.pi / 180), -5.0 * (np.pi / 180)])
@pytest.mark.parametrize("stepsCommanded", [0, 5, -5])
@pytest.mark.parametrize("stepAngle", [0.01 * (np.pi / 180), 0.5 * (np.pi / 180), 1.0 * (np.pi / 180)])
@pytest.mark.parametrize("stepTime", [0.1, 0.5, 1.0])
@pytest.mark.parametrize("accuracy", [1e-12])
def test_stepperMotorProfilerTestFunction(show_plots, initialMotorAngle, stepsCommanded, stepAngle, stepTime, accuracy):
    r"""
    **Validation Test Description**

    This unit test ensures that the stepper motor profiler module correctly actuates the stepper motor from an initial
    angle to a final reference angle, given an input integer number of commanded steps contained in the
    ``motorStepCommand`` input message. The initial motor angle and number of commanded steps are varied so that the
    module is shown to work for both positive and negative steps. The motor states are profiled for each step using
    a bang-bang constant positive, constant negative acceleration profile. The motor acceleration is determined from
    the given constant step angle and constant step time.

    **Test Parameters**

    Args:
        initialMotorAngle (float): [rad] Initial stepper motor angle
        stepsCommanded (int): [steps] Number of steps commanded to the stepper motor
        stepAngle (float): [rad] Angle the stepper motor moves through for a single step (constant)
        stepTime (float): [sec] Time required for a single motor step (constant)
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    This unit test checks that the final motor angle matches the reference motor angele. The reference motor angle is
    determined from the initial motor angle and number of commanded steps. The test also checks that the module
    keeps track of the motor step count correctly by comparing the final motor step count with the number of steps
    commanded from the module ``motorStepCommand`` input message.

    """
    [testResults, testMessage] = stepperMotorProfilerTestFunction(show_plots, initialMotorAngle, stepsCommanded, stepAngle, stepTime, accuracy)

    assert testResults < 1, testMessage


def stepperMotorProfilerTestFunction(show_plots, initialMotorAngle, stepsCommanded, stepAngle, stepTime, accuracy):
    """Call this routine directly to run the unit test."""
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testProcessRate = macros.sec2nano(0.1)     # Set process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the stepperMotorProfiler module to be tested
    StepperMotorProfiler = stepperMotorProfiler.stepperMotorProfiler()
    StepperMotorProfiler.ModelTag = "StepperMotorProfiler"
    rotAxis_M = np.array([1.0, 0.0, 0.0])
    StepperMotorProfiler.rotAxis_M = rotAxis_M
    StepperMotorProfiler.thetaInit = initialMotorAngle
    StepperMotorProfiler.stepAngle = stepAngle
    StepperMotorProfiler.stepTime = stepTime
    StepperMotorProfiler.thetaDDotMax = stepAngle / (0.25 * stepTime * stepTime)
    StepperMotorProfiler.r_FM_M = np.array([0.0, 0.0, 0.0])
    StepperMotorProfiler.rPrime_FM_M = np.array([0.0, 0.0, 0.0])
    StepperMotorProfiler.rPrimePrime_FM_M = np.array([0.0, 0.0, 0.0])

    # Add the test module to the runtime call list
    unitTestSim.AddModelToTask(unitTaskName, StepperMotorProfiler)

    # Create the StepperMotorProfiler input message
    MotorStepCommandMessageData = messaging.MotorStepCommandMsgPayload()
    MotorStepCommandMessageData.stepsCommanded = stepsCommanded
    MotorStepCommandMessage = messaging.MotorStepCommandMsg().write(MotorStepCommandMessageData)
    StepperMotorProfiler.motorStepCommandInMsg.subscribeTo(MotorStepCommandMessage)

    # Log the test module output message for data comparison
    stepperMotorDataLog = StepperMotorProfiler.stepperMotorOutMsg.recorder()
    prescribedDataLog = StepperMotorProfiler.prescribedMotionOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, stepperMotorDataLog)
    unitTestSim.AddModelToTask(unitTaskName, prescribedDataLog)

    # Initialize the simulation, set the sim run time, and execute the simulation
    unitTestSim.InitializeSimulation()
    actuateTime = stepTime * np.abs(stepsCommanded)  # [sec] Time for the motor to actuate to the desired angle
    holdTime = 5  # [sec] Time the simulation will continue while holding the final angle
    unitTestSim.ConfigureStopTime(macros.sec2nano(actuateTime + holdTime))
    unitTestSim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    timespan = stepperMotorDataLog.times()
    theta = (180 / np.pi) * stepperMotorDataLog.theta
    thetaDot = (180 / np.pi) * stepperMotorDataLog.thetaDot
    thetaDDot = (180 / np.pi) * stepperMotorDataLog.thetaDDot
    motorStepCount = stepperMotorDataLog.stepCount
    motorCommandedSteps = stepperMotorDataLog.stepsCommanded
    sigma_FM = prescribedDataLog.sigma_FM
    omega_FM_F = prescribedDataLog.omega_FM_F
    omegaPrime_FM_F = prescribedDataLog.omegaPrime_FM_F

    # Only show plots if the motor actuates
    if (stepsCommanded == 0):
        show_plots = False

    # Plot motor angle
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

    # Check to ensure the initial angle converged to the reference angle
    desiredMotorAngleTrue = initialMotorAngle + (stepsCommanded * stepAngle)
    if not unitTestSupport.isDoubleEqual(theta[-1], (180 / np.pi) * desiredMotorAngleTrue, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + StepperMotorProfiler.ModelTag + " Final motor angle does not match the reference angle.")

    # Check to ensure the initial angle rate converged to the reference angle rate
    if not unitTestSupport.isDoubleEqual(thetaDot[-1], 0.0, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: " + StepperMotorProfiler.ModelTag + " Final motor angle rate does not match the reference rate.")

    # Check the motor achieved the commanded steps
    if not (stepsCommanded == motorStepCount[-1]):
        testFailCount += 1
        testMessages.append("FAILED: " + StepperMotorProfiler.ModelTag + " Motor did not complete the number of commanded steps.")
        print(stepsCommanded)
        print(motorStepCount[-1])

    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unitTestScript can be run as a stand-along python script
#
if __name__ == "__main__":
    stepperMotorProfilerTestFunction(
                 True,
                 0.0,                       # initialMotorAngle
                 10,                        # stepsCommanded
                 1.0 * (np.pi / 180),       # stepAngle
                 1.0,                       # stepTime
                 1e-12                      # accuracy
               )
