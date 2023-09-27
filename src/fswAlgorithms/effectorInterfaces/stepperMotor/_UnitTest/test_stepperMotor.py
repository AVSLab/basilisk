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
#   Module Name:        stepperMotor
#   Author:             Shamsa SabekZaei
#   Creation Date:      Aug 25, 2023
#

import pytest
import inspect
import matplotlib.pyplot as plt
import numpy as np
import os
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import stepperMotor
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

@pytest.mark.parametrize("stepAngle", [0.01 * (np.pi / 180), 0.5 * (np.pi / 180) , 1 * (np.pi / 180)])
@pytest.mark.parametrize("stepTime", [0.1, 0.5, 1])
@pytest.mark.parametrize("initialMotorAngle", [0, 60 * (np.pi / 180)])
@pytest.mark.parametrize("desiredMotorAngle", [0, 10.6 * (np.pi / 180), 60.005 * (np.pi / 180)])
# @pytest.mark.parametrize("accuracy", [1e-12])

def test_stepperMotorTestFunction(show_plots, stepAngle, stepTime, initialMotorAngle, desiredMotorAngle):
    r"""
    **Validation Test Description**

    This unit test ensures that the stepper motor is properly computed, where the unput of deisred angle will give us the right number of motor steps. 
    **Test Parameters**

    Args:
        stepAngle (float): [rad] Initial PRV angle of the F frame with respect to the M frame
        stepTime (float): [sec]
        initialMotorAngle (float): [rad] Reference PRV angle of the F frame with respect to the M frame
        desiredMotorAngle (float): [rad/s^2] Maximum angular acceleration for the attitude maneuver
        accuracy (float):

    **Description of Variables Being Tested**

    This unit test ensures that the profiled 1 DOF rotational attitude maneuver is properly computed for a series of
    initial and reference PRV angles and maximum angular accelerations. The final prescribed angle ``theta_FM_Final``
    and angular velocity magnitude ``thetaDot_Final`` are compared with the reference values ``theta_Ref`` and
    ``thetaDot_Ref``, respectively.
    """
    [testResults, testMessage] = stepperMotorTestFunction(show_plots, stepAngle, stepTime, initialMotorAngle, desiredMotorAngle)

    assert testResults < 1, testMessage

def stepperMotorTestFunction(show_plots, stepAngle, stepTime, initialMotorAngle, desiredMotorAngle):
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

    # Create an instance of the stepperMotor module to be tested
    StepperMotorConfig = stepperMotor.StepperMotorConfig()
    StepperMotorWrap = unitTestSim.setModelDataWrap(StepperMotorConfig)
    StepperMotorWrap.ModelTag = "stepperMotor"
    StepperMotorConfig.stepAngle = stepAngle
    StepperMotorConfig.stepTime = stepTime
    StepperMotorConfig.initAngle = initialMotorAngle

    # Add the stepperMotor test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, StepperMotorWrap, StepperMotorConfig)

    # Create the stepperMotor input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = desiredMotorAngle
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData)
    StepperMotorConfig.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)

    # Log the test module output message for data comparison
    motorStepCountLog = StepperMotorConfig.motorStepCountOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, motorStepCountLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Calculate required number of steps for validation
    trueNumSteps = (desiredMotorAngle - initialMotorAngle) / stepAngle

    # If the desired motor angle is not a multiple of the step angle, the number of steps calculated is not an integer
    # and it must be rounded to the nearest whole step
    lowerStepFraction = trueNumSteps - np.floor(trueNumSteps)
    upperStepFraction = np.ceil(trueNumSteps) - trueNumSteps
    if (upperStepFraction > lowerStepFraction):
        trueNumSteps = np.floor(trueNumSteps)
    else:
        trueNumSteps = np.ceil(trueNumSteps)

    # If the desired motor angle is not a multiple of the step angle, a new desired angle is calculated
    newMotorDesiredAngle = initialMotorAngle + (trueNumSteps * stepAngle)

    # Set the simulation time
    actuateTime = stepTime * trueNumSteps  # [sec] Time for the motor to actuate to the desired angle
    holdTime = 5  # [sec] Time the simulation will continue while holding the final angle
    unitTestSim.ConfigureStopTime(macros.sec2nano(actuateTime + holdTime))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    # Pull the logged motor step data
    numSteps = motorStepCountLog.numSteps

    # Check that the correct number of steps was calculated
    if (numSteps[0] != trueNumSteps):
            testFailCount += 1
            testMessages.append("FAILED: " + StepperMotorWrap.ModelTag + " Number of required motor steps do not match")

    # Check to make sure the message was written correctly
    if (trueNumSteps != 0):
        if (np.count_nonzero(numSteps) != 1):
            testFailCount += 1
            testMessages.append("FAILED: " + StepperMotorWrap.ModelTag + " MotorStepCountMsg was incorrectly written")
    else:
        if (np.count_nonzero(numSteps) != 0):
            testFailCount += 1
            testMessages.append("FAILED: " + StepperMotorWrap.ModelTag + " MotorStepCountMsg was incorrectly written")


# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
if __name__ == "__main__":
    stepperMotorTestFunction(
                 False,
                 1 * (np.pi / 180),     # stepAngle
                 1,                     # stepTime
                 0,                     # initialAngle
                 10 * (np.pi / 180),    # desiredAngle
               )