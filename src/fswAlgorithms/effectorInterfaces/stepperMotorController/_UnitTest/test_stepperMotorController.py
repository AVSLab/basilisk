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
#   Module Name:        stepperMotorController
#   Author:             Shamsa SabekZaei and Leah Kiner
#   Creation Date:      Aug 25, 2023
#

import inspect
import matplotlib.pyplot as plt
import numpy as np
import os
import pytest
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import stepperMotorController
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

@pytest.mark.parametrize("stepAngle", [0.008 * (np.pi / 180), 0.01 * (np.pi / 180), 0.5 * (np.pi / 180)])
@pytest.mark.parametrize("stepTime", [0.008, 0.1, 0.5])
@pytest.mark.parametrize("initialMotorAngle", [-5 * (np.pi / 180), 0.0, 60.0 * (np.pi / 180)])
@pytest.mark.parametrize("desiredMotorAngle", [0.0, 10.6 * (np.pi / 180), 60.0051 * (np.pi / 180)])
def test_stepperMotorControllerTestFunction(show_plots, stepAngle, stepTime, initialMotorAngle, desiredMotorAngle):
    r"""
    **Validation Test Description**

    This unit test ensures that the stepper motor controller module correctly determines the number of steps required to actuate
    from an initial angle to a final reference angle. The initial and desired motor angles are varied so that both
    positive and negative steps are taken. It should be noted that the motor angles are descretized by a constant
    ``stepAngle``; therefore the motor cannot actuate to any desired angle. The desired motor angles are chosen in this
    test so that several cases require the desired angle to be adjusted to the nearest multiple of the motor step angle.
    In other words, this test introduces cases where the computed number of required steps is not an integer. For these
    cases, the determined number of steps must be rounded to the nearest whole step.

    **Test Parameters**

    Args:
        stepAngle (float): [rad] Angle the stepper motor moves through for a single step (constant)
        stepTime (float): [sec] Time required for a single motor step (constant)
        initialMotorAngle (float): [rad] Initial stepper motor angle
        desiredMotorAngle (float): [rad] Desired stepper motor angle

    **Description of Variables Being Tested**

    The module-computed number of required stepper motor steps is checked to match the true number of motor steps
    computed in this script. The first element of the module ``motorStepCommand`` output message is checked to match
    the number of steps determined in this script.

    """
    [testResults, testMessage] = stepperMotorControllerTestFunction(show_plots, stepAngle, stepTime, initialMotorAngle, desiredMotorAngle)

    assert testResults < 1, testMessage

def stepperMotorControllerTestFunction(show_plots, stepAngle, stepTime, initialMotorAngle, desiredMotorAngle):
    testFailCount = 0                                        # Zero the unit test result counter
    testMessages = []                                        # Create an empty array to store the test log messages
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testProcessRate = macros.sec2nano(stepTime)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the stepperMotorController module to be tested
    StepperMotorController = stepperMotorController.stepperMotorController()
    StepperMotorController.ModelTag = "stepperMotorController"
    StepperMotorController.stepAngle = stepAngle
    StepperMotorController.stepTime = stepTime
    StepperMotorController.initAngle = initialMotorAngle
    StepperMotorController.currentAngle = initialMotorAngle

    # Add the stepperMotor test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, StepperMotorController)

    # Create the stepperMotorController input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = desiredMotorAngle
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData)
    StepperMotorController.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)

    # Log the test module output message for data comparison
    motorStepCommandLog = StepperMotorController.motorStepCommandOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, motorStepCommandLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Calculate required number of steps for validation
    if (initialMotorAngle > 0):
        trueNumSteps = (desiredMotorAngle - (np.ceil(initialMotorAngle/stepAngle)*stepAngle)) / stepAngle
    else:
        trueNumSteps = (desiredMotorAngle - (np.floor(initialMotorAngle/stepAngle)*stepAngle)) / stepAngle

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
    actuateTime = stepTime * np.abs(trueNumSteps)  # [sec] Time for the motor to actuate to the desired angle
    holdTime = 5  # [sec] Time the simulation will continue while holding the final angle
    unitTestSim.ConfigureStopTime(macros.sec2nano(actuateTime + holdTime))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    # Pull the logged motor step data
    stepsCommanded = motorStepCommandLog.stepsCommanded

    # Check that the correct number of steps was calculated
    if (stepsCommanded[0] != trueNumSteps):
        testFailCount += 1
        testMessages.append("\nFAILED: " + StepperMotorController.ModelTag + " Number of required motor steps do not match")

    return [testFailCount, ''.join(testMessages)]


# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
if __name__ == "__main__":
    stepperMotorControllerTestFunction(
         False,
         1.0 * (np.pi / 180),     # stepAngle
         1,                       # stepTime
         0.0,                     # initialAngle
         10.0 * (np.pi / 180),    # desiredAngle
    )
