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
#   Creation Date:      Sept 28, 2023
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

@pytest.mark.parametrize("stepAngle", [1.0 * (np.pi / 180)])
@pytest.mark.parametrize("stepTime", [1.0])
@pytest.mark.parametrize("initialMotorAngle", [0.0])
@pytest.mark.parametrize("desiredMotorAngle1", [10.0 * (np.pi / 180), -10.0 * (np.pi / 180)])
@pytest.mark.parametrize("desiredMotorAngle2", [0.0, 10.0 * (np.pi / 180), 5 * (np.pi / 180)])
@pytest.mark.parametrize("interruptFraction", [0.0, 0.25, 0.5, 0.75])
def test_stepperMotorControllerTestFunction(show_plots, stepAngle, stepTime, initialMotorAngle, desiredMotorAngle1, desiredMotorAngle2, interruptFraction):
    r"""
    **Validation Test Description**

    This unit test ensures that the stepper motor controller module correctly determines the number of steps required to actuate
    from an initial angle to a final reference angle. An interrupted message is introduced during the motor actuation
    that specifies a new final reference angle. The initial and desired motor angles are varied so that combinations of
    both positive and negative steps are taken. The time of step interruption is varied to ensure that once a step
    begins, it is completed regardless of when the interrupted message is written. Because the other unit test script
    for this module checked the module functionality for various motor step angles and desired angles that are not
    multiples of the motor step angle, the step angle and desired angles chosen in this script are set to simple values.

    **Test Parameters**

    Args:
        stepAngle (float): [rad] Angle the stepper motor moves through for a single step (constant)
        stepTime (float): [sec] Time required for a single motor step (constant)
        initialMotorAngle (float): [rad] Initial stepper motor angle
        desiredMotorAngle1 (float): [rad] Desired stepper motor angle 1
        desiredMotorAngle2 (float): [rad] Desired stepper motor angle 2
        interruptFraction (float): Specifies what fraction of a step is completed when the interrupted message is written

    **Description of Variables Being Tested**

    The module-computed number of required stepper motor steps for both simulation chunks are checked with the true
    number of motor steps computed in this script. Because the other unit test script for this module checked that
    messages are correctly written, this validation is not repeated in this script.

    """

    [testResults, testMessage] = stepperMotorControllerTestFunction(show_plots, stepAngle, stepTime, initialMotorAngle, desiredMotorAngle1, desiredMotorAngle2, interruptFraction)

    assert testResults < 1, testMessage

def stepperMotorControllerTestFunction(show_plots, stepAngle, stepTime, initialMotorAngle, desiredMotorAngle1, desiredMotorAngle2, interruptFraction):
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
    HingedRigidBodyMessageData.theta = desiredMotorAngle1
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData)
    StepperMotorController.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)

    # Log the test module output message for data comparison
    motorStepCommandLog = StepperMotorController.motorStepCommandOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, motorStepCommandLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Calculate required number of steps for validation
    trueNumSteps1 = (desiredMotorAngle1 - (np.ceil(initialMotorAngle/stepAngle)*stepAngle)) / stepAngle

    # If the desired motor angle is not a multiple of the step angle, the number of steps calculated is not an integer
    # and it must be rounded to the nearest whole step
    lowerStepFraction = trueNumSteps1 - np.floor(trueNumSteps1)
    upperStepFraction = np.ceil(trueNumSteps1) - trueNumSteps1
    if (upperStepFraction > lowerStepFraction):
        trueNumSteps1 = np.floor(trueNumSteps1)
    else:
        trueNumSteps1 = np.ceil(trueNumSteps1)

    # If the desired motor angle is not a multiple of the step angle, a new desired angle is calculated
    newMotorDesiredAngle = initialMotorAngle + (trueNumSteps1 * stepAngle)

    # Set the simulation time
    actuateTime1 = stepTime * np.abs(trueNumSteps1)  # [sec] Time for the motor to actuate to the desired angle
    simTime1 = (actuateTime1 / 2) + (interruptFraction * stepTime)  # [sec] Time before the first message is interrupted
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime1))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    # Create the second interruption message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = desiredMotorAngle2
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData, unitTestSim.TotalSim.CurrentNanos)
    StepperMotorController.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)

    # Calculate number of steps to actuate from interrupted motor position to the second desired motor angle
    if (trueNumSteps1 > 0):
        interruptedMotorAngle = initialMotorAngle + ((simTime1 / stepTime) * stepAngle)
        # Ensure the interrupted motor angle is set to the next multiple of the motor step angle
        # (If the motor is interrupted during a step)
        interruptedMotorAngle = np.ceil(interruptedMotorAngle / stepAngle) * stepAngle
    else:
        interruptedMotorAngle = initialMotorAngle - ((simTime1 / stepTime) * stepAngle)
        # Ensure the interrupted motor angle is set to the next multiple of the motor step angle
        # (If the motor is interrupted during a step)
        interruptedMotorAngle = np.floor(interruptedMotorAngle / stepAngle) * stepAngle

    trueNumSteps2 = (desiredMotorAngle2 - interruptedMotorAngle) / stepAngle

    # If the desired motor angle is not a multiple of the step angle, the number of steps calculated is not an integer
    # and it must be rounded to the nearest whole step
    lowerStepFraction = trueNumSteps2 - np.floor(trueNumSteps2)
    upperStepFraction = np.ceil(trueNumSteps2) - trueNumSteps2
    if (upperStepFraction > lowerStepFraction):
        trueNumSteps2 = np.floor(trueNumSteps2)
    else:
        trueNumSteps2 = np.ceil(trueNumSteps2)

    # If the desired motor angle is not a multiple of the step angle, a new desired angle is calculated
    newMotorDesiredAngle2 = interruptedMotorAngle + (trueNumSteps2 * stepAngle)

    # Set the simulation time for chunk 2
    actuateTime2 = stepTime * np.abs(trueNumSteps2)  # [sec] Time for the motor to actuate to the desired angle
    holdTime = 5  # [sec] Time the simulation will continue while holding the final angle
    simTime2 = actuateTime2 + holdTime
    unitTestSim.ConfigureStopTime(macros.sec2nano(simTime1 + simTime2))

    # Execute simulation chunk 2
    unitTestSim.ExecuteSimulation()

    # Pull the logged motor step data
    stepsCommanded = motorStepCommandLog.stepsCommanded

    # Check that the correct number of steps were calculated
    if ((stepsCommanded[0] != trueNumSteps1) or (stepsCommanded[-1] != trueNumSteps2)):
        testFailCount += 1
        testMessages.append("\nFAILED: " + StepperMotorController.ModelTag + " Number of required motor steps do not match")
        if (stepsCommanded[0] != trueNumSteps1):
            print("STEP CALCULATION 1 INCORRECT")
        if (stepsCommanded[-1] != trueNumSteps2):
            print("STEP CALCULATION 2 INCORRECT")

    # Manual check that module outputs match the expected true result
    print("True Steps:")
    print(trueNumSteps1)
    print(trueNumSteps2)
    print("Module Calculation:")
    print(stepsCommanded[0])
    print(stepsCommanded[-1])

    return [testFailCount, ''.join(testMessages)]


# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
if __name__ == "__main__":
    stepperMotorControllerTestFunction(
                 False,
                 1.0 * (np.pi / 180),     # stepAngle
                 1.0,                     # stepTime
                 0.0,                     # initialMotorAngle
                 10.0 * (np.pi / 180),   # desiredMotorAngle1,
                 5.0 * (np.pi / 180),     # desiredMotorAngle2
                 0.0                     # interruptFraction
               )
