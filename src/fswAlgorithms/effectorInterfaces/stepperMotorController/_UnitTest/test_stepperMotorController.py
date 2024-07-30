#
#  ISC License
#
#  Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder
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

import inspect
import os

import numpy as np
import pytest
from Basilisk.architecture import bskLogging
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import stepperMotorController
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

@pytest.mark.parametrize("motorStepAngle", [0.008 * macros.D2R, 0.01 * macros.D2R, 0.5 * macros.D2R])
@pytest.mark.parametrize("motorStepTime", [0.008, 0.1, 0.5])
@pytest.mark.parametrize("motorThetaInit", [-5.0 * macros.D2R, 0.0, 60.0 * macros.D2R])
@pytest.mark.parametrize("motorThetaRef", [0.0, 10.6 * macros.D2R, 60.0051 * macros.D2R])
def test_stepperMotorController(show_plots, motorStepAngle, motorStepTime, motorThetaInit, motorThetaRef):
    r"""
    **Validation Test Description**

    This unit test ensures that the stepper motor controller module correctly determines the number of steps required to actuate
    from an initial angle to a final reference angle. The initial and desired motor angles are varied so that both
    positive and negative steps are taken. It should be noted that the motor angles are descretized by a constant
    ``motorStepAngle``; therefore the motor cannot actuate to any desired angle. The desired motor angles are chosen in this
    test so that several cases require the desired angle to be adjusted to the nearest multiple of the motor step angle.
    In other words, this test introduces cases where the computed number of required steps is not an integer. For these
    cases, the determined number of steps must be rounded to the nearest whole step.

    **Test Parameters**

    Args:
        motorStepAngle (float): [rad] Angle the stepper motor moves through for a single step (constant)
        motorStepTime (float): [sec] Time required for a single motor step (constant)
        motorThetaInit (float): [rad] Initial stepper motor angle
        motorThetaRef (float): [rad] Desired stepper motor angle

    **Description of Variables Being Tested**

    The module-computed number of required stepper motor steps is checked to match the true number of motor steps
    computed in this script. The first element of the module ``motorStepCommand`` output message is checked to match
    the number of steps determined in this script.

    """

    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testProcessRate = macros.sec2nano(motorStepTime)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the stepperMotorController module to be tested
    motorController = stepperMotorController.StepperMotorController()
    motorController.ModelTag = "stepperMotorController"
    motorController.setStepAngle(motorStepAngle)
    motorController.setStepTime(motorStepTime)
    motorController.setThetaInit(motorThetaInit)
    unitTestSim.AddModelToTask(unitTaskName, motorController)

    # Create the stepperMotorController input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = motorThetaRef
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData)
    motorController.motorRefAngleInMsg.subscribeTo(HingedRigidBodyMessage)

    # Log the test module output message for data comparison
    motorStepCommandLog = motorController.motorStepCommandOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, motorStepCommandLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Calculate required number of steps for validation
    if (motorThetaInit > 0):
        trueNumSteps = (motorThetaRef - (np.ceil(motorThetaInit/motorStepAngle)*motorStepAngle)) / motorStepAngle
    else:
        trueNumSteps = (motorThetaRef - (np.floor(motorThetaInit/motorStepAngle)*motorStepAngle)) / motorStepAngle

    # If the desired motor angle is not a multiple of the step angle, the number of steps calculated is not an integer
    # and it must be rounded to the nearest whole step
    lowerStepFraction = trueNumSteps - np.floor(trueNumSteps)
    upperStepFraction = np.ceil(trueNumSteps) - trueNumSteps
    if (upperStepFraction > lowerStepFraction):
        trueNumSteps = np.floor(trueNumSteps)
    else:
        trueNumSteps = np.ceil(trueNumSteps)

    # If the desired motor angle is not a multiple of the step angle, a new desired angle is calculated
    newMotorDesiredAngle = motorThetaInit + (trueNumSteps * motorStepAngle)

    # Set the simulation time
    actuateTime = motorStepTime * np.abs(trueNumSteps)  # [sec] Time for the motor to actuate to the desired angle
    holdTime = 5  # [sec] Time the simulation will continue while holding the final angle
    unitTestSim.ConfigureStopTime(macros.sec2nano(actuateTime + holdTime))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    # Pull the logged motor step data
    stepsCommanded = motorStepCommandLog.stepsCommanded

    # Check that the correct number of steps was calculated
    accuracy = 1e-12
    np.testing.assert_allclose(stepsCommanded[0],
                               trueNumSteps,
                               atol=accuracy,
                               verbose=True)


if __name__ == "__main__":
    test_stepperMotorController(
         False,
         1.0 * macros.D2R,  # motorStepAngle
         1.0,  # motorStepTime
         0.0,  # initialAngle
         10.0 * macros.D2R,  # desiredAngle
    )
