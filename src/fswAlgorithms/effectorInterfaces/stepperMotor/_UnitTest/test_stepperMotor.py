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
from Basilisk.fswAlgorithms import stepperMotor # import the module that is to be tested
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


# Vary the reference angle for pytest
#Shamsa: thetaint is gonna excute twice one in 0 and once in 2*np.pi/3

@pytest.mark.parametrize("desiredAngle", [0, 2*np.pi/3])
@pytest.mark.parametrize("accuracy", [1e-12])
def test_stepperMotorTestFunction(show_plots, desiredAngle, accuracy):
    r"""
    **Validation Test Description**

    This unit test ensures that the stepper motor is properly computed, where the unput of deisred angle will give us the right number of motor steps. 
    **Test Parameters**

    Args:
        desiredAngle (float): [rad] desired angle value
        thetaInit (float): [rad] Initial PRV angle of the F frame with respect to the M frame
        thetaRef (float): [rad] Reference PRV angle of the F frame with respect to the M frame
        thetaDDotMax (float): [rad/s^2] Maximum angular acceleration for the attitude maneuver

    **Description of Variables Being Tested**

    This unit test ensures that the profiled 1 DOF rotational attitude maneuver is properly computed for a series of
    initial and reference PRV angles and maximum angular accelerations. The final prescribed angle ``theta_FM_Final``
    and angular velocity magnitude ``thetaDot_Final`` are compared with the reference values ``theta_Ref`` and
    ``thetaDot_Ref``, respectively.
    """
    [testResults, testMessage] = stepperMotorTestFunction(show_plots, desiredAngle, accuracy)

#assertion statement checks whether a given condition is True.
    # If the condition is False, an AssertionError is raised, and an optional error message (testMessage) can be
    # provided to explain the reason for the assertion failure.
    assert testResults < 1, testMessage


def stepperMotorTestFunction(show_plots, desiredAngle1, desiredAngle2, desiredAngle3, desiredAngle4,desiredAngle5, desiredAngle6, accuracy):
    testFailCount = 0                                        # Zero the unit test result counter
    testMessages = []                                        # Create an empty array to store the test log messages
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create the test thread
    testProcessRate = macros.sec2nano(1)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create an instance of the stepperMotor module to be tested
    StepperMotorConfig = stepperMotor.StepperMotorConfig()
    StepperMotorWrap = unitTestSim.setModelDataWrap(StepperMotorConfig)
    StepperMotorWrap.ModelTag = "stepperMotor"
    stepAngle = 1
    StepperMotorConfig.stepAngle = stepAngle
    motorRPM = 0.5
    stepTime = 60 / ( motorRPM * (360 / stepAngle) )
    StepperMotorConfig.stepTime = stepTime

    # Add the stepperMotor test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, StepperMotorWrap, StepperMotorConfig)

    # Create the stepperMotor input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = desiredAngle1
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData)
    StepperMotorConfig.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)

    # Log the test module output message for data comparison
    stepCountMsgLog = StepperMotorConfig.motorStepCountOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, stepCountMsgLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Set the simulation time
    unitTestSim.ConfigureStopTime(macros.sec2nano(30))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    # Create the stepperMotor input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsg_C_zeroMsgPayload()
    print("")
    HingedRigidBodyMessageData.theta = desiredAngle2
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData, unitTestSim.TotalSim.CurrentNanos)
    StepperMotorConfig.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)

    # Set the simulation time
    unitTestSim.ConfigureStopTime(macros.sec2nano(30) + macros.sec2nano(30))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    # Create the stepperMotor input message
    print("")
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsgPayload()
    HingedRigidBodyMessageData.theta = desiredAngle3
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData, unitTestSim.TotalSim.CurrentNanos)
    StepperMotorConfig.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)

    # Set the simulation time
    unitTestSim.ConfigureStopTime(macros.sec2nano(30) + macros.sec2nano(30) +  macros.sec2nano(30))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    # Create the stepperMotor input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsg_C_zeroMsgPayload()
    HingedRigidBodyMessageData.theta = desiredAngle4
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData, unitTestSim.TotalSim.CurrentNanos)
    StepperMotorConfig.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)

    # Set the simulation time
    unitTestSim.ConfigureStopTime(macros.sec2nano(30) + macros.sec2nano(30) + macros.sec2nano(30) + macros.sec2nano(30))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

        # Create the stepperMotor input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsg_C_zeroMsgPayload()
    HingedRigidBodyMessageData.theta = desiredAngle5
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData, unitTestSim.TotalSim.CurrentNanos)
    StepperMotorConfig.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)

    # Set the simulation time
    unitTestSim.ConfigureStopTime(macros.sec2nano(30) + macros.sec2nano(30) + macros.sec2nano(30) + macros.sec2nano(30)+ macros.sec2nano(30))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()
        # Create the stepperMotor input message
    HingedRigidBodyMessageData = messaging.HingedRigidBodyMsg_C_zeroMsgPayload()
    HingedRigidBodyMessageData.theta = desiredAngle6
    HingedRigidBodyMessage = messaging.HingedRigidBodyMsg().write(HingedRigidBodyMessageData, unitTestSim.TotalSim.CurrentNanos)
    StepperMotorConfig.spinningBodyInMsg.subscribeTo(HingedRigidBodyMessage)

    # Set the simulation time
    unitTestSim.ConfigureStopTime(macros.sec2nano(30) + macros.sec2nano(30) + macros.sec2nano(30) + macros.sec2nano(30)+ macros.sec2nano(30)+ macros.sec2nano(30))

    # Begin the simulation
    unitTestSim.ExecuteSimulation()

    # Extract the logged data for plotting and data comparison
    print("")
    print("Number of steps are")
    print(stepCountMsgLog.numSteps)
    print(len(stepCountMsgLog.numSteps))


# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
if __name__ == "__main__":
    stepperMotorTestFunction(
                 True,
                 10,     # desiredAngle1
                 30,     # desiredAngle2
                 70,     # desiredAngle3
                 42,     # desiredAngle3
                 32,
                 52,
                 1e-12   # accuracy
               )