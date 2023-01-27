#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        motorThermal
#   Author:             João Vaz Carneiro
#   Creation Date:      March 4, 2021
#

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import encoder                    # import the module that is to be tested
from Basilisk.architecture import messaging                      # import the message definitions
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport



# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
@pytest.mark.parametrize("accuracy", [1e-8])

def test_encoder(show_plots, accuracy):
    r"""
    **Validation Test Description**

    This unit test script tests the the features of an encoder, namely discretization and signal failures. It sets up
    the reaction wheel speed message and encoder modules and runs the simulation for a number of steps. The script tests
    each functionality of the module, including the discretization of the wheel speeds and the signal failures, such as
    the signal being turned off or being stuck at the previous iteration.

    In the reaction wheel message, three wheels are considered with varying values. Throughout the simulation, the
    encoder module assumes different operating states, such as nominal (everything works as intended), off (wheel speeds
    are off) and stuck (wheel speeds remain constant).

    The limitations of this test are the same as the ones discussed on the module's .rst file. Also, the value of the
    accuracy used is the limit for the test to pass.

    **Test Parameters**

    Args:
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    In this file we are checking the values of the variables

    - ``wheelSpeedsEncoded``

    which represents the array of reaction wheel speeds. This array is compared to the ``truewheelSpeedsEncoded`` array,
    which contains the expected values of the wheel speeds after encoding.
    """
    [testResults, testMessage] = encoderTest(show_plots, accuracy)
    assert testResults < 1, testMessage


def encoderTest(show_plots, accuracy):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    # Create simulation variable names
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #

    testProcessRate = macros.sec2nano(1)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    #
    #   setup the simulation tasks/objects
    #

    #
    # Create RW speed message
    #

    speedMsgData = messaging.RWSpeedMsgPayload()
    speedMsgData.wheelSpeeds = [100, 200, 300]
    speedMsg = messaging.RWSpeedMsg().write(speedMsgData)

    numRW = 3

    #
    #   Setup the reaction wheel speed encoder
    #

    wheelSpeedEncoder = encoder.Encoder()
    wheelSpeedEncoder.ModelTag = 'rwSpeedsEncoder'
    wheelSpeedEncoder.clicksPerRotation = 2
    wheelSpeedEncoder.numRW = numRW
    wheelSpeedEncoder.rwSpeedInMsg.subscribeTo(speedMsg)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, wheelSpeedEncoder)

    #
    # log data
    #

    # log the RW speeds
    wheelSpeedEncodedLog = wheelSpeedEncoder.rwSpeedOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, wheelSpeedEncodedLog)

    #
    #   initialize Simulation
    #

    unitTestSim.InitializeSimulation()
    numSteps = 0

    # run the sim
    unitTestSim.TotalSim.SingleStepProcesses()
    numSteps += 1
    unitTestSim.TotalSim.SingleStepProcesses()
    numSteps += 1
    unitTestSim.TotalSim.SingleStepProcesses()
    numSteps += 1

    # update the encoder to be OFF
    wheelSpeedEncoder.rwSignalState = [encoder.SIGNAL_OFF]*numRW

    # run the sim
    unitTestSim.TotalSim.SingleStepProcesses()
    numSteps += 1

    # update the wheel speeds
    speedMsgData.wheelSpeeds = [500, 400, 300]
    speedMsg = messaging.RWSpeedMsg().write(speedMsgData)
    wheelSpeedEncoder.rwSpeedInMsg.subscribeTo(speedMsg)
    wheelSpeedEncoder.rwSignalState = [encoder.SIGNAL_NOMINAL] * numRW

    # run the sim
    unitTestSim.TotalSim.SingleStepProcesses()
    numSteps += 1

    # change the wheel speeds but make the encoder stuck
    speedMsgData.wheelSpeeds = [100, 200, 300]
    speedMsg = messaging.RWSpeedMsg().write(speedMsgData)
    wheelSpeedEncoder.rwSpeedInMsg.subscribeTo(speedMsg)
    wheelSpeedEncoder.rwSignalState = [encoder.SIGNAL_STUCK] * numRW

    # run the sim
    unitTestSim.TotalSim.SingleStepProcesses()
    numSteps += 1

    #
    # retrieve the logged data
    #

    wheelSpeedsEncoded = np.array(wheelSpeedEncodedLog.wheelSpeeds)

    #
    # set the truth vectors
    #

    trueWheelSpeedsEncoded = np.array([[100., 200., 300.],
                                       [ 97.38937226, 197.92033718, 298.45130209],
                                       [100.53096491, 201.06192983, 298.45130209],
                                       [0., 0., 0.],
                                       [499.51323192, 398.98226701, 298.45130209],
                                       [499.51323192, 398.98226701, 298.45130209]])

    #
    # compare the module results to the true values
    #

    fail = 0
    for i in range(numSteps):
        # check a vector values
        if not unitTestSupport.isArrayEqual(wheelSpeedsEncoded[i, 0:3], trueWheelSpeedsEncoded[i, :], 3, accuracy):
            fail += 1

    if fail > 0:
        testFailCount += 1

    if not testFailCount:
        print("PASSED")
    else:
        testMessages.append("FAILED: Encoder Test")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# Run this unitTest as a stand-along python script
#
if __name__ == "__main__":
    test_encoder(
        False,  # show_plots
        1e-8    # accuracy
    )
