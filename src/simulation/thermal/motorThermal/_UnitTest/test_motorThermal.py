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
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import motorThermal                    # import the module that is to be tested
from Basilisk.architecture import messaging                      # import the message definitions
from Basilisk.simulation import spacecraft
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeRW
from Basilisk.simulation import reactionWheelStateEffector


# Uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.  Note that the order in that you add the parametrize method
# matters for the documentation in that it impacts the order in which the test arguments are shown.
# The first parametrize arguments are shown last in the pytest argument list
@pytest.mark.parametrize("accuracy", [1e-8])

def test_motorThermal(show_plots, accuracy):
    r"""
    **Validation Test Description**

    This unit test script tests the temperature modelling of a general motor (in this case a reaction wheel). It sets up
    the reaction wheel and runs 4 test scenarios:

    - Nominal: the motor starts at the same temperature as the air surrounding it (20 degrees Celsius).
    - Motor is colder: the motor starts at a lower temperature (0 degrees Celsius) than the ambient temperature (20
      degrees Celsius).
    - Motor is hotter: the motor starts at a higher temperature (20 degrees Celsius) than the ambient temperature (0
      degrees Celsius).
    - Motor is hotter and friction is accounted for: the motor starts at a higher temperature (20 degrees Celsius) than
      the ambient temperature (0 degrees Celsius) and friction is modelled.

    The sole reaction wheel is set up using the reaction wheel state effector, while taking advantage of the ability to
    change the reaction wheel's properties on the fly. For simplicity, the torque is constant and set to the maximum
    value that the reaction wheel can handle. Finally, the temperature modelling comes from this motorThermal module.

    The limitations of this test are the same as the ones discussed on the module's .rst file. also, the value of the
    accuracy used is the limit for the test to pass.

    **Test Parameters**

    Args:
        accuracy (float): absolute accuracy value used in the validation tests

    **Description of Variables Being Tested**

    In this file we are checking the values of the variables

    - ``rwTemp``

    which represents the array of motor temperatures. This array is compared to the ``truthTemp`` array, which contains
    the true values of the temperature.
    """
    [testResults, testMessage] = motorThermalTest(show_plots, accuracy)
    assert testResults < 1, testMessage


def motorThermalTest(show_plots, accuracy):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    # Create simulation variable names
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.sec2nano(1.)

    #
    #  create the simulation process
    #

    testProcessRate = macros.sec2nano(0.1)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    #
    #   setup the simulation tasks/objects
    #

    # create the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    #
    # add RW devices
    #

    # make a fresh RW factory instance, this is critical to run multiple times
    rwFactory = simIncludeRW.rwFactory()

    # store the RW dynamical model type
    varRWModel = messaging.BalancedWheels

    # create the reaction wheels
    RW = rwFactory.create('Honeywell_HR16',
                          [1, 0, 0],  # gsHat_B
                          Omega=4000.,  # RPM
                          maxMomentum=50.,
                          RWModel=varRWModel,
                          useRWfriction=True
                          )
    numRW = rwFactory.getNumOfDevices()

    # set maximum values for RW speed
    # RW1.Omega_max = 500 * macros.RPM

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "ReactionWheel"
    rwFactory.addToSpacecraft(rwStateEffector.ModelTag, rwStateEffector, scObject)

    # set RW torque command
    cmdArray = messaging.ArrayMotorTorqueMsgPayload()
    cmdArray.motorTorque = [50, 0, 0]  # [Nm]
    cmdMsg = messaging.ArrayMotorTorqueMsg().write(cmdArray)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(cmdMsg)

    #
    #   Setup the temperature modelling
    #

    thermalModel = motorThermal.MotorThermal()
    thermalModel.ModelTag = 'rwThermals'
    thermalModel.currentTemperature = 0  # [ºC]
    thermalModel.efficiency = 0.5
    thermalModel.ambientThermalResistance = 10
    thermalModel.motorHeatCapacity = 10
    thermalModel.rwStateInMsg.subscribeTo(rwStateEffector.rwOutMsgs[0])


    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, rwStateEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)
    unitTestSim.AddModelToTask(unitTaskName, thermalModel)

    #
    # log data
    #

    # log the RW temperature
    tempLog = thermalModel.temperatureOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, tempLog)

    #
    #   initialize Simulation
    #

    unitTestSim.InitializeSimulation()
    numTests = 0

    # run the first test (nominal)
    unitTestSim.ConfigureStopTime(simulationTime)
    unitTestSim.ExecuteSimulation()
    numTests += 1

    # change the test conditions
    thermalModel.currentTemperature = 0
    thermalModel.ambientTemperature = 20

    # run the second test (motor is colder than ambient)
    unitTestSim.ConfigureStopTime(2*simulationTime)
    unitTestSim.ExecuteSimulation()
    numTests += 1

    # change the test conditions
    thermalModel.currentTemperature = 20
    thermalModel.ambientTemperature = 0

    # run the second test (motor is hotter than ambient)
    unitTestSim.ConfigureStopTime(3*simulationTime)
    unitTestSim.ExecuteSimulation()
    numTests += 1

    # change the test conditions
    thermalModel.currentTemperature = 20
    thermalModel.ambientTemperature = 0
    RW.useRWfriction = True

    # run the second test (motor is hotter than ambient and friction is used)
    unitTestSim.ConfigureStopTime(4*simulationTime)
    unitTestSim.ExecuteSimulation()
    numTests += 1

    #
    # retrieve the logged data
    #

    rwTemp = np.array(tempLog.temperature)

    #
    # set the truth vectors
    #

    trueTemp = np.array([0.83985244, 1.67941113, 2.51867637, 3.35764846, 4.19632769,
                         5.03471435, 5.87280873, 6.71061112, 7.54812182, 8.38534113, 0.86531353,
                         1.73030786, 2.59498331, 3.45934019, 4.32337882, 5.18709952, 6.05050261,
                         6.91358841, 7.77635723, 8.6388094, 20.83077463, 21.6612646, 22.49147018,
                         23.32139167, 24.15102935, 24.9803835, 25.8094544, 26.63824235, 27.46674761,
                         28.29497048, 20.83623573, 21.67218133, 22.50783709, 23.34320331, 24.17828028,
                         25.01306827, 25.84756759, 26.68177852, 27.51570134, 28.34933636])

    #
    # compare the module results to the true values
    #

    # do the comparison

    for i in range(numTests):
        # check a vector values
        if not unitTestSupport.isArrayEqual(rwTemp[10*i+1:10*i+11], trueTemp[10*i:10*i+10], 10, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Motor Temperature Test failed test " + str(i+1) + "\n")

    if not testFailCount:
        print("PASSED")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


#
# Run this unitTest as a stand-along python script
#
if __name__ == "__main__":
    test_motorThermal(
        False,  # show_plots
        1e-8    # accuracy
    )
