#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        reactionWheelStateEffector_RWUpdate
#   Author:             João Vaz Carneiro
#   Creation Date:      January 23, 2020
#

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.simulation import spacecraft
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeRW
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.architecture import messaging

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(conditionstring)


@pytest.mark.parametrize("accuracy", [1e-8])
def test_RWUpdate(show_plots, accuracy):
    r"""
    **Validation Test Description**

    The objective of this script is to test the functionality of changing the reaction wheel (RW) characteristics while
    the simulation is running. It starts by testing the initial setup, and then does four additional tests: the first
    two change the maximum allowed torque, the third one changes the maximum power limit, and the final one changes the 
    current wheel speeds and maximum allowed wheel speeds. All these tests rely on the fact that, when a maximum or 
    minimum value is surpassed, the applied torque is capped accordingly.

    As this test script is not parameterized, only one version of this script will run.

    **Description of Variables Being Tested**

    As discussed, in this file we check the values of the applied torque for each reaction wheel ``i``:

    - ``rwStateEffector.rwOutMsgs[i].u_current``

    For ease of use, this data is stored in ``dataRW``, which is then used to check if the tests are being passed.
    """
    [testResults, testMessage] = RWUpdateTest(show_plots, accuracy)
    assert testResults < 1, testMessage

def RWUpdateTest(show_plots, accuracy):

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

    testProcessRate = macros.sec2nano(1.)  # update process rate update time
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
    RW1 = rwFactory.create('Honeywell_HR16'
                           , [1, 0, 0]  # gsHat_B
                           , Omega=500.  # RPM
                           , maxMomentum=50.
                           , RWModel=varRWModel
                           )
    RW2 = rwFactory.create('Honeywell_HR16'
                           , [0, 1, 0]  # gsHat_B
                           , Omega=500.  # RPM
                           , maxMomentum=50.
                           , RWModel=varRWModel
                           )
    RW3 = rwFactory.create('Honeywell_HR16'
                           , [0, 0, 1]  # gsHat_B
                           , Omega=500.  # RPM
                           , maxMomentum=50.
                           , RWModel=varRWModel
                           )
    numRW = rwFactory.getNumOfDevices()

    # create RW object container and tie to spacecraft object
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "ReactionWheel"
    rwFactory.addToSpacecraft(rwStateEffector.ModelTag, rwStateEffector, scObject)

    # set RW torque command
    cmdArray = messaging.ArrayMotorTorqueMsgPayload()
    cmdArray.motorTorque = [0.4, 0.1, -0.5]  # [Nm]
    cmdMsg = messaging.ArrayMotorTorqueMsg().write(cmdArray)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(cmdMsg)
    trueTorque = [[0.4, 0., -0.5]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, rwStateEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    #
    # log data
    #

    rwLogs = []
    for item in range(numRW):
        rwLogs.append(rwStateEffector.rwOutMsgs[item].recorder())
        unitTestSim.AddModelToTask(unitTaskName, rwLogs[item])

    #
    #   initialize Simulation
    #

    unitTestSim.InitializeSimulation()
    numTests = 0

    #
    # First test
    #

    # set the maximum torque values
    RW1.u_max = 0.5
    RW2.u_max = 0.5
    RW3.u_max = 0.5
    RW1.u_min = 0.2
    RW2.u_min = 0.2
    RW3.u_min = 0.2

    # configure a simulation stop time and execute the simulation run
    unitTestSim.ConfigureStopTime(simulationTime)
    unitTestSim.ExecuteSimulation()
    numTests += 1

    # expected output
    trueTorque.append([0.4, 0., -0.5])

    #
    # Second test
    #

    # reset the maximum torque values
    RW1.u_max = 0.3
    RW2.u_max = 0.3
    RW3.u_max = 0.3
    RW1.u_min = 0.05
    RW2.u_min = 0.05
    RW3.u_min = 0.05

    # reconfigure a simulation stop time and re-execute the simulation run
    unitTestSim.ConfigureStopTime(2*simulationTime)
    unitTestSim.ExecuteSimulation()
    numTests += 1

    # expected output
    trueTorque.append([0.3, 0.1, -0.3])

    #
    # Third test
    #

    # reset the maximum torque values
    RW1.P_max = 0.7
    RW2.P_max = 0.7
    RW3.P_max = 0.7
    RW1.Omega = 7
    RW2.Omega = -5
    RW3.Omega = 2

    # reconfigure a simulation stop time and re-execute the simulation run
    unitTestSim.ConfigureStopTime(3*simulationTime)
    unitTestSim.ExecuteSimulation()
    numTests += 1

    # expected output
    trueTorque.append([0.1, 0.1, -0.3])

    #
    # Fourth test
    #

    # reset the RW speed and set maximum values
    RW1.P_max = -1
    RW2.P_max = -1
    RW3.P_max = -1
    RW1.Omega = 100*macros.RPM
    RW2.Omega = -200*macros.RPM
    RW3.Omega = 50*macros.RPM
    RW1.Omega_max = 100*macros.RPM
    RW2.Omega_max = 100*macros.RPM
    RW3.Omega_max = 100*macros.RPM

    # reconfigure a simulation stop time and re-execute the simulation run
    unitTestSim.ConfigureStopTime(4*simulationTime)
    unitTestSim.ExecuteSimulation()
    numTests += 1

    # expected output
    trueTorque.append([0., 0.1, -0.3])

    #
    # retrieve the logged data
    #

    dataRW = []
    for i in range(numRW):
        dataRW.append(rwLogs[i].u_current)
    dataRW = np.array(dataRW)

    #
    # compare the module results to the true values
    #

    # do the comparison
    for i in range(numTests+1):
        # check a vector values
        if not unitTestSupport.isArrayEqual(dataRW[:, i], trueTorque[i], numRW, accuracy):
            testFailCount += 1
            if i == 0:
                testMessages.append("FAILED: Reaction Wheel Update Test failed setup")
            else:
                testMessages.append("FAILED: Reaction Wheel Update Test failed test " + str(i) + "\n")

    if not testFailCount:
        print("PASSED")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

#
# Run this unitTest as a standalone python script
#
if __name__ == "__main__":
    test_RWUpdate(
               False        # show_plots
               , 1e-8       # accuracy
    )
