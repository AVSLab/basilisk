# ISC License
#
# Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import os
import numpy as np
import math
import pytest
import inspect

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.simulation import spacecraft, thrusterStateEffector, svIntegrators
from Basilisk.architecture import messaging
import matplotlib.pyplot as plt


class ResultsStore:
    def __init__(self):
        self.PassFail = []

    def texSnippet(self):
        for i in range(len(self.PassFail)):
            snippetName = 'Result' + str(i)
            if self.PassFail[i] == 'PASSED':
                textColor = 'ForestGreen'
            elif self.PassFail[i] == 'FAILED':
                textColor = 'Red'
            texSnippet = r'\textcolor{' + textColor + '}{' + self.PassFail[i] + '}'
            unitTestSupport.writeTeXSnippet(snippetName, texSnippet, path)


@pytest.fixture(scope="module")
def testFixture():
    listRes = ResultsStore()
    yield listRes
    listRes.texSnippet()


def thrusterEffectorAllTests(show_plots):
    [testResults, testMessage] = test_unitThrusters(show_plots)

# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)


@pytest.mark.parametrize("thrustNumber, initialConditions, duration, long_angle, lat_angle, location, rate", [
    (1, 0., 2.0, 30., 15., [[1.125], [0.5], [2.0]], macros.sec2nano(0.01)),
    (1, 1., 0.0, 30., 15., [[1.125], [0.5], [2.0]], macros.sec2nano(0.01)),
    (1, 0., 2.0, 60., -15., [[-1.125], [0.5], [-2.0]], macros.sec2nano(0.01)),
    (1, 1., 0.0, 60., -15., [[-1.125], [0.5], [-2.0]], macros.sec2nano(0.01)),
    (2, 0., 2.0, 30., 15., [[1.125], [0.5], [2.0]], macros.sec2nano(0.01)),
    (2, 1., 0.0, 30., 15., [[1.125], [0.5], [2.0]], macros.sec2nano(0.01)),
])
# provide a unique test method name, starting with test_
def test_unitThrusters(testFixture, show_plots, thrustNumber, initialConditions, duration, long_angle, lat_angle, location, rate):
    r"""
    **Validation Test Description**

    This unit test script tests the stateEffector implementation of thrusters. It sets up the thruster module and runs
    a combination of 6 different scenarios. Each scenario uses either one or two thrusters, while also changing the
    thruster's locations and whether thruster 1 is firing or not.

    For information on how the thruster module works and what the closed-form solution for the ``thrustFactor`` variable
    is, see :ref:`thrusterStateEffector`. Given the ``thrustFactor`` :math:`\kappa`, the thrust is computed as follows:

    .. math::
        \textbf{F} = \kappa \cdot F_{max} \cdot \hat{n}

    where :math:`\hat{n}` is the thruster's direction vector. The torque is computed by:

    .. math::
        \textbf{T} = \textbf{r}\times\textbf{F}

    where :math:`\textbf{r}` corresponds to the thruster's position relative to the spacecraft's center of mass. The
    mass flow rate is given by:

    .. math::
        \dot{m} = \dfrac{F}{g\cdot I_{sp}}

    where :math:`g` is Earth's gravitational acceleration and :math:`I_{sp}` is the thruster's specific impulse.

    **Test Parameters**

    Args:
        thrustNumber (int): number of thrusters used in the simulation
        initialConditions (float): initial value of the ``thrustFactor`` variable for thruster 1. Thruster always starts
        off.
        duration (float): duration of the thrust in seconds.
        long_angle (float): longitude angle in degrees for thruster 1. Thruster 2 is also impacted by this value.
        lat_angle (float): latitude angle in degrees for thruster 1. Thruster 2 is also impacted by this value.
        location (float): location of thruster 1.
        rate (int): simulation rate in nanoseconds.

    **Description of Variables Being Tested**

    In this file we are checking the values of the variables

    - ``thrForce``
    - ``thrTorque``
    - ``mDot``

    All these variables are compared to the true values from the closed-form expressions given in :ref:`thrusterStateEffector`.
    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitThrusters(testFixture, show_plots, thrustNumber, initialConditions, duration, long_angle,
                                               lat_angle, location, rate)
    assert testResults < 1, testMessage


# Run the test
def unitThrusters(testFixture, show_plots, thrustNumber, initialConditions, duration, long_angle, lat_angle, location, rate):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    #  Create a simulation and set the rate
    TotalSim = SimulationBaseClass.SimBaseClass()
    testRate = int(rate)  # Parametrized rate of test

    # Create the process and task
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    testProc = TotalSim.CreateNewProcess(unitProcessName)
    testProc.addTask(TotalSim.CreateNewTask(unitTaskName, testRate))

    # Create the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # Define initial conditions of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
    scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]

    # Constants for thruster creation
    g = 9.80665
    Isp = 226.7

    #  Create the thrusters
    thrusterSet = thrusterStateEffector.ThrusterStateEffector()
    thrusterSet.ModelTag = "ACSThrusterDynamics"

    #  Create thruster characteristic parameters (position, angle thrust, ISP, time of thrust) for thruster 1
    long_angle_deg = long_angle  # Parametrized angle of thrust
    lat_angle_deg = lat_angle
    long_angle_rad = long_angle_deg * math.pi / 180.0
    lat_angle_rad = lat_angle_deg * math.pi / 180.0
    thruster1 = thrusterStateEffector.THRSimConfig()
    thruster1.thrLoc_B = location  # Parametrized location for thruster
    thruster1.thrDir_B = [[math.cos(long_angle_rad) * math.cos(lat_angle_rad)],
                          [math.sin(long_angle_rad) * math.cos(lat_angle_rad)], [math.sin(lat_angle_rad)]]
    thruster1.MaxThrust = 10.0
    thruster1.steadyIsp = 226.7
    thruster1.MinOnTime = 0.006
    thruster1.cutoffFrequency = 5
    thrusterSet.addThruster(thruster1)

    loc1 = np.array([thruster1.thrLoc_B[0][0], thruster1.thrLoc_B[1][0], thruster1.thrLoc_B[2][0]])
    dir1 = np.array([thruster1.thrDir_B[0][0], thruster1.thrDir_B[1][0], thruster1.thrDir_B[2][0]])

    #  Create thruster characteristic parameters for thruster 2
    if thrustNumber == 2:
        thruster2 = thrusterStateEffector.THRSimConfig()
        thruster2.thrLoc_B = np.array([[1.], [0.0], [0.0]]).reshape([3, 1])
        thruster2.thrDir_B = np.array(
            [[math.cos(long_angle_rad + math.pi / 4.) * math.cos(lat_angle_rad - math.pi / 4.)],
             [math.sin(long_angle_rad + math.pi / 4.) * math.cos(lat_angle_rad - math.pi / 4.)],
             [math.sin(lat_angle_rad - math.pi / 4.)]]).reshape([3, 1])
        thruster2.MaxThrust = 20.0
        thruster2.steadyIsp = 226.7
        thruster2.MinOnTime = 0.006
        thruster2.cutoffFrequency = 2
        thrusterSet.addThruster(thruster2)

        loc2 = np.array([thruster2.thrLoc_B[0][0], thruster2.thrLoc_B[1][0], thruster2.thrLoc_B[2][0]])
        dir2 = np.array([thruster2.thrDir_B[0][0], thruster2.thrDir_B[1][0], thruster2.thrDir_B[2][0]])

    # Set the initial conditions
    thrusterSet.kappaInit = messaging.DoubleVector([initialConditions])

    # Add the thrusters to the spacecraft
    scObject.addStateEffector(thrusterSet)

    # Save state
    dataRec = thrusterSet.thrusterOutMsgs[0].recorder(testRate)

    # Add both modules and the recorder to tasks
    TotalSim.AddModelToTask(unitTaskName, scObject)
    TotalSim.AddModelToTask(unitTaskName, thrusterSet)
    TotalSim.AddModelToTask(unitTaskName, dataRec)

    #  Define the start of the thrust and its duration
    thrDurationTime = macros.sec2nano(2.0)

    # Log variables of interest
    TotalSim.AddVariableForLogging('ACSThrusterDynamics.forceOnBody_B', testRate, 0, 2)
    TotalSim.AddVariableForLogging('ACSThrusterDynamics.torqueOnBodyPntB_B', testRate, 0, 2)
    TotalSim.AddVariableForLogging('ACSThrusterDynamics.mDotTotal', testRate, 0, 0)

    #  Configure a single thruster firing, create a message for it
    ThrustMessage = messaging.THRArrayOnTimeCmdMsgPayload()
    if thrustNumber == 1:
        ThrustMessage.OnTimeRequest = [duration]
    if thrustNumber == 2:
        ThrustMessage.OnTimeRequest = [duration, 2.]
    thrCmdMsg = messaging.THRArrayOnTimeCmdMsg().write(ThrustMessage)
    thrusterSet.cmdsInMsg.subscribeTo(thrCmdMsg)

    # Initialize the simulation
    TotalSim.InitializeSimulation()

    # Close all plots
    plt.close("all")

    # Run the simulation
    TotalSim.ConfigureStopTime(TotalSim.TotalSim.CurrentNanos + int(thrDurationTime))
    TotalSim.ExecuteSimulation()

    # Plot the thrust factor if needed
    dataThrustFactor = dataRec.thrustFactor
    plt.figure(1)
    plt.plot(dataRec.times() * macros.NANO2SEC, dataThrustFactor)
    plt.xlabel('Time [s]')
    plt.ylabel('Thrust Factor')
    if show_plots:
        plt.show()

    # Gather the Force, Torque and Mass Rate results
    thrForce = TotalSim.GetLogVariableData('ACSThrusterDynamics.forceOnBody_B')
    thrTorque = TotalSim.GetLogVariableData('ACSThrusterDynamics.torqueOnBodyPntB_B')
    mDot = TotalSim.GetLogVariableData('ACSThrusterDynamics.mDotTotal')

    # Save the time vector
    timeSec = dataRec.times() * macros.NANO2SEC

    # Generate the truth data (force, torque and mass rate)
    expectedThrustData = np.zeros([3, np.shape(thrForce)[0]])
    expectedTorqueData = np.zeros([3, np.shape(thrTorque)[0]])
    expectedMDot = np.zeros([np.shape(mDot)[0], 1])
    for i in range(np.shape(thrForce)[0]):
        if thrustNumber == 1:
            # Compute the thrust force
            if duration == 0.:
                force1 = initialConditions * np.exp(- thruster1.cutoffFrequency * timeSec[i]) * thruster1.MaxThrust * dir1
                expectedThrustData[0:3, i] = force1
            else:
                force1 = (1.0 + (initialConditions - 1.0) * np.exp(- thruster1.cutoffFrequency * timeSec[i])) * thruster1.MaxThrust * dir1
                expectedThrustData[0:3, i] = force1
            # Compute the torque
            expectedTorqueData[0:3, i] = np.cross(loc1, force1)
            # Compute the mass flow rate
            expectedMDot[i, 0] = thruster1.MaxThrust / (g * Isp)
        else:
            # Compute the thrust force
            if duration == 0.:
                force1 = initialConditions * np.exp(- thruster1.cutoffFrequency * timeSec[i]) * thruster1.MaxThrust * dir1
                force2 = (1.0 - np.exp(- thruster2.cutoffFrequency * timeSec[i])) * thruster2.MaxThrust * dir2
                expectedThrustData[0:3, i] = force1 + force2
            else:
                force1 = (1.0 + (initialConditions - 1.0) * np.exp(- thruster1.cutoffFrequency * timeSec[i])) * thruster1.MaxThrust * dir1
                force2 = (1.0 - np.exp(- thruster2.cutoffFrequency * timeSec[i])) * thruster2.MaxThrust * dir2
                expectedThrustData[0:3, i] = force1 + force2
            # Compute the torque
            expectedTorqueData[0:3, i] = np.cross(loc1, force1) + np.cross(loc2, force2)
            # Compute the mass flow rate
            expectedMDot[i, 0] = (thruster1.MaxThrust + thruster2.MaxThrust) / (g * Isp)

    # Modify expected values for comparison and define errorTolerance
    TruthForce = np.transpose(expectedThrustData)
    TruthTorque = np.transpose(expectedTorqueData)
    ErrTolerance = 1E-3

    # Compare Force values
    thrForce = np.delete(thrForce, 0, axis=1)  # remove time column
    testFailCount, testMessages = unitTestSupport.compareArray(TruthForce, thrForce, ErrTolerance, "Force",
                                                               testFailCount, testMessages)

    # Compare Torque values
    thrTorque = np.delete(thrTorque, 0, axis=1)  # remove time column
    testFailCount, testMessages = unitTestSupport.compareArray(TruthTorque, thrTorque, ErrTolerance, "Torque",
                                                               testFailCount, testMessages)

    # Compare mass flow rate values
    mDot = np.delete(mDot, 0, axis=1)
    ErrTolerance = 1E-6
    for i in range(0, len(np.array(mDot))):
        if not unitTestSupport.isArrayEqual(np.array(mDot)[i, :], expectedMDot[i, :], 1, ErrTolerance):
            testFailCount += 1
            testMessages.append('M dot failure')

    if testFailCount == 0:
        print("PASSED")
        testFixture.PassFail.append("PASSED")
    else:
        testFixture.PassFail.append("FAILED")
        print(testMessages)

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


if __name__ == "__main__":
    unitThrusters(ResultsStore(), True, 1, 0.0, 2.0, 30., 15., [[1.125], [0.5], [2.0]], macros.sec2nano(0.01))
