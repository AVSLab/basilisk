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

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros, RigidBodyKinematics as rbk, \
    simulationArchTypes
from Basilisk.simulation import spacecraft, thrusterDynamicEffector
from Basilisk.architecture import messaging
import matplotlib.pyplot as plt


def thrusterEffectorAllTests(show_plots):
    [testResults, testMessage] = test_unitThrusters(show_plots)


# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)


@pytest.mark.parametrize("long_angle, lat_angle, location, rate", [
    (30., 15., [[1.125], [0.5], [2.0]], macros.sec2nano(0.01)),  # 1 thruster, thrust on
])
# provide a unique test method name, starting with test_
def test_unitThrusters(show_plots, long_angle, lat_angle, location, rate):
    r"""
    This unit test checks the functionality of attaching a dynamic thruster to a body other than the hub.
    """
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitThrusters(show_plots, long_angle, lat_angle, location, rate)
    assert testResults < 1, testMessage


# Run the test
def unitThrusters(show_plots, long_angle, lat_angle, location, rate):
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    #  Create a simulation and set the rate
    TotalSim = SimulationBaseClass.SimBaseClass()
    testRate = int(rate)  # Parametrized rate of test

    # Create the process and task
    unitTaskName1 = "unitTask1"  # arbitrary name (don't change)
    unitTaskName2 = "unitTask2"  # arbitrary name (don't change)
    unitProcessName1 = "TestProcess1"  # arbitrary name (don't change)
    unitProcessName2 = "TestProcess2"  # arbitrary name (don't change)
    testProc1 = TotalSim.CreateNewProcess(unitProcessName1, 10)
    testProc1.addTask(TotalSim.CreateNewTask(unitTaskName1, testRate))
    testProc2 = TotalSim.CreateNewProcess(unitProcessName2, 0)
    testProc2.addTask(TotalSim.CreateNewTask(unitTaskName2, testRate))

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
    thrusterSet = thrusterDynamicEffector.ThrusterDynamicEffector()
    thrusterSet.ModelTag = "ACSThrusterDynamics"

    #  Create thruster characteristic parameters (position, angle thrust, ISP, time of thrust) for thruster 1
    long_angle_deg = long_angle  # Parametrized angle of thrust
    lat_angle_deg = lat_angle
    long_angle_rad = long_angle_deg * math.pi / 180.0
    lat_angle_rad = lat_angle_deg * math.pi / 180.0
    thruster = thrusterDynamicEffector.THRSimConfig()
    thruster.thrLoc_B = location  # Parametrized location for thruster
    thruster.thrDir_B = [[math.cos(long_angle_rad) * math.cos(lat_angle_rad)],
                         [math.sin(long_angle_rad) * math.cos(lat_angle_rad)], [math.sin(lat_angle_rad)]]
    thruster.MaxThrust = 10.0
    thruster.steadyIsp = 226.7
    thruster.MinOnTime = 0.006
    thruster.cutoffFrequency = 5
    thruster.MaxSwirlTorque = 0.0
    thrusterSet.addThruster(thruster)

    # Create the process and task that contains the Python modules
    pyTaskName = "pyTask"
    pyProcessName = "pyProcess"
    pyModulesProcess = TotalSim.CreateNewPythonProcess(pyProcessName, 5)
    pyModulesProcess.createPythonTask(pyTaskName, testRate, True, 1)

    # Set up the dcm and location
    dcm_BF = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
    r_FB_B = [0, 0, 1]

    # Create the module
    pyModule = attachedBodyModule("attachedBody", dcm_BF, r_FB_B, True, 100)
    pyModulesProcess.addModelToTask(pyTaskName, pyModule)

    # Attach messages
    pyModule.scInMsg.subscribeTo(scObject.scStateOutMsg)
    thrusterSet.connectAttachedBody(pyModule.bodyOutMsg)

    # Define the location and direction with respect to the platform
    loc = np.array([thruster.thrLoc_B[0][0], thruster.thrLoc_B[1][0], thruster.thrLoc_B[2][0]])
    dir = np.array([thruster.thrDir_B[0][0], thruster.thrDir_B[1][0], thruster.thrDir_B[2][0]])

    # Update the direction and location of the thruster to the hub
    dir = dcm_BF.dot(dir)
    loc = dcm_BF.dot(loc) + r_FB_B

    # Add the thrusters to the spacecraft
    scObject.addDynamicEffector(thrusterSet)

    # Save state
    dataRec = thrusterSet.thrusterOutMsgs[0].recorder(testRate)

    # Add both modules and the recorder to tasks
    TotalSim.AddModelToTask(unitTaskName1, scObject)
    TotalSim.AddModelToTask(unitTaskName2, thrusterSet)
    TotalSim.AddModelToTask(unitTaskName2, dataRec)

    #  Define the start of the thrust and its duration
    testDurationTime = 2.0

    # Log variables of interest
    TotalSim.AddVariableForLogging('ACSThrusterDynamics.forceExternal_B', testRate, 0, 2)
    TotalSim.AddVariableForLogging('ACSThrusterDynamics.torqueExternalPntB_B', testRate, 0, 2)

    # Initialize the simulation
    TotalSim.InitializeSimulation()

    # Close all plots
    plt.close("all")

    #  Configure a single thruster firing, create a message for it
    ThrustMessage = messaging.THRArrayOnTimeCmdMsgPayload()
    thrDuration = testDurationTime / 2
    ThrustMessage.OnTimeRequest = [thrDuration]
    thrCmdMsg = messaging.THRArrayOnTimeCmdMsg().write(ThrustMessage)
    thrusterSet.cmdsInMsg.subscribeTo(thrCmdMsg)

    # Run the simulation
    TotalSim.ConfigureStopTime(macros.sec2nano(testDurationTime))
    TotalSim.ExecuteSimulation()

    # Gather the Force, Torque and Mass Rate results
    thrForce = TotalSim.GetLogVariableData('ACSThrusterDynamics.forceExternal_B')
    thrTorque = TotalSim.GetLogVariableData('ACSThrusterDynamics.torqueExternalPntB_B')

    # Generate the truth data (force, torque and mass rate)
    expectedThrustData = np.zeros([3, np.shape(thrForce)[0]])
    expectedTorqueData = np.zeros([3, np.shape(thrTorque)[0]])
    for i in range(np.shape(thrForce)[0]):
        # Compute the thrust force (zero at the first time step)
        if i < int(round(macros.sec2nano(thrDuration) / testRate)) + 1 and i != 0:
            thrustFactor = 1
        else:
            thrustFactor = 0
        force = thrustFactor * thruster.MaxThrust * dir
        expectedThrustData[0:3, i] = force
        # Compute the torque
        expectedTorqueData[0:3, i] = np.cross(loc, force)

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

    if testFailCount == 0:
        print("PASSED")
    else:
        print(testMessages)

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


class attachedBodyModule(simulationArchTypes.PythonModelClass):
    def __init__(self, modelName, dcm_BF, r_FB_B, modelActive=True, modelPriority=-1):
        super(attachedBodyModule, self).__init__(modelName, modelActive, modelPriority)

        # Input spacecraft state structure message
        self.scInMsg = messaging.SCStatesMsgReader()

        # Output body state message
        self.bodyOutMsg = messaging.SCStatesMsg()

        # Save dcm and location
        self.dcm_BF = dcm_BF
        self.r_FB_B = r_FB_B

    def reset(self, currentTime):
        return

    def updateState(self, currentTime):
        # Read input message
        scMsgBuffer = self.scInMsg()

        # Create output message buffer
        bodyOutMsgBuffer = messaging.SCStatesMsgPayload()

        # Grab the spacecraft hub states
        sigma_BN = scMsgBuffer.sigma_BN
        dcm_BN = rbk.MRP2C(sigma_BN)
        omega_BN_B = scMsgBuffer.omega_BN_B
        r_BN_N = scMsgBuffer.r_BN_N

        # Compute the attached body states relative to the hub
        dcm_FB = np.transpose(self.dcm_BF)
        sigma_FB = rbk.C2MRP(dcm_FB)
        sigma_FN = rbk.addMRP(np.array(sigma_BN), sigma_FB)
        omega_FB_F = dcm_FB.dot(omega_BN_B)
        r_FN_N = r_BN_N + np.transpose(dcm_BN).dot(np.array(self.r_FB_B))

        # Write the output message information
        bodyOutMsgBuffer.sigma_BN = sigma_FN
        bodyOutMsgBuffer.omega_BN_B = omega_FB_F
        bodyOutMsgBuffer.r_BN_N = r_FN_N
        self.bodyOutMsg.write(bodyOutMsgBuffer, currentTime, self.moduleID)

        return


if __name__ == "__main__":
    unitThrusters(False, 30., 15., [[1.125], [0.5], [2.0]], macros.sec2nano(0.01))
