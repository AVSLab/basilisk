#
#  ISC License
#
#  Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Module Name:        vizInterface
#   Author:             Jack Fox
#   Creation Date:      November 4, 2024
#

import inspect
import os
import pytest
import numpy as np

# Protobuffer specific
try:
    import vizMessage_pb2
    import google.protobuf.internal.decoder as decoder
    protoFound = True
except ModuleNotFoundError:
    protoFound = False

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


# Import all modules that are going to be called in this simulation
from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody, unitTestSupport,
                                vizSupport, simIncludeThruster)
from Basilisk.simulation import spacecraft
from Basilisk.architecture import messaging
from Basilisk.simulation import thrusterDynamicEffector
import pytest
import time
try:
    from Basilisk.simulation import vizInterface
except ImportError:
    pass


# Uncomment this line if this test is to be skipped in the global unit test run, adjust message as needed.
# @pytest.mark.skipif(conditionstring)
# Uncomment this line if this test has an expected failure, adjust message as needed.
# @pytest.mark.xfail(conditionstring)
# Provide a unique test method name, starting with 'test_'.
# The following 'parametrize' function decorator provides the parameters and expected results for each
# of the multiple test runs for this test.  Note that the order in that you add the parametrize method
# matters for the documentation in that it impacts the order in which the test arguments are shown.
# The first parametrize arguments are shown last in the pytest argument list
@pytest.mark.parametrize("accuracy", [1e-8])
def test_vizInterface(show_plots, accuracy):
    r"""
    **Validation Test Description**

    This unit test script tests the vizInterface module. Though this module is largely hand-tested due to its
    interactive nature, this script tests the packed protobuffers that are produces in the saved binary file to ensure
    all elements are captured as expected.

    **Test Parameters**

    Args:
        accuracy (float): absolute accuracy value used in the validation tests

    """
    [testResults, testMessage] = vizInterfaceTest(show_plots, accuracy)
    assert testResults < 1, testMessage


def vizInterfaceTest(show_plots, accuracy):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    # Early quit if protobuf or Vizard not configured
    if not protoFound or not vizSupport.vizFound:
        return [testFailCount, ''.join(testMessages)]

    # Create simulation variable names
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    #
    # Create the simulation process
    #
    testProcessRate = macros.sec2nano(1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    frames = 10
    simulationTime = macros.sec2nano(frames)

    #
    # Set up the simulation tasks/objects
    #

    # Create the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "testSat"
    # Add spacecraft object to the simulation process
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True
    mu = planet.mu
    # Attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject)

    #
    # Set up orbit
    #
    oe = orbitalMotion.ClassicElements()
    rGEO = 42000. * 1000  # meters
    oe.a = rGEO
    oe.e = 0.00001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 0 * macros.D2R
    oe.omega = 0 * macros.D2R
    oe.f = 0 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)

    #
    # Initialize Spacecraft States with the initialization variables
    #
    scObject.hub.r_CN_NInit = rN                   # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN                   # m/s - v_BN_N
    scObject.hub.omega_BN_BInit = (0.1, 0.2, 0.3)  # rad/s - sigma_BN

    # Create spacecraft data container
    scData = vizInterface.VizSpacecraftData()
    scData.spacecraftName = scObject.ModelTag
    scData.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    samplingTime = unitTestSupport.samplingTime(simulationTime, testProcessRate, 100)

    # Create data recorders
    scState_dataRec = scObject.scStateOutMsg.recorder(samplingTime)
    unitTestSim.AddModelToTask(unitTaskName, scState_dataRec)

    sName = "testVizInterface"
    viz = vizSupport.enableUnityVisualization(unitTestSim, unitTaskName, scObject
                                              , saveFile=sName)

    viz.settings.orbitLinesOn = 1
    viz.settings.spacecraftCSon = 1
    viz.settings.keyboardLiveInput = "abcd"

    #
    #   Initialize/execute simulation
    #
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(simulationTime)
    unitTestSim.ExecuteSimulation()

    # Read in binary save file, parse message list
    msgList = read_protobuf_messages("./_VizFiles/" + sName + "_UnityViz.bin")

    # Assert file size
    assert len(msgList) == frames, "File is missing messages"

    # Check spacecraft states (pos, vel, rot)
    checkSpacecraftStates(msgList, scState_dataRec, accuracy)

    # Check celestial bodies
    checkCelestialBodies(msgList)

    # Check settings
    checkSettings(msgList, testProcessRate)

    # Delete binary file
    os.remove("./_VizFiles/" + sName + "_UnityViz.bin")

    # Each test method requires a single assert method to be called
    # This check below just makes sure no subtest failures were found
    return [testFailCount, ''.join(testMessages)]


# Parses varint from file
def read_varint(file):
    """Reads a varint from the file."""
    varint_buffer = []
    while True:
        byte = file.read(1)
        if not byte:
            raise EOFError("Unexpected end of file while reading varint.")
        varint_buffer.append(byte)
        # If the highest bit is 0, this is the last byte of the varint.
        if ord(byte) < 0x80:
            break
    # Convert the list of bytes into a bytes object.
    varint_bytes = b"".join(varint_buffer)
    # Decode the varint from the bytes object.
    message_size, _ = decoder._DecodeVarint32(varint_bytes, 0)
    return message_size


# Parses protobuffer messages from binary file
def read_protobuf_messages(fname):
    messages = []
    with open(fname, 'rb') as f:
        while True:
            try:
                # Read the varint that indicates the message size.
                message_size = read_varint(f)

                # Now read the serialized message based on the decoded size.
                serialized_message = f.read(message_size)
                if len(serialized_message) != message_size:
                    raise EOFError("File ended unexpectedly while reading a message.")

                # Parse the message into a Protobuf object.
                message = vizMessage_pb2.VizMessage()
                message.ParseFromString(serialized_message)

                # Append the parsed message to the list.
                messages.append(message)

            except EOFError:
                # Break the loop if we reach the end of the file.
                break

    return messages


# Checks spacecraft states between data recorder and saved binary
def checkSpacecraftStates(msgList, scState_dataRec, accuracy):

    r_BN_N = scState_dataRec.r_BN_N
    v_BN_N = scState_dataRec.v_BN_N
    sigma_BN = scState_dataRec.sigma_BN

    n = len(msgList)

    for i in range(n):
        msg_i = msgList[i]
        # Num spacecraft check
        assert len(msg_i.spacecraft) == 1, "Number of spacecraft mismatch"
        # Position check
        protoPos = msg_i.spacecraft[0].position
        recPos = r_BN_N[i+1][0:3]
        assert np.isclose(protoPos, recPos, 0, accuracy).all(), "Position mismatch"
        # Velocity check
        protoVel = msg_i.spacecraft[0].velocity
        recVel = v_BN_N[i+1][0:3]
        assert np.isclose(protoVel, recVel, 0, accuracy).all(), "Velocity mismatch"
        # Rotation check
        protoRot = msg_i.spacecraft[0].rotation
        recRot = sigma_BN[i+1][0:3]
        assert np.isclose(protoRot, recRot, 0, accuracy).all(), "Rotation mismatch"


# Checks number of celestial bodies and central body pos/vel
def checkCelestialBodies(msgList):
    n = len(msgList)
    for i in range(n):
        msg_i = msgList[i]
        # Number of celestial bodies check
        assert len(msg_i.celestialBodies) == 1, "Celestial bodies mismatch"
        # Central body checks
        assert msg_i.celestialBodies[0].position == [0.0, 0.0, 0.0], "Celestial body position mismatch"
        assert msg_i.celestialBodies[0].velocity == [0.0, 0.0, 0.0], "Celestial body velocity mismatch"


# Checks for settings message at first timestep, validates contents
def checkSettings(msgList, testProcessRate):
    n = len(msgList)
    for i in range(n):
        msg_i = msgList[i]

        assert msg_i.currentTime.frameNumber == i+1, "Frame number is incorrect"
        assert msg_i.currentTime.simTimeElapsed == testProcessRate*(i+1), "Sim time elapsed is incorrect"

        if i == 0:
            # Check for settings
            assert msg_i.HasField("settings"), "Should have settings message at first timestep"
            assert msg_i.HasField("epoch"), "Should have epoch message at first timestep"
            # Validate specific settings
            assert msg_i.settings.orbitLinesOn == 1, "Orbit lines not on"
            assert msg_i.settings.spacecraftCSon == 1, "Spacecraft CS not on"
            assert msg_i.settings.keyboardLiveInput == "abcd", "Incorrect key listeners"
        else:
            # Check for absence of settings
            assert not msg_i.HasField("settings"), "Should only have settings message at first timestep"


#
# Run this unitTest as a stand-along python script
#
if __name__ == "__main__":
    test_vizInterface(
        False,  # show_plots
        1e-8    # accuracy
    )
