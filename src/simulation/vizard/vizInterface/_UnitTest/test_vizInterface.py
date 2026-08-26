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

import gc
import inspect
import os
import tempfile

import numpy as np
import psutil
import pytest

# Protobuffer-specific modules are generated with vizInterface.
from Basilisk import hasBuildFeature

vizInterfaceEnabled = hasBuildFeature("vizInterface")
pytestmark = pytest.mark.skipif(
    not vizInterfaceEnabled,
    reason="Requires Basilisk built with --vizInterface True",
)
if vizInterfaceEnabled:
    from Basilisk.utilities.vizProtobuffer import vizMessage_pb2
    import google.protobuf.internal.decoder as decoder

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)


# Import all modules that are going to be called in this simulation
from Basilisk.utilities import (
    SimulationBaseClass,
    macros,
    orbitalMotion,
    simIncludeGravBody,
    vizSupport,
    simIncludeThruster,
)
from Basilisk.utilities import simHelpers
from Basilisk.simulation import spacecraft
from Basilisk.architecture import messaging, bskLogging
from Basilisk.simulation import thrusterDynamicEffector
import time
if vizInterfaceEnabled:
    from Basilisk.simulation import vizInterface


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
def test_vizInterface(show_plots, accuracy, tmp_path):
    r"""
    **Validation Test Description**

    This unit test script tests the ``vizInterface`` module. Though this module is largely hand-tested due to its
    interactive nature, this script tests the packed protocol buffers in the saved binary file to ensure all elements
    are captured as expected.

    :param show_plots: Flag controlling whether validation plots are displayed.
    :param accuracy: Absolute accuracy used in the validation checks.
    :param tmp_path: Temporary directory for the generated protobuf file.
    """
    output_file = tmp_path / "testVizInterface_UnityViz.bin"
    [testResults, testMessage] = vizInterfaceTest(show_plots, accuracy, output_file)
    assert testResults < 1, testMessage


def test_vizInterface_long_gravity_body_name_reset():
    r"""
    **Validation Test Description**

    This test verifies that ``vizInterface`` can reset with a celestial body name longer than the fixed
    ``SpicePlanetStateMsgPayload.PlanetName`` storage.  The full body name remains available to the Vizard
    protocol buffer path, while the default SPICE payload name must be copied without overflowing its backing array.
    """
    reset_time = 0  # [ns]
    long_body_name = "body_" + "x" * 128

    grav_body = vizInterface.GravBodyInfo()
    grav_body.bodyName = long_body_name

    viz = vizInterface.VizInterface()
    viz.gravBodyInformation = vizInterface.GravBodyInfoVector([grav_body])

    with pytest.raises(bskLogging.BasiliskError, match="celestial body name is 133 characters"):
        viz.Reset(reset_time)


def test_vizInterface_closes_output_files(tmp_path):
    """Verify that output files are released on reset and module destruction.

    Windows does not permit removing a file while the writing process still
    holds it open, so these removals also exercise the stream lifetime.

    :param tmp_path: Temporary directory for the generated protobuf files.
    """
    first_output = tmp_path / "first.bin"
    second_output = tmp_path / "second.bin"
    reset_time = 0  # [ns]

    viz = vizInterface.VizInterface()
    viz.saveFile = True
    viz.protoFilename = str(first_output)
    viz.Reset(reset_time)

    viz.protoFilename = str(second_output)
    viz.Reset(reset_time)
    first_output.unlink()

    del viz
    second_output.unlink()


@pytest.mark.flaky(reruns=3, reruns_delay=0)
def test_vizInterface_broadcast_without_subscriber():
    r"""Verify that broadcast-only execution remains memory-bounded without a Vizard subscriber.

    This exercises the publisher serialization path and the no-save-file return
    path over multiple frames. RSS is sampled after an initial warm-up so one-time
    Python, protobuf, and ZeroMQ allocations are excluded from the measured growth.
    A subscriber is intentionally absent because a ZeroMQ publisher must be able
    to operate without one.
    """
    taskRate = macros.sec2nano(0.01)  # [ns]
    warmupFrames = 200
    measuredFrames = 2000
    totalFrames = warmupFrames + measuredFrames
    warmupTime = warmupFrames * taskRate  # [ns]
    simulationTime = totalFrames * taskRate  # [ns]
    # Comparable macOS soak runs grew about 0.5 MiB with the fix and more than
    # 4.5 MiB with the original leak; this limit allows for allocator noise.
    maxAllowedGrowth = 3.0  # [MiB]

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcess = unitTestSim.CreateNewProcess("broadcastProcess")
    testProcess.addTask(unitTestSim.CreateNewTask("broadcastTask", taskRate))

    spacecraftModel = spacecraft.Spacecraft()
    spacecraftModel.ModelTag = "broadcastSat"
    unitTestSim.AddModelToTask("broadcastTask", spacecraftModel)

    viz = vizSupport.enableUnityVisualization(
        unitTestSim,
        "broadcastTask",
        spacecraftModel,
        saveFile=None,
        liveStream=False,
        broadcastStream=True,
    )

    viz.pubComAddress = "127.0.0.1"
    viz.pubPortNumber = "*"

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(warmupTime)
    unitTestSim.ExecuteSimulation()

    process = psutil.Process(os.getpid())
    gc.collect()
    initialMemory = process.memory_info().rss / 1024**2  # [MiB]

    unitTestSim.ConfigureStopTime(simulationTime)
    unitTestSim.ExecuteSimulation()

    gc.collect()
    finalMemory = process.memory_info().rss / 1024**2  # [MiB]
    memoryGrowth = finalMemory - initialMemory  # [MiB]

    assert viz.FrameNumber == totalFrames
    assert memoryGrowth < maxAllowedGrowth, (
        f"Broadcast-only RSS grew by {memoryGrowth:.2f} MiB over {measuredFrames} frames; "
        f"expected less than {maxAllowedGrowth:.2f} MiB."
    )


def vizInterfaceTest(show_plots, accuracy, output_file):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

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

    samplingTime = simHelpers.samplingTime(simulationTime, testProcessRate, 100)

    # Create data recorders
    scState_dataRec = scObject.scStateOutMsg.recorder(samplingTime)
    unitTestSim.AddModelToTask(unitTaskName, scState_dataRec)

    viz = vizSupport.enableUnityVisualization(unitTestSim, unitTaskName, scObject
                                              , saveFile=str(output_file))

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
    msgList = read_protobuf_messages(output_file)

    # Assert file size
    assert len(msgList) == frames, "File is missing messages"

    # Check spacecraft states (pos, vel, rot)
    checkSpacecraftStates(msgList, scState_dataRec, accuracy)

    # Check celestial bodies
    checkCelestialBodies(msgList)

    # Check settings
    checkSettings(msgList, testProcessRate)

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
# Run this unit test as a stand-alone Python script
#
if __name__ == "__main__":
    if not vizInterfaceEnabled:
        raise RuntimeError("Requires Basilisk built with --vizInterface True")
    with tempfile.TemporaryDirectory() as temporary_directory:
        output_file = os.path.join(temporary_directory, "testVizInterface_UnityViz.bin")
        [testResults, testMessage] = vizInterfaceTest(
            False,  # show_plots
            1e-8,   # [-] accuracy
            output_file,
        )
        assert testResults < 1, testMessage
