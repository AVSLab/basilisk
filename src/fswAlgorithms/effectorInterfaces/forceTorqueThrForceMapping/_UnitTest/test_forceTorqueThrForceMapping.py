#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder
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

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import forceTorqueThrForceMapping
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import fswSetupThrusters
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport


def test_forceTorqueThrForceMapping1():
    r"""
    **Test Description**

    This pytest ensures that the forceTorqueThrForce module can compute a valid solution for cases where:
    1. There is a direction where no thrusters point - ensures matrix invertibility is handled

    """

    # Test 1 - No thrusters pointing in one direction, CoM offset
    rcsLocationData = [
        [-0.86360, -0.82550, 1.79070],
        [-0.82550, -0.86360, 1.79070],
        [0.82550, 0.86360, 1.79070],
        [0.86360, 0.82550, 1.79070],
        [-0.86360, -0.82550, -1.79070],
        [-0.82550, -0.86360, -1.79070],
        [0.82550, 0.86360, -1.79070],
        [0.86360, 0.82550, -1.79070],
    ]

    rcsDirectionData = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ]

    requested_torque = [0.4, 0.2, 0.4]

    requested_force = [0.9, 1.1, 0.0]

    CoM_B = [0.1, 0.1, 0.1]

    [testResults, testMessage] = forceTorqueThrForceMappingTestFunction(
        rcsLocationData,
        rcsDirectionData,
        requested_torque,
        requested_force,
        CoM_B,
        True,
    )

    assert testResults < 1, testMessage


def test_forceTorqueThrForceMapping2():
    r"""
    **Test Description**

    This pytest ensures that the forceTorqueThrForce module can compute a valid solution for the case
    where there is zero requested torque in a connected input message, but a requested non-zero force

    """

    # Test 1 - No thrusters pointing in one direction, CoM offset
    rcsLocationData = [
        [-0.86360, -0.82550, 1.79070],
        [-0.82550, -0.86360, 1.79070],
        [0.82550, 0.86360, 1.79070],
        [0.86360, 0.82550, 1.79070],
        [-0.86360, -0.82550, -1.79070],
        [-0.82550, -0.86360, -1.79070],
        [0.82550, 0.86360, -1.79070],
        [0.86360, 0.82550, -1.79070],
    ]

    rcsDirectionData = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ]

    requested_force = [0.9, 1.1, 0.0]

    CoM_B = [0.1, 0.1, 0.1]

    requested_torque = [0.0, 0.0, 0.0]

    [testResults, testMessage] = forceTorqueThrForceMappingTestFunction(
        rcsLocationData,
        rcsDirectionData,
        requested_torque,
        requested_force,
        CoM_B,
        True,
    )

    assert testResults < 1, testMessage


def test_forceTorqueThrForceMapping3():
    r"""
    **Test Description**

    This pytest ensures that the forceTorqueThrForce module can compute a valid solution for the case
    where there is no torque input message, but a requested non-zero force

    """

    # Test 1 - No thrusters pointing in one direction, CoM offset
    rcsLocationData = [
        [-0.86360, -0.82550, 1.79070],
        [-0.82550, -0.86360, 1.79070],
        [0.82550, 0.86360, 1.79070],
        [0.86360, 0.82550, 1.79070],
        [-0.86360, -0.82550, -1.79070],
        [-0.82550, -0.86360, -1.79070],
        [0.82550, 0.86360, -1.79070],
        [0.86360, 0.82550, -1.79070],
    ]

    rcsDirectionData = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ]

    requested_force = [0.9, 1.1, 0.0]

    CoM_B = [0.1, 0.1, 0.1]

    requested_torque = [0.0, 0.0, 0.0]

    [testResults, testMessage] = forceTorqueThrForceMappingTestFunction(
        rcsLocationData,
        rcsDirectionData,
        requested_torque,
        requested_force,
        CoM_B,
        False,
    )

    assert testResults < 1, testMessage


def test_forceTorqueThrForceMapping4():
    r"""
    **Test Description**

    This pytest ensures that the forceTorqueThrForce module can compute a valid solution for the case where
    Thrusters point in each direction

    """

    rcsLocationData = [
        [-1, -1, 1],
        [-1, -1, 1],
        [-1, -1, 1],
        [1, 1, 1],
        [1, 1, 1],
        [1, 1, 1],
        [1, 1, -1],
        [1, 1, -1],
        [1, 1, -1],
        [-1, -1, -1],
        [-1, -1, -1],
        [-1, -1, -1],
    ]

    rcsDirectionData = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 0.0, -1.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]

    CoM_B = [0.1, 0.1, 0.1]
    requested_torque = [0.0, 0.0, 0.0]
    requested_force = [0.9, 1.1, 1.0]

    [testResults, testMessage] = forceTorqueThrForceMappingTestFunction(
        rcsLocationData,
        rcsDirectionData,
        requested_torque,
        requested_force,
        CoM_B,
        True,
    )
    assert testResults < 1, testMessage


def calculate_force_torque(thruster_forces, locations, directions, CoM_B):
    """Calculate the resulting force and torque from thruster forces"""
    total_force = np.zeros(3)
    total_torque = np.zeros(3)

    for i, force in enumerate(thruster_forces):
        # Force contribution
        force_vector = force * directions[i]
        total_force += force_vector

        # Torque contribution (r x F)
        r_rel_com = locations[i] - CoM_B
        torque_contribution = np.cross(r_rel_com, force_vector)
        total_torque += torque_contribution

    return total_torque, total_force


def forceTorqueThrForceMappingTestFunction(
    rcsLocation, rcsDirection, requested_torque, requested_force, CoM_B, torqueInMsgFlag
):
    """Test method"""
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = forceTorqueThrForceMapping.forceTorqueThrForceMapping()
    module.ModelTag = "forceTorqueThrForceMappingTag"
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Configure blank module input messages
    cmdTorqueInMsgData = messaging.CmdTorqueBodyMsgPayload()
    cmdTorqueInMsgData.torqueRequestBody = requested_torque
    cmdTorqueInMsg = messaging.CmdTorqueBodyMsg().write(cmdTorqueInMsgData)

    cmdForceInMsgData = messaging.CmdForceBodyMsgPayload()
    cmdForceInMsgData.forceRequestBody = requested_force
    cmdForceInMsg = messaging.CmdForceBodyMsg().write(cmdForceInMsgData)

    numThrusters = len(rcsLocation)
    maxThrust = 3.0  # N
    MAX_EFF_CNT = messaging.MAX_EFF_CNT
    rcsLocationData = np.zeros((MAX_EFF_CNT, 3))
    rcsDirectionData = np.zeros((MAX_EFF_CNT, 3))

    rcsLocationData[0 : len(rcsLocation)] = rcsLocation

    rcsDirectionData[0 : len(rcsLocation)] = rcsDirection

    fswSetupThrusters.clearSetup()
    for i in range(numThrusters):
        fswSetupThrusters.create(rcsLocationData[i], rcsDirectionData[i], maxThrust)
    thrConfigInMsg = fswSetupThrusters.writeConfigMessage()

    vehConfigInMsgData = messaging.VehicleConfigMsgPayload()
    vehConfigInMsgData.CoM_B = CoM_B
    vehConfigInMsg = messaging.VehicleConfigMsg().write(vehConfigInMsgData)

    # subscribe input messages to module
    if torqueInMsgFlag:
        module.cmdTorqueInMsg.subscribeTo(cmdTorqueInMsg)
    module.cmdForceInMsg.subscribeTo(cmdForceInMsg)
    module.thrConfigInMsg.subscribeTo(thrConfigInMsg)
    module.vehConfigInMsg.subscribeTo(vehConfigInMsg)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(0.5))
    unitTestSim.ExecuteSimulation()

    # Get actual thruster forces
    actual_thruster_forces = np.array(
        module.thrForceCmdOutMsg.read().thrForce[0 : len(rcsLocation)]
    )

    actual_torque, actual_force = calculate_force_torque(
        actual_thruster_forces,
        np.array(rcsLocation),
        np.array(rcsDirection),
        np.array(CoM_B),
    )

    # Print detailed comparison
    print(f"Requested torque: {requested_torque}")
    print(f"Actual torque:    {actual_torque}")
    print(f"Torque error:     {actual_torque - requested_torque}")
    print(f"Requested force:  {requested_force}")
    print(f"Actual force:     {actual_force}")
    print(f"Force error:      {actual_force - requested_force}")
    print(f"Thruster forces:  {actual_thruster_forces}")

    # Validate thruster force constraints (0 <= force <= maxThrust)
    for i, force in enumerate(actual_thruster_forces):
        if force < -1e-10:
            testFailCount += 1
            testMessages.append(f"FAILED: Thruster {i} has negative force: {force}\n")
        elif force > maxThrust + 1e-10:
            testFailCount += 1
            testMessages.append(
                f"FAILED: Thruster {i} exceeds max thrust ({maxThrust} N): {force}\n"
            )

    # Test force/torque accuracy using compareVector
    tolerance = 1e-3
    requested_torque_force = np.array([requested_torque + requested_force])
    actual_torque_force = np.array([np.concatenate([actual_torque, actual_force])])

    testFailCount, testMessages = unitTestSupport.compareVector(
        requested_torque_force,
        actual_torque_force,
        tolerance,
        "Force/Torque accuracy",
        testFailCount,
        testMessages,
    )

    # First set a non-zero force/torque request
    cmdTorqueInMsgData.torqueRequestBody = [1.0, 0.5, 0.7]
    cmdTorqueInMsg = messaging.CmdTorqueBodyMsg().write(cmdTorqueInMsgData)
    module.cmdTorqueInMsg.subscribeTo(cmdTorqueInMsg)

    cmdForceInMsgData.forceRequestBody = [1.0, 0.5, 0.7]
    cmdForceInMsg = messaging.CmdForceBodyMsg().write(cmdForceInMsgData)
    module.cmdForceInMsg.subscribeTo(cmdForceInMsg)

    unitTestSim.InitializeSimulation()
    unitTestSim.ExecuteSimulation()

    # Now call Reset() and verify output is zeroed
    module.Reset(0)  # Pass 0 as currentSimNanos
    expectedForce = [0.0] * messaging.MAX_EFF_CNT
    testFailCount, testMessages = unitTestSupport.compareVector(
        np.array(expectedForce),
        np.array(module.thrForceCmdOutMsg.read().thrForce),
        tolerance,
        "Reset() zeroed thruster force",
        testFailCount,
        testMessages,
    )

    print(f"Accuracy used: {tolerance}")

    if testFailCount == 0:
        print("PASSED: " + module.ModelTag)
    else:
        print(testMessages)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    test_forceTorqueThrForceMapping1()
    test_forceTorqueThrForceMapping2()
    test_forceTorqueThrForceMapping3()
    test_forceTorqueThrForceMapping4()
