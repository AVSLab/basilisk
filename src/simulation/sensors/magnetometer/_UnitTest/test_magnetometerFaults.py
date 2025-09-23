#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

bskPath = path.split("src")[0]

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import (
    unitTestSupport,
)  # general support file with common unit test functions
from Basilisk.simulation import magnetometer
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk


@pytest.mark.parametrize(
    "axis0State, axis1State, axis2State, stuckValue, spikeProb, spikeAmount, errTol",
    [
        # --- Nominal ---
        (
            magnetometer.NOMINAL,
            magnetometer.NOMINAL,
            magnetometer.NOMINAL,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            0.01,
        ),
        # --- StuckValue cases ---
        (
            magnetometer.MAG_FAULT_STUCK_VALUE,
            magnetometer.NOMINAL,
            magnetometer.NOMINAL,
            [5.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            0.01,
        ),  # X stuck
        (
            magnetometer.NOMINAL,
            magnetometer.MAG_FAULT_STUCK_VALUE,
            magnetometer.NOMINAL,
            [0.0, -10.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            0.01,
        ),  # Y stuck
        (
            magnetometer.NOMINAL,
            magnetometer.NOMINAL,
            magnetometer.MAG_FAULT_STUCK_VALUE,
            [0.0, 0.0, 2.5],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            0.01,
        ),  # Z stuck
        (
            magnetometer.MAG_FAULT_STUCK_VALUE,
            magnetometer.MAG_FAULT_STUCK_VALUE,
            magnetometer.MAG_FAULT_STUCK_VALUE,
            [1.0, -2.0, 3.0],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            0.01,
        ),  # all stuck at different values
        # --- StuckCurrent cases ---
        (
            magnetometer.MAG_FAULT_STUCK_CURRENT,
            magnetometer.NOMINAL,
            magnetometer.NOMINAL,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            0.01,
        ),  # X stuck at current
        (
            magnetometer.NOMINAL,
            magnetometer.MAG_FAULT_STUCK_CURRENT,
            magnetometer.NOMINAL,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            0.01,
        ),  # Y stuck at current
        (
            magnetometer.NOMINAL,
            magnetometer.NOMINAL,
            magnetometer.MAG_FAULT_STUCK_CURRENT,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            0.01,
        ),  # Z stuck at current
        (
            magnetometer.MAG_FAULT_STUCK_CURRENT,
            magnetometer.MAG_FAULT_STUCK_CURRENT,
            magnetometer.MAG_FAULT_STUCK_CURRENT,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            0.01,
        ),  # all stuck at current
        # --- Spiking deterministic (prob=1) ---
        (
            magnetometer.MAG_FAULT_SPIKING,
            magnetometer.NOMINAL,
            magnetometer.NOMINAL,
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [10.0, 1.0, 1.0],
            0.01,
        ),  # X always spikes ×10
        (
            magnetometer.NOMINAL,
            magnetometer.MAG_FAULT_SPIKING,
            magnetometer.NOMINAL,
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, -3.0, 1.0],
            0.01,
        ),  # Y always spikes ×-3
        (
            magnetometer.NOMINAL,
            magnetometer.NOMINAL,
            magnetometer.MAG_FAULT_SPIKING,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 1.0, 0.0],
            0.01,
        ),  # Z always spikes to zero
        # --- Spiking probabilistic ---
        (
            magnetometer.MAG_FAULT_SPIKING,
            magnetometer.NOMINAL,
            magnetometer.NOMINAL,
            [0.0, 0.0, 0.0],
            [0.2, 0.0, 0.0],
            [2.0, 1.0, 1.0],
            0.01,
        ),  # X spikes ~20% of steps ×2
        (
            magnetometer.NOMINAL,
            magnetometer.MAG_FAULT_SPIKING,
            magnetometer.MAG_FAULT_SPIKING,
            [0.0, 0.0, 0.0],
            [0.0, 0.5, 0.5],
            [1.0, 2.0, -1.0],
            0.01,
        ),  # Y spikes ×2, Z spikes inverted ×-1
        # --- Mixed faults ---
        (
            magnetometer.MAG_FAULT_STUCK_VALUE,
            magnetometer.MAG_FAULT_STUCK_CURRENT,
            magnetometer.MAG_FAULT_SPIKING,
            [4.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 1.0, 3.0],
            0.01,
        ),  # X stuck=4, Y stuck-current, Z spikes ×3
        (
            magnetometer.MAG_FAULT_SPIKING,
            magnetometer.MAG_FAULT_STUCK_VALUE,
            magnetometer.NOMINAL,
            [0.0, -7.0, 0.0],
            [1.0, 0.0, 0.0],
            [5.0, 1.0, 1.0],
            0.01,
        ),  # X spikes ×5, Y stuck=-7
    ],
)
def test_magnetometer_faults(
    axis0State, axis1State, axis2State, stuckValue, spikeProb, spikeAmount, errTol
):
    [testResults, testMessages] = run(
        axis0State, axis1State, axis2State, stuckValue, spikeProb, spikeAmount, errTol
    )
    assert testResults < 1, testMessages
    __tracebackhide__ = True


def run(axis0State, axis1State, axis2State, stuckValue, spikeProb, spikeAmount, errTol):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate_s = 0.01
    testProcessRate = macros.sec2nano(
        testProcessRate_s
    )  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    testModule = magnetometer.Magnetometer()
    testModule.ModelTag = "TAM_sensor"

    # Add module to the task
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # Set-up fake magnetic field
    magFieldMsg = messaging.MagneticFieldMsgPayload()
    trueMagField = [
        1e-5,
        2e-5,
        1.5e-5,
    ]  # [T] true magnetic field outputs in inertial frame
    magFieldMsg.magField_N = trueMagField
    magMsg = messaging.MagneticFieldMsg().write(magFieldMsg)
    testModule.magInMsg.subscribeTo(magMsg)

    # Set-up fake attitude
    satelliteStateMsg = messaging.SCStatesMsgPayload()
    satelliteStateMsg.sigma_BN = [0.3, 0.2, 0.0]
    scMsg = messaging.SCStatesMsg().write(satelliteStateMsg)
    testModule.stateInMsg.subscribeTo(scMsg)

    # Sensor set-up
    yaw = 0.7854  # [rad]
    pitch = 1.0  # [rad]
    roll = 0.1  # [rad]
    dcm_SB_py = rbk.euler3212C([yaw, pitch, roll])  # for checking the dcm_SB
    dcm_SB = testModule.setBodyToSensorDCM(yaw, pitch, roll)

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = testModule.tamDataOutMsg.recorder(testProcessRate)
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Configure the simulation
    unitTestSim.InitializeSimulation()

    # Simulate each time step
    angles = np.linspace(0.0, 2 * np.pi, 100)  # example small number of steps
    fault_activated = False
    trueTam_S_list = []
    for i in range(len(angles)):
        # Update the attitude
        sigma_BN = [0.3, 0.2, np.tan(angles[i] / 4.0)]  # current attitude
        satelliteStateMsg.sigma_BN = sigma_BN
        scMsg = messaging.SCStatesMsg().write(satelliteStateMsg)
        testModule.stateInMsg.subscribeTo(scMsg)

        # Activate fault halfway through the simulation
        if i >= len(angles) // 2 and not fault_activated:
            for i, state in enumerate([axis0State, axis1State, axis2State]):
                if state == magnetometer.NOMINAL:
                    testModule.setFaultState(i, magnetometer.NOMINAL)
                elif state == magnetometer.MAG_FAULT_STUCK_VALUE:
                    testModule.setFaultState(i, magnetometer.MAG_FAULT_STUCK_VALUE)
                    testModule.stuckValue = stuckValue
                elif state == magnetometer.MAG_FAULT_STUCK_CURRENT:
                    testModule.setFaultState(i, magnetometer.MAG_FAULT_STUCK_CURRENT)
                elif state == magnetometer.MAG_FAULT_SPIKING:
                    testModule.setFaultState(i, magnetometer.MAG_FAULT_SPIKING)
                    testModule.spikeProbability = spikeProb
                    testModule.spikeAmount = spikeAmount
                else:
                    NotImplementedError("Fault type specified does not exist.")
            fault_activated = True

        # Step simulation
        unitTestSim.TotalSim.SingleStepProcesses()

        # Compute true field in sensor frame
        dcm_BN = rbk.MRP2C(sigma_BN)
        dcm_SN = np.dot(dcm_SB, dcm_BN)
        trueTam_S = np.dot(dcm_SN, trueMagField)
        trueTam_S_list.append(trueTam_S)

    # This pulls the actual data log from the simulation run.
    tamData = dataLog.tam_S
    print("tamData:", tamData)
    print("trueTam_S:", trueTam_S_list)

    # Compare the module results to the true values
    true_half = trueTam_S_list[len(angles) // 2 - 1]
    spike_count = [0, 0, 0]
    for i in range(len(tamData)):
        true = trueTam_S_list[i]
        measured = tamData[i]
        if i < len(angles) // 2:
            if not unitTestSupport.isArrayEqualRelative(measured, true, 3, errTol):
                testFailCount += 1
                testMessages.append(
                    f"TAM data without fault failed comparison with {errTol * 100}% tolerance"
                )
        else:
            for j, state in enumerate([axis0State, axis1State, axis2State]):
                if state == magnetometer.NOMINAL and not unitTestSupport.isDoubleEqual(
                    measured[j], true[j], errTol
                ):
                    testFailCount += 1
                    testMessages.append(
                        f"TAM data without fault failed comparison with {errTol * 100}% tolerance"
                    )
                elif (
                    state == magnetometer.MAG_FAULT_STUCK_VALUE
                    and not unitTestSupport.isDoubleEqual(
                        measured[j], stuckValue[j], errTol
                    )
                ):
                    testFailCount += 1
                    testMessages.append(
                        f"TAM data with stuck value fault failed comparison with {errTol * 100}% tolerance"
                    )
                elif (
                    state == magnetometer.MAG_FAULT_STUCK_CURRENT
                    and not unitTestSupport.isDoubleEqual(
                        measured[j], true_half[j], errTol
                    )
                ):
                    testFailCount += 1
                    testMessages.append(
                        f"TAM data with stuck current fault failed comparison with {errTol * 100}% tolerance"
                    )
                elif state == magnetometer.MAG_FAULT_SPIKING:
                    expected_spike = true[j] * spikeAmount[j]
                    if spikeProb[j] >= 1.0:
                        if not unitTestSupport.isDoubleEqual(
                            measured[j], expected_spike, errTol
                        ):
                            testFailCount += 1
                            testMessages.append(
                                f"TAM data with deterministic spiking fault failed comparison with {errTol * 100}% tolerance"
                            )
                    else:
                        if not (
                            unitTestSupport.isDoubleEqual(
                                measured[j], expected_spike, errTol
                            )
                            or unitTestSupport.isDoubleEqual(
                                measured[j], true[j], errTol
                            )
                        ):
                            testFailCount += 1
                            testMessages.append(
                                f"TAM data with probabilistic spiking fault failed comparison with {errTol * 100}% tolerance"
                            )

                        # Count spikes for later fraction check
                        if abs(measured[j] / true[j] - 1) > errTol:
                            spike_count[j] += 1
                else:
                    NotImplementedError("Fault type specified does not exist.")

    tolerance_factor = 1.5
    for j, state in enumerate([axis0State, axis1State, axis2State]):
        if state == magnetometer.MAG_FAULT_SPIKING and spikeProb[j] < 1.0:
            total_steps = len(angles) // 2  # steps with fault active
            spike_fraction = spike_count[j] / total_steps
            spike_threshold = spikeProb[j] * tolerance_factor
            if spike_fraction > spike_threshold:
                testFailCount += 1
                testMessages.append(
                    f"TAM data spike fraction {spike_fraction:.2f} exceeded threshold {spike_threshold:.2f}"
                )

    #   print out success or failure message
    if testFailCount == 0:
        print("PASSED: " + testModule.ModelTag)
    else:
        print("Failed: " + testModule.ModelTag)
    print(
        "This test uses a relative accuracy value of " + str(errTol * 100) + " percent"
    )

    return [testFailCount, "".join(testMessages)]


#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_magnetometer_faults(
        magnetometer.NOMINAL,  # axis0State
        magnetometer.NOMINAL,  # axis1State
        magnetometer.NOMINAL,  # axis2State
        [0.0, 0.0, 0.0],  # stuckValue
        [0.0, 0.0, 0.0],  # spikeProb
        [0.0, 0.0, 0.0],  # spikeAmount
        0.01,  # errTol
    )
