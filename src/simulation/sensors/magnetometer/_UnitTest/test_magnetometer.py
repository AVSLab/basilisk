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
#   Module Name:        Magnetometer - TAM
#   Author:             Demet Cilden-Guler
#   Creation Date:      September 25, 2019
#

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

bskPath = path.split('src')[0]

# Import all of the modules that we are going to be called in this simulation
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
from Basilisk.simulation import magnetometer
from Basilisk.architecture import messaging
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk


@pytest.mark.parametrize("useNoiseStd, useBias, useMinOut, useMaxOut, useScaleFactor, errTol", [
    (True, True, True, True, True, 0.05),      # With noise - use 5% tolerance
    (True, False, False, True, True, 0.05),    # With noise - use 5% tolerance
    (False, True, True, True, True, 0.01),     # No noise - can use tighter 1% tolerance
    (False, False, False, True, True, 0.01),   # No noise - can use tighter 1% tolerance
])
def test_module(show_plots, useNoiseStd, useBias, useMinOut, useMaxOut, useScaleFactor, errTol):
    [testResults, testMessages] = run(show_plots, useNoiseStd, useBias, useMinOut, useMaxOut, useScaleFactor, errTol)
    assert testResults < 1, testMessages
    __tracebackhide__ = True

def run(show_plots, useNoiseStd, useBias, useMinOut, useMaxOut, useScaleFactor, errTol):
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    unitTaskName = "unitTask"               # arbitrary name (don't change)
    unitProcessName = "TestProcess"         # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate_s = 0.01
    testProcessRate = macros.sec2nano(testProcessRate_s)     # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Construct algorithm and associated C++ container
    testModule = magnetometer.Magnetometer()
    testModule.ModelTag = "TAM_sensor"
    NoiseStd = [3e-9, 3e-9, 3e-9]  # Tesla
    bias = [1e-6, 1e-6, 1e-5]  # Tesla
    minOut = -1e-4  # Tesla
    maxOut = 1e-4  # Tesla

    if useNoiseStd:
        testModule.senNoiseStd = NoiseStd
    if useBias:
        testModule.senBias = bias
    if useScaleFactor:
        testModule.scaleFactor = 2
    if useMinOut & useMaxOut:
        testModule.minOutput = minOut
        testModule.maxOutput = maxOut

    # Add module to the task
    unitTestSim.AddModelToTask(unitTaskName, testModule)

    # Set-up fake magnetic field
    magFieldMsg = messaging.MagneticFieldMsgPayload()
    trueMagField = [1e-5, 2e-5, 1.5e-5]  # [T] true magnetic field outputs in inertial frame
    magFieldMsg.magField_N = trueMagField
    magMsg = messaging.MagneticFieldMsg().write(magFieldMsg)
    testModule.magInMsg.subscribeTo(magMsg)

    # Set-up fake attitude
    satelliteStateMsg = messaging.SCStatesMsgPayload()
    angles = np.linspace(0., 2 * np.pi, 59000)
    sigmas = np.zeros(len(angles))
    for i in range(len(sigmas)):  # convert rotation angle about 3rd axis to MRP
        sigmas[i] = np.tan(angles[i] / 4.)  # This is iterated through in the execution for loop
        satelliteStateMsg.sigma_BN = [0.3, 0.2, sigmas[i]]
    scMsg = messaging.SCStatesMsg().write(satelliteStateMsg)
    testModule.stateInMsg.subscribeTo(scMsg)
    dcm_BN = rbk.MRP2C(satelliteStateMsg.sigma_BN)

    # Sensor set-up
    yaw = 0.7854  # [rad]
    pitch = 1.0  # [rad]
    roll = 0.1  # [rad]
    dcm_SB_py = rbk.euler3212C([yaw, pitch, roll])  # for checking the dcm_SB
    dcm_SB = testModule.setBodyToSensorDCM(yaw, pitch, roll)
    dcm_SN = np.dot(dcm_SB, dcm_BN)
    trueTam_S =  np.dot(dcm_SN, trueMagField)

    if useBias:
        trueTam_S += bias  # Tesla
    if useScaleFactor:
        trueTam_S *= 2

    for i in range(len(trueTam_S)):
        if useMinOut & useMaxOut:
            if trueTam_S[i] < minOut:
                trueTam_S[i] = minOut
            if trueTam_S[i] > maxOut:
                trueTam_S[i] = maxOut

    # Setup logging on the test module output message so that we get all the writes to it
    dataLog = testModule.tamDataOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Need to call the self-init and cross-init methods
    unitTestSim.InitializeSimulation()

    unitTestSim.TotalSim.SingleStepProcesses()

    # This pulls the actual data log from the simulation run.
    tamData = dataLog.tam_S
    print(tamData)
    print(trueTam_S)

    if useNoiseStd:
        if not unitTestSupport.isArrayEqualRelative(tamData[0], trueTam_S, 3, errTol):
            testFailCount += 1
            testMessages.append(f"TAM data with noise failed comparison with {errTol*100}% tolerance")
    else:
        # For non-noisy data we can use stricter comparison
        if not unitTestSupport.isArrayEqual(tamData[0], trueTam_S, 3, errTol):
            testFailCount += 1
            testMessages.append(f"TAM data failed comparison with {errTol*100}% tolerance")

    #   print out success or failure message
    if testFailCount == 0:
        print("PASSED: " + testModule.ModelTag)
    else:
        print("Failed: " + testModule.ModelTag)
    print("This test uses a relative accuracy value of " + str(errTol*100) + " percent")

    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unitTestScript can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_module(              # update "module" in function name
                 False,  # show_plots
                 True,   # useNoiseStd
                 True,   # useBias
                 True,   # useMinOut
                 True,   # useMaxOut
                 True,   # useScaleFactor
                 1e-2    # errTol
               )
