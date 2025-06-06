
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


#
#   Integrated Unit Test Script
#   Purpose:  Run a test of the star tracker module
#   Author:  John Alcorn
#   Creation Date:  October 12, 2016
#

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import starTracker
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions


# methods
def listStack(vec,simStopTime,unitProcRate):
    # returns a list duplicated the number of times needed to be consistent with module output
    return [vec] * int(simStopTime/(float(unitProcRate)/float(macros.sec2nano(1))))

def setRandomWalk(self, senNoiseStd = 0.0, errorBounds = [[1e6],[1e6],[1e6]]):
    # sets the module random walk variables
    PMatrix = [[senNoiseStd, 0., 0.], [0., senNoiseStd, 0.], [0., 0., senNoiseStd]]
    self.PMatrix = PMatrix
    self.walkBounds = errorBounds

def safe_EP2PRV(quaternion):
    """Safely convert from Euler Parameters to Principal Rotation Vector"""
    eps = 1e-12

    # Check if quaternion represents identity rotation (no rotation)
    if np.allclose(quaternion, [1, 0, 0, 0], atol=eps):
        return np.array([0, 0, 0])

    # Ensure quaternion is normalized
    quaternion = quaternion / np.linalg.norm(quaternion)

    # Calculate rotation angle
    cos_phi_2 = quaternion[0]
    sin_phi_2 = np.sqrt(1 - cos_phi_2 * cos_phi_2)

    # Handle small rotation case
    if sin_phi_2 < eps:
        # For very small rotations, return scaled vector part
        return 2 * quaternion[1:4]

    # Normal case - calculate PRV
    phi = 2 * np.arctan2(sin_phi_2, cos_phi_2)
    e = quaternion[1:4] / sin_phi_2
    return phi * e

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useFlag, testCase", [
    (False,'basic'),
    (False,'noise'),
    (False,'walk bounds')
])

# provide a unique test method name, starting with test_
def test_unitSimStarTracker(show_plots, useFlag, testCase):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimStarTracker(show_plots, useFlag, testCase)
    assert testResults < 1, testMessage


def unitSimStarTracker(show_plots, useFlag, testCase):
    testFail = False
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcName = "TestProcess"  # arbitrary name (don't change)

    # initialize SimulationBaseClass
    unitSim = SimulationBaseClass.SimBaseClass()

    # create the task and specify the integration update time
    unitProcRate = macros.sec2nano(0.1)
    unitProcRate_s = macros.NANO2SEC*unitProcRate
    unitProc = unitSim.CreateNewProcess(unitProcName)
    unitProc.addTask(unitSim.CreateNewTask(unitTaskName, unitProcRate))

    # configure module
    StarTracker = starTracker.StarTracker()
    StarTracker.ModelTag = "StarTracker"
    setRandomWalk(StarTracker)

    # configure module input message
    OutputStateData = messaging.SCStatesMsgPayload()
    OutputStateData.r_BN_N = [0,0,0]
    OutputStateData.v_BN_N = [0,0,0]
    OutputStateData.sigma_BN = [0,0,0]
    OutputStateData.omega_BN_B = [0,0,0]
    OutputStateData.TotalAccumDVBdy = [0,0,0]
    OutputStateData.MRPSwitchCount = 0

    trueVector = dict()
    print(testCase)
    if testCase == 'basic':
        # this test verifies basic input and output
        simStopTime = 0.5
        sigma = np.array([-0.390614710591786, -0.503642740963740, 0.462959869561285])
        OutputStateData.sigma_BN = sigma
        trueVector['qInrtl2Case'] = listStack(rbk.MRP2EP(sigma),simStopTime,unitProcRate)
        trueVector['timeTag'] =  np.arange(0,0+simStopTime*1E9,unitProcRate_s*1E9)

    elif testCase == 'noise':
        simStopTime = 10.0
        noiseStd = 0.005
        walkBound = 0.0075

        # Adjust noise for time step
        dt = unitProcRate_s
        adjustedNoiseStd = noiseStd  # Remove sqrt(dt) scaling since it's handled internally

        # Set up noise parameters
        setRandomWalk(StarTracker, adjustedNoiseStd, [[walkBound]]*3)

        # Set initial state
        sigma = np.array([0.0, 0.0, 0.0])
        OutputStateData.sigma_BN = sigma

        # Set expected values
        expected_std = noiseStd  # Expect direct noise level
        trueVector['qInrtl2Case'] = [expected_std] * 3

    elif testCase == 'walk bounds':
        # this test checks the walk bounds of random walk
        simStopTime = 1000.
        noiseStd = 0.01
        walkBound = 0.1
        setRandomWalk(StarTracker, noiseStd, [[walkBound],[walkBound],[walkBound]])
        sigma = np.array([0,0,0])
        OutputStateData.sigma_BN = sigma
        trueVector['qInrtl2Case'] = [walkBound + noiseStd*3] * 3
        trueVector['timeTag'] =  np.arange(0,0+simStopTime*1E9,unitProcRate_s*1E9)

    else:
        raise Exception('invalid test case')

    # add module to the task
    unitSim.AddModelToTask(unitTaskName, StarTracker)

    # log module output message
    dataLog = StarTracker.sensorOutMsg.recorder()
    unitSim.AddModelToTask(unitTaskName, dataLog)

    # configure spacecraft state message
    scMsg = messaging.SCStatesMsg().write(OutputStateData)
    StarTracker.scStateInMsg.subscribeTo(scMsg)

    unitSim.InitializeSimulation()
    unitSim.ConfigureStopTime(macros.sec2nano(simStopTime))
    unitSim.ExecuteSimulation()

    # pull message log data and assemble into dict
    moduleOutput = dataLog.qInrtl2Case

    # convert quaternion output to prv
    moduleOutput2 = np.zeros([int(simStopTime/unitProcRate_s)+1, 3])
    for i in range(0, int(simStopTime/unitProcRate_s)+1):
        moduleOutput2[i] = safe_EP2PRV(moduleOutput[i])

    if not 'accuracy' in vars():
        accuracy = 1e-6

    if testCase == 'noise':
        for i in range(0,3):
            mean_val = np.abs(np.mean(moduleOutput2[:,i]))
            std_val = np.abs(np.std(moduleOutput2[:,i]))
            expected_std = trueVector['qInrtl2Case'][i]

            # Error thresholds must incorporate room for random walk.
            mean_threshold = 0.02
            std_threshold = 0.25

            if mean_val > mean_threshold or np.abs(std_val - expected_std)/expected_std > std_threshold:
                testFail = True
                testMessages.append(f"Axis {i} failed: mean={mean_val:.6f}, std={std_val:.6f}")
                break

    elif testCase == 'walk bounds':
        for i in range(0,3):
            print(np.max(np.abs(np.asarray(moduleOutput2[i]))))
            if np.max(np.abs(np.asarray(moduleOutput2[i]))) > trueVector['qInrtl2Case'][i]:
                testFail = True
                break

    else:
        for i in range(0,len(trueVector['qInrtl2Case'])):
            if not unitTestSupport.isArrayEqual(moduleOutput[i], trueVector['qInrtl2Case'][i], 3, accuracy):
                testFail = True
                break

    if testFail:
        testFailCount += 1
        testMessages.append("FAILED: " + StarTracker.ModelTag + " Module failed unit test")

    np.set_printoptions(precision=16)

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


# This statement below ensures that the unit test script can be run as a
# stand-along python script
if __name__ == "__main__":
    test_unitSimStarTracker(
        False, # show_plots
        False, # useFlag
        'walk bounds' # testCase
    )
