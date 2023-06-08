
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
#   Purpose:  Run a test of the reaction wheel sim module
#   Author:  John Alcorn
#   Creation Date:  November 14, 2016
#

import inspect
import os

import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.architecture import messaging


# methods
def listStack(vec,simStopTime,unitProcRate):
    # returns a list duplicated the number of times needed to be consistent with module output
    return [vec] * int(simStopTime/(float(unitProcRate)/float(macros.sec2nano(1))))


def writeNewRWCmds(self, u_cmd, numRW):
    # create standard vector from SWIG template (see .i file)
    NewRWCmdsVec = messaging.RWCmdMsgPayloadVector(numRW)

    cmds = messaging.RWCmdMsgPayload()
    for i in range(0, numRW):
        cmds.u_cmd = u_cmd[i]
        NewRWCmdsVec[i] = cmds  # set the data
        self.NewRWCmds = NewRWCmdsVec  # set in module


def defaultReactionWheel():
    RW = messaging.RWConfigMsgPayload()
    RW.rWB_B = [[0.], [0.], [0.]]
    RW.gsHat_B = [[1.], [0.], [0.]]
    RW.w2Hat0_B = [[0.], [1.], [0.]]
    RW.w3Hat0_B = [[0.], [0.], [1.]]
    RW.RWModel = reactionWheelStateEffector.BalancedWheels
    return RW


def asEigen(v):
    out = []
    for i in range(0, len(v)):
        out.append([v[i]])
    return out

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useFlag, testCase", [
    (False, 'saturation'),
    (False, 'minimum'),
    (False, 'speedSaturation'),
    (False, 'powerSaturation')
])


# provide a unique test method name, starting with test_
def test_unitSimReactionWheel(show_plots, useFlag, testCase):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimReactionWheel(show_plots, useFlag, testCase)
    assert testResults < 1, testMessage


def unitSimReactionWheel(show_plots, useFlag, testCase):
    testFail = False
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    # configure module
    ReactionWheel = reactionWheelStateEffector.ReactionWheelStateEffector()
    ReactionWheel.ModelTag = "ReactionWheel"

    numRW = 2

    RWs = []
    for i in range(0, numRW):
        RWs.append(defaultReactionWheel())

    expOut = dict()  # expected output

    print(testCase)
    if testCase == 'basic':
        pass

    elif testCase == 'saturation':
        RWs.append(defaultReactionWheel())
        RWs[0].u_max = 1.
        RWs[1].u_max = 2.
        RWs[2].u_max = 2.
        u_cmd = [-1.2, 1.5, 2.5]
        writeNewRWCmds(ReactionWheel, u_cmd, len(RWs))

        expOut['u_current'] = [-1., 1.5, 2.]

    elif testCase == 'minimum':
        RWs[0].u_min = .1
        RWs[1].u_min = .0
        u_cmd = [-.09, 0.0001]
        writeNewRWCmds(ReactionWheel, u_cmd, len(RWs))

        expOut['u_current'] = [0., 0.0001]

    elif testCase == 'speedSaturation':
        RWs.append(defaultReactionWheel())
        RWs[0].Omega_max = 50.
        RWs[1].Omega_max = 50.
        RWs[2].Omega_max = 50.
        RWs[0].Omega = 49.
        RWs[1].Omega = 51.
        RWs[2].Omega = -52.
        u_cmd = [1.5, 1.5, 1.5]
        writeNewRWCmds(ReactionWheel, u_cmd, len(RWs))

        expOut['u_current'] = [1.5, 0.0, 1.5]

    elif testCase == 'powerSaturation':
        RWs.append(defaultReactionWheel())
        RWs[0].P_max = 1.
        RWs[1].P_max = 1.
        RWs[2].P_max = 1.
        RWs[0].Omega = 50.
        RWs[1].Omega = 50.
        RWs[2].Omega = 50.
        u_cmd = [0.01, -0.04, 0.04]
        writeNewRWCmds(ReactionWheel, u_cmd, len(RWs))

        expOut['u_current'] = [0.01, -0.02, 0.02]

    else:
        raise Exception('invalid test case')

    for i in range(0, len(RWs)):
        ReactionWheel.addReactionWheel(RWs[i])

    ReactionWheel.ConfigureRWRequests(0.)

    if 'accuracy' not in vars():
        accuracy = 1e-10

    for outputName in list(expOut.keys()):
        for i in range(0, len(RWs)):
            if expOut[outputName][i] != ReactionWheel.ReactionWheelData[i].u_current:
                print("expected: " + str(expOut[outputName][i]))
                print("got :" + str(ReactionWheel.ReactionWheelData[i].u_current))
                testFail = 1
                break
        if testFail:
            break

    if testFail:
        testFailCount += 1
        testMessages.append("FAILED: " + ReactionWheel.ModelTag + " Module failed " +
                            outputName + " unit test")

    np.set_printoptions(precision=16)

    # print out success message if no errors were found
    if testFailCount == 0:
        print("PASSED ")
        colorText = 'ForestGreen'
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        passedText = r'\textcolor{' + colorText + '}{' + "FAILED" + '}'

    # Write some snippets for AutoTex
    snippetName = testCase + 'PassFail'
    unitTestSupport.writeTeXSnippet(snippetName, passedText, path)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


# This statement below ensures that the unit test script can be run as a
# standalone python script
if __name__ == "__main__":
    test_unitSimReactionWheel(
        False,  # show_plots
        False,  # useFlag
        'speedSaturation'  # testCase
    )
