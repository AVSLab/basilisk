
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
#   Purpose:  Run a test of the VSCMG sim module
#   Author:  John Alcorn
#   Creation Date:  November 14, 2016
#

import numpy as np
import pytest
from Basilisk.architecture import messaging
from Basilisk.simulation import vscmgStateEffector
from Basilisk.utilities import macros


# methods
def listStack(vec,simStopTime,unitProcRate):
    # returns a list duplicated the number of times needed to be consistent with module output
    return [vec] * int(simStopTime/(float(unitProcRate)/float(macros.sec2nano(1))))

def writeNewVSCMGCmds(self,u_s_cmd,u_g_cmd,numVSCMG):
    NewVSCMGCmdsVec = messaging.VSCMGCmdMsgPayloadVector(numVSCMG)
    cmds = messaging.VSCMGCmdMsgPayload()
    for i in range(0,numVSCMG):
        cmds.u_s_cmd = u_s_cmd[i]
        cmds.u_g_cmd = u_g_cmd[i]
        NewVSCMGCmdsVec[i] = cmds  # set the data
    self.newVSCMGCmds = NewVSCMGCmdsVec  # set in module

def defaultVSCMG(VSCMG):
    VSCMG.rGB_B = [[0.],[0.],[0.]]
    VSCMG.gsHat0_B = [[1.],[0.],[0.]]
    VSCMG.gtHat0_B = [[1.],[0.],[0.]]
    VSCMG.ggHat_B = [[1.],[0.],[0.]]
    VSCMG.w2Hat0_B = [[0.],[1.],[0.]]
    VSCMG.w3Hat0_B = [[0.],[0.],[1.]]
    VSCMG.theta = 0.
    VSCMG.u_s_current = 0.
    VSCMG.u_s_max = 0.
    VSCMG.u_s_min = 0.
    VSCMG.u_s_f = 0.
    VSCMG.u_g_current = 0.
    VSCMG.u_g_max = 0.
    VSCMG.u_g_min = 0.
    VSCMG.u_g_f = 0.
    VSCMG.Omega = 0.
    VSCMG.gamma = 0.
    VSCMG.gammaDot = 0.
    VSCMG.Omega_max = 1000.
    VSCMG.gammaDot_max = -1
    VSCMG.IW1 = 0.
    VSCMG.IW2 = 0.
    VSCMG.IW3 = 0.
    VSCMG.U_s = 0.
    VSCMG.U_d = 0.
    VSCMG.massW = 0.
    VSCMG.massG = 0.
    VSCMG.wheelLinearFrictionRatio = 0.
    VSCMG.VSCMGModel = 0
    return

def asEigen(v):
    out = []
    for i in range(0,len(v)):
        out.append([v[i]])
    return out

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useFlag, testCase", [
    (False,'saturation'),
    (False,'minimum'),
    (False,'friction')
])

# provide a unique test method name, starting with test_
def test_unitSimVSCMG(show_plots, useFlag, testCase):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimVSCMG(show_plots, useFlag, testCase)
    assert testResults < 1, testMessage


def unitSimVSCMG(show_plots, useFlag, testCase):
    testFail = False
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    # configure module
    VSCMG = vscmgStateEffector.VSCMGStateEffector()
    VSCMG.ModelTag = "VSCMG"

    numVSCMG = 2

    VSCMGs = []
    for i in range(0,numVSCMG):
        msg = messaging.VSCMGConfigMsgPayload()
        defaultVSCMG(msg)
        VSCMGs.append(msg)

    expOut = dict()  # expected output

    if testCase == 'basic':
        pass

    elif testCase == 'saturation':
        VSCMGs[0].u_s_max = 1.
        VSCMGs[1].u_s_max = 2.
        VSCMGs[0].u_g_max = 1.
        VSCMGs[1].u_g_max = 2.
        u_s_cmd = [-1.2,1.5]
        u_g_cmd = [-1.2,1.5]
        writeNewVSCMGCmds(VSCMG,u_s_cmd,u_g_cmd,len(VSCMGs))

        expOut['u_s_current'] = [-1.,1.5]

    elif testCase == 'minimum':
        VSCMGs[0].u_s_min = .1
        VSCMGs[1].u_s_min = .0
        VSCMGs[0].u_g_min = .1
        VSCMGs[1].u_g_min = .0
        u_s_cmd = [-.09,0.0001]
        u_g_cmd = [-.09,0.0001]
        writeNewVSCMGCmds(VSCMG,u_s_cmd,u_g_cmd,len(VSCMGs))

        expOut['u_s_current'] = [0.,0.0001]

    elif testCase == 'friction':
        u_s_f = [0.1,0.]
        u_g_f = [0.1,0.]
        Omega = [-20.,0.]
        Omega_max = [100.,0.]
        gammaDot = [-20.,0.]
        gammaDot_max = [100.,0.]
        wheelLinearFrictionRatio = [0.1,0.]
        gimbalLinearFrictionRatio = [0.1,0.]
        for i in range(0,numVSCMG):
            VSCMGs[i].u_s_f = u_s_f[i]
            VSCMGs[i].Omega = Omega[i]
            VSCMGs[i].Omega_max = Omega_max[i]
            VSCMGs[i].wheelLinearFrictionRatio = wheelLinearFrictionRatio[i]
            VSCMGs[i].u_g_f = u_g_f[i]
            VSCMGs[i].gammaDot = gammaDot[i]
            VSCMGs[i].gammaDot_max = gammaDot_max[i]
            VSCMGs[i].gimbalLinearFrictionRatio = gimbalLinearFrictionRatio[i]
        u_s_cmd = [-1.,0.]
        u_g_cmd = [-1.,0.]
        writeNewVSCMGCmds(VSCMG,u_s_cmd,u_g_cmd,len(VSCMGs))

        expOut['u_s_current'] = np.asarray(u_s_cmd) + np.asarray(u_s_f)

    else:
        raise Exception('invalid test case')

    for i in range(0,len(VSCMGs)):
        VSCMG.AddVSCMG(VSCMGs[i])

    VSCMG.ConfigureVSCMGRequests(0.)

    if not 'accuracy' in vars():
        accuracy = 1e-10

    for outputName in list(expOut.keys()):
        for i in range(0,numVSCMG):
            if expOut[outputName][i] != getattr(VSCMG.VSCMGData[i], outputName):
                print("expected: " + str(expOut[outputName][i]))
                print("got :" + str(getattr(VSCMG.VSCMGData[i], outputName)))
                testFail = 1
                break
        if testFail:
            break

    if testFail:
        testFailCount += 1
        testMessages.append("FAILED: " + VSCMG.ModelTag + " Module failed " +
                            outputName + " unit test")

    np.set_printoptions(precision=16)

    # print out success message if no errors were found
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
    test_unitSimVSCMG(
        False, # show_plots
        False, # useFlag
        'friction' # testCase
    )
