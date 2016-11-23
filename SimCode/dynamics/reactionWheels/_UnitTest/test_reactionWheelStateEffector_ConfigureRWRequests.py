''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''
#
#   Integrated Unit Test Script
#   Purpose:  Run a test of the reaction wheel sim module
#   Author:  John Alcorn
#   Creation Date:  November 14, 2016
#

import pytest
import sys, os, inspect
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import ctypes
import math
import csv
import logging

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('basilisk')
sys.path.append(splitPath[0]+'/basilisk/modules')
sys.path.append(splitPath[0]+'/basilisk/PythonModules')

import MessagingAccess
import SimulationBaseClass
import unitTestSupport  # general support file with common unit test functions
import macros
import reactionWheelStateEffector
import sim_model
import RigidBodyKinematics as rbk
import spacecraftPlus

# methods
def listStack(vec,simStopTime,unitProcRate):
    # returns a list duplicated the number of times needed to be consistent with module output
    return [vec] * int(simStopTime/(float(unitProcRate)/float(macros.sec2nano(1))))

def writeNewRWCmds(self,u_cmd,numRW):
    NewRWCmdsVec = reactionWheelStateEffector.RWCmdVector(numRW) # create standard vector from SWIG template (see .i file)
    cmds = reactionWheelStateEffector.RWCmdStruct()
    for i in range(0,numRW):
        cmds.u_cmd = u_cmd[i]
        NewRWCmdsVec[i] = cmds # set the data
        self.NewRWCmds = NewRWCmdsVec # set in module

def defaultReactionWheel():
    RW = reactionWheelStateEffector.ReactionWheelConfigData()
    RW.typeName = ''
    RW.rWB_S = [[0.],[0.],[0.]]
    RW.gsHat_S = [[1.],[0.],[0.]]
    RW.gtHat0_S = [[0.],[1.],[0.]]
    RW.ggHat0_S = [[0.],[0.],[1.]]
    RW.rWB_B = [[0.],[0.],[0.]]
    RW.gsHat_B = [[1.],[0.],[0.]]
    RW.gtHat0_B = [[0.],[1.],[0.]]
    RW.ggHat0_B = [[0.],[0.],[1.]]
    RW.theta = 0.
    RW.u_current = 0.
    RW.u_max = 0.
    RW.u_min = 0.
    RW.u_f = 0.
    RW.Omega = 0.
    RW.Omega_max = 0.
    RW.Js = 0.
    RW.Jt = 0.
    RW.Jg = 0.
    RW.U_s = 0.
    RW.U_d = 0.
    RW.mass = 0.
    RW.linearFrictionRatio = 0.
    RW.RWModel = 0
    return RW

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
def test_unitSimReactionWheel(show_plots, useFlag, testCase):
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
    for i in range(0,numRW):
        RWs.append(defaultReactionWheel())


    expOut = dict() # expected output

    print testCase
    if testCase is 'basic':
        pass

    elif testCase is 'saturation':
        RWs[0].u_max = 1.
        RWs[1].u_max = 2.
        u_cmd = [-1.2,1.5]
        writeNewRWCmds(ReactionWheel,u_cmd,len(RWs))

        expOut['u_current'] = [-1.,1.5]

    elif testCase is 'minimum':
        RWs[0].u_min = .1
        RWs[1].u_min = .0
        u_cmd = [-.09,0.0001]
        writeNewRWCmds(ReactionWheel,u_cmd,len(RWs))

        expOut['u_current'] = [0.,0.0001]

    elif testCase is 'friction':
        u_f = [0.1,0.]
        Omega = [-20.,0.]
        Omega_max = [100.,0.]
        linearFrictionRatio = [0.1,0.]
        for i in range(0,numRW):
            RWs[i].u_f = u_f[i]
            RWs[i].Omega = Omega[i]
            RWs[i].Omega_max = Omega_max[i]
            RWs[i].linearFrictionRatio = linearFrictionRatio[i]
        u_cmd = [-1.,0.]
        writeNewRWCmds(ReactionWheel,u_cmd,len(RWs))

        expOut['u_current'] = np.asarray(u_cmd) + np.asarray(u_f)

    else:
        raise Exception('invalid test case')

    for i in range(0,len(RWs)):
        ReactionWheel.AddReactionWheel(RWs[i])

    ReactionWheel.ConfigureRWRequests(0.)



    if not 'accuracy' in vars():
        accuracy = 1e-10

    for outputName in expOut.keys():
        for i in range(0,numRW):
            if expOut[outputName][i] != getattr(ReactionWheel.ReactionWheelData[i],outputName):
                print "expected: " + str(expOut[outputName][i])
                print "got :" + str(getattr(ReactionWheel.ReactionWheelData[i],outputName))
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
        print   "PASSED "

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


# This statement below ensures that the unit test script can be run as a
# stand-along python script
if __name__ == "__main__":
    test_unitSimReactionWheel(
        False, # show_plots
        False, # useFlag
        'friction' # testCase
    )
