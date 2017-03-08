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
import sys, os, inspect
import numpy
import pytest
import math

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

import SimulationBaseClass
import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
import macros
import spacecraftPlus
import sim_model
import ctypes
import gravityEffector
import spice_interface
import simIncludeThruster
import thrusterDynamicEffector
import vehicleConfigData
import fuelTank
import fuelSloshParticle

def isArrayEqualRelative(result, truth, dim, accuracy):
    # the result array is of dimension dim+1, as the first entry is the time stamp
    # the truth array is of dimesion dim, no time stamp
    if dim < 1:
        print "Incorrect array dimension " + dim + " sent to isArrayEqual"
        return 0

    if len(result)==0:
        print "Result array was empty"
        return 0

    if len(truth)==0:
        print "Truth array was empty"
        return 0

    if unitTestSupport.foundNAN(result): return 0

    for i in range(0,dim):
        if truth[i] == 0:
            return 1 if result[i+1] == 0 else 0
        if math.fabs((result[i+1] - truth[i])/truth[i]) > accuracy:
            return 0    # return 0 to indicate the array's are not equal
    return 1            # return 1 to indicate the two array's are equal

unitTestSupport.isArrayEqualRelative = isArrayEqualRelative

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def tankModelTest(show_plots):
    [testResults, testMessage] = test_tankModelConstantVolume(show_plots)
    assert testResults < 1, testMessage

def test_tankModelConstantVolume(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    model = fuelTank.cvar.FuelTankModelConstantVolume
    model.propMassInit = 10;
    model.r_TB_BInit = [[1],[1],[1]]
    model.radiusTankInit = 5
    
    trials = [(0, 0), (10, -1), (5, -1)] #mFuel, mDotFuel
    true_ITankPntT_B =      [
                                [0,0,0,0,0,0,0,0,0],
                                [100,0,0,0,100,0,0,0,100],
                                [50,0,0,0,50,0,0,0,50]
                            ]
    true_IPrimeTankPntT_B = [
                                [0,0,0,0,0,0,0,0,0],
                                [-10,0,0,0,-10,0,0,0,-10],
                                [-10,0,0,0,-10,0,0,0,-10]
                            ]
    true_r_TB_B =           [
                                [1,1,1],
                                [1,1,1],
                                [1,1,1]
                            ]
    true_rPrime_TB_B =      [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    true_rPPrime_TB_B =     [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    
    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(*trial)
        dataITank = model.ITankPntT_B
        dataITank = [0] + [dataITank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataITank, true_ITankPntT_B[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed ITankPntT_B test")

        dataIPrimeTank = model.IPrimeTankPntT_B
        dataIPrimeTank = [0] + [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataIPrimeTank, true_IPrimeTankPntT_B[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed IPrimeTankPntT_B test")

        dataR = model.r_TB_B
        dataR = [0] + [dataR[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataR, true_r_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed r_TB_B test")

        dataRPrime = model.rPrime_TB_B
        dataRPrime = [0] + [dataRPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPrime, true_rPrime_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed rPrime_TB_B test")

        dataRPPrime = model.rPPrime_TB_B
        dataRPPrime = [0] + [dataRPPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPPrime, true_rPPrime_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed rPPrime_TB_B test")

    if testFailCount == 0:
        print "PASSED: " + " Fuel Tank constant volume unit test"

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def test_tankModelConstantDensity(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    model = fuelTank.cvar.FuelTankModelConstantDensity
    model.propMassInit = 10;
    model.r_TB_BInit = [[1],[1],[1]]
    model.radiusTankInit = 5
    
    trials = [(0, 0), (10, -1), (5, -1)] #mFuel, mDotFuel
    true_ITankPntT_B =      [
                                [0,0,0,0,0,0,0,0,0],
                                [100,0,0,0,100,0,0,0,100],
                                [31.498026247371826,0,0,0,31.498026247371826,0,0,0,31.498026247371826]
                            ]
    true_IPrimeTankPntT_B = [
                                [0,0,0,0,0,0,0,0,0],
                                [-16.666666666666668,0,0,0,-16.666666666666668,0,0,0,-16.666666666666668],
                                [-10.499342082457275,0,0,0,-10.499342082457275,0,0,0,-10.499342082457275]
                            ]
    true_r_TB_B =           [
                                [1,1,1],
                                [1,1,1],
                                [1,1,1]
                            ]
    true_rPrime_TB_B =      [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    true_rPPrime_TB_B =     [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    
    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(*trial)
        dataITank = model.ITankPntT_B
        dataITank = [0] + [dataITank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataITank, true_ITankPntT_B[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed ITankPntT_B test")

        dataIPrimeTank = model.IPrimeTankPntT_B
        dataIPrimeTank = [0] + [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataIPrimeTank, true_IPrimeTankPntT_B[idx],9,accuracy):
            print dataIPrimeTank, idx
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed IPrimeTankPntT_B test")

        dataR = model.r_TB_B
        dataR = [0] + [dataR[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataR, true_r_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed r_TB_B test")

        dataRPrime = model.rPrime_TB_B
        dataRPrime = [0] + [dataRPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPrime, true_rPrime_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed rPrime_TB_B test")

        dataRPPrime = model.rPPrime_TB_B
        dataRPPrime = [0] + [dataRPPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPPrime, true_rPPrime_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed rPPrime_TB_B test")

    if testFailCount == 0:
        print "PASSED: " + " Fuel Tank constant volume unit test"

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

@pytest.mark.xfail() # need to find true values
def test_tankModelEmptying(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    model = fuelTank.cvar.FuelTankModelEmptying
    model.propMassInit = 10
    model.r_TB_BInit = [[1],[1],[1]]
    model.radiusTankInit = 5
    model.k3 = [[0],[0],[1]]
    
    trials = [(0, 0), (10, -1), (5, -1)] #mFuel, mDotFuel
    true_ITankPntT_B =      [
                                [0,0,0,0,0,0,0,0,0],
                                [100,0,0,0,100,0,0,0,100],
                                [31.498026247371826,0,0,0,31.498026247371826,0,0,0,31.498026247371826]
                            ]
    true_IPrimeTankPntT_B = [
                                [0,0,0,0,0,0,0,0,0],
                                [-16.666666666666668,0,0,0,-16.666666666666668,0,0,0,-16.666666666666668],
                                [-10.499342082457275,0,0,0,-10.499342082457275,0,0,0,-10.499342082457275]
                            ]
    true_r_TB_B =           [
                                [1,1,1],
                                [1,1,1],
                                [1,1,1]
                            ]
    true_rPrime_TB_B =      [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    true_rPPrime_TB_B =     [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    
    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(*trial)
        dataITank = model.ITankPntT_B
        dataITank = [0] + [dataITank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataITank, true_ITankPntT_B[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank emptying unit test failed ITankPntT_B test")

        dataIPrimeTank = model.IPrimeTankPntT_B
        dataIPrimeTank = [0] + [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataIPrimeTank, true_IPrimeTankPntT_B[idx],9,accuracy):
            print dataIPrimeTank, idx
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank emptying unit test failed IPrimeTankPntT_B test")

        dataR = model.r_TB_B
        dataR = [0] + [dataR[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataR, true_r_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank emptying unit test failed r_TB_B test")

        dataRPrime = model.rPrime_TB_B
        dataRPrime = [0] + [dataRPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPrime, true_rPrime_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank emptying unit test failed rPrime_TB_B test")

        dataRPPrime = model.rPPrime_TB_B
        dataRPPrime = [0] + [dataRPPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPPrime, true_rPPrime_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank emptying unit test failed rPPrime_TB_B test")

    if testFailCount == 0:
        print "PASSED: " + " Fuel Tank constant volume unit test"

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def test_tankModelUniformBurn(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    model = fuelTank.cvar.FuelTankModelUniformBurn
    model.propMassInit = 10
    model.r_TB_BInit = [[1],[1],[1]]
    model.radiusTankInit = 5
    model.lengthTank = 5;
    
    trials = [(0, 0), (10, -1), (5, -1)] #mFuel, mDotFuel
    true_ITankPntT_B =      [
                                [0,0,0,0,0,0,0,0,0],
                                [83.33333333333334,0,0,0,83.33333333333334,0,0,0,125],
                                [41.66666666666667,0,0,0,41.66666666666667,0,0,0,62.5]
                            ]
    true_IPrimeTankPntT_B = [
                                [0,0,0,0,0,0,0,0,0],
                                [-8.3333333333334,0,0,0,-8.3333333333334,0,0,0,-12.5],
                                [-8.3333333333334,0,0,0,-8.3333333333334,0,0,0,-12.5]
                            ]
    true_r_TB_B =           [
                                [1,1,1],
                                [1,1,1],
                                [1,1,1]
                            ]
    true_rPrime_TB_B =      [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    true_rPPrime_TB_B =     [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    
    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(*trial)
        dataITank = model.ITankPntT_B
        dataITank = [0] + [dataITank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataITank, true_ITankPntT_B[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank centrifugal burn unit test failed ITankPntT_B test")

        dataIPrimeTank = model.IPrimeTankPntT_B
        dataIPrimeTank = [0] + [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataIPrimeTank, true_IPrimeTankPntT_B[idx],9,accuracy):
            print dataIPrimeTank, idx
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank centrifugal burn unit test failed IPrimeTankPntT_B test")

        dataR = model.r_TB_B
        dataR = [0] + [dataR[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataR, true_r_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank centrifugal burn unit test failed r_TB_B test")

        dataRPrime = model.rPrime_TB_B
        dataRPrime = [0] + [dataRPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPrime, true_rPrime_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank centrifugal burn unit test failed rPrime_TB_B test")

        dataRPPrime = model.rPPrime_TB_B
        dataRPPrime = [0] + [dataRPPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPPrime, true_rPPrime_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank centrifugal burn unit test failed rPPrime_TB_B test")

    if testFailCount == 0:
        print "PASSED: " + " Fuel Tank constant volume unit test"

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


def test_tankModelCentrifugalBurn(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    model = fuelTank.cvar.FuelTankModelCentrifugalBurn
    model.propMassInit = 10
    model.r_TB_BInit = [[1],[1],[1]]
    model.radiusTankInit = 5
    model.lengthTank = 5
    
    trials = [(0, 0), (10, -1), (5, -1)] #mFuel, mDotFuel
    true_ITankPntT_B =      [
                                [0,0,0,0,0,0,0,0,0],
                                [83.33333333333334,0,0,0,83.33333333333334,0,0,0,125],
                                [57.291666666666671,0,0,0,57.291666666666671,0,0,0,93.75]
                            ]
    true_IPrimeTankPntT_B = [
                                [0,0,0,0,0,0,0,0,0],
                                [-2.0833333333333335,0,0,0,-2.0833333333333335,0,0,0,0.0],
                                [-8.3333333333333339,0,0,0,-8.3333333333333339,0,0,0,-12.500000000000002]
                            ]
    true_r_TB_B =           [
                                [1,1,1],
                                [1,1,1],
                                [1,1,1]
                            ]
    true_rPrime_TB_B =      [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    true_rPPrime_TB_B =     [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    
    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(*trial)
        dataITank = model.ITankPntT_B
        dataITank = [0] + [dataITank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataITank, true_ITankPntT_B[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank uniform burn unit test failed ITankPntT_B test")

        dataIPrimeTank = model.IPrimeTankPntT_B
        dataIPrimeTank = [0] + [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataIPrimeTank, true_IPrimeTankPntT_B[idx],9,accuracy):
            print dataIPrimeTank, idx
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank uniform burn unit test failed IPrimeTankPntT_B test")

        dataR = model.r_TB_B
        dataR = [0] + [dataR[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataR, true_r_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank uniform burn unit test failed r_TB_B test")

        dataRPrime = model.rPrime_TB_B
        dataRPrime = [0] + [dataRPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPrime, true_rPrime_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank uniform burn unit test failed rPrime_TB_B test")

        dataRPPrime = model.rPPrime_TB_B
        dataRPPrime = [0] + [dataRPPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPPrime, true_rPPrime_TB_B[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank uniform burn unit test failed rPPrime_TB_B test")

    if testFailCount == 0:
        print "PASSED: " + " Fuel Tank constant volume unit test"

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    test_tankModelTest(show_plots)
