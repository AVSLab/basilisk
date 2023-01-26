
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


import inspect
import os

from Basilisk.simulation import fuelTank
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_


def test_tankModelConstantVolume(show_plots=False):
    """Module Unit Test"""
    [testResults, testMessage] = tankModelConstantVolume(show_plots)
    assert testResults < 1, testMessage


def test_tankModelConstantDensity(show_plots=False):
    """Module Unit Test"""
    [testResults, testMessage] = tankModelConstantDensity(show_plots)
    assert testResults < 1, testMessage


def test_tankModelEmptying(show_plots=False):
    """Module Unit Test"""
    [testResults, testMessage] = tankModelEmptying(show_plots)
    assert testResults < 1, testMessage


def test_tankModelUniformBurn(show_plots=False):
    """Module Unit Test"""
    [testResults, testMessage] = tankModelUniformBurn(show_plots)
    assert testResults < 1, testMessage


def test_tankModelCentrifugalBurn(show_plots=False):
    """Module Unit Test"""
    [testResults, testMessage] = tankModelCentrifugalBurn(show_plots)
    assert testResults < 1, testMessage


def tankModelConstantVolume(show_plots):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    model = fuelTank.cvar.FuelTankModelConstantVolume
    model.propMassInit = 10
    model.r_TcT_TInit = [[1],[1],[1]]
    model.radiusTankInit = 5
    
    trials = [(0, 0), (10, -1), (5, -1)] #mFuel, mDotFuel
    true_ITankPntT_T =      [
                                [0,0,0,0,0,0,0,0,0],
                                [100,0,0,0,100,0,0,0,100],
                                [50,0,0,0,50,0,0,0,50]
                            ]
    true_IPrimeTankPntT_T = [
                                [0,0,0,0,0,0,0,0,0],
                                [-10,0,0,0,-10,0,0,0,-10],
                                [-10,0,0,0,-10,0,0,0,-10]
                            ]
    true_r_TcT_T =           [
                                [1,1,1],
                                [1,1,1],
                                [1,1,1]
                            ]
    true_rPrime_TcT_T =      [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    true_rPPrime_TcT_T =     [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    
    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(trial[0])
        model.computeTankPropDerivs(*trial)
        dataITank = model.ITankPntT_T
        dataITank = [dataITank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataITank, true_ITankPntT_T[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed ITankPntT_T test")

        dataIPrimeTank = model.IPrimeTankPntT_T
        dataIPrimeTank = [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataIPrimeTank, true_IPrimeTankPntT_T[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed IPrimeTankPntT_T test")

        dataR = model.r_TcT_T
        dataR = [dataR[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataR, true_r_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed r_TcT_T test")

        dataRPrime = model.rPrime_TcT_T
        dataRPrime = [dataRPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPrime, true_rPrime_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed rPrime_TcT_T test")

        dataRPPrime = model.rPPrime_TcT_T
        dataRPPrime = [dataRPPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPPrime, true_rPPrime_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed rPPrime_TcT_T test")

    if testFailCount == 0:
        print("PASSED: " + " Fuel Tank constant volume unit test")

    snippetName = 'ConstVolPassFail'
    passFail(testFailCount, snippetName)

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def tankModelConstantDensity(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    model = fuelTank.cvar.FuelTankModelConstantDensity
    model.propMassInit = 10;
    model.r_TcT_TInit = [[1],[1],[1]]
    model.radiusTankInit = 5
    
    trials = [(0, 0), (10, -1), (5, -1)] #mFuel, mDotFuel
    true_ITankPntT_T =      [
                                [0,0,0,0,0,0,0,0,0],
                                [100,0,0,0,100,0,0,0,100],
                                [31.498026247371826,0,0,0,31.498026247371826,0,0,0,31.498026247371826]
                            ]
    true_IPrimeTankPntT_T = [
                                [0,0,0,0,0,0,0,0,0],
                                [-16.666666666666668,0,0,0,-16.666666666666668,0,0,0,-16.666666666666668],
                                [-10.499342082457275,0,0,0,-10.499342082457275,0,0,0,-10.499342082457275]
                            ]
    true_r_TcT_T =           [
                                [1,1,1],
                                [1,1,1],
                                [1,1,1]
                            ]
    true_rPrime_TcT_T =      [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    true_rPPrime_TcT_T =     [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    
    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(trial[0])
        model.computeTankPropDerivs(*trial)
        dataITank = model.ITankPntT_T
        dataITank = [dataITank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataITank, true_ITankPntT_T[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed ITankPntT_T test")

        dataIPrimeTank = model.IPrimeTankPntT_T
        dataIPrimeTank = [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataIPrimeTank, true_IPrimeTankPntT_T[idx],9,accuracy):
            print(dataIPrimeTank, idx)
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed IPrimeTankPntT_T test")

        dataR = model.r_TcT_T
        dataR = [dataR[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataR, true_r_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed r_TcT_T test")

        dataRPrime = model.rPrime_TcT_T
        dataRPrime = [dataRPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPrime, true_rPrime_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed rPrime_TcT_T test")

        dataRPPrime = model.rPPrime_TcT_T
        dataRPPrime = [dataRPPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPPrime, true_rPPrime_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank constant volume unit test failed rPPrime_TcT_T test")

    if testFailCount == 0:
        print("PASSED: " + " Fuel Tank constant volume unit test")

    snippetName = 'ConstDensPassFail'
    passFail(testFailCount, snippetName)

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def tankModelEmptying(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    model = fuelTank.cvar.FuelTankModelEmptying
    model.propMassInit = 10
    model.r_TcT_TInit = [[1],[1],[1]]
    model.radiusTankInit = 5
    
    trials = [(0, 0), (10, -1), (5, -1)] #mFuel, mDotFuel
    true_ITankPntT_T =      [
                                [0,0,0,0,0,0,0,0,0],
                                [100,0,0,0,100,0,0,0,100],
                                [50.0,0,0,0,50.0,0,0,0,50]
                            ]
    true_IPrimeTankPntT_T = [
                                [0,0,0,0,0,0,0,0,0],
                                [0,0,0,0,0,0,0,0,0],
                                [-8.75,0,0,0,-8.75,0,0,0,-12.5]
                            ]
    true_r_TcT_T =           [
                                [1,1,1-5.0],
                                [1,1,1],
                                [1,1,1.0-15.0/8.0]
                            ]
    true_rPrime_TcT_T =      [
                                [0,0,0],
                                [0,0,0],
                                [0,0,-3.0/8.0]
                            ]
    true_rPPrime_TcT_T =     [
                                [0,0,0],
                                [0,0,0],
                                [0,0,-17.0/30.0]
                            ]
    
    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(trial[0])
        model.computeTankPropDerivs(*trial)
        dataITank = model.ITankPntT_T
        dataITank = [dataITank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqual(dataITank, true_ITankPntT_T[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank emptying unit test failed ITankPntT_T test")

        dataIPrimeTank = model.IPrimeTankPntT_T
        dataIPrimeTank = [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataIPrimeTank, true_IPrimeTankPntT_T[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank emptying unit test failed IPrimeTankPntT_T test")

        dataR = model.r_TcT_T
        dataR = [dataR[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataR, true_r_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank emptying unit test failed r_TcT_T test")

        dataRPrime = model.rPrime_TcT_T
        dataRPrime = [dataRPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPrime, true_rPrime_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank emptying unit test failed rPrime_TcT_T test")

        dataRPPrime = model.rPPrime_TcT_T
        dataRPPrime = [dataRPPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPPrime, true_rPPrime_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank emptying unit test failed rPPrime_TcT_T test")

    if testFailCount == 0:
        print("PASSED: " + " Fuel Tank constant volume unit test")

    snippetName = 'EmptyingPassFail'
    passFail(testFailCount, snippetName)

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def tankModelUniformBurn(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    model = fuelTank.cvar.FuelTankModelUniformBurn
    model.propMassInit = 10
    model.r_TcT_TInit = [[1],[1],[1]]
    model.radiusTankInit = 5
    model.lengthTank = 5;
    
    trials = [(0, 0), (10, -1), (5, -1)] #mFuel, mDotFuel
    true_ITankPntT_T =      [
                                [0,0,0,0,0,0,0,0,0],
                                [83.33333333333334,0,0,0,83.33333333333334,0,0,0,125],
                                [41.66666666666667,0,0,0,41.66666666666667,0,0,0,62.5]
                            ]
    true_IPrimeTankPntT_T = [
                                [0,0,0,0,0,0,0,0,0],
                                [-8.3333333333334,0,0,0,-8.3333333333334,0,0,0,-12.5],
                                [-8.3333333333334,0,0,0,-8.3333333333334,0,0,0,-12.5]
                            ]
    true_r_TcT_T =           [
                                [1,1,1],
                                [1,1,1],
                                [1,1,1]
                            ]
    true_rPrime_TcT_T =      [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    true_rPPrime_TcT_T =     [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    
    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(trial[0])
        model.computeTankPropDerivs(*trial)
        dataITank = model.ITankPntT_T
        dataITank = [dataITank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataITank, true_ITankPntT_T[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank centrifugal burn unit test failed ITankPntT_T test")

        dataIPrimeTank = model.IPrimeTankPntT_T
        dataIPrimeTank = [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataIPrimeTank, true_IPrimeTankPntT_T[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank centrifugal burn unit test failed IPrimeTankPntT_T test")

        dataR = model.r_TcT_T
        dataR = [dataR[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataR, true_r_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank centrifugal burn unit test failed r_TcT_T test")

        dataRPrime = model.rPrime_TcT_T
        dataRPrime = [dataRPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPrime, true_rPrime_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank centrifugal burn unit test failed rPrime_TcT_T test")

        dataRPPrime = model.rPPrime_TcT_T
        dataRPPrime = [dataRPPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPPrime, true_rPPrime_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank centrifugal burn unit test failed rPPrime_TcT_T test")

    if testFailCount == 0:
        print("PASSED: " + " Fuel Tank constant volume unit test")

    snippetName = 'UniformBurnPassFail'
    passFail(testFailCount, snippetName)

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]


def tankModelCentrifugalBurn(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    model = fuelTank.cvar.FuelTankModelCentrifugalBurn
    model.propMassInit = 10
    model.r_TcT_TInit = [[1],[1],[1]]
    model.radiusTankInit = 5
    model.lengthTank = 5
    
    trials = [(0, 0), (10, -1), (5, -1)] #mFuel, mDotFuel
    true_ITankPntT_T =      [
                                [0,0,0,0,0,0,0,0,0],
                                [83.33333333333334,0,0,0,83.33333333333334,0,0,0,125],
                                [57.291666666666671,0,0,0,57.291666666666671,0,0,0,93.75]
                            ]
    true_IPrimeTankPntT_T = [
                                [0,0,0,0,0,0,0,0,0],
                                [-2.0833333333333335,0,0,0,-2.0833333333333335,0,0,0,0.0],
                                [-8.3333333333333339,0,0,0,-8.3333333333333339,0,0,0,-12.500000000000002]
                            ]
    true_r_TcT_T =           [
                                [1,1,1],
                                [1,1,1],
                                [1,1,1]
                            ]
    true_rPrime_TcT_T =      [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    true_rPPrime_TcT_T =     [
                                [0,0,0],
                                [0,0,0],
                                [0,0,0]
                            ]
    
    accuracy = 1e-8
    for idx, trial in enumerate(trials):
        model.computeTankProps(trial[0])
        model.computeTankPropDerivs(*trial)
        dataITank = model.ITankPntT_T
        dataITank = [dataITank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataITank, true_ITankPntT_T[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank uniform burn unit test failed ITankPntT_T test")

        dataIPrimeTank = model.IPrimeTankPntT_T
        dataIPrimeTank = [dataIPrimeTank[i][j] for i in range(3) for j in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataIPrimeTank, true_IPrimeTankPntT_T[idx],9,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank uniform burn unit test failed IPrimeTankPntT_T test")

        dataR = model.r_TcT_T
        dataR = [dataR[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataR, true_r_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank uniform burn unit test failed r_TcT_T test")

        dataRPrime = model.rPrime_TcT_T
        dataRPrime = [dataRPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPrime, true_rPrime_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank uniform burn unit test failed rPrime_TcT_T test")

        dataRPPrime = model.rPPrime_TcT_T
        dataRPPrime = [dataRPPrime[i][0] for i in range(3)]
        if not unitTestSupport.isArrayEqualRelative(dataRPPrime, true_rPPrime_TcT_T[idx],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Tank uniform burn unit test failed rPPrime_TcT_T test")

    if testFailCount == 0:
        print("PASSED: " + " Fuel Tank constant volume unit test")

    snippetName = 'CentrifugalPassFail'
    passFail(testFailCount, snippetName)

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def passFail(testFailCountInput, snippetName):
    if testFailCountInput < 1:
        textMsg = 'PASSED'
        textColor = 'ForestGreen'
    else:
        textMsg = 'FAILED'
        textColor = 'Red'

    texSnippet =  r'\textcolor{' + textColor + '}{'+ textMsg + '}'
    unitTestSupport.writeTeXSnippet(snippetName, texSnippet, path)


if __name__ == "__main__":
    # tankModelConstantVolume(True)
    tankModelConstantDensity(True)
    # tankModelEmptying(False)
