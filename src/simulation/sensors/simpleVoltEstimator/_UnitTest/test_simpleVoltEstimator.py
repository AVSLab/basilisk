
# ISC License
#
# Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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




import os

import matplotlib.pyplot as plt
import numpy
from Basilisk.architecture import messaging
from Basilisk.simulation import simpleVoltEstimator
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

def test_unitSimpleVoltEstimator(show_plots):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimpleVoltEstimator(show_plots)
    assert testResults < 1, testMessage


def unitSimpleVoltEstimator(show_plots):
    path = os.path.dirname(os.path.abspath(__file__))
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    unitTestProc = unitTestSim.CreateNewProcess(unitProcessName)
    # create the task and specify the integration update time
    unitTestProc.addTask(unitTestSim.CreateNewTask(unitTaskName, int(1E8)))

    # Now initialize the modules that we are using
    sVoltObject = simpleVoltEstimator.SimpleVoltEstimator()
    unitTestSim.AddModelToTask(unitTaskName, sVoltObject)

    scPotential = -2000.

    voltMessageData = messaging.VoltMsgPayload()
    voltMessageData.voltage = scPotential

    # Volt output Message
    voltMsg = messaging.VoltMsg().write(voltMessageData)
    sVoltObject.voltInMsg.subscribeTo(voltMsg)

    sVoltObject.ModelTag = "SimpleVoltageEstimation"
    voltBound = numpy.array([1000.0])
    voltSigma = 50.0

    pMatrix = [voltSigma]
    errorBounds = [1000.]

    sVoltObject.walkBounds = errorBounds
    sVoltObject.PMatrix = pMatrix

    # setup logging
    dataVoltLog = sVoltObject.voltOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataVoltLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(int(60 * 144.0 * 1E9))
    unitTestSim.ExecuteSimulation()

    # pull simulation data
    volt = dataVoltLog.voltage

    countAllow = volt.shape[0] * 0.3 / 100.

    # make sure there are not too many error counts (voltage difference exceeding voltage bound)
    voltDiffCount = 0
    i = 0
    while i < volt.shape[0]:
        voltDiff = volt[i] - scPotential
        if abs(voltDiff) > voltBound:
            voltDiffCount += 1
        i += 1

    errorCounts = [voltDiffCount]

    for count in errorCounts:
        if count > countAllow:
            testFailCount += 1
            testMessages.append("FAILED: Too many error counts - " + str(count))

    # now make sure there are enough occasions where voltage difference comes close to voltage bound
    sigmaThreshold = 0.8
    voltDiffCount = 0
    i = 0
    while i < volt.shape[0]:
        voltDiff = volt[i] - scPotential
        if abs(voltDiff) > voltBound*sigmaThreshold:
            voltDiffCount += 1
        i += 1

    errorCounts = [voltDiffCount]

    for count in errorCounts:
        if count < 1:
            testFailCount += 1
            testMessages.append("FAILED: Too few error counts - " + str(count))

    plt.figure(1)
    plt.clf()
    plt.figure(1, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(dataVoltLog.times() * 1.0E-9, volt[:])

    plt.xlabel('Time (s)')
    plt.ylabel('Voltage (V)')
    unitTestSupport.writeFigureLaTeX('SimpleVolt', 'Simple Voltage Estimator Voltage Signal', plt,
                                     r'height=0.4\textwidth, keepaspectratio', path)
    if show_plots:
        plt.show()
        plt.close('all')

    # check if BSK_ERROR is returned if pMatrix is wrong size
    pMatrixBad = [[0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                  [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]]
    stateBoundsBad = [[0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.]]
    sVoltObject.walkBounds = stateBoundsBad
    sVoltObject.PMatrix = pMatrixBad

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(int(1E8))
    unitTestSim.ExecuteSimulation()

    # print out success message if no error were found
    if testFailCount == 0:
        print("PASSED")

    assert testFailCount < 1, testMessages
    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
if __name__ == "__main__":
    unitSimpleVoltEstimator(True)
