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

import math

import matplotlib.pyplot as plt
import numpy
from Basilisk.architecture import messaging
from Basilisk.simulation import planetNav
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


def test_planetNav(show_plots):
    r"""
    **Validation Test Description**

    This unit test is designed to functionally test the simulation model outputs as well as get complete code
    path coverage. The test design is broken up into three main parts:

    1. Error Bound Enforcement: The simulation is run for 2.4 hours and the error bounds for all of the signals are
       tested. This test length is long enough to see both the walk in the signal and the noise, all the while not
       being so long as to slow down the test. The test ensures that the bounds are crossed no more than 30% of the
       time.

    2. Error Bound Usage:  The error signals are checked for all of the model parameters over the course of the
       simulation to ensure that the error gets to at least 80% of its maximum error bound at least once, ensuring that
       noise is indeed properly introduced.

    3. Corner Case Check: The simulation is intentionally given bad inputs to ensure that it alerts the user and
       does not crash.

    **Test Parameters**

    These tests are considered to pass if during the whole simulation time of 144 minutes, all the variables need to
    stay within an allowable statistical error. This means that they must stay within their bounds 30% of the time.

    At the same time, we want each of the errors to get to 80% of their respective error bounds at least once during
    the run.

    The test used for the planetNav module tests the statistics of the Gauss Markov process, making sure that we
    are getting the variability we want. In order to do so, no specific scenario is necessary. Therefore the position of
    the planet is set generically.  The planet position is [10000, 0, 0]^T
    """
    [testResults, testMessage] = planetNavTestFunction(show_plots)
    assert testResults < 1, testMessage


def planetNavTestFunction(show_plots):
    """Test method"""
    testFailCount = 0
    testMessages = []
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"

    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProcessRate = macros.sec2nano(0.1)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # setup module to be tested
    module = planetNav.PlanetNav()
    module.ModelTag = "planetNavTag"
    unitTestSim.AddModelToTask(unitTaskName, module)

    # Configure blank module input messages
    ephemerisInMsgData = messaging.EphemerisMsgPayload()
    ephemerisInMsgData.r_BdyZero_N = [10000.0, 0.0, 0.0]
    ephemerisInMsgData.v_BdyZero_N = [0., 0.0, 0.0]
    ephemerisInMsgData.sigma_BN = [0.0, 0.0, 0.0]
    ephemerisInMsgData.omega_BN_B = [0.0, 0.0, 0.0]
    ephemerisInMsg = messaging.EphemerisMsg().write(ephemerisInMsgData)

    # subscribe input messages to module
    module.ephemerisInMsg.subscribeTo(ephemerisInMsg)

    module.ModelTag = "PlanetNavigation"
    posBound = numpy.array([1000.0] * 3)
    velBound = numpy.array([1.0] * 3)
    attBound = numpy.array([5E-3] * 3)
    rateBound = numpy.array([0.02] * 3)

    posSigma = 5.0
    velSigma = 0.035
    attSigma = 1.0 / 360.0 * math.pi / 180.0
    rateSigma = 0.05 * math.pi / 180.0

    pMatrix = [[posSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., posSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., posSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., velSigma, 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., velSigma, 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., velSigma, 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., attSigma, 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., attSigma, 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., attSigma, 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., rateSigma, 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., rateSigma, 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., rateSigma]]

    errorBounds = [[1000.], [1000.], [1000.], [1.], [1.], [1.], [0.005], [0.005], [0.005], [0.02], [0.02], [0.02]]

    module.walkBounds = errorBounds
    module.PMatrix = pMatrix
    module.crossTrans = True
    module.crossAtt = False

    # setup output message recorder objects
    ephemerisOutMsgRec = module.ephemerisOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, ephemerisOutMsgRec)

    # Execute the simulation
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(int(60 * 144.0 * 1E9))
    unitTestSim.ExecuteSimulation()

    # Pull module data and make sure it is correct
    r_BN_N = ephemerisOutMsgRec.r_BdyZero_N
    v_BN_N = ephemerisOutMsgRec.v_BdyZero_N
    sigma_BN = ephemerisOutMsgRec.sigma_BN
    omega_BN_B = ephemerisOutMsgRec.omega_BN_B

    countAllow = r_BN_N.shape[0] * 0.3/100.

    posDiffCount = 0
    velDiffCount = 0
    attDiffCount = 0
    rateDiffCount = 0
    i = 0
    while i < r_BN_N.shape[0]:
        posVecDiff = r_BN_N[i, 0:] - ephemerisInMsgData.r_BdyZero_N
        velVecDiff = v_BN_N[i, 0:] - ephemerisInMsgData.v_BdyZero_N
        attVecDiff = sigma_BN[i, 0:] - ephemerisInMsgData.sigma_BN
        rateVecDiff = omega_BN_B[i, 0:] - ephemerisInMsgData.omega_BN_B
        j = 0
        while j < 3:
            if abs(posVecDiff[j]) > posBound[j]:
                posDiffCount += 1
            if abs(velVecDiff[j]) > velBound[j]:
                velDiffCount += 1
            if abs(attVecDiff[j]) > attBound[j]:
                attDiffCount += 1
            if abs(rateVecDiff[j]) > rateBound[j]:
                rateDiffCount += 1
            j += 1
        i += 1

    errorCounts = [posDiffCount, velDiffCount, attDiffCount, rateDiffCount]

    for count in errorCounts:
        if count > countAllow:
            testFailCount += 1
            testMessages.append("FAILED: Too many error counts  -" + str(count))

    sigmaThreshold = 0.8
    posDiffCount = 0
    velDiffCount = 0
    attDiffCount = 0
    rateDiffCount = 0
    i = 0
    while i < r_BN_N.shape[0]:
        posVecDiff = r_BN_N[i,0:] - ephemerisInMsgData.r_BdyZero_N
        velVecDiff = v_BN_N[i,0:] - ephemerisInMsgData.v_BdyZero_N
        attVecDiff = sigma_BN[i,0:] - ephemerisInMsgData.sigma_BN
        rateVecDiff = omega_BN_B[i,0:] - ephemerisInMsgData.omega_BN_B
        j=0
        while j<3:
            if abs(posVecDiff[j]) > posBound[j]*sigmaThreshold:
                posDiffCount += 1
            if abs(velVecDiff[j]) > velBound[j]*sigmaThreshold:
                velDiffCount += 1
            if abs(attVecDiff[j]) > attBound[j]*sigmaThreshold:
                attDiffCount += 1
            if abs(rateVecDiff[j]) > rateBound[j]*sigmaThreshold:
                rateDiffCount += 1
            j += 1
        i += 1

    errorCounts = [posDiffCount, velDiffCount, attDiffCount, rateDiffCount]

    for count in errorCounts:
        if count < 1:
            testFailCount += 1
            testMessages.append("FAILED: Too few error counts - " + str(count))

    plt.figure(1)
    plt.clf()
    plt.figure(1, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(ephemerisOutMsgRec.times() * 1.0E-9, r_BN_N[:,0], label='x-position')
    plt.plot(ephemerisOutMsgRec.times() * 1.0E-9, r_BN_N[:,1], label='y-position')
    plt.plot(ephemerisOutMsgRec.times() * 1.0E-9, r_BN_N[:,2], label='z-position')

    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')

    plt.figure(2)
    plt.clf()
    plt.figure(2, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(ephemerisOutMsgRec.times() * 1.0E-9, v_BN_N[:,0], label='x-velocity')
    plt.plot(ephemerisOutMsgRec.times() * 1.0E-9, v_BN_N[:,1], label='y-velocity')
    plt.plot(ephemerisOutMsgRec.times() * 1.0E-9, v_BN_N[:,2], label='z-velocity')

    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')

    plt.figure(3)
    plt.clf()
    plt.figure(3, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(ephemerisOutMsgRec.times() * 1.0E-9, sigma_BN[:, 0], label='x-rotation')
    plt.plot(ephemerisOutMsgRec.times() * 1.0E-9, sigma_BN[:, 1], label='y-rotation')
    plt.plot(ephemerisOutMsgRec.times() * 1.0E-9, sigma_BN[:, 2], label='z-rotation')

    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('Attitude (rad)')

    plt.figure(4)
    plt.clf()
    plt.figure(4, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(ephemerisOutMsgRec.times() * 1.0E-9, omega_BN_B[:, 0], label='x-angular vel.')
    plt.plot(ephemerisOutMsgRec.times() * 1.0E-9, omega_BN_B[:, 1], label='y-angular vel.')
    plt.plot(ephemerisOutMsgRec.times() * 1.0E-9, omega_BN_B[:, 2], label='z-angular vel.')

    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('Attitude (rad)')

    if show_plots:
        plt.show()
        plt.close('all')

    # Corner case usage
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
    module.walkBounds = stateBoundsBad
    module.PMatrix = pMatrixBad

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


if __name__ == "__main__":
    test_planetNav(True)


