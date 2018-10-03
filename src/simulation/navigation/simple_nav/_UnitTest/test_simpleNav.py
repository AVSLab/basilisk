''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


import matplotlib.pyplot as plt
import numpy
import math
import os
from Basilisk.simulation import simple_nav
from Basilisk.simulation import spice_interface
from Basilisk.utilities import MessagingAccess
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import sim_model
from Basilisk.utilities import unitTestSupport


def listNorm(inputList):
   normValue = 0.0
   for elem in inputList:
      normValue += elem*elem
   normValue = math.sqrt(normValue)
   i=0
   while i<len(inputList):
      inputList[i] = inputList[i]/normValue
      i += 1

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

def test_unitSimpleNav(show_plots):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimpleNav(show_plots)
    assert testResults < 1, testMessage


def unitSimpleNav(show_plots):
    path = os.path.dirname(os.path.abspath(__file__))
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()

    unitTestProc = unitTestSim.CreateNewProcess(unitProcessName)
    # create the task and specify the integration update time
    unitTestProc.addTask(unitTestSim.CreateNewTask(unitTaskName, int(1E8)))

    #Now initialize the modules that we are using.  I got a little better as I went along
    sNavObject = simple_nav.SimpleNav()
    unitTestSim.AddModelToTask(unitTaskName, sNavObject)

    spiceMessage = spice_interface.SpicePlanetStateSimMsg()
    stateMessage = simple_nav.SCPlusStatesSimMsg()
    vehPosition = [10000.0, 0.0, 0.0]
    sunPosition = [10000.0, 1000.0, 0.0]

    stateMessage.r_BN_N = vehPosition 
    spiceMessage.PositionVector = sunPosition
    spiceMessage.PlanetName = "sun"

    # Inertial State output Message
    inputMessageSize =  stateMessage.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          "inertial_state_output",
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    unitTestSim.TotalSim.WriteMessageData("inertial_state_output",
                                          inputMessageSize,
                                          0,
                                          stateMessage)

    # Sun Planet Data Message
    inputMessageSize =  spiceMessage.getStructSize()
    unitTestSim.TotalSim.CreateNewMessage(unitProcessName,
                                          "sun_planet_data",
                                          inputMessageSize,
                                          2)  # number of buffers (leave at 2 as default, don't make zero)
    unitTestSim.TotalSim.WriteMessageData("sun_planet_data",
                                          inputMessageSize,
                                          0,
                                          spiceMessage)

    sNavObject.ModelTag = "SimpleNavigation"
    posBound = numpy.array([1000.0] * 3)
    velBound = numpy.array([1.0] * 3)
    attBound = numpy.array([5E-3] * 3)
    rateBound = numpy.array([0.02] * 3)
    sunBound = numpy.array([5.0 * math.pi / 180.0] * 3)
    dvBound = numpy.array([0.053] * 3)

    posSigma = 5.0
    velSigma = 0.035
    attSigma = 1.0 / 360.0 * math.pi / 180.0
    rateSigma = 0.05 * math.pi / 180.0
    sunSigma = math.pi / 180.0
    dvSigma = 0.1 * math.pi / 180.0

    pMatrix = [[posSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., posSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., posSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., velSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., velSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., velSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., attSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., attSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., attSigma, 0., 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., rateSigma, 0., 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., rateSigma, 0., 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., rateSigma, 0., 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., sunSigma, 0., 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., sunSigma, 0., 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., sunSigma, 0., 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., dvSigma, 0., 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., dvSigma, 0.],
               [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., dvSigma],
               ]
    errorBounds = [[1000.], [1000.], [1000.], [1.], [1.], [1.], [0.005], [0.005], [0.005], [0.02], [0.02], [0.02],
                   [5.0 * math.pi / 180.0], [5.0 * math.pi / 180.0], [5.0 * math.pi / 180.0], [0.053], [0.053], [0.053]]

    sNavObject.walkBounds = errorBounds
    sNavObject.PMatrix = pMatrix
    sNavObject.crossTrans = True
    sNavObject.crossAtt = False

    unitTestSim.TotalSim.logThisMessage("simple_att_nav_output", int(1E8))
    unitTestSim.TotalSim.logThisMessage("simple_trans_nav_output", int(1E8))
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(int(60 * 144.0 * 1E9))
    unitTestSim.ExecuteSimulation()

    # def obtainMessageVector(MessageName, MessageModule, MessageObj, MessageCount,
    #                         SimContainer, VarName, VarType, startIndex, stopIndex,
    #                         messageType=sim_model.messageBuffer):
    moduleName = 'Basilisk.simulation.simple_nav'
    posNav = MessagingAccess.obtainMessageVector("simple_trans_nav_output", moduleName,
                                                 'NavTransIntMsg', 60 * 144 * 10, unitTestSim.TotalSim, 'r_BN_N',
                                                 'double', 0, 2, sim_model.logBuffer)
    velNav = MessagingAccess.obtainMessageVector("simple_trans_nav_output", moduleName,
                                                 'NavTransIntMsg', 60 * 144 * 10, unitTestSim.TotalSim, 'v_BN_N',
                                                 'double', 0, 2, sim_model.logBuffer)
    attNav = MessagingAccess.obtainMessageVector("simple_att_nav_output", moduleName,
                                                 'NavAttIntMsg', 60 * 144 * 10, unitTestSim.TotalSim, 'sigma_BN',
                                                 'double', 0, 2, sim_model.logBuffer)
    rateNav = MessagingAccess.obtainMessageVector("simple_att_nav_output", moduleName,
                                                  'NavAttIntMsg', 60 * 144 * 10, unitTestSim.TotalSim, 'omega_BN_B',
                                                  'double', 0, 2, sim_model.logBuffer)
    dvNav = MessagingAccess.obtainMessageVector("simple_trans_nav_output", moduleName,
                                                'NavTransIntMsg', 60 * 144 * 10, unitTestSim.TotalSim, 'vehAccumDV',
                                                'double', 0, 2, sim_model.logBuffer)
    sunNav = MessagingAccess.obtainMessageVector("simple_att_nav_output", moduleName,
                                                 'NavAttIntMsg', 60 * 144 * 10, unitTestSim.TotalSim, 'vehSunPntBdy',
                                                 'double', 0, 2, sim_model.logBuffer)

    sunHatPred = numpy.array(sunPosition)-numpy.array(vehPosition)
    listNorm(sunHatPred)

    countAllow = posNav.shape[0] * 0.3/100.

    posDiffCount = 0
    velDiffCount = 0
    attDiffCount = 0
    rateDiffCount = 0
    dvDiffCount = 0
    sunDiffCount = 0
    i=0
    while i< posNav.shape[0]:
        posVecDiff = posNav[i,1:] - vehPosition
        velVecDiff = velNav[i,1:]
        attVecDiff = attNav[i,1:]
        rateVecDiff = rateNav[i,1:]
        dvVecDiff = dvNav[i,1:]
        sunVecDiff = math.acos(numpy.dot(sunNav[i, 1:], sunHatPred))
        j=0
        while j<3:
            if(abs(posVecDiff[j]) > posBound[j]):
                posDiffCount += 1
            if(abs(velVecDiff[j]) > velBound[j]):
                velDiffCount += 1
            if(abs(attVecDiff[j]) > attBound[j]):
                attDiffCount += 1
            if(abs(rateVecDiff[j]) > rateBound[j]):
                rateDiffCount += 1
            if(abs(dvVecDiff[j]) > dvBound[j]):
                dvDiffCount += 1
            j+=1
        if(abs(sunVecDiff) > 4.0*math.sqrt(3.0)*sunBound[0]):
            sunDiffCount += 1
        i+= 1

    errorCounts = [posDiffCount, velDiffCount, attDiffCount, rateDiffCount,
        dvDiffCount, sunDiffCount]

    for count in errorCounts:
        if count > countAllow:
            testFailCount += 1
            testMessages.append("FAILED: Too many error counts  -" + str(count))

    sigmaThreshold = 0.8
    posDiffCount = 0
    velDiffCount = 0
    attDiffCount = 0
    rateDiffCount = 0
    dvDiffCount = 0
    sunDiffCount = 0
    i=0
    while i< posNav.shape[0]:
        posVecDiff = posNav[i,1:] - vehPosition
        velVecDiff = velNav[i,1:]
        attVecDiff = attNav[i,1:]
        rateVecDiff = rateNav[i,1:]
        dvVecDiff = dvNav[i,1:]
        sunVecDiff = math.acos(numpy.dot(sunNav[i, 1:], sunHatPred))
        j=0
        while j<3:
            if(abs(posVecDiff[j]) > posBound[j]*sigmaThreshold):
                posDiffCount += 1
            if(abs(velVecDiff[j]) > velBound[j]*sigmaThreshold):
                velDiffCount += 1
            if(abs(attVecDiff[j]) > attBound[j]*sigmaThreshold):
                attDiffCount += 1
            if(abs(rateVecDiff[j]) > rateBound[j]*sigmaThreshold):
                rateDiffCount += 1
            if(abs(dvVecDiff[j]) > dvBound[j]*sigmaThreshold):
                dvDiffCount += 1
            j+=1
        if(abs(sunVecDiff) > 4.0*math.sqrt(3.0)*sunBound[0]*sigmaThreshold):
            sunDiffCount += 1
        i+= 1

    errorCounts = [posDiffCount, velDiffCount, attDiffCount, rateDiffCount,
        dvDiffCount, sunDiffCount]

    for count in errorCounts:
        if count < 1:
            testFailCount += 1
            testMessages.append("FAILED: Too few error counts -" + str(count))

    plt.figure(1)
    plt.clf()
    plt.figure(1, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(posNav[:,0] * 1.0E-9 , posNav[:,1], label='x-position')
    plt.plot(posNav[:,0] * 1.0E-9, posNav[:,2], label='y-position')
    plt.plot(posNav[:,0] * 1.0E-9, posNav[:,3], label='z-position')

    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    unitTestSupport.writeFigureLaTeX('SimpleNavPos', 'Simple Navigation Position Signal', plt, 'height=0.4\\textwidth, keepaspectratio', path)
    if show_plots:
        plt.show()
        plt.close('all')

    plt.figure(2)
    plt.clf()
    plt.figure(2, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(attNav[:,0] * 1.0E-9 , attNav[:, 1], label='x-rotation')
    plt.plot(attNav[:,0] * 1.0E-9 , attNav[:, 2], label='y-rotation')
    plt.plot(attNav[:,0] * 1.0E-9 , attNav[:, 3], label='z-rotation')

    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('Attitude (rad)')
    unitTestSupport.writeFigureLaTeX('SimpleNavAtt', 'Simple Navigation Att Signal', plt, 'height=0.4\\textwidth, keepaspectratio', path)
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
    # stateBoundsBad = [[0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.]]
    stateBoundsBad = [[0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.]]
    sNavObject.walkBounds = stateBoundsBad
    sNavObject.PMatrix = pMatrixBad

    sNavObject.inputStateName = "random_name"
    sNavObject.inputSunName = "weirdly_not_the_sun"
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(int(1E8))
    unitTestSim.ExecuteSimulation()

    # print out success message if no error were found
    if testFailCount == 0:
        print   "PASSED"

    assert testFailCount < 1, testMessages
    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
if __name__ == "__main__":
    unitSimpleNav(False)
