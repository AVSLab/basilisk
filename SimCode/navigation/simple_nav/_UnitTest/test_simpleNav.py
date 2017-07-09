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
#Very simple simulation.  Just sets up and calls the SPICE interface.  Could 
#be the basis for a unit test of SPICE

import pytest
import sys, os, inspect


filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

import matplotlib.pyplot as plt
import numpy
import ctypes
import math

#sys.path.append(os.environ['SIMULATION_BASE']+'/modules')
#sys.path.append(os.environ['SIMULATION_BASE']+'/PythonModules/')

#Import all of the modules that we are going to call in this simulation
import simple_nav
import spice_interface
import MessagingAccess
import SimulationBaseClass
import sim_model
import unitTestSupport


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

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("UseFlag", [False]
  )

def test_unitSimpleNav(show_plots, UseFlag):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimpleNav(show_plots, UseFlag)
    assert testResults < 1, testMessage


def unitSimpleNav(show_plots, UseFlag):
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
    posBound = [1000.0]*3
    velBound = [1.0]*3
    attBound = [5E-3]*3
    rateBound = [0.02]*3
    sunBound = [5.0*math.pi/180.0]*3
    dvBound = [0.053]*3

    posSigma = 5.0
    velSigma = 0.05
    attSigma = 5.0/3600.0*math.pi/180.0
    rateSigma = 0.05*math.pi/180.0
    sunSigma = 0.1*math.pi/180.0
    dvSigma = 0.1*math.pi/180.0

    pMatrix = [0.0]*18*18
    pMatrix[0*18+0] = pMatrix[1*18+1] = pMatrix[2*18+2] = posSigma
    pMatrix[3*18+3] = pMatrix[4*18+4] = pMatrix[5*18+5] = velSigma
    pMatrix[6*18+6] = pMatrix[7*18+7] = pMatrix[8*18+8] = attSigma
    pMatrix[9*18+9] = pMatrix[10*18+10] = pMatrix[11*18+11] = rateSigma
    pMatrix[12*18+12] = pMatrix[13*18+13] = pMatrix[14*18+14] = sunSigma
    pMatrix[15*18+15] = pMatrix[16*18+16] = pMatrix[17*18+17] = dvSigma
    errorBounds = []
    errorBounds.extend(posBound)
    errorBounds.extend(velBound)
    errorBounds.extend(attBound)
    errorBounds.extend(rateBound)
    errorBounds.extend(sunBound)
    errorBounds.extend(dvBound)

    sNavObject.walkBounds = sim_model.DoubleVector(errorBounds)
    sNavObject.PMatrix = sim_model.DoubleVector(pMatrix)
    sNavObject.crossTrans = True
    sNavObject.crossAtt = False

    unitTestSim.TotalSim.logThisMessage("simple_att_nav_output", int(1E8))
    unitTestSim.TotalSim.logThisMessage("simple_trans_nav_output", int(1E8))
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(int(60*144.0*1E9))
    unitTestSim.ExecuteSimulation()

    posNav = MessagingAccess.obtainMessageVector("simple_trans_nav_output", 'simple_nav',
        'NavTransIntMsg', 60*144*10, unitTestSim.TotalSim, 'r_BN_N', 'double', 0, 2, sim_model.logBuffer)
    velNav = MessagingAccess.obtainMessageVector("simple_trans_nav_output", 'simple_nav',
        'NavTransIntMsg', 60*144*10, unitTestSim.TotalSim, 'v_BN_N', 'double', 0, 2, sim_model.logBuffer)
    attNav = MessagingAccess.obtainMessageVector("simple_att_nav_output", 'simple_nav',
        'NavAttIntMsg', 60*144*10, unitTestSim.TotalSim, 'sigma_BN', 'double', 0, 2, sim_model.logBuffer)
    rateNav = MessagingAccess.obtainMessageVector("simple_att_nav_output", 'simple_nav',
        'NavAttIntMsg', 60*144*10, unitTestSim.TotalSim, 'omega_BN_B', 'double', 0, 2, sim_model.logBuffer)
    dvNav = MessagingAccess.obtainMessageVector("simple_trans_nav_output", 'simple_nav',
        'NavTransIntMsg', 60*144*10, unitTestSim.TotalSim, 'vehAccumDV', 'double', 0, 2, sim_model.logBuffer)
    sunNav = MessagingAccess.obtainMessageVector("simple_att_nav_output", 'simple_nav',
        'NavAttIntMsg', 60*144*10, unitTestSim.TotalSim, 'vehSunPntBdy', 'double', 0, 2, sim_model.logBuffer)


    sunHatPred = numpy.array(sunPosition)-numpy.array(vehPosition)
    listNorm(sunHatPred)

    countAllow = posNav.shape[0] * 0.3 * 100


    sigmaThreshold = 0.0
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
            if(abs(posVecDiff[j]) > posBound[j] + posSigma*sigmaThreshold):
                posDiffCount += 1
            if(abs(velVecDiff[j]) > velBound[j] + velSigma*sigmaThreshold):
                velDiffCount += 1
            if(abs(attVecDiff[j]) > attBound[j] + attSigma*sigmaThreshold):
                attDiffCount += 1
            if(abs(rateVecDiff[j]) > rateBound[j] + rateSigma*sigmaThreshold):
                rateDiffCount += 1
            if(abs(dvVecDiff[j]) > dvBound[j] + dvSigma*sigmaThreshold):
                dvDiffCount += 1
            j+=1
        if(abs(sunVecDiff) > 4.0*math.sqrt(3.0)*sunBound[0] + 4.0*sunSigma*sigmaThreshold):
            sunDiffCount += 1
        i+= 1

    errorCounts = [posDiffCount, velDiffCount, attDiffCount, rateDiffCount,
        dvDiffCount, sunDiffCount]
    i=0
    print errorCounts
    for count in errorCounts:
        if count > countAllow:
            print "Too many error counts for element: "
            print count
            testFailCount += 1
            testMessages.append("FAILED: Too many error counts for element: %(DiffVal)i \n" % \
                            {"i": i})


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
    plt.close()

    plt.figure(1, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.plot(attNav[:,0] * 1.0E-9 , attNav[:, 1], label='x-rotation')
    plt.plot(attNav[:,0] * 1.0E-9 , attNav[:, 2], label='y-rotation')
    plt.plot(attNav[:,0] * 1.0E-9 , attNav[:, 3], label='z-rotation')

    plt.legend(loc='upper left')
    plt.xlabel('Time (s)')
    plt.ylabel('Attitude (rad)')
    if show_plots:
        plt.show()
    unitTestSupport.writeFigureLaTeX('SimpleNavAtt', 'Simple Navigation Att Signal', plt, 'height=0.4\\textwidth, keepaspectratio', path)


    # Corner case usage

    pMatrixBad = [0.0]*12*12
    stateBoundsBad = [0.0]*14
    sNavObject.walkBounds = sim_model.DoubleVector(stateBoundsBad)
    sNavObject.PMatrix = sim_model.DoubleVector(pMatrixBad)
    sNavObject.inputStateName = "random_name"
    sNavObject.inputSunName = "weirdly_not_the_sun"
    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(int(1E8))
    unitTestSim.ExecuteSimulation()

    # print out success message if no error were found
    if testFailCount == 0:
        print   " \n PASSED "

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]


# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    test_unitSimpleNav(False, # show_plots
                       False
                   )
