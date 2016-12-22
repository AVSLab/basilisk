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
import hingedRigidBodyStateEffector
import fuelSloshParticle
import fuelTank
import sim_model
import macros
import ctypes

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def spacecraftPlusAllTest(show_plots):
    [testResults, testMessage] = test_hubPropagate(show_plots)
    assert testResults < 1, testMessage

def test_hubPropagate(show_plots):
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages
    
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    
    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.TotalSim.terminateSimulation()
    
    # Create test thread
    testProcessRate = macros.sec2nano(0.1)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    unitTestSim.particle1 = fuelSloshParticle.FuelSloshParticle()
    unitTestSim.particle2 = fuelSloshParticle.FuelSloshParticle()
    unitTestSim.particle3 = fuelSloshParticle.FuelSloshParticle()

    # Define Variables for particle 1
    unitTestSim.particle1.massFSP = 10
    unitTestSim.particle1.k = 100.0
    unitTestSim.particle1.c = 0.0
    unitTestSim.particle1.rPB_B = [[0.1], [0], [-0.1]]
    unitTestSim.particle1.pHat_B = [[1], [0], [0]]
    unitTestSim.particle1.nameOfRhoState = "fuelSloshParticleRho1"
    unitTestSim.particle1.nameOfRhoDotState = "fuelSloshParticleRhoDot1"

    # Define Variables for particle 2
    unitTestSim.particle2.massFSP = 20
    unitTestSim.particle2.k = 100.0
    unitTestSim.particle2.c = 0.0
    unitTestSim.particle2.rPB_B = [[0], [0], [0.1]]
    unitTestSim.particle2.pHat_B = [[0], [1], [0]]
    unitTestSim.particle2.nameOfRhoState = "fuelSloshParticleRho2"
    unitTestSim.particle2.nameOfRhoDotState = "fuelSloshParticleRhoDot2"

    # Define Variables for particle 3
    unitTestSim.particle3.massFSP = 15
    unitTestSim.particle3.k = 100.0
    unitTestSim.particle3.c = 0.0
    unitTestSim.particle3.rPB_B = [[-0.1], [0], [0.1]]
    unitTestSim.particle3.pHat_B = [[0], [0], [1]]
    unitTestSim.particle3.nameOfRhoState = "fuelSloshParticleRho3"
    unitTestSim.particle3.nameOfRhoDotState = "fuelSloshParticleRhoDot3"

    #define the fuel tank
    unitTestSim.tank1 = fuelTank.FuelTank()
    unitTestSim.tank1.radiusTank = 0;
    unitTestSim.tank1.rTB_B = [[0],[0],[0]]
    unitTestSim.tank1.nameOfMassState = "fuelTankMass1"
    unitTestSim.tank1.pushFuelSloshParticle(unitTestSim.particle1)
    unitTestSim.tank1.pushFuelSloshParticle(unitTestSim.particle2)
    unitTestSim.tank1.pushFuelSloshParticle(unitTestSim.particle3)

    # Add panels to spaceCraft
    # this next line is not working
    scObject.addStateEffector(unitTestSim.tank1)
    
    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)
    
    unitTestSim.InitializeSimulation()

    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")
    omegaRef = scObject.dynManager.getStateObject("hubOmega")
    rho1Ref = scObject.dynManager.getStateObject("fuelSloshParticleRho1")
    rhoDot1Ref = scObject.dynManager.getStateObject("fuelSloshParticleRhoDot1")
    rho2Ref = scObject.dynManager.getStateObject("fuelSloshParticleRho2")
    rhoDot2Ref = scObject.dynManager.getStateObject("fuelSloshParticleRhoDot2")
    rho3Ref = scObject.dynManager.getStateObject("fuelSloshParticleRho3")
    rhoDot3Ref = scObject.dynManager.getStateObject("fuelSloshParticleRhoDot3")

    posRef.setState([[0.0], [0.0], [0.0]])
    velRef.setState([[0.0], [0.0], [0.0]])
    sigmaRef.setState([[0.0], [0.0], [0.0]])
    omegaRef.setState([[0.1], [-0.1], [0.1]])
    rho1Ref.setState([[0.05]])
    rhoDot1Ref.setState([[0.0]])
    rho2Ref.setState([[-0.025]])
    rhoDot2Ref.setState([[0.0]])
    rho3Ref.setState([[-0.015]])
    rhoDot3Ref.setState([[0.0]])

    scObject.hub.mHub = 750
    scObject.hub.rBcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    stopTime = 60.0*10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    dataPos = posRef.getState()
    dataPos = [[stopTime, dataPos[0][0], dataPos[1][0], dataPos[2][0]]]
    dataSigma =  sigmaRef.getState()
    dataSigma = [[stopTime, dataSigma[0][0], dataSigma[1][0], dataSigma[2][0]]]

    truePos = [
                [-1.36388979e-01, -1.70517452e-01, -3.27473799e-02]
                ]
    trueSigma = [
                  [-2.46306074e-01, 8.25425414e-01, -3.37112618e-01]
                  ]

    accuracy = 1e-8
    for i in range(0,len(truePos)):
        print(dataPos[i])
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataPos[i],truePos[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED:  Fuel Slosh failed pos unit test at t=" + str(dataPos[i][0]*macros.NANO2SEC) + "sec\n")

    for i in range(0,len(trueSigma)):
        print(dataSigma[i])
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataSigma[i],trueSigma[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED:  Fuel Slosh failed attitude unit test at t=" + str(dataSigma[i][0]*macros.NANO2SEC) + "sec\n")

    if testFailCount == 0:
        print "PASSED: " + " Fuel Slosh Test"
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    spacecraftPlusAllTest(False)
