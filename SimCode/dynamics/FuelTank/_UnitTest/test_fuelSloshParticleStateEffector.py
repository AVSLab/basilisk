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
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    unitTestSim.particle1 = fuelSloshParticle.FuelSloshParticle()
    unitTestSim.particle2 = fuelSloshParticle.FuelSloshParticle()
    unitTestSim.particle3 = fuelSloshParticle.FuelSloshParticle()

    # Define Variables for particle 1
    unitTestSim.particle1.k = 100.0
    unitTestSim.particle1.c = 0.0
    unitTestSim.particle1.r_PB_B = [[0.1], [0], [-0.1]]
    unitTestSim.particle1.pHat_B = [[1], [0], [0]]
    unitTestSim.particle1.nameOfRhoState = "fuelSloshParticleRho1"
    unitTestSim.particle1.nameOfRhoDotState = "fuelSloshParticleRhoDot1"
    unitTestSim.particle1.nameOfMassState = "fuelSloshParticleMass1"

    # Define Variables for particle 2
    unitTestSim.particle2.k = 100.0
    unitTestSim.particle2.c = 0.0
    unitTestSim.particle2.r_PB_B = [[0], [0], [0.1]]
    unitTestSim.particle2.pHat_B = [[0], [1], [0]]
    unitTestSim.particle2.nameOfRhoState = "fuelSloshParticleRho2"
    unitTestSim.particle2.nameOfRhoDotState = "fuelSloshParticleRhoDot2"
    unitTestSim.particle2.nameOfMassState = "fuelSloshParticleMass2"

    # Define Variables for particle 3
    unitTestSim.particle3.k = 100.0
    unitTestSim.particle3.c = 0.0
    unitTestSim.particle3.r_PB_B = [[-0.1], [0], [0.1]]
    unitTestSim.particle3.pHat_B = [[0], [0], [1]]
    unitTestSim.particle3.nameOfRhoState = "fuelSloshParticleRho3"
    unitTestSim.particle3.nameOfRhoDotState = "fuelSloshParticleRhoDot3"
    unitTestSim.particle3.nameOfMassState = "fuelSloshParticleMass3"

    #define the fuel tank
    unitTestSim.tank1 = fuelTank.FuelTank()
    unitTestSim.tank1.setTankModel(fuelTank.TANK_MODEL_CONSTANT_VOLUME)
    tankModel = fuelTank.cvar.FuelTankModelConstantVolume
    tankModel.propMassInit = 30.0
    tankModel.r_TcT_TInit = [[0.0],[0.0],[0.0]]
    tankModel.radiusTankInit = 0.5
    unitTestSim.tank1.r_TB_B = [[0],[0],[0.1]]
    unitTestSim.tank1.nameOfMassState = "fuelTankMass1"
    unitTestSim.tank1.pushFuelSloshParticle(unitTestSim.particle1)
    unitTestSim.tank1.pushFuelSloshParticle(unitTestSim.particle2)
    unitTestSim.tank1.pushFuelSloshParticle(unitTestSim.particle3)
    unitTestSim.tank1.updateOnly = True

    # Add panels to spaceCraft
    # this next line is not working
    scObject.addStateEffector(unitTestSim.tank1)
    
    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)
    
    unitTestSim.InitializeSimulation()

    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbKinEnergy", testProcessRate, 0, 0, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotEnergy", testProcessRate, 0, 0, 'double')

    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")
    sigmaRef = scObject.dynManager.getStateObject("hubSigma")
    omegaRef = scObject.dynManager.getStateObject("hubOmega")
    rho1Ref = scObject.dynManager.getStateObject("fuelSloshParticleRho1")
    rhoDot1Ref = scObject.dynManager.getStateObject("fuelSloshParticleRhoDot1")
    mass1Ref = scObject.dynManager.getStateObject("fuelSloshParticleMass1")
    rho2Ref = scObject.dynManager.getStateObject("fuelSloshParticleRho2")
    rhoDot2Ref = scObject.dynManager.getStateObject("fuelSloshParticleRhoDot2")
    mass2Ref = scObject.dynManager.getStateObject("fuelSloshParticleMass2")
    rho3Ref = scObject.dynManager.getStateObject("fuelSloshParticleRho3")
    rhoDot3Ref = scObject.dynManager.getStateObject("fuelSloshParticleRhoDot3")
    mass3Ref = scObject.dynManager.getStateObject("fuelSloshParticleMass3")
    massTank = scObject.dynManager.getStateObject(unitTestSim.tank1.nameOfMassState)

    posRef.setState([[0.0], [0.0], [0.0]])
    velRef.setState([[0.0], [0.0], [0.0]])
    sigmaRef.setState([[0.0], [0.0], [0.0]])
    omegaRef.setState([[0.1], [-0.1], [0.1]])
    rho1Ref.setState([[0.05]])
    rhoDot1Ref.setState([[0.0]])
    mass1Ref.setState([[10.0]])
    rho2Ref.setState([[-0.025]])
    rhoDot2Ref.setState([[0.0]])
    mass2Ref.setState([[20.0]])
    rho3Ref.setState([[-0.015]])
    rhoDot3Ref.setState([[0.0]])
    mass3Ref.setState([[15.0]])
    massTank.setState([[30]])

    scObject.hub.mHub = 750
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    stopTime = 2.5
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    orbKinEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbKinEnergy")
    orbAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
    rotAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")
    rotEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotEnergy")

    dataPos = posRef.getState()
    dataPos = [[stopTime, dataPos[0][0], dataPos[1][0], dataPos[2][0]]]
    dataSigma =  sigmaRef.getState()
    dataSigma = [[stopTime, dataSigma[0][0], dataSigma[1][0], dataSigma[2][0]]]

    truePos = [
                [0.0004742421817176357, 9.64154405238357e-05, 0.000528986131060958]
                ]
    trueSigma = [
                  [0.06115213876020642, -0.06561968675506792, 0.061244392141876255]
                  ]

    initialOrbAngMom_N = [
                [orbAngMom_N[0,1], orbAngMom_N[0,2], orbAngMom_N[0,3]]
                ]

    finalOrbAngMom = [
                [orbAngMom_N[-1,0], orbAngMom_N[-1,1], orbAngMom_N[-1,2], orbAngMom_N[-1,3]]
                 ]

    initialRotAngMom_N = [
                [rotAngMom_N[0,1], rotAngMom_N[0,2], rotAngMom_N[0,3]]
                ]

    finalRotAngMom = [
                [rotAngMom_N[-1,0], rotAngMom_N[-1,1], rotAngMom_N[-1,2], rotAngMom_N[-1,3]]
                 ]

    initialOrbKinEnergy = [
                [orbKinEnergy[0,1]]
                ]

    finalOrbKinEnergy = [
                [orbKinEnergy[-1,0], orbKinEnergy[-1,1]]
                 ]

    initialRotEnergy = [
                [rotEnergy[0,1]]
                ]

    finalRotEnergy = [
                [rotEnergy[-1,0], rotEnergy[-1,1]]
                 ]

    plt.figure(1)
    plt.plot(orbAngMom_N[:,0]*1e-9, orbAngMom_N[:,1] - orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, orbAngMom_N[:,2] - orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, orbAngMom_N[:,3] - orbAngMom_N[0,3])
    plt.title("Change in Orbital Angular Momentum")
    plt.figure(2)
    plt.plot(rotAngMom_N[:,0]*1e-9, rotAngMom_N[:,1] - rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, rotAngMom_N[:,2] - rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, rotAngMom_N[:,3] - rotAngMom_N[0,3])
    plt.title("Change in Rotational Angular Momentum")
    plt.figure(3)
    plt.plot(orbKinEnergy[:,0]*1e-9, orbKinEnergy[:,1] - orbKinEnergy[0,1])
    plt.title("Change in Orbital Kinetic Energy")
    plt.figure(4)
    plt.plot(rotEnergy[:,0]*1e-9, rotEnergy[:,1] - rotEnergy[0,1])
    plt.title("Change in Rotational Energy")
    if show_plots == True:
        plt.show()

    accuracy = 1e-8
    for i in range(0,len(truePos)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataPos[i],truePos[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED:  Fuel Slosh failed pos unit test")

    for i in range(0,len(trueSigma)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataSigma[i],trueSigma[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED:  Fuel Slosh failed attitude unit test")

    for i in range(0,len(initialOrbAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i],initialOrbAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Slosh unit test failed orbital angular momentum unit test")

    for i in range(0,len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i],initialRotAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Slosh unit test failed rotational angular momentum unit test")

    for i in range(0,len(initialRotEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotEnergy[i],initialRotEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Slosh unit test failed rotational energy unit test")

    for i in range(0,len(initialOrbKinEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbKinEnergy[i],initialOrbKinEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Fuel Slosh unit test failed orbital kinetic energy unit test")

    if testFailCount == 0:
        print "PASSED: " + " Fuel Slosh Test"

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    spacecraftPlusAllTest(False)
