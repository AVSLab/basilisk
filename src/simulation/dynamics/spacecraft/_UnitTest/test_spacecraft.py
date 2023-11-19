
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

import numpy
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.simulation import spacecraft
from Basilisk.utilities import macros
from Basilisk.simulation import gravityEffector
from Basilisk.simulation import extForceTorque
from Basilisk.utilities import RigidBodyKinematics
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import GravityGradientEffector
from Basilisk.architecture import messaging

def addTimeColumn(time, data):
    return numpy.transpose(numpy.vstack([[time], numpy.transpose(data)]))

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_


@pytest.mark.parametrize("function", ["SCTranslation"
                                      , "SCTransAndRotation"
                                      , "SCRotation"
                                      , "SCTransBOE"
                                      , "SCPointBVsPointC"
                                      , "scOptionalRef"
                                      , "scAccumDV"
                                      ])
def test_spacecraftAllTest(show_plots, function):
    """Module Unit Test"""
    if function == "scOptionalRef":
        [testResults, testMessage] = eval(function + '(show_plots, 1e-3)')
    elif function == "scAccumDV":
        [testResults, testMessage] = eval(function + '()')
    else:
        [testResults, testMessage] = eval(function + '(show_plots)')
    assert testResults < 1, testMessage


def SCTranslation(show_plots):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.01)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    unitTestSim.earthGravBody.planetName = "earth_planet_data"
    unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
    unitTestSim.earthGravBody.isCentralBody = True

    scObject.gravField.gravBodies = spacecraft.GravBodyVector([unitTestSim.earthGravBody])

    dataLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Define initial conditions of the spacecraft
    scObject.hub.mHub = 100
    scObject.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]

    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totOrbEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    unitTestSim.InitializeSimulation()
    accuracy = 1e-3
    if not unitTestSupport.isArrayEqual(scObject.scStateOutMsg.read().r_BN_N,
                                        [item for sublist in scObject.hub.r_CN_NInit for item in sublist],
                                        3, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: SCHub Translation test failed init pos msg unit test")
    if not unitTestSupport.isArrayEqual(scObject.scStateOutMsg.read().v_BN_N,
                                        [item for sublist in scObject.hub.v_CN_NInit for item in sublist],
                                        3, accuracy):
        testFailCount += 1
        testMessages.append("FAILED: SCHub Translation test failed init pos msg unit test")


    stopTime = 10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()
    orbAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbAngMomPntN_N)
    orbEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbEnergy)

    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInOrbitalAngularMomentumTranslationOnly", "Change in Orbital Angular Momentum Translation Only", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInOrbitalEnergyTranslationOnly", "Change in Orbital Energy Translation Only", plt, r"width=0.8\textwidth", path)
    if show_plots:
        plt.show()
        plt.close('all')

    moduleOutput = dataLog.r_BN_N

    truePos = [
                [-4072255.7737936215, 7456050.4649078, 5258610.029627514]
                ]

    initialOrbAngMom_N = [
                [orbAngMom_N[0,1], orbAngMom_N[0,2], orbAngMom_N[0,3]]
                ]

    finalOrbAngMom = [
                [orbAngMom_N[-1,1], orbAngMom_N[-1,2], orbAngMom_N[-1,3]]
                 ]

    initialOrbEnergy = [
                [orbEnergy[0,1]]
                ]

    finalOrbEnergy = [
                [orbEnergy[-1,1]]
                 ]

    accuracy = 1e-10
    for i in range(0,len(truePos)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(moduleOutput[-1,:],truePos[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: SCHub Translation test failed pos unit test")

    for i in range(0,len(initialOrbAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i],initialOrbAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: SCHub Translation test failed orbital angular momentum unit test")

    for i in range(0,len(initialOrbEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i],initialOrbEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: SCHub Translation test failed orbital energy unit test")

    if testFailCount == 0:
        print("PASSED: " + " SCHub Translation Integrated Sim Test")

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def SCTransAndRotation(show_plots):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    unitTestSim.earthGravBody.planetName = "earth_planet_data"
    unitTestSim.earthGravBody.mu = 0.3986004415E+15  # meters!
    unitTestSim.earthGravBody.isCentralBody = True

    scObject.gravField.gravBodies = spacecraft.GravBodyVector([unitTestSim.earthGravBody])

    dataLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Define initial conditions of the spacecraft
    scObject.hub.mHub = 100
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    scObject.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.5], [-0.4], [0.7]]

    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    unitTestSim.InitializeSimulation()

    stopTime = 10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    orbAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbAngMomPntN_N)
    rotAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotAngMomPntC_N)
    rotEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotEnergy)
    orbEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbEnergy)

    r_BN_NOutput = dataLog.r_BN_N
    sigma_BNOutput = dataLog.sigma_BN

    truePos = [
                [-4072255.7737936215, 7456050.4649078, 5258610.029627514]
                ]

    trueSigma = [
                [3.73034285e-01,  -2.39564413e-03,   2.08570797e-01]
                ]

    initialOrbAngMom_N = [
                [orbAngMom_N[0,1], orbAngMom_N[0,2], orbAngMom_N[0,3]]
                ]

    finalOrbAngMom = [
                [orbAngMom_N[-1,1], orbAngMom_N[-1,2], orbAngMom_N[-1,3]]
                 ]

    initialRotAngMom_N = [
                [rotAngMom_N[0,1], rotAngMom_N[0,2], rotAngMom_N[0,3]]
                ]

    finalRotAngMom = [
                [rotAngMom_N[-1,1], rotAngMom_N[-1,2], rotAngMom_N[-1,3]]
                 ]

    initialOrbEnergy = [
                [orbEnergy[0,1]]
                ]

    finalOrbEnergy = [
                [orbEnergy[-1,1]]
                 ]

    initialRotEnergy = [
                [rotEnergy[0,1]]
                ]

    finalRotEnergy = [
                [rotEnergy[-1,1]]
                 ]

    plt.close('all')
    plt.figure()
    plt.clf()
    plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInOrbitalAngularMomentumTranslationAndRotation", "Change in Orbital Angular Momentum Translation And Rotation", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInOrbitalEnergyTranslationAndRotation", "Change in Orbital Energy Translation And Rotation", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInRotationalAngularMomentumTranslationAndRotation", "Change in Rotational Angular Momentum Translation And Rotation", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(rotEnergy[:,0]*1e-9, (rotEnergy[:,1] - rotEnergy[0,1])/rotEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInRotationalEnergyTranslationAndRotation", "Change in Rotational Energy Translation And Rotation", plt, r"width=0.8\textwidth", path)
    if show_plots:
        plt.show()
        plt.close('all')

    accuracy = 1e-8
    for i in range(0,len(truePos)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(r_BN_NOutput[-1,:],truePos[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed pos unit test")

    for i in range(0,len(trueSigma)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(sigma_BNOutput[-1,:],trueSigma[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed attitude unit test")

    accuracy = 1e-10
    for i in range(0,len(initialOrbAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i],initialOrbAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed orbital angular momentum unit test")

    for i in range(0,len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i],initialRotAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed rotational angular momentum unit test")

    for i in range(0,len(initialRotEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotEnergy[i],initialRotEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed rotational energy unit test")

    for i in range(0,len(initialOrbEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i],initialOrbEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed orbital energy unit test")

    if testFailCount == 0:
        print("PASSED: " + " Spacecraft Translation and Rotation Integrated Sim Test")

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def SCRotation(show_plots):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    timeStep = 0.001
    testProcessRate = macros.sec2nano(timeStep)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    dataLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Define initial conditions of the spacecraft
    scObject.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    scObject.hub.omega_BN_BInit = [[0.5], [-0.4], [0.7]]

    # BOE for rotational dynamics
    h = numpy.dot(numpy.asarray(scObject.hub.IHubPntBc_B),numpy.asarray(scObject.hub.omega_BN_BInit).flatten())
    H = numpy.linalg.norm(h)
    n3_B = -h/H

    # Find DCM
    n2_B = numpy.zeros(3)
    n2_B[1] = 0.1
    n2_B[0] = -n2_B[1]*n3_B[1]/n3_B[0]
    n2_B = n2_B/numpy.linalg.norm(n2_B)
    n1_B = numpy.cross(n2_B,n3_B)
    n1_B = n1_B/(numpy.linalg.norm(n1_B))
    dcm_BN = numpy.zeros([3,3])
    dcm_BN[:,0] = n1_B
    dcm_BN[:,1] = n2_B
    dcm_BN[:,2] = n3_B
    h3_N = numpy.array([0,0,-H])
    h3_B = numpy.dot(dcm_BN,h3_N)
    h3_Ncheck = numpy.dot(dcm_BN.transpose(),h3_B)
    sigmaCalc = RigidBodyKinematics.C2MRP(dcm_BN)
    scObject.hub.sigma_BNInit = [[sigmaCalc[0]], [sigmaCalc[1]], [sigmaCalc[2]]]

    scObjectLog = scObject.logger(["totRotAngMomPntC_N", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    unitTestSim.InitializeSimulation()

    stopTime = 10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    rotAngMom_N = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotAngMomPntC_N)
    rotEnergy = unitTestSupport.addTimeColumn(scObjectLog.times(), scObjectLog.totRotEnergy)
    rotAngMomMag = numpy.zeros(len(rotAngMom_N))
    for i in range(0,len(rotAngMom_N)):
        rotAngMomMag[i] = numpy.linalg.norm(numpy.asarray(rotAngMom_N[i,1:4]))

    trueSigma = [
                [5.72693314e-01,   5.10734375e-01,  -3.07377611e-01]
                ]

    initialRotAngMom_N = [
                [numpy.linalg.norm(numpy.asarray(rotAngMom_N[0,1:4]))]
                ]

    finalRotAngMom = [
                [numpy.linalg.norm(numpy.asarray(rotAngMom_N[-1,1:4]))]
                 ]

    initialRotEnergy = [
                [rotEnergy[0,1]]
                ]

    finalRotEnergy = [
                [rotEnergy[-1,1]]
                 ]

    moduleOutput = addTimeColumn(dataLog.times(), dataLog.sigma_BN)
    omega_BNOutput = addTimeColumn(dataLog.times(), dataLog.omega_BN_B)

    check = 0
    for i in range(0,len(moduleOutput)):
        if check == 0 and moduleOutput[i+1,2] < moduleOutput[i,2]:
            check = 1
        if check == 1 and moduleOutput[i+1,2] > moduleOutput[i,2]:
            check = 2
            index = i+1
            break

    sigmaBeforeSwitch = moduleOutput[index-1,1:4]
    sigmaBeforeBefore = moduleOutput[index-2,1:4]
    sigmaAfterSwitch = moduleOutput[index,:]
    deltaT = (moduleOutput[index-1,0] - moduleOutput[index-2,0])*1e-9
    yPrime = (sigmaBeforeSwitch - sigmaBeforeBefore)/deltaT
    sigmaGhost = sigmaBeforeSwitch + yPrime*deltaT
    sigmaAfterAnalytical = - sigmaGhost/numpy.dot(numpy.linalg.norm(numpy.asarray(sigmaGhost)),numpy.linalg.norm(numpy.asarray(sigmaGhost)))

    timeArray = numpy.zeros(5)
    sigmaArray = numpy.zeros([3,5])
    omegaAnalyticalArray = numpy.zeros([3,5])
    omegaArray = numpy.zeros([4,5])
    for i in range(0, 5):
        idx = int(stopTime/timeStep*(i+1)/5)
        timeArray[i] = moduleOutput[idx, 0]
        sigmaArray[:, i] = moduleOutput[idx, 1:4]
        sigma = sigmaArray[:, i]
        sigmaNorm = numpy.linalg.norm(sigma)
        sigma1 = sigma[0]
        sigma2 = sigma[1]
        sigma3 = sigma[2]
        omegaArray[:,i] = omega_BNOutput[idx, :]
        omegaAnalyticalArray[0,i] = -H/(1 + sigmaNorm**2)**2*(8*sigma1*sigma3 - 4*sigma2*(1 - sigmaNorm**2))/scObject.hub.IHubPntBc_B[0][0]
        omegaAnalyticalArray[1,i] = -H/(1 + sigmaNorm**2)**2*(8*sigma2*sigma3 + 4*sigma1*(1 - sigmaNorm**2))/scObject.hub.IHubPntBc_B[1][1]
        omegaAnalyticalArray[2,i] = -H/(1 + sigmaNorm**2)**2*(4*(-sigma1**2 - sigma2**2 + sigma3**2) + (1 - sigmaNorm**2)**2)/scObject.hub.IHubPntBc_B[2][2]

    plt.close("all")    # clear out earlier figures

    plt.figure()
    plt.clf()
    plt.plot(moduleOutput[:,0]*1e-9, moduleOutput[:,1], moduleOutput[:,0]*1e-9, moduleOutput[:,2], moduleOutput[:,0]*1e-9, moduleOutput[:,3])
    plt.plot(moduleOutput[index,0]*1e-9, moduleOutput[index,1],'bo')
    plt.plot(moduleOutput[index,0]*1e-9, sigmaGhost[0],'yo')
    plt.plot(moduleOutput[index-1,0]*1e-9, moduleOutput[index-1,1],'bo')
    plt.xlabel("Time (s)")
    plt.ylabel("MRPs")
    unitTestSupport.writeFigureLaTeX("scPlusMRPs", "Attitude of Spacecraft in MRPs", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(moduleOutput[index - 3: index + 3,0]*1e-9, moduleOutput[index - 3: index + 3,1],"b")
    plt.plot(moduleOutput[index-1,0]*1e-9, moduleOutput[index-1,1],'bo', label = "Basilisk " + r"$\sigma_{1,t-1}$")
    plt.plot(moduleOutput[index,0]*1e-9, moduleOutput[index,1],'ro', label = "Basilisk " + r"$\sigma_{1,t}$")
    plt.plot(moduleOutput[index,0]*1e-9, sigmaGhost[0],'ko', label = "Basilisk " + r"$\sigma_{1,0}$")
    plt.plot([moduleOutput[index-1,0]*1e-9, moduleOutput[index,0]*1e-9], [moduleOutput[index-1,1], sigmaGhost[0]],'--k')
    axes = plt.gca()
    axes.set_ylim([-0.5,0.5])
    plt.legend(loc ='upper right',numpoints = 1)
    plt.xlabel("Time (s)")
    plt.ylabel("MRPs")
    unitTestSupport.writeFigureLaTeX("scPlusMRPSwitching", "MRP Switching", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMomMag - rotAngMomMag[0])/rotAngMomMag[0])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInRotationalAngularMomentumRotationOnly", "Change in Rotational Angular Momentum Rotation Only", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(rotEnergy[:,0]*1e-9, (rotEnergy[:,1] - rotEnergy[0,1])/rotEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("scPlusChangeInRotationalEnergyRotationOnly", "Change in Rotational Energy Rotation Only", plt, r"width=0.8\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(omega_BNOutput[:,0]*1e-9,omega_BNOutput[:,1],label = r"$\omega_1$" + " Basilisk")
    plt.plot(omega_BNOutput[:,0]*1e-9,omega_BNOutput[:,2],label = r"$\omega_2$" + " Basilisk")
    plt.plot(omega_BNOutput[:,0]*1e-9,omega_BNOutput[:,3], label = r"$\omega_3$" + " Basilisk")
    plt.plot(timeArray*1e-9,omegaAnalyticalArray[0,:],'bo', label = r"$\omega_1$" + " BOE")
    plt.plot(timeArray*1e-9,omegaAnalyticalArray[1,:],'go', label = r"$\omega_2$" + " BOE")
    plt.plot(timeArray*1e-9,omegaAnalyticalArray[2,:],'ro', label = r"$\omega_3$" + " BOE")
    plt.xlabel("Time (s)")
    plt.ylabel("Angular Velocity (rad/s)")
    plt.legend(loc ='lower right',numpoints = 1, prop = {'size': 6.5})
    unitTestSupport.writeFigureLaTeX("scPlusBasiliskVsBOECalcForRotation", "Basilisk Vs BOE Calc For Rotation", plt, r"width=0.8\textwidth", path)
    if show_plots:
        plt.show()
        plt.close("all")

    moduleOutput = dataLog.sigma_BN
    accuracy = 1e-8
    for i in range(0,len(trueSigma)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(moduleOutput[-1],trueSigma[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed attitude unit test")

    accuracy = 1e-10
    for i in range(0,len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i],initialRotAngMom_N[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation and Rotation Integrated test failed rotational angular momentum unit test")

    for i in range(0,len(initialRotEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotEnergy[i],initialRotEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Rotation Integrated test failed rotational energy unit test")

    omegaArray = (numpy.delete(omegaArray, 0, 0)).transpose()
    omegaAnalyticalArray = omegaAnalyticalArray.transpose()
    for i in range(0,len(omegaAnalyticalArray)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(omegaArray[i],omegaAnalyticalArray[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Rotation Integrated test Rotational BOE unit test")

    accuracy = 1e-5
    if not unitTestSupport.isArrayEqualRelative(numpy.delete(sigmaAfterSwitch, 0,), sigmaAfterAnalytical,1,accuracy):
        testFailCount += 1
        testMessages.append("FAILED: Spacecraft Rotation Integrated test failed MRP Switching unit test")

    if testFailCount == 0:
        print("PASSED: " + "Spacecraft Rotation Integrated test")

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def SCTransBOE(show_plots):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    timeStep = 0.1
    testProcessRate = macros.sec2nano(timeStep)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Define conditions for the forces and times
    F1 = 3.
    F2 = -7.
    t1 = 3.
    t2 = 6.
    t3 = 10.

    # Add external force and torque
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    extFTObject.extTorquePntB_B = [[0], [0], [0]]
    extFTObject.extForce_B = [[F1], [0], [0]]
    scObject.addDynamicEffector(extFTObject)
    unitTestSim.AddModelToTask(unitTaskName, extFTObject)

    dataLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Define initial conditions of the spacecraft
    scObject.hub.mHub = 100
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    unitTestSim.InitializeSimulation()

    stopTime = t1
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    extFTObject.extTorquePntB_B = [[0], [0], [0]]
    extFTObject.extForce_B = [[0], [0], [0]]

    stopTime = t2
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    extFTObject.extTorquePntB_B = [[0], [0], [0]]
    extFTObject.extForce_B = [[F2], [0], [0]]

    stopTime = t3
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    r_BN_NOutput = addTimeColumn(dataLog.times(), dataLog.r_BN_N)
    v_BN_NOutput = addTimeColumn(dataLog.times(), dataLog.v_BN_N)

    # BOE calcs
    a1 = F1/scObject.hub.mHub
    a2 = F2/scObject.hub.mHub
    v1 = a1*t1
    v2 = v1
    v3 = v2 + a2*(t3-t2)
    x1 = 0.5*v1*t1
    x2 = x1 + v2*(t2-t1)
    t0 = t2 - v2/a2
    x3 = x2 + 0.5*v2*(t0-t2) + 0.5*v3*(t3-t0)

    # truth and Basilisk
    truthV = [v1, v2, v3]
    truthX = [x1, x2, x3]

    basiliskV = [v_BN_NOutput[int(t1/timeStep), 1], v_BN_NOutput[int(t2/timeStep), 1], v_BN_NOutput[int(t3/timeStep), 1]]
    basiliskX = [r_BN_NOutput[int(t1/timeStep), 1], r_BN_NOutput[int(t2/timeStep), 1], r_BN_NOutput[int(t3/timeStep), 1]]

    plt.close('all')
    plt.figure()
    plt.clf()
    plt.plot(r_BN_NOutput[:,0]*1e-9, r_BN_NOutput[:,1],'-b',label = "Basilisk")
    plt.plot([t1, t2, t3], [x1, x2, x3],'ro',markersize = 6.5,label = "BOE")
    plt.xlabel('time (s)')
    plt.ylabel('X (m)')
    plt.legend(loc ='upper left',numpoints = 1)
    PlotName = "scPlusTranslationPositionBOE"
    PlotTitle = "Translation Position BOE"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(v_BN_NOutput[:,0]*1e-9, v_BN_NOutput[:,1],'-b',label = "Basilisk")
    plt.plot([t1, t2, t3], [v1, v2, v3],'ro',markersize = 6.5,label = "BOE")
    plt.xlabel('time (s)')
    plt.ylabel('X velocity (m/s)')
    plt.legend(loc ='lower left',numpoints = 1)
    PlotName = "scPlusTranslationVelocityBOE"
    PlotTitle = "Translation Velocity BOE"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
    if show_plots:
        plt.show()
        plt.close('all')

    accuracy = 1e-10
    for i in range(0,3):
        # check a vector values
        if abs((truthX[i] - basiliskX[i])/truthX[i]) > accuracy:
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation BOE Integrated test failed pos unit test")

    for i in range(0,3):
        # check a vector values
        if abs((truthV[i] - basiliskV[i])/truthV[i]) > accuracy:
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Translation BOE Integrated test failed velocity unit test")

    if testFailCount == 0:
        print("PASSED: " + " Spacecraft Translation BOE Integrated Sim Test")

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def SCPointBVsPointC(show_plots):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.01)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Define location of force
    rFBc_B = numpy.array([0.3, -0.7, 0.4])
    force_B = numpy.array([0.5, 0.6, -0.2])
    torquePntC_B = numpy.cross(rFBc_B,force_B)

    # Add external force and torque
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    extFTObject.extTorquePntB_B = [[torquePntC_B[0]], [torquePntC_B[1]], [torquePntC_B[2]]]
    extFTObject.extForce_B = [[force_B[0]], [force_B[1]], [force_B[2]]]
    scObject.addDynamicEffector(extFTObject)
    unitTestSim.AddModelToTask(unitTaskName, extFTObject)

    dataLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Define initial conditions of the spacecraft
    scObject.hub.mHub = 100
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    scObject.hub.r_CN_NInit = [[0.0],	[0.0],	[0.0]]
    scObject.hub.v_CN_NInit = [[0.0],	[0.0],	[0.0]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.5], [-0.4], [0.7]]

    unitTestSim.InitializeSimulation()

    stopTime = 10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    r_CN_NOutput1 = addTimeColumn(dataLog.times(), dataLog.r_CN_N)
    sigma_BNOutput1 = addTimeColumn(dataLog.times(), dataLog.sigma_BN)

    ####################

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.01)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Define location of force
    rBcB_B = numpy.array([0.4, 0.5, 0.2])
    rFB_B = rBcB_B + rFBc_B
    torquePntB_B = numpy.cross(rFB_B,force_B)

    # Add external force and torque
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    extFTObject.extTorquePntB_B = [[torquePntB_B[0]], [torquePntB_B[1]], [torquePntB_B[2]]]
    extFTObject.extForce_B = [[force_B[0]], [force_B[1]], [force_B[2]]]
    scObject.addDynamicEffector(extFTObject)
    unitTestSim.AddModelToTask(unitTaskName, extFTObject)

    dataLog2 = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog2)

    # Define initial conditions of the spacecraft
    scObject.hub.mHub = 100
    scObject.hub.r_BcB_B = [[rBcB_B[0]], [rBcB_B[1]], [rBcB_B[2]]]
    scObject.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    scObject.hub.r_CN_NInit = [[0.0],	[0.0],	[0.0]]
    scObject.hub.v_CN_NInit = [[0.0],	[0.0],	[0.0]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.5], [-0.4], [0.7]]

    unitTestSim.InitializeSimulation()

    stopTime = 10.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    r_CN_NOutput2 = addTimeColumn(dataLog2.times(), dataLog2.r_CN_N)
    sigma_BNOutput2 = addTimeColumn(dataLog2.times(), dataLog2.sigma_BN)

    plt.figure()
    plt.clf()
    plt.plot(r_CN_NOutput1[:,0]*1e-9, r_CN_NOutput1[:,1], 'k', label = 'Torque About Point C', linewidth=3.0)
    plt.plot(r_CN_NOutput1[:,0]*1e-9,r_CN_NOutput1[:,2], 'k', r_CN_NOutput1[:,0]*1e-9, r_CN_NOutput1[:,3], 'k', linewidth=3.0)
    plt.plot(r_CN_NOutput2[:,0]*1e-9, r_CN_NOutput2[:,1], '--c', label = 'Torque About Point B')
    plt.plot(r_CN_NOutput2[:,0]*1e-9,r_CN_NOutput2[:,2], '--c', r_CN_NOutput2[:,0]*1e-9, r_CN_NOutput1[:,3], '--c')
    plt.xlabel('Time (s)')
    plt.ylabel('Inertial Position (m)')
    plt.legend(loc ='upper left', handlelength=3.5)
    PlotName = "scPlusPointBVsPointCTranslation"
    PlotTitle = "PointB Vs PointC Translation"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(sigma_BNOutput1[:,0]*1e-9, sigma_BNOutput1[:,1], 'k', label = 'Torque About Point C', linewidth=3.0)
    plt.plot(sigma_BNOutput1[:,0]*1e-9, sigma_BNOutput1[:,2], 'k', sigma_BNOutput1[:,0]*1e-9, sigma_BNOutput1[:,3], 'k', linewidth=3.0)
    plt.plot(sigma_BNOutput2[:,0]*1e-9, sigma_BNOutput2[:,1], '--c', label = 'Torque About Point B')
    plt.plot(sigma_BNOutput2[:,0]*1e-9, sigma_BNOutput2[:,2], '--c', sigma_BNOutput2[:,0]*1e-9, sigma_BNOutput2[:,3], '--c')
    plt.xlabel('Time (s)')
    plt.ylabel('MRPs')
    plt.legend(loc ='upper right', handlelength=3.5)
    PlotName = "scPlusPointBVsPointCAttitude"
    PlotTitle = "PointB Vs PointC Attitude"
    format = r"width=0.8\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)
    if show_plots:
        plt.show()
        plt.close('all')

    accuracy = 1e-8
    if not unitTestSupport.isArrayEqualRelative(r_CN_NOutput1[-1,1:4],r_CN_NOutput2[-1,1:4],3,accuracy):
        testFailCount += 1
        testMessages.append("FAILED: Spacecraft Point B Vs Point C test failed pos unit test")

    if not unitTestSupport.isArrayEqualRelative(sigma_BNOutput1[-1,1:4],sigma_BNOutput2[-1,1:4],3,accuracy):
        testFailCount += 1
        testMessages.append("FAILED: Spacecraft Point B Vs Point C test failed attitude unit test")

    if testFailCount == 0:
        print("PASSED: " + " Spacecraft Point B Vs Point C Integrated Sim Test")

    assert testFailCount < 1, testMessages

    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

@pytest.mark.parametrize("accuracy", [1e-3])
def scOptionalRef(show_plots, accuracy):
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    timeStep = 0.1
    testProcessRate = macros.sec2nano(timeStep)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    dataLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # add Earth
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # add gravity gradient effector
    ggEff = GravityGradientEffector.GravityGradientEffector()
    ggEff.ModelTag = scObject.ModelTag
    ggEff.addPlanetName(earth.planetName)
    scObject.addDynamicEffector(ggEff)
    unitTestSim.AddModelToTask(unitTaskName, ggEff)

    # Define initial conditions of the spacecraft
    scObject.hub.mHub = 100
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    scObject.hub.r_CN_NInit = [[7000000.0],	[0.0],	[0.0]]
    scObject.hub.v_CN_NInit = [[7000.0],	[0.0],	[0.0]]
    scObject.hub.sigma_BNInit = [[0.5], [0.4], [0.3]]
    scObject.hub.sigma_BNInit = [[0.], [0.], [1.0]]
    scObject.hub.omega_BN_BInit = [[0.5], [-0.4], [0.7]]

    # write attitude reference message
    attRef = messaging.AttRefMsgPayload()
    attRef.sigma_RN = [0.0, 0.0, 1.0]
    attRef.omega_RN_N = [0.0001, 0.0002, 0.0003]
    attRefMsg = messaging.AttRefMsg().write(attRef)
    scObject.attRefInMsg.subscribeTo(attRefMsg)

    # write translational reference message
    transRef = messaging.TransRefMsgPayload()
    transRef.r_RN_N = [1000000, 2000000, 3000000]
    transRef.v_RN_N = [2000, 3000, 4000]
    transRefMsg = messaging.TransRefMsg().write(transRef)
    scObject.transRefInMsg.subscribeTo(transRefMsg)

    unitTestSim.InitializeSimulation()

    stopTime = 0.2
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    omegaOut = dataLog.omega_BN_B
    sigmaOut = dataLog.sigma_BN
    r_RN_Out = dataLog.r_BN_N
    v_RN_Out = dataLog.v_BN_N

    trueSigma = [attRef.sigma_RN]*3
    trueOmega = [[-0.0001, -0.0002, 0.0003]]*3
    truer_RN_N = [transRef.r_RN_N]*3
    truev_RN_N = [transRef.v_RN_N]*3

    testFailCount, testMessages = unitTestSupport.compareArray(trueSigma, sigmaOut,
                                                               accuracy, "sigma_BN",
                                                               testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareArray(trueOmega, omegaOut,
                                                               accuracy, "omega_BN_B",
                                                               testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareArray(truer_RN_N, r_RN_Out,
                                                               accuracy, "r_RN_N",
                                                               testFailCount, testMessages)
    testFailCount, testMessages = unitTestSupport.compareArray(truev_RN_N, v_RN_Out,
                                                               accuracy, "v_RN_N",
                                                               testFailCount, testMessages)

    if testFailCount == 0:
        print("PASSED: scPlus setting optional reference state input message")
    else:
        print("FAILED: scPlus setting optional reference state input message")

    assert testFailCount < 1, testMessages

    return [testFailCount, ''.join(testMessages)]

def scAccumDV():
    """Module Unit Test"""
    # The __tracebackhide__ setting influences pytest showing of tracebacks:
    # the mrp_steering_tracking() function will not be shown unless the
    # --fulltrace command line option is specified.
    __tracebackhide__ = True

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty list to store test log messages

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    timeStep = 0.1
    testProcessRate = macros.sec2nano(timeStep)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    dataLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # add Earth
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    # Define initial conditions of the spacecraft
    scObject.hub.mHub = 100
    scObject.hub.r_BcB_B = [[0.0], [100.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[500, 0.0, 0.0], [0.0, 200, 0.0], [0.0, 0.0, 300]]
    scObject.hub.r_CN_NInit = [[-7000000.0],	[0.0],	[0.0]]
    scObject.hub.v_CN_NInit = [[0.0],	[7000.0],	[0.0]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [numpy.pi/180]]

    unitTestSim.InitializeSimulation()

    stopTime = 0.5
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    dataAccumDV_CN_B = dataLog.TotalAccumDVBdy
    dataAccumDV_BN_B = dataLog.TotalAccumDV_BN_B

    accuracy = 1e-10
    truth_dataAccumDV_CN_B = [0.0, 0.0, 0.0]
    v_r = numpy.cross(numpy.array(scObject.hub.omega_BN_BInit).T, -numpy.array(scObject.hub.r_BcB_B).T)[0]
    truth_dataAccumDV_BN_B = numpy.zeros(3)
    for i in range(len(dataLog.times())-1):
        if not unitTestSupport.isArrayEqual(dataAccumDV_CN_B[i+1],truth_dataAccumDV_CN_B,3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Point C Accumulated DV test failed pos unit test")

        truth_dataAccumDV_BN_B += numpy.matmul(RigidBodyKinematics.MRP2C(dataLog.sigma_BN[i+1]),
                                              numpy.matmul(RigidBodyKinematics.MRP2C(dataLog.sigma_BN[i+1]).T,v_r) -
                                              numpy.matmul(RigidBodyKinematics.MRP2C(dataLog.sigma_BN[i]).T,v_r))
        if not unitTestSupport.isArrayEqual(dataAccumDV_BN_B[i+1],truth_dataAccumDV_BN_B,3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Spacecraft Point B Accumulated DV test failed pos unit test")

    if testFailCount == 0:
        print("PASSED: Spacecraft Accumulated DV tests with offset CoM")

    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    # scAttRef(True, 1e-3)
    # SCTranslation(True)
    # SCTransAndRotation(True)
    # SCRotation(True)
    # SCTransBOE(True)
    # SCPointBVsPointC(True)
    scOptionalRef(True, 0.001)