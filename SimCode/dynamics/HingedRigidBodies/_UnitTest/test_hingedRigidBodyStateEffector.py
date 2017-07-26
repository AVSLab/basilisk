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
import spacecraftPlus
import hingedRigidBodyStateEffector
import macros
import gravityEffector
import ExtForceTorque

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_
def hingedRigidBodyAllTest(show_plots):
    [testResults, testMessage] = test_hingedRigidBodyGravity(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = test_hingedRigidBodyNoGravity(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = test_hingedRigidBodyNoGravityDamping(show_plots)
    assert testResults < 1, testMessage
    [testResults, testMessage] = test_hingedRigidBodyTransient(show_plots)
    assert testResults < 1, testMessage

def test_hingedRigidBodyGravity(show_plots):
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

    # Create two hinged rigid bodies
    unitTestSim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    unitTestSim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unitTestSim.panel1.mass = 100.0
    unitTestSim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel1.d = 1.5
    unitTestSim.panel1.k = 100.0
    unitTestSim.panel1.c = 0.0
    unitTestSim.panel1.r_HB_B = [[0.5], [0.0], [1.0]]
    unitTestSim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel1.nameOfThetaState = "hingedRigidBodyTheta1"
    unitTestSim.panel1.nameOfThetaDotState = "hingedRigidBodyThetaDot1"
    unitTestSim.panel1.thetaInit = 5*numpy.pi/180.0
    unitTestSim.panel1.thetaDotInit = 0.0

    # Define Variables for panel 2
    unitTestSim.panel2.mass = 100.0
    unitTestSim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel2.d = 1.5
    unitTestSim.panel2.k = 100.0
    unitTestSim.panel2.c = 0.0
    unitTestSim.panel2.r_HB_B = [[-0.5], [0.0], [1.0]]
    unitTestSim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel2.nameOfThetaState = "hingedRigidBodyTheta2"
    unitTestSim.panel2.nameOfThetaDotState = "hingedRigidBodyThetaDot2"
    unitTestSim.panel2.thetaInit = 0.0
    unitTestSim.panel2.thetaDotInit = 0.0

    # Add panels to spaceCraft
    scObject.addStateEffector(unitTestSim.panel1)
    scObject.addStateEffector(unitTestSim.panel2)

    # Define mass properties of the rigid part of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Add Earth gravity to the sim
    unitTestSim.earthGravBody = gravityEffector.GravBodyData()
    unitTestSim.earthGravBody.bodyInMsgName = "earth_planet_data"
    unitTestSim.earthGravBody.outputMsgName = "earth_display_frame_data"
    unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
    unitTestSim.earthGravBody.isCentralBody = True
    unitTestSim.earthGravBody.useSphericalHarmParams = False
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([unitTestSim.earthGravBody])

    # Log the spacecraft state message
    unitTestSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Add energy and momentum variables to log
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbEnergy", testProcessRate, 0, 0, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotEnergy", testProcessRate, 0, 0, 'double')

    stopTime = 2.5
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    sigmaOut = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName+'.sigma_BN',range(3))

    orbEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbEnergy")
    orbAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
    rotAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")
    rotEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotEnergy")

    dataSigma = [sigmaOut[-1]]

    trueSigma = [[0.06170318243240492, -0.07089090074412899, 0.06409500412692531]]

    initialOrbAngMom_N = [[orbAngMom_N[0,1], orbAngMom_N[0,2], orbAngMom_N[0,3]]]

    finalOrbAngMom = [orbAngMom_N[-1]]

    initialRotAngMom_N = [[rotAngMom_N[0,1], rotAngMom_N[0,2], rotAngMom_N[0,3]]]

    finalRotAngMom = [rotAngMom_N[-1]]

    initialOrbEnergy = [[orbEnergy[0,1]]]

    finalOrbEnergy = [orbEnergy[-1]]

    initialRotEnergy = [[rotEnergy[0,1]]]

    finalRotEnergy = [rotEnergy[-1]]

    plt.figure()
    plt.clf()
    plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    PlotName = "ChangeInOrbitalAngularMomentumGravity"
    PlotTitle = "Change in Orbital Angular Momentum with Gravity"
    format = "width=0.8\\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    PlotName = "ChangeInOrbitalEnergyGravity"
    PlotTitle = "Change in Orbital Energy with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    PlotName = "ChangeInRotationalAngularMomentumGravity"
    PlotTitle = "Change In Rotational Angular Momentum with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rotEnergy[:,0]*1e-9, (rotEnergy[:,1] - rotEnergy[0,1])/rotEnergy[0,1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    PlotName = "ChangeInRotationalEnergyGravity"
    PlotTitle = "Change In Rotational Energy with Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    if show_plots == True:
        plt.show()

    accuracy = 1e-10
    for i in range(0,len(trueSigma)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataSigma[i],trueSigma[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED:  Hinged Rigid Body integrated test failed gravity attitude test")

    for i in range(0,len(initialOrbAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i],initialOrbAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Hinged Rigid Body integrated test failed gravity orbital angular momentum unit test")

    for i in range(0,len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i],initialRotAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Hinged Rigid Body integrated test failed gravity rotational angular momentum unit test")

    for i in range(0,len(initialRotEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotEnergy[i],initialRotEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Hinged Rigid Body integrated test failed gravity rotational energy unit test")

    for i in range(0,len(initialOrbEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i],initialOrbEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Hinged Rigid Body integrated test failed gravity orbital energy unit test")

    if testFailCount == 0:
        print "PASSED: " + " Hinged Rigid Body gravity integrated test"

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def test_hingedRigidBodyNoGravity(show_plots):
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

    unitTestSim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    unitTestSim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unitTestSim.panel1.mass = 100.0
    unitTestSim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel1.d = 1.5
    unitTestSim.panel1.k = 100.0
    unitTestSim.panel1.c = 0.0
    unitTestSim.panel1.r_HB_B = [[0.5], [0.0], [1.0]]
    unitTestSim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel1.nameOfThetaState = "hingedRigidBodyTheta1"
    unitTestSim.panel1.nameOfThetaDotState = "hingedRigidBodyThetaDot1"
    unitTestSim.panel1.thetaInit = 5*numpy.pi/180.0
    unitTestSim.panel1.thetaDotInit = 0.0

    # Define Variables for panel 2
    unitTestSim.panel2.mass = 100.0
    unitTestSim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel2.d = 1.5
    unitTestSim.panel2.k = 100.0
    unitTestSim.panel2.c = 0.0
    unitTestSim.panel2.r_HB_B = [[-0.5], [0.0], [1.0]]
    unitTestSim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel2.nameOfThetaState = "hingedRigidBodyTheta2"
    unitTestSim.panel2.nameOfThetaDotState = "hingedRigidBodyThetaDot2"
    unitTestSim.panel2.thetaInit = 0.0
    unitTestSim.panel2.thetaDotInit = 0.0

    # Add panels to spaceCraft
    scObject.addStateEffector(unitTestSim.panel1)
    scObject.addStateEffector(unitTestSim.panel2)

    # Define mass properties of the rigid part of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[0.1], [-0.4], [0.3]]
    scObject.hub.v_CN_NInit = [[-0.2], [0.5], [0.1]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    unitTestSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)
    
    unitTestSim.InitializeSimulation()

    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbEnergy", testProcessRate, 0, 0, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotEnergy", testProcessRate, 0, 0, 'double')

    stopTime = 2.5
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    sigmaOut = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName+'.sigma_BN',range(3))
    rOut_BN_N = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N',range(3))
    vOut_CN_N = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName+'.v_CN_N',range(3))

    orbEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbEnergy")
    orbAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
    rotAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")
    rotEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotEnergy")

    # Get the last sigma and position
    dataSigma = [sigmaOut[-1]]
    dataPos = [rOut_BN_N[-1]]

    truePos = [[-0.15832794740648992, 1.122481716747217, -0.37975995949382907]]

    trueSigma = [[0.06170318243240492, -0.07089090074412899, 0.06409500412692531]]

    initialOrbAngMom_N = [[orbAngMom_N[0,1], orbAngMom_N[0,2], orbAngMom_N[0,3]]]

    finalOrbAngMom = [orbAngMom_N[-1]]

    initialRotAngMom_N = [[rotAngMom_N[0,1], rotAngMom_N[0,2], rotAngMom_N[0,3]]]

    finalRotAngMom = [rotAngMom_N[-1]]

    initialOrbEnergy = [[orbEnergy[0,1]]]

    finalOrbEnergy = [orbEnergy[-1]]

    initialRotEnergy = [[rotEnergy[0,1]]]

    finalRotEnergy = [rotEnergy[-1]]

    plt.figure()
    plt.clf()
    plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    PlotName = "ChangeInOrbitalAngularMomentumNoGravity"
    PlotTitle = "Change in Orbital Angular Momentum No Gravity"
    format = "width=0.8\\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    PlotName = "ChangeInOrbitalEnergyNoGravity"
    PlotTitle = "Change in Orbital Energy No Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    PlotName = "ChangeInRotationalAngularMomentumNoGravity"
    PlotTitle = "Change In Rotational Angular Momentum No Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rotEnergy[:,0]*1e-9, (rotEnergy[:,1] - rotEnergy[0,1])/rotEnergy[0,1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    PlotName = "ChangeInRotationalEnergyNoGravity"
    PlotTitle = "Change In Rotational Energy No Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(vOut_CN_N[:,0]*1e-9, vOut_CN_N[:,1], vOut_CN_N[:,0]*1e-9, vOut_CN_N[:,2], vOut_CN_N[:,0]*1e-9, vOut_CN_N[:,3])
    plt.xlabel('time (s)')
    plt.ylabel('m/s')
    PlotName = "VelocityOfCenterOfMassNoGravity"
    PlotTitle = "Velocity Of Center Of Mass No Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(vOut_CN_N[:,0]*1e-9, (vOut_CN_N[:,1] - vOut_CN_N[0,1])/vOut_CN_N[0,1], vOut_CN_N[:,0]*1e-9, (vOut_CN_N[:,2] - vOut_CN_N[0,2])/vOut_CN_N[0,2], vOut_CN_N[:,0]*1e-9, (vOut_CN_N[:,3] - vOut_CN_N[0,3])/vOut_CN_N[0,3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    PlotName = "ChangeInVelocityOfCenterOfMassNoGravity"
    PlotTitle = "Change In Velocity Of Center Of Mass No Gravity"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    if show_plots == True:
        plt.show()

    accuracy = 1e-10
    for i in range(0,len(truePos)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataPos[i],truePos[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED:  Hinged Rigid Body integrated test failed position test")

    for i in range(0,len(trueSigma)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(dataSigma[i],trueSigma[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED:  Hinged Rigid Body integrated test failed attitude test")

    for i in range(0,len(initialOrbAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i],initialOrbAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Hinged Rigid Body integrated test failed orbital angular momentum unit test")

    for i in range(0,len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i],initialRotAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Hinged Rigid Body integrated test failed rotational angular momentum unit test")

    for i in range(0,len(initialRotEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotEnergy[i],initialRotEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Hinged Rigid Body integrated test failed rotational energy unit test")

    for i in range(0,len(initialOrbEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i],initialOrbEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Hinged Rigid Body integrated test failed orbital energy unit test")

    if testFailCount == 0:
        print "PASSED: " + " Hinged Rigid Body integrated test"

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def test_hingedRigidBodyNoGravityDamping(show_plots):
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

    unitTestSim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    unitTestSim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unitTestSim.panel1.mass = 100.0
    unitTestSim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel1.d = 1.5
    unitTestSim.panel1.k = 100.0
    unitTestSim.panel1.c = 6.0
    unitTestSim.panel1.r_HB_B = [[0.5], [0.0], [1.0]]
    unitTestSim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel1.nameOfThetaState = "hingedRigidBodyTheta1"
    unitTestSim.panel1.nameOfThetaDotState = "hingedRigidBodyThetaDot1"
    unitTestSim.panel1.thetaInit = 5*numpy.pi/180.0
    unitTestSim.panel1.thetaDotInit = 0.0

    # Define Variables for panel 2
    unitTestSim.panel2.mass = 100.0
    unitTestSim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel2.d = 1.5
    unitTestSim.panel2.k = 100.0
    unitTestSim.panel2.c = 7.0
    unitTestSim.panel2.r_HB_B = [[-0.5], [0.0], [1.0]]
    unitTestSim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.panel2.nameOfThetaState = "hingedRigidBodyTheta2"
    unitTestSim.panel2.nameOfThetaDotState = "hingedRigidBodyThetaDot2"
    unitTestSim.panel2.thetaInit = 0.0
    unitTestSim.panel2.thetaDotInit = 0.0

    # Add panels to spaceCraft
    scObject.addStateEffector(unitTestSim.panel1)
    scObject.addStateEffector(unitTestSim.panel2)

    # Define mass properties of the rigid part of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[0.1], [-0.4], [0.3]]
    scObject.hub.v_CN_NInit = [[-0.2], [0.5], [0.1]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    unitTestSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)

    unitTestSim.InitializeSimulation()

    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbEnergy", testProcessRate, 0, 0, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", testProcessRate, 0, 2, 'double')

    stopTime = 2.5
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    vOut_CN_N = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName+'.v_CN_N',range(3))

    orbEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbEnergy")
    orbAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
    rotAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")

    initialOrbAngMom_N = [[orbAngMom_N[0,1], orbAngMom_N[0,2], orbAngMom_N[0,3]]]

    finalOrbAngMom = [orbAngMom_N[-1]]

    initialRotAngMom_N = [[rotAngMom_N[0,1], rotAngMom_N[0,2], rotAngMom_N[0,3]]]

    finalRotAngMom = [rotAngMom_N[-1]]

    initialOrbEnergy = [[orbEnergy[0,1]]]

    finalOrbEnergy = [orbEnergy[-1]]

    plt.figure()
    plt.clf()
    plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    PlotName = "ChangeInOrbitalAngularMomentumNoGravityDamping"
    PlotTitle = "Change in Orbital Angular Momentum No Gravity with Damping"
    format = "width=0.8\\textwidth"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    PlotName = "ChangeInOrbitalEnergyNoGravityDamping"
    PlotTitle = "Change in Orbital Energy No Gravity with Damping"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    PlotName = "ChangeInRotationalAngularMomentumNoGravityDamping"
    PlotTitle = "Change In Rotational Angular Momentum No Gravity with Damping"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(vOut_CN_N[:,0]*1e-9, vOut_CN_N[:,1], vOut_CN_N[:,0]*1e-9, vOut_CN_N[:,2], vOut_CN_N[:,0]*1e-9, vOut_CN_N[:,3])
    plt.xlabel('time (s)')
    plt.ylabel('m/s')
    PlotName = "VelocityOfCenterOfMassNoGravityDamping"
    PlotTitle = "Velocity Of Center Of Mass No Gravity with Damping"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    plt.figure()
    plt.clf()
    plt.plot(vOut_CN_N[:,0]*1e-9, (vOut_CN_N[:,1] - vOut_CN_N[0,1])/vOut_CN_N[0,1], vOut_CN_N[:,0]*1e-9, (vOut_CN_N[:,2] - vOut_CN_N[0,2])/vOut_CN_N[0,2], vOut_CN_N[:,0]*1e-9, (vOut_CN_N[:,3] - vOut_CN_N[0,3])/vOut_CN_N[0,3])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    PlotName = "ChangeInVelocityOfCenterOfMassNoGravityDamping"
    PlotTitle = "Change In Velocity Of Center Of Mass No Gravity with Damping"
    unitTestSupport.writeFigureLaTeX(PlotName, PlotTitle, plt, format, path)

    if show_plots == True:
        plt.show()

    accuracy = 1e-10
    for i in range(0,len(initialOrbAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i],initialOrbAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Hinged Rigid Body integrated test with damping failed orbital angular momentum unit test")

    for i in range(0,len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i],initialRotAngMom_N[i],3,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Hinged Rigid Body integrated test with damping failed rotational angular momentum unit test")

    for i in range(0,len(initialOrbEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i],initialOrbEnergy[i],1,accuracy):
            testFailCount += 1
            testMessages.append("FAILED: Hinged Rigid Body integrated test with damping failed orbital energy unit test")

    if testFailCount == 0:
        print "PASSED: " + " Hinged Rigid Body integrated test with damping"

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def test_hingedRigidBodyTransient(show_plots):
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
    stepSize = 0.1
    testProcessRate = macros.sec2nano(stepSize)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    unitTestSim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    unitTestSim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    unitTestSim.panel1.mass = 100.0
    unitTestSim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel1.d = 1.5
    unitTestSim.panel1.k = 100.0
    unitTestSim.panel1.c = 75
    unitTestSim.panel1.r_HB_B = [[0.5], [1.0], [0.0]]
    unitTestSim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]]
    unitTestSim.panel1.nameOfThetaState = "hingedRigidBodyTheta1"
    unitTestSim.panel1.nameOfThetaDotState = "hingedRigidBodyThetaDot1"
    unitTestSim.panel1.thetaInit = 0.0
    unitTestSim.panel1.thetaDotInit = 0.0

    # Define Variables for panel 2
    unitTestSim.panel2.mass = 100.0
    unitTestSim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    unitTestSim.panel2.d = 1.5
    unitTestSim.panel2.k = 100.0
    unitTestSim.panel2.c = 75
    unitTestSim.panel2.r_HB_B = [[-0.5], [1.0], [0.0]]
    unitTestSim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
    unitTestSim.panel2.nameOfThetaState = "hingedRigidBodyTheta2"
    unitTestSim.panel2.nameOfThetaDotState = "hingedRigidBodyThetaDot2"
    unitTestSim.panel2.thetaInit = 0.0
    unitTestSim.panel2.thetaDotInit = 0.0

    # Add panels to spaceCraft
    scObject.addStateEffector(unitTestSim.panel1)
    scObject.addStateEffector(unitTestSim.panel2)

    # Add external force and torque
    extFTObject = ExtForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "externalDisturbance"
    extFTObject.extTorquePntB_B = [[0], [0], [0]]
    extFTObject.extForce_B = [[0], [1], [0]]
    scObject.addDynamicEffector(extFTObject)
    unitTestSim.AddModelToTask(unitTaskName, extFTObject)

    # Define mass properties of the rigid part of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]
    scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    unitTestSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)

    unitTestSim.InitializeSimulation()

    unitTestSim.AddVariableForLogging("spacecraftBody.dynManager.getStateObject('hingedRigidBodyTheta1').getState()", testProcessRate, 0, 0, 'double')
    unitTestSim.AddVariableForLogging("spacecraftBody.dynManager.getStateObject('hingedRigidBodyTheta2').getState()", testProcessRate, 0, 0, 'double')

    stopTime = 60.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    theta1Out = unitTestSim.GetLogVariableData("spacecraftBody.dynManager.getStateObject('hingedRigidBodyTheta1').getState()")
    theta2Out = unitTestSim.GetLogVariableData("spacecraftBody.dynManager.getStateObject('hingedRigidBodyTheta2').getState()")

    rOut_BN_N = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N',range(3))
    sigmaOut_BN = unitTestSim.pullMessageLogData(scObject.scStateOutMsgName+'.sigma_BN',range(3))
    thetaOut = 4.0*numpy.arctan(sigmaOut_BN[:,3])

    # Developing the lagrangian result
    # Define initial values
    spacecraft = spacecraftClass()
    spacecraft.hub.mass = scObject.hub.mHub
    spacecraft.hub.Inertia = scObject.hub.IHubPntBc_B[2][2]
    # Define variables for panel1
    spacecraft.panel1.mass = unitTestSim.panel1.mass
    spacecraft.panel1.Inertia = unitTestSim.panel1.IPntS_S[1][1]
    spacecraft.panel1.Rhinge = numpy.linalg.norm(numpy.asarray(unitTestSim.panel1.r_HB_B))
    spacecraft.panel1.beta = numpy.arctan2(unitTestSim.panel1.r_HB_B[1][0],unitTestSim.panel1.r_HB_B[0][0])
    spacecraft.panel1.thetaH = 0.0
    spacecraft.panel1.d = unitTestSim.panel1.d
    spacecraft.panel1.k = unitTestSim.panel1.k
    spacecraft.panel1.c = unitTestSim.panel1.c
    # Define variables for panel2
    spacecraft.panel2.mass = unitTestSim.panel2.mass
    spacecraft.panel2.Inertia = unitTestSim.panel2.IPntS_S[1][1]
    spacecraft.panel2.Rhinge = numpy.linalg.norm(numpy.asarray(unitTestSim.panel2.r_HB_B))
    spacecraft.panel2.beta = numpy.arctan2(unitTestSim.panel2.r_HB_B[1][0],unitTestSim.panel2.r_HB_B[0][0])
    spacecraft.panel2.thetaH = numpy.pi
    spacecraft.panel2.d = unitTestSim.panel2.d
    spacecraft.panel2.k = unitTestSim.panel2.k
    spacecraft.panel2.c = unitTestSim.panel2.c
    # Define body force and torque
    spacecraft.xThrust_B = 0.0
    spacecraft.yThrust_B = extFTObject.extForce_B[1][0]
    spacecraft.Torque = 0.0

    # Define initial conditions of the sim
    time = numpy.arange(0.0,stopTime + stepSize,stepSize).flatten()
    x0 = numpy.zeros(10)
    x0[3] = unitTestSim.panel1.thetaInit
    x0[4] = -unitTestSim.panel2.thetaInit

    X = numpy.zeros((len(x0),len(time)))
    X[:,0] = x0
    for j in range (1,(len(time))):
        X[:, j] = rk4(planarFlexFunction, X[:, j-1], stepSize, time[j-1], spacecraft)

    # Find steady state value
    variablesIn = boxAndWingParameters()
    variablesIn.k = spacecraft.panel1.k
    variablesIn.d = spacecraft.panel1.d
    variablesIn.F = spacecraft.yThrust_B
    variablesIn.mSC = spacecraft.hub.mass + spacecraft.panel1.mass + spacecraft.panel2.mass
    variablesIn.mSP = spacecraft.panel1.mass
    thetaSSGuess = -0.01
    tolerance = 1e-10
    thetaSS = newtonRapshon(boxAndWingsFandFPrime,thetaSSGuess,tolerance,variablesIn)

    plt.figure()
    plt.clf()
    plt.plot(time, X[0,:],'-b',label = "Lagrangian")
    plt.plot(rOut_BN_N[:,0]*1e-9, (rOut_BN_N[:,1]-rOut_BN_N[0,1]),'-r',label = "Basilsik")
    plt.legend(loc ='upper right')
    plt.title('X Position')

    plt.figure()
    plt.clf()
    plt.plot(time, X[1,:],'-b',label = "Lagrangian")
    plt.plot(rOut_BN_N[:,0]*1e-9, (rOut_BN_N[:,2]-rOut_BN_N[0,2]),'r',label = "Basilsik")
    plt.legend(loc ='upper right')
    plt.title('Y Position')

    plt.figure()
    plt.clf()
    plt.plot(time, X[2,:],'-b',label = "Lagrangian")
    plt.plot(sigmaOut_BN[:,0]*1e-9, thetaOut,'-r',label = "Basilsik")
    plt.legend(loc ='upper right')
    plt.title('Theta')

    plt.figure()
    plt.clf()
    plt.plot(time, X[3,:],'-b',label = "Lagrangian")
    plt.plot(theta1Out[:,0]*1e-9, theta1Out[:,1],'-r',label = "Basilsik")
    plt.plot(theta1Out[-1,0]*1e-9, thetaSS,'ok',label = "BOE Calculation")
    plt.legend(loc ='upper right',numpoints = 1)
    plt.title('Theta1')

    plt.figure()
    plt.clf()
    plt.plot(time, -X[4,:],'-b',label = "Lagrangian")
    plt.plot(theta2Out[:,0]*1e-9, theta2Out[:,1],'-r',label = "Basilsik")
    plt.legend(loc ='upper right')
    plt.title('Theta2')

    plt.show(show_plots)

    accuracy = 1e-10
    # for i in range(0,len(initialOrbAngMom_N)):
    #     # check a vector values
    #     if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i],initialOrbAngMom_N[i],3,accuracy):
    #         testFailCount += 1
    #         testMessages.append("FAILED: Hinged Rigid Body integrated test Transient Test failed position comparison ")

    if testFailCount == 0:
        print "PASSED: " + " Hinged Rigid Body Transient Integrated test"

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

def planarFlexFunction(x, t, variables):
    theta = x[2]
    theta1 = x[3]
    theta2 = x[4]
    xHubDot = x[5]
    yHubDot = x[6]
    thetaDot = x[7]
    theta1Dot = x[8]
    theta2Dot = x[9]
    # Define variables for hub
    mHub = variables.hub.mass
    IHub = variables.hub.Inertia
    # Define variables for panel1
    mSP1 = variables.panel1.mass
    ISP1 = variables.panel1.Inertia
    Rhinge1 = variables.panel1.Rhinge
    beta1 = variables.panel1.beta
    thetaH1 = variables.panel1.thetaH
    d1 = variables.panel1.d
    k1 = variables.panel1.k
    c1 = variables.panel1.c
    # Define variables for panel2
    mSP2 = variables.panel2.mass
    ISP2 = variables.panel2.Inertia
    Rhinge2 = variables.panel2.Rhinge
    beta2 = variables.panel2.beta
    thetaH2 = variables.panel2.thetaH
    d2 = variables.panel2.d
    k2 = variables.panel2.k
    c2 = variables.panel2.c
    Tx_B = variables.xThrust_B
    Ty_B = variables.yThrust_B
    Torque = variables.Torque

    # Convert Tx_B and Ty_B to the inertial frame
    dcm_BN = numpy.array([[numpy.cos(theta), numpy.sin(theta)],
                          [-numpy.sin(theta), numpy.cos(theta)]])
    Thrust_N = numpy.dot(dcm_BN.transpose(),numpy.array([[Tx_B],[Ty_B]]))
    Tx = Thrust_N[0,0]
    Ty = Thrust_N[1,0]

    matrixA = numpy.zeros((5,5))
    vectorB = numpy.zeros(5)
    # Populate X Translation Equation
    matrixA[0,0] = 1.0
    matrixA[0,1] = 0.0
    matrixA[0,2] = -1/(mHub + mSP1 + mSP2)*(mSP1*Rhinge1*numpy.sin(beta1 + theta) + mSP2*Rhinge2*numpy.sin(beta2 + theta) + d1*mSP1*numpy.sin(thetaH1 + theta + theta1) + d2*mSP2*numpy.sin(thetaH2 + theta + theta2))
    matrixA[0,3] = -1/(mHub + mSP1 + mSP2)*(d1*mSP1*numpy.sin(thetaH1 + theta + theta1))
    matrixA[0,4] = -1/(mHub + mSP1 + mSP2)*(d2*mSP2*numpy.sin(thetaH2 + theta + theta2))
    vectorB[0] = 1/(mHub + mSP1 + mSP2)*(Tx + mSP1*Rhinge1*numpy.cos(beta1 + theta)*thetaDot**2 + mSP2*Rhinge2*numpy.cos(beta2 + theta)*thetaDot**2 + d1*mSP1*numpy.cos(thetaH1 + theta + theta1)*thetaDot**2 + d2*mSP2*numpy.cos(thetaH2 + theta + theta2)*thetaDot**2
               + 2*d1*mSP1*numpy.cos(thetaH1 + theta + theta1)*thetaDot*theta1Dot + d1*mSP1*numpy.cos(thetaH1 + theta + theta1)*theta1Dot**2 + 2*d2*mSP2*numpy.cos(thetaH2 + theta + theta2)*thetaDot*theta2Dot
               + d2*mSP2*numpy.cos(thetaH2 + theta + theta2)*theta2Dot**2)
    # Populate Y Translation Equation
    matrixA[1,0] = 0.0
    matrixA[1,1] = 1.0
    matrixA[1,2] = 1/(mHub + mSP1 + mSP2)*(mSP1*Rhinge1*numpy.cos(beta1 + theta) + mSP2*Rhinge2*numpy.cos(beta2 + theta) + d1*mSP1*numpy.cos(thetaH1 + theta + theta1) + d2*mSP2*numpy.cos(thetaH2 + theta + theta2))
    matrixA[1,3] = 1/(mHub + mSP1 + mSP2)*(d1*mSP1*numpy.cos(thetaH1 + theta + theta1))
    matrixA[1,4] = 1/(mHub + mSP1 + mSP2)*(d2*mSP2*numpy.cos(thetaH2 + theta + theta2))
    vectorB[1] = 1/(mHub + mSP1 + mSP2)*(Ty + mSP1*Rhinge1*numpy.sin(beta1 + theta)*thetaDot**2 + mSP2*Rhinge2*numpy.sin(beta2 + theta)*thetaDot**2 + d1*mSP1*numpy.sin(thetaH1 + theta + theta1)*thetaDot**2 + d2*mSP2*numpy.sin(thetaH2 + theta + theta2)*thetaDot**2
               + 2*d1*mSP1*numpy.sin(thetaH1 + theta + theta1)*thetaDot*theta1Dot + d1*mSP1*numpy.sin(thetaH1 + theta + theta1)*theta1Dot**2 + 2*d2*mSP2*numpy.sin(thetaH2 + theta + theta2)*thetaDot*theta2Dot
               + d2*mSP2*numpy.sin(thetaH2 + theta + theta2)*theta2Dot**2)
    # Populate theta Equation
    matrixA[2,0] = -1/(IHub + ISP1 + ISP2 + d1**2*mSP1 + d2**2*mSP2 + mSP1*Rhinge1**2 + mSP2*Rhinge2**2 + 2*d1*mSP1*Rhinge1*numpy.cos(beta1 - thetaH1 - theta1) + 2*d2*mSP2*Rhinge2*numpy.cos(beta2 - thetaH2 - theta2))*(mSP1*Rhinge1*numpy.sin(beta1 + theta)
                 + mSP2*Rhinge2*numpy.sin(beta2 + theta) + d1*mSP1*numpy.sin(thetaH1 + theta + theta1) + d2*mSP2*numpy.sin(thetaH2 + theta + theta2))
    matrixA[2,1] = 1/(IHub + ISP1 + ISP2 + d1**2*mSP1 + d2**2*mSP2 + mSP1*Rhinge1**2 + mSP2*Rhinge2**2 + 2*d1*mSP1*Rhinge1*numpy.cos(beta1 - thetaH1 - theta1) + 2*d2*mSP2*Rhinge2*numpy.cos(beta2 - thetaH2 - theta2))*(mSP1*Rhinge1*numpy.cos(beta1 + theta)
                 + mSP2*Rhinge2*numpy.cos(beta2 + theta) + d1*mSP1*numpy.cos(thetaH1 + theta + theta1) + d2*mSP2*numpy.cos(thetaH2 + theta + theta2))
    matrixA[2,2] = 1.0
    matrixA[2,3] = 1/(IHub + ISP1 + ISP2 + d1**2*mSP1 + d2**2*mSP2 + mSP1*Rhinge1**2 + mSP2*Rhinge2**2 + 2*d1*mSP1*Rhinge1*numpy.cos(beta1 - thetaH1 - theta1) + 2*d2*mSP2*Rhinge2*numpy.cos(beta2 - thetaH2 - theta2))*(ISP1
                 + d1**2*mSP1 + d1*mSP1*Rhinge1*numpy.cos(beta1 - thetaH1 - theta1))
    matrixA[2,4] = 1/(IHub + ISP1 + ISP2 + d1**2*mSP1 + d2**2*mSP2 + mSP1*Rhinge1**2 + mSP2*Rhinge2**2 + 2*d1*mSP1*Rhinge1*numpy.cos(beta1 - thetaH1 - theta1) + 2*d2*mSP2*Rhinge2*numpy.cos(beta2 - thetaH2 - theta2))*(ISP2
                 + d2**2*mSP2 + d2*mSP2*Rhinge2*numpy.cos(beta2 - thetaH2 - theta2))
    vectorB[2] = 1/(IHub + ISP1 + ISP2 + d1**2*mSP1 + d2**2*mSP2 + mSP1*Rhinge1**2 + mSP2*Rhinge2**2 + 2*d1*mSP1*Rhinge1*numpy.cos(beta1 - thetaH1 - theta1) + 2*d2*mSP2*Rhinge2*numpy.cos(beta2 - thetaH2 - theta2))*(Torque
              - 2*d1*mSP1*Rhinge1*numpy.sin(beta1 - thetaH1 - theta1)*thetaDot*theta1Dot - d1*mSP1*Rhinge1*numpy.sin(beta1 - thetaH1 - theta1)*theta1Dot**2
              - 2*d2*mSP2*Rhinge2*numpy.sin(beta2 - thetaH2 - theta2)*thetaDot*theta2Dot - d2*mSP2*Rhinge2*numpy.sin(beta2 - thetaH2 - theta2)*theta2Dot**2)
    # Populate theta1 Equation
    matrixA[3,0] = -1/(ISP1 + d1**2*mSP1)*(d1*mSP1*numpy.sin(thetaH1 + theta + theta1))
    matrixA[3,1] = 1/(ISP1 + d1**2*mSP1)*(d1*mSP1*numpy.cos(thetaH1 + theta + theta1))
    matrixA[3,2] = 1/(ISP1 + d1**2*mSP1)*(ISP1 + d1**2*mSP1 + d1*mSP1*Rhinge1*numpy.cos(beta1 - thetaH1 - theta1))
    matrixA[3,3] = 1.0
    matrixA[3,4] = 0.0
    vectorB[3] = 1/(ISP1 + d1**2*mSP1)*(-k1*theta1 + d1*mSP1*Rhinge1*numpy.sin(beta1 - thetaH1 - theta1)*thetaDot**2 - c1*theta1Dot)
    # Populate theta2 Equation
    matrixA[4,0] = -1/(ISP2 + d2**2*mSP2)*(d2*mSP2*numpy.sin(thetaH2 + theta + theta2))
    matrixA[4,1] = 1/(ISP2 + d2**2*mSP2)*(d2*mSP2*numpy.cos(thetaH2 + theta + theta2))
    matrixA[4,2] = 1/(ISP2 + d2**2*mSP2)*(ISP2 + d2**2*mSP2 + d2*mSP2*Rhinge2*numpy.cos(beta2 - thetaH2 - theta2))
    matrixA[4,3] = 0.0
    matrixA[4,4] = 1.0
    vectorB[4] = 1/(ISP2 + d2**2*mSP2)*(-k2*theta2 + d2*mSP2*Rhinge2*numpy.sin(beta2 - thetaH2 - theta2)*thetaDot**2 - c2*theta2Dot)

    Xdot = numpy.zeros(len(x))
    # Populate Trivial derivatives
    Xdot[0] = xHubDot
    Xdot[1] = yHubDot
    Xdot[2] = thetaDot
    Xdot[3] = theta1Dot
    Xdot[4] = theta2Dot
    # Calculate nontrivial derivatives
    result = numpy.dot(numpy.linalg.inv(matrixA),vectorB)
    Xdot[5:10] = result

    return Xdot

def rk4(Fn, X, h, t, varargin):
    k1 = h*Fn(X, t, varargin)
    k2 = h*Fn(X+k1/2, t+h/2, varargin)
    k3 = h*Fn(X+k2/2, t+h/2, varargin)
    k4 = h*Fn(X+k3, t+h, varargin)
    Z = X + (k1 + 2*k2 + 2*k3 + k4)/6.0
    return Z

class solarPanel:
    mass = 0.0
    Inertia = 0.0
    Rhinge = 0.0
    beta = 0.0
    thetaH = 0.0
    d = 0.0
    k = 0.0
    c = 0.0

class hubClass:
    mass = 0.0
    Inertia = 0.0

class spacecraftClass:
    panel1 = solarPanel()
    panel2 = solarPanel()
    hub = hubClass()
    xThrust_B = 0.0
    yThrust_B = 0.0
    Torque = 0.0

def newtonRapshon(funcAndDervi,guess,tolerance,variables):
    xOld = guess
    for i in range(1,101):
        fx, fPrimex = funcAndDervi(xOld, variables)
        xNew = xOld - fx/fPrimex
        if abs(xNew - xOld) < tolerance:
            break
        xOld = xNew
    return xNew

def boxAndWingsFandFPrime(theta,variables):
    # Define variables
    F = variables.F
    mSC = variables.mSC
    k = variables.k
    mSP = variables.mSP
    d = variables.d
    aSP = F/mSC
    fX = k*theta + mSP*aSP*d*numpy.cos(theta)
    fPrimeX = k - mSP*aSP*d*numpy.sin(theta)
    return fX, fPrimeX

class boxAndWingParameters:
    F = 0
    mSC = 0
    k = 0
    mSP = 0
    d = 0

if __name__ == "__main__":
    test_hingedRigidBodyTransient(True)
