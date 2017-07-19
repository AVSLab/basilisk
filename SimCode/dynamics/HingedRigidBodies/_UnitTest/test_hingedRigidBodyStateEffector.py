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

if __name__ == "__main__":
    test_hingedRigidBodyNoGravityDamping(True)
