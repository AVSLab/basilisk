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
import sys, os, inspect
import numpy
import matplotlib.pyplot as plt
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import nHingedRigidBodyStateEffector
from Basilisk.simulation import gravityEffector
from Basilisk.utilities import macros

@pytest.mark.parametrize("useFlag, testCase", [
    (False,'NoGravity'),
    (False,'Gravity')
])

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

def test_nHingedRigidBodyAllTest(show_plots,useFlag,testCase):
    [testResults, testMessage] = nHingedRigidBody(show_plots,useFlag,testCase)
    assert testResults < 1, testMessage

def nHingedRigidBody(show_plots,useFlag,testCase):
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
    testProcessRate = macros.sec2nano(0.0001)  # update process rate update time
    plottingRate = 0.01
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    unitTestSim.effector1 = nHingedRigidBodyStateEffector.NHingedRigidBodyStateEffector()
    unitTestSim.effector2 = nHingedRigidBodyStateEffector.NHingedRigidBodyStateEffector()
    unitTestSim.panel = nHingedRigidBodyStateEffector.HingedPanel()

    unitTestSim.effector1.r_HB_B = [[0.5], [0.0], [1.0]]
    unitTestSim.effector1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.effector1.nameOfThetaState = "nHingedRigidBody1Theta"
    unitTestSim.effector1.nameOfThetaDotState = "nHingedRigidBody1ThetaDot"

    unitTestSim.effector2.r_HB_B = [[-0.5], [0.0], [1.0]]
    unitTestSim.effector2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.effector2.nameOfThetaState = "nHingedRigidBody2Theta"
    unitTestSim.effector2.nameOfThetaDotState = "nHingedRigidBody2ThetaDot"

    # Define Variable for a panel
    unitTestSim.panel.mass = 50.0
    unitTestSim.panel.IPntS_S = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]
    unitTestSim.panel.d = 0.75
    unitTestSim.panel.k = 500.0
    unitTestSim.panel.c = 0.0
    unitTestSim.panel.thetaInit = 5*numpy.pi/180.0
    unitTestSim.panel.thetaDotInit = 0.0
    unitTestSim.panel.theta_0 = 0.0

    # Add panels to effector 4 to one 3 to the other
    unitTestSim.effector1.addHingedPanel(unitTestSim.panel)
    unitTestSim.panel.thetaInit = 0.0
    unitTestSim.effector1.addHingedPanel(unitTestSim.panel)
    unitTestSim.effector1.addHingedPanel(unitTestSim.panel)
    unitTestSim.effector1.addHingedPanel(unitTestSim.panel)
    # 3 on effector 2
    unitTestSim.effector2.addHingedPanel(unitTestSim.panel)
    unitTestSim.effector2.addHingedPanel(unitTestSim.panel)
    unitTestSim.effector2.addHingedPanel(unitTestSim.panel)
    
    # Add effector to spaceCraft
    scObject.addStateEffector(unitTestSim.effector1)
    scObject.addStateEffector(unitTestSim.effector2)

    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[0.1], [-0.4], [0.3]]
    scObject.hub.v_CN_NInit = [[-0.2], [0.5], [0.1]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    if testCase == 'Gravity':
        unitTestSim.earthGravBody = gravityEffector.GravBodyData()
        unitTestSim.earthGravBody.bodyInMsgName = "earth_planet_data"
        unitTestSim.earthGravBody.outputMsgName = "earth_display_frame_data"
        unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
        unitTestSim.earthGravBody.isCentralBody = True
        unitTestSim.earthGravBody.useSphericalHarmParams = False
        scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([unitTestSim.earthGravBody])
        scObject.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
        scObject.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]

    unitTestSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)

    unitTestSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)
    
    unitTestSim.InitializeSimulation()

    unitTestSim.AddVariableForLogging("spacecraftBody.dynManager.getStateObject('nHingedRigidBody1Theta').getState()", plottingRate, 0, 3, 'double')
    unitTestSim.AddVariableForLogging("spacecraftBody.dynManager.getStateObject('nHingedRigidBody2Theta').getState()", plottingRate, 0, 2, 'double')

    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbEnergy", plottingRate, 0, 0, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", plottingRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", plottingRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotEnergy", plottingRate, 0, 0, 'double')

    stopTime = 1.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    nHingedRigidBody1ThetasOut = unitTestSim.GetLogVariableData("spacecraftBody.dynManager.getStateObject('nHingedRigidBody1Theta').getState()")
    nHingedRigidBody2ThetasOut = unitTestSim.GetLogVariableData("spacecraftBody.dynManager.getStateObject('nHingedRigidBody2Theta').getState()")

    orbEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbEnergy")
    orbAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totOrbAngMomPntN_N")
    rotAngMom_N = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotAngMomPntC_N")
    rotEnergy = unitTestSim.GetLogVariableData(scObject.ModelTag + ".totRotEnergy")

    initialOrbAngMom_N = [[orbAngMom_N[0, 1], orbAngMom_N[0, 2], orbAngMom_N[0, 3]]]

    finalOrbAngMom = [orbAngMom_N[-1]]

    initialRotAngMom_N = [[rotAngMom_N[0, 1], rotAngMom_N[0, 2], rotAngMom_N[0, 3]]]

    finalRotAngMom = [rotAngMom_N[-1]]

    initialOrbEnergy = [[orbEnergy[0, 1]]]

    finalOrbEnergy = [orbEnergy[-1]]

    initialRotEnergy = [[rotEnergy[0, 1]]]

    finalRotEnergy = [rotEnergy[-1]]

    plt.close("all")

    plt.figure()
    plt.clf()
    plt.plot(orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,1] - orbAngMom_N[0,1])/orbAngMom_N[0,1], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,2] - orbAngMom_N[0,2])/orbAngMom_N[0,2], orbAngMom_N[:,0]*1e-9, (orbAngMom_N[:,3] - orbAngMom_N[0,3])/orbAngMom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("ChangeInOrbitalAngularMomentum" + testCase, "Change in Orbital Angular Momentum " + testCase, plt, "width=0.8\\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("ChangeInOrbitalEnergy" + testCase, "Change in Orbital Energy " + testCase, plt, "width=0.8\\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("ChangeInRotationalAngularMomentum" + testCase, "Change in Rotational Angular Momentum " + testCase, plt, "width=0.8\\textwidth", path)
    plt.figure()
    plt.clf()
    plt.plot(rotEnergy[:,0]*1e-9, (rotEnergy[:,1] - rotEnergy[0,1])/rotEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    unitTestSupport.writeFigureLaTeX("ChangeInRotationalEnergy" + testCase, "Change in Rotational Energy " + testCase, plt, "width=0.8\\textwidth", path)

    plt.figure()
    plt.clf()
    plt.plot(nHingedRigidBody1ThetasOut[:,0]*1e-9, nHingedRigidBody1ThetasOut[:,1]*180/numpy.pi,'-b')
    plt.xlabel('Time (s)')
    plt.ylabel('Panel 1 Theta 1 (deg)')

    plt.figure()
    plt.clf()
    plt.plot(nHingedRigidBody1ThetasOut[:,0]*1e-9, nHingedRigidBody1ThetasOut[:,2]*180/numpy.pi,'-b')
    plt.xlabel('Time (s)')
    plt.ylabel('Panel 1 Theta 2 (deg)')

    plt.figure()
    plt.clf()
    plt.plot(nHingedRigidBody1ThetasOut[:,0]*1e-9, nHingedRigidBody1ThetasOut[:,3]*180/numpy.pi,'-b')
    plt.xlabel('Time (s)')
    plt.ylabel('Panel 1 Theta 3 (deg)')

    plt.figure()
    plt.clf()
    plt.plot(nHingedRigidBody1ThetasOut[:,0]*1e-9, nHingedRigidBody1ThetasOut[:,4]*180/numpy.pi,'-b')
    plt.xlabel('Time (s)')
    plt.ylabel('Panel 1 Theta 4 (deg)')

    plt.figure()
    plt.clf()
    plt.plot(nHingedRigidBody2ThetasOut[:,0]*1e-9, nHingedRigidBody2ThetasOut[:,1]*180/numpy.pi,'-b')
    plt.xlabel('Time (s)')
    plt.ylabel('Panel 2 Theta 1 (deg)')

    plt.figure()
    plt.clf()
    plt.plot(nHingedRigidBody2ThetasOut[:,0]*1e-9, nHingedRigidBody2ThetasOut[:,2]*180/numpy.pi,'-b')
    plt.xlabel('Time (s)')
    plt.ylabel('Panel 2 Theta 2 (deg)')

    plt.figure()
    plt.clf()
    plt.plot(nHingedRigidBody2ThetasOut[:,0]*1e-9, nHingedRigidBody2ThetasOut[:,3]*180/numpy.pi,'-b')
    plt.xlabel('Time (s)')
    plt.ylabel('Panel 2 Theta 3 (deg)')

    if show_plots:
        plt.show()
        plt.close("all")

    accuracy = 1e-10

    for i in range(0, len(initialOrbAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbAngMom[i], initialOrbAngMom_N[i], 3, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: N Hinged Rigid Body integrated test failed orbital angular momentum unit test")

    for i in range(0, len(initialRotAngMom_N)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotAngMom[i], initialRotAngMom_N[i], 3, accuracy):
            testFailCount += 1
            testMessages.append(
                "FAILED: N Hinged Rigid Body integrated test failed rotational angular momentum unit test")

    for i in range(0, len(initialRotEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalRotEnergy[i], initialRotEnergy[i], 1, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: N Hinged Rigid Body integrated test failed rotational energy unit test")

    for i in range(0, len(initialOrbEnergy)):
        # check a vector values
        if not unitTestSupport.isArrayEqualRelative(finalOrbEnergy[i], initialOrbEnergy[i], 1, accuracy):
            testFailCount += 1
            testMessages.append("FAILED: N Hinged Rigid Body integrated test failed orbital energy unit test")

    if testFailCount == 0:
        print "PASSED: " + " N Hinged Rigid Body integrated test"

    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    nHingedRigidBody(True, False, "Gravity")
