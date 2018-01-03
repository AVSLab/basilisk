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
import matplotlib.pyplot as plt

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import nHingedRigidBodyStateEffector
from Basilisk.utilities import macros

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

def test_nHingedRigidBodyAllTest(show_plots):
    [testResults, testMessage] = nHingedRigidBodyNoGravity(show_plots)
    assert testResults < 1, testMessage

def nHingedRigidBodyNoGravity(show_plots):
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
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    unitTestSim.effector1 = nHingedRigidBodyStateEffector.NHingedRigidBodyStateEffector()
    unitTestSim.effector2 = nHingedRigidBodyStateEffector.NHingedRigidBodyStateEffector()
    unitTestSim.panel1 = nHingedRigidBodyStateEffector.HingedPanel()
    unitTestSim.panel2 = nHingedRigidBodyStateEffector.HingedPanel()

    unitTestSim.effector1.r_HB_B = [[0.5], [0.0], [1.0]]
    unitTestSim.effector1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.effector1.nameOfThetaState = "hingedRigidBody1Theta"
    unitTestSim.effector1.nameOfThetaDotState = "hingedRigidBody1ThetaDot"

    unitTestSim.effector2.r_HB_B = [[-0.5], [0.0], [1.0]]
    unitTestSim.effector2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    unitTestSim.effector2.nameOfThetaState = "hingedRigidBody2Theta"
    unitTestSim.effector2.nameOfThetaDotState = "hingedRigidBody2ThetaDot"

    # Define Variable for panel 1
    unitTestSim.panel1.mass = 50.0
    unitTestSim.panel1.IPntS_S = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]
    unitTestSim.panel1.d = 0.75
    unitTestSim.panel1.k = 100.0
    unitTestSim.panel1.c = 0.0
    unitTestSim.panel1.thetaInit = 5*numpy.pi/180.0
    unitTestSim.panel1.thetaDotInit = 0.0
    unitTestSim.panel1.theta_0 = 0.0

    # Define Variables for panel 2
    unitTestSim.panel2.mass = 50.0
    unitTestSim.panel2.IPntS_S = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]
    unitTestSim.panel2.d = 0.75
    unitTestSim.panel2.k = 100.0
    unitTestSim.panel2.c = 0.0
    unitTestSim.panel2.thetaInit = 0.0
    unitTestSim.panel2.thetaDotInit = 0.0
    unitTestSim.panel2.theta_0 = 0.0

    # Add panels to effector
    unitTestSim.effector1.addHingedPanel(unitTestSim.panel1)
    unitTestSim.effector1.addHingedPanel(unitTestSim.panel2)
    unitTestSim.effector1.addHingedPanel(unitTestSim.panel2)
    unitTestSim.effector1.addHingedPanel(unitTestSim.panel1)
    unitTestSim.effector2.addHingedPanel(unitTestSim.panel1)
    unitTestSim.effector2.addHingedPanel(unitTestSim.panel2)
    unitTestSim.effector2.addHingedPanel(unitTestSim.panel2)
    unitTestSim.effector2.addHingedPanel(unitTestSim.panel1)
    
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

    unitTestSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, testProcessRate)
    
    unitTestSim.InitializeSimulation()

    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbEnergy", testProcessRate, 0, 0, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totOrbAngMomPntN_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotAngMomPntC_N", testProcessRate, 0, 2, 'double')
    unitTestSim.AddVariableForLogging(scObject.ModelTag + ".totRotEnergy", testProcessRate, 0, 0, 'double')

    stopTime = 1.0
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

    initialOrbAngMom_N = [[orbAngMom_N[0, 1], orbAngMom_N[0, 2], orbAngMom_N[0, 3]]]

    finalOrbAngMom = [orbAngMom_N[-1]]

    initialRotAngMom_N = [[rotAngMom_N[0, 1], rotAngMom_N[0, 2], rotAngMom_N[0, 3]]]

    finalRotAngMom = [rotAngMom_N[-1]]

    initialOrbEnergy = [[orbEnergy[0, 1]]]

    finalOrbEnergy = [orbEnergy[-1]]

    initialRotEnergy = [[rotEnergy[0, 1]]]

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

    plt.show(show_plots)
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
    nHingedRigidBodyNoGravity(True)
