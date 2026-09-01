
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
import sys

import matplotlib.pyplot as plt
import numpy
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('SimCode')
sys.path.append(splitPath[0] + '/modules')
sys.path.append(splitPath[0] + '/PythonModules')

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import simHelpers
from Basilisk.simulation import spacecraft
from Basilisk.simulation import dualHingedRigidBodyStateEffector
from Basilisk.simulation import extForceTorque
from Basilisk.simulation import nHingedRigidBodyStateEffector
from Basilisk.simulation import gravityEffector
from Basilisk.utilities import macros
from Basilisk.utilities import pythonVariableLogger


@pytest.mark.parametrize("testCase", [
    ('NoGravity'),
    ('Gravity')
])

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

def test_nHingedRigidBodyAllTest(show_plots, testCase):
    """
In this integrated test there are two hinged rigid bodies connected to the spacecraft hub, one with 4 \
interconnected panels and one with 3 interconnected panels.  Depending on the scenario, there are different \
success criteria.  Each scenario checks the conservation of orbital angular momentum, the conservation of orbital \
energy, the conservation of rotational angular momentum and the conservation of rotational energy.


**Test Parameters:**

- testCase: [string]
    defines whether or not the gravity is included in this test.

**Description of Variables Being Tested**

This test checks the conservation of the spacecraft orbital angular momentum, the rotational angular momentum, \
the orbital energy and the rotational energy.

**Test Descriptions:**

testCase == 'Gravity'
In this test the simulation is placed into orbit around Earth with point gravity and has no damping in \
the hinged rigid bodies.

testCase == 'NoGravity'
In this test, the spacecraft is placed in free space (no gravity) and has no damping in the hinged rigid bodies.

The following figures show the conservation of the quantities described in the success criteria for each scenario. \
The conservation plots are all relative difference plots. All conservation plots show integration error which is the \
desired result. In the python test these values are automatically checked therefore when the tests pass, these \
values have all been confirmed to be conserved.

    """
    [testResults, testMessage] = nHingedRigidBody(show_plots, testCase)
    assert testResults < 1, testMessage


def test_nHingedRigidBodyOutputMessagesMatchDual():
    """
    Verify both output-message vectors against the equivalent dual-hinged model.

    A two-panel N-hinged effector and a dual-hinged effector are given identical geometry, mass
    properties, and initial states on the same spacecraft. The hub starts with a non-identity
    attitude and nonzero translational and angular velocity so that every inertial transformation
    contributes. The test runs for two steps and compares each panel's angle, angle rate, inertial
    position, inertial velocity, attitude, and angular velocity.
    """
    timeStep = 0.01  # [s]
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.SetProgressBar(False)
    unitTestSim.CreateNewProcess("TestProcess").addTask(
        unitTestSim.CreateNewTask("unitTask", macros.sec2nano(timeStep)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg*m^2]
    scObject.hub.r_CN_NInit = [[1000.0], [-2000.0], [3000.0]]  # [m]
    scObject.hub.v_CN_NInit = [[-2.0], [3.0], [1.0]]  # [m/s]
    scObject.hub.sigma_BNInit = [[0.1], [-0.2], [0.05]]  # [-]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.2]]  # [rad/s]

    panelMass = 50.0  # [kg]
    panelHalfLength = 0.75  # [m]
    panelInertia = [[50.0, 0.0, 0.0], [0.0, 25.0, 0.0], [0.0, 0.0, 25.0]]  # [kg*m^2]
    springConstant = 100.0  # [N*m/rad]
    dampingCoefficient = 0.0  # [N*m*s/rad]
    thetaInit = [0.1, -0.2]  # [rad]
    thetaDotInit = [-0.01, 0.02]  # [rad/s]
    hingePosition_B = [[0.5], [-1.5], [-0.5]]  # [m]
    dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]  # [-]

    nHinged = nHingedRigidBodyStateEffector.NHingedRigidBodyStateEffector()
    nHinged.r_HB_B = hingePosition_B
    nHinged.dcm_HB = dcm_HB
    for panelIndex in range(2):
        panel = nHingedRigidBodyStateEffector.HingedPanel()
        panel.mass = panelMass
        panel.d = panelHalfLength
        panel.k = springConstant
        panel.c = dampingCoefficient
        panel.IPntS_S = panelInertia
        panel.thetaInit = thetaInit[panelIndex]
        panel.thetaDotInit = thetaDotInit[panelIndex]
        panel.theta_0 = 0.0  # [rad]
        nHinged.addHingedPanel(panel)

    dualHinged = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    dualHinged.mass1 = panelMass
    dualHinged.mass2 = panelMass
    dualHinged.d1 = panelHalfLength
    dualHinged.d2 = panelHalfLength
    dualHinged.l1 = 2.0 * panelHalfLength  # [m]
    dualHinged.thetaH2S1 = 0.0  # [rad] the N hinged chain carries no fixed offset between hinges
    dualHinged.k1 = springConstant
    dualHinged.k2 = springConstant
    dualHinged.c1 = dampingCoefficient
    dualHinged.c2 = dampingCoefficient
    dualHinged.IPntS1_S1 = panelInertia
    dualHinged.IPntS2_S2 = panelInertia
    dualHinged.theta1Init = thetaInit[0]
    dualHinged.theta2Init = thetaInit[1]
    dualHinged.theta1DotInit = thetaDotInit[0]
    dualHinged.theta2DotInit = thetaDotInit[1]
    dualHinged.r_H1B_B = hingePosition_B
    dualHinged.dcm_H1B = dcm_HB

    scObject.addStateEffector(nHinged)
    scObject.addStateEffector(dualHinged)
    unitTestSim.AddModelToTask("unitTask", scObject)

    nStateRecorders = [message.recorder() for message in nHinged.nHingedRigidBodyOutMsgs]
    nConfigRecorders = [message.recorder() for message in nHinged.nHingedRigidBodyConfigLogOutMsgs]
    dualStateRecorders = [message.recorder() for message in dualHinged.dualHingedRigidBodyOutMsgs]
    dualConfigRecorders = [message.recorder() for message in dualHinged.dualHingedRigidBodyConfigLogOutMsgs]
    for recorder in nStateRecorders + nConfigRecorders + dualStateRecorders + dualConfigRecorders:
        unitTestSim.AddModelToTask("unitTask", recorder)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(2.0 * timeStep))
    unitTestSim.ExecuteSimulation()

    accuracy = 1e-13
    for panelIndex in range(2):
        numpy.testing.assert_allclose(
            nStateRecorders[panelIndex].theta, dualStateRecorders[panelIndex].theta,
            rtol=0.0, atol=accuracy)
        numpy.testing.assert_allclose(
            nStateRecorders[panelIndex].thetaDot, dualStateRecorders[panelIndex].thetaDot,
            rtol=0.0, atol=accuracy)
        for fieldName in ("r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"):
            numpy.testing.assert_allclose(
                getattr(nConfigRecorders[panelIndex], fieldName),
                getattr(dualConfigRecorders[panelIndex], fieldName),
                rtol=0.0, atol=accuracy)


@pytest.mark.parametrize("segment, shouldRaise", [(1, False), (3, False), (0, True), (4, True)])
def test_nHingedRigidBodyDynamicEffectorSegmentBounds(segment, shouldRaise):
    """
    Verify that dynamic effectors can attach only to existing panels.

    A three-panel chain accepts its first and last panel numbers and rejects the adjacent values
    outside the valid one-based range.

    **Test Parameters:**

    - segment: [int]
        one-based panel number supplied to ``addDynamicEffector``
    - shouldRaise: [bool]
        whether the panel number is outside the valid range
    """
    effector = nHingedRigidBodyStateEffector.NHingedRigidBodyStateEffector()
    for _ in range(3):
        effector.addHingedPanel(nHingedRigidBodyStateEffector.HingedPanel())

    child = extForceTorque.ExtForceTorque()
    if shouldRaise:
        with pytest.raises(BasiliskError, match="non-existent panel"):
            effector.addDynamicEffector(child, segment)
    else:
        effector.addDynamicEffector(child, segment)


@pytest.mark.parametrize("chain, expectedError", [
    ('UnequalMass', "same mass and the same hinge"),
    ('UnequalDistance', "same mass and the same hinge"),
    ('NoPanels', "at least one hinged panel"),
    ('MasslessPanels', "panel mass must be greater than 0"),
    ('UnequalInertia', None),
])
def test_nHingedRigidBodyPanelValidation(chain, expectedError):
    """
The equations of motion are derived for a chain of identical panels, factoring one panel mass and
one hinge to center of mass distance out of the sums that run over the chain. An uneven chain
integrates without complaint and a massless one reaches the hub states as NaN, so initialization
must reject both. The inertia is not factored out, so dissimilar inertia tensors must initialize.

**Test Parameters:**

- chain: [string]
    panel-chain configuration to initialize
- expectedError: [string]
    text the rejection message must carry, or None when the chain must initialize
    """
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg*m^2]

    effector = nHingedRigidBodyStateEffector.NHingedRigidBodyStateEffector()
    if chain != 'NoPanels':
        for factor in [1.0, 2.0]:
            panel = nHingedRigidBodyStateEffector.HingedPanel()
            massScale = 0.0 if chain == 'MasslessPanels' else (factor if chain == 'UnequalMass' else 1.0)  # [-]
            panel.mass = 50.0 * massScale  # [kg]
            panel.d = 0.75 * (factor if chain == 'UnequalDistance' else 1.0)  # [m]
            panel.k = 500.0  # [N*m/rad]
            panel.c = 0.0  # [N*m*s/rad]
            inertiaScale = factor if chain == 'UnequalInertia' else 1.0  # [-]
            panel.IPntS_S = (inertiaScale * numpy.diag([50.0, 25.0, 25.0])).tolist()  # [kg*m^2]
            effector.addHingedPanel(panel)
    scObject.addStateEffector(effector)

    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.CreateNewProcess("TestProcess").addTask(
        unitTestSim.CreateNewTask("unitTask", macros.sec2nano(0.0001)))
    unitTestSim.AddModelToTask("unitTask", scObject)

    if expectedError is not None:
        with pytest.raises(BasiliskError, match=expectedError):
            unitTestSim.InitializeSimulation()
    else:
        unitTestSim.InitializeSimulation()


def nHingedRigidBody(show_plots, testCase):
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
    testProcessRate = macros.sec2nano(0.0001)  # update process rate update time
    plottingRate = 0.01
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    unitTestSim.effector1 = nHingedRigidBodyStateEffector.NHingedRigidBodyStateEffector()
    unitTestSim.effector2 = nHingedRigidBodyStateEffector.NHingedRigidBodyStateEffector()
    unitTestSim.panel = nHingedRigidBodyStateEffector.HingedPanel()

    unitTestSim.effector1.r_HB_B = [[0.5], [0.0], [1.0]]
    unitTestSim.effector1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]

    unitTestSim.effector2.r_HB_B = [[-0.5], [0.0], [1.0]]
    unitTestSim.effector2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

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
        unitTestSim.earthGravBody.planetName = "earth_planet_data"
        unitTestSim.earthGravBody.mu = 0.3986004415E+15 # meters!
        unitTestSim.earthGravBody.isCentralBody = True
        scObject.gravField.gravBodies = spacecraft.GravBodyVector([unitTestSim.earthGravBody])
        scObject.hub.r_CN_NInit = [[-4020338.690396649],	[7490566.741852513],	[5248299.211589362]]
        scObject.hub.v_CN_NInit = [[-5199.77710904224],	[-3436.681645356935],	[1041.576797498721]]

    dataLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    scObjectLog = scObject.logger(["totOrbEnergy", "totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    theta1Name = unitTestSim.effector1.nameOfThetaState
    theta2Name = unitTestSim.effector2.nameOfThetaState
    stateLog = pythonVariableLogger.PythonVariableLogger({
        "theta1": lambda _: scObject.dynManager.getStateObject(theta1Name).getState(),
        "theta2": lambda _: scObject.dynManager.getStateObject(theta2Name).getState(),
    })
    unitTestSim.AddModelToTask(unitTaskName, stateLog)

    unitTestSim.InitializeSimulation()

    stopTime = 1.0
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    nHingedRigidBody1ThetasOut = simHelpers.addTimeColumn(stateLog.times(), stateLog.theta1)
    nHingedRigidBody2ThetasOut = simHelpers.addTimeColumn(stateLog.times(), stateLog.theta2)

    orbEnergy = simHelpers.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbEnergy)
    orbAngMom_N = simHelpers.addTimeColumn(scObjectLog.times(), scObjectLog.totOrbAngMomPntN_N)
    rotAngMom_N = simHelpers.addTimeColumn(scObjectLog.times(), scObjectLog.totRotAngMomPntC_N)
    rotEnergy = simHelpers.addTimeColumn(scObjectLog.times(), scObjectLog.totRotEnergy)

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
    plt.suptitle("Change in Orbital Angular Momentum ")
    plt.figure()
    plt.clf()
    plt.plot(orbEnergy[:,0]*1e-9, (orbEnergy[:,1] - orbEnergy[0,1])/orbEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    plt.suptitle("Change in Orbital Energy ")
    plt.figure()
    plt.clf()
    plt.plot(rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,1] - rotAngMom_N[0,1])/rotAngMom_N[0,1], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,2] - rotAngMom_N[0,2])/rotAngMom_N[0,2], rotAngMom_N[:,0]*1e-9, (rotAngMom_N[:,3] - rotAngMom_N[0,3])/rotAngMom_N[0,3])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    plt.suptitle("Change in Rotational Angular Momentum")
    plt.figure()
    plt.clf()
    plt.plot(rotEnergy[:,0]*1e-9, (rotEnergy[:,1] - rotEnergy[0,1])/rotEnergy[0,1])
    plt.xlabel("Time (s)")
    plt.ylabel("Relative Difference")
    plt.suptitle("Change in Rotational Energy")

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

    finalOrbAngMom = numpy.delete(finalOrbAngMom, 0, axis=1)  # remove time column
    finalRotAngMom = numpy.delete(finalRotAngMom, 0, axis=1)  # remove time column
    finalRotEnergy = numpy.delete(finalRotEnergy, 0, axis=1)  # remove time column
    finalOrbEnergy = numpy.delete(finalOrbEnergy, 0, axis=1)  # remove time column

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
        print("PASSED: " + " N Hinged Rigid Body integrated test")
        print("Error tolerance for all tests was" + str(accuracy))


    assert testFailCount < 1, testMessages
    # return fail count and join into a single string all messages in the list
    # testMessage
    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    nHingedRigidBody(True,  "Gravity")
