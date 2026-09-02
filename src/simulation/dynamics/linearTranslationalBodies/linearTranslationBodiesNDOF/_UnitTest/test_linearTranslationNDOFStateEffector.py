# ISC License
#
# Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


#
#   Unit Test Script
#   Module Name:        linearTranslationNDOF
#   Author:             Peter Johnson
#   Creation Date:      March 7, 2024
#

import inspect
import os

import numpy as np
import pytest
import numpy
import matplotlib.pyplot as plt

# plt.rcParams['text.usetex'] = True

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import (
    SimulationBaseClass,
    macros,
)
from Basilisk.simulation import spacecraft, linearTranslationNDOFStateEffector, gravityEffector
from Basilisk.simulation import linearTranslationOneDOFStateEffector
from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError


def randomValidInertia():
    r"""Generate a random diagonal inertia tensor that is physically realizable."""
    secondMoments = np.random.uniform(2.5, 50.0, 3)  # [kg m^2]
    principalInertias = np.array([
        secondMoments[1] + secondMoments[2],
        secondMoments[0] + secondMoments[2],
        secondMoments[0] + secondMoments[1]
    ])  # [kg m^2]
    return np.diag(principalInertias)


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail() # need to update how the RW states are defined
# provide a unique test method name, starting with test_

@pytest.mark.parametrize("function", ["translatingBodyNoInput"
    , "translatingBodyLockAxis"
    , "translatingBodyCommandedForce"])
def test_translatingBody(show_plots, function):
    r"""
    **Validation Test Description**

    This unit test sets up a spacecraft with four single-axis translating rigid bodies attached to a rigid hub. Each
    translating body's center of mass is off-center from the translating axis and the position of the axis is arbitrary.
    The scenario includes gravity acting on both the spacecraft and the effector.

    **Description of Variables Being Tested**

    In this file we are checking the principles of conservation of energy and angular momentum. Both the orbital and
    rotational energy and angular momentum must be maintained when conservative forces like gravity are present.
    Therefore, the values of the variables

    - ``finalOrbAngMom``
    - ``finalOrbEnergy``
    - ``finalRotAngMom``
    - ``finalRotEnergy``

    against their initial values.
    """
    testFunction = globals().get(function)

    if testFunction is None:
        raise ValueError(f"Function '{function}' not found in global scope")

    testFunction(show_plots)

def test_translatingBodyOutputMessagesMatchOneDOF():
    """
    Verify both output-message vectors against the equivalent one-DOF model.

    A single-body N-DOF effector and a one-DOF effector are given identical geometry, mass
    properties, and initial states on their own spacecraft in one simulation. The hub starts with a
    non-identity attitude and nonzero translational and angular velocity so that every inertial
    transformation contributes. The displacement, displacement rate, inertial position, inertial
    velocity, attitude, and angular velocity are compared at every step.
    """
    timeStep = 0.01  # [s]
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.SetProgressBar(False)
    testProc = unitTestSim.CreateNewProcess("TestProcess")
    testProc.addTask(unitTestSim.CreateNewTask("unitTask", macros.sec2nano(timeStep)))

    mass = 20.0  # [kg]
    k = 100.0  # [N/m]
    rhoInit = 0.4  # [m]
    rhoDotInit = 0.05  # [m/s]
    fHat = [[3.0 / 5.0], [4.0 / 5.0], [0.0]]
    r_FcF_F = [[-1.0], [1.0], [0.5]]  # [m]
    r_F0B_B = [[-1.0], [1.0], [0.0]]  # [m]
    IPntFc_F = [[50.0, 0.0, 0.0], [0.0, 80.0, 0.0], [0.0, 0.0, 60.0]]  # [kg*m^2]
    dcm_FB = [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]

    nDofEffector = linearTranslationNDOFStateEffector.LinearTranslationNDOFStateEffector()
    nDofEffector.ModelTag = "translatingBodyNDOF"
    body = linearTranslationNDOFStateEffector.TranslatingBody()
    body.setMass(mass)
    body.setK(k)
    body.setC(0.0)  # [N*s/m]
    body.setRhoInit(rhoInit)
    body.setRhoDotInit(rhoDotInit)
    body.setFHat_P(fHat)
    body.setR_FcF_F(r_FcF_F)
    body.setR_F0P_P(r_F0B_B)
    body.setIPntFc_F(IPntFc_F)
    body.setDCM_FP(dcm_FB)
    nDofEffector.addTranslatingBody(body)

    oneDofEffector = linearTranslationOneDOFStateEffector.LinearTranslationOneDOFStateEffector()
    oneDofEffector.ModelTag = "translatingBodyOneDOF"
    oneDofEffector.setMass(mass)
    oneDofEffector.setK(k)
    oneDofEffector.setC(0.0)  # [N*s/m]
    oneDofEffector.setRhoInit(rhoInit)
    oneDofEffector.setRhoDotInit(rhoDotInit)
    oneDofEffector.setFHat_B(fHat)
    oneDofEffector.setR_FcF_F(r_FcF_F)
    oneDofEffector.setR_F0B_B(r_F0B_B)
    oneDofEffector.setIPntFc_F(IPntFc_F)
    oneDofEffector.setDCM_FB(dcm_FB)

    recorders = []
    for effector in (nDofEffector, oneDofEffector):
        scObject = spacecraft.Spacecraft()
        scObject.ModelTag = "spacecraft" + effector.ModelTag
        scObject.hub.mHub = 750.0  # [kg]
        scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]  # [m]
        scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg*m^2]
        scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]  # [m]
        scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]  # [m/s]
        scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
        scObject.hub.omega_BN_BInit = [[0.5], [-0.4], [0.6]]  # [rad/s]

        earthGravBody = gravityEffector.GravBodyData()
        earthGravBody.planetName = "earth_planet_data"
        earthGravBody.mu = 0.3986004415E+15  # [m^3/s^2]
        earthGravBody.isCentralBody = True
        scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

        scObject.addStateEffector(effector)
        unitTestSim.AddModelToTask("unitTask", effector)
        unitTestSim.AddModelToTask("unitTask", scObject)

    for outMsg, configMsg in ((nDofEffector.translatingBodyOutMsgs[0],
                               nDofEffector.translatingBodyConfigLogOutMsgs[0]),
                              (oneDofEffector.translatingBodyOutMsg,
                               oneDofEffector.translatingBodyConfigLogOutMsg)):
        stateRec = outMsg.recorder()
        configRec = configMsg.recorder()
        unitTestSim.AddModelToTask("unitTask", stateRec)
        unitTestSim.AddModelToTask("unitTask", configRec)
        recorders.append((stateRec, configRec))

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(10 * timeStep))
    unitTestSim.ExecuteSimulation()

    (nDofState, nDofConfig), (oneDofState, oneDofConfig) = recorders
    accuracy = 1e-10
    np.testing.assert_allclose(nDofState.rho, oneDofState.rho, rtol=accuracy,
                               err_msg="Displacement does not match the one-DOF effector.")
    np.testing.assert_allclose(nDofState.rhoDot, oneDofState.rhoDot, rtol=accuracy,
                               err_msg="Displacement rate does not match the one-DOF effector.")
    for field in ("r_BN_N", "v_BN_N", "sigma_BN", "omega_BN_B"):
        np.testing.assert_allclose(getattr(nDofConfig, field), getattr(oneDofConfig, field),
                                   rtol=accuracy, atol=accuracy,
                                   err_msg=field + " does not match the one-DOF effector.")


# axis and mass [kg] of each body in the chain, outward from the hub. A body given no mass keeps the
# zero default, since the setter rejects a non-positive mass, and is given a zero inertia to match.
X_AXIS = [[1.0], [0.0], [0.0]]
Y_AXIS = [[0.0], [1.0], [0.0]]
Z_AXIS = [[0.0], [0.0], [1.0]]
XY_AXIS = [[1.0], [1.0], [0.0]]
VALIDATION_CHAINS = {
    'Valid': [(20.0, X_AXIS), (15.0, Y_AXIS)],
    'MasslessJoint': [(None, X_AXIS), (20.0, Y_AXIS)],
    'MasslessChain': [(None, X_AXIS), (None, Y_AXIS), (20.0, Z_AXIS)],
    'MasslessOutermost': [(20.0, X_AXIS), (None, Y_AXIS)],
    'CollinearMassless': [(None, X_AXIS), (20.0, X_AXIS)],
    # pairwise independent axes that collectively span only two dimensions
    'CoplanarMassless': [(None, X_AXIS), (None, Y_AXIS), (20.0, XY_AXIS)],
    'NoBodies': [],
    'SkewedDCM': [(20.0, X_AXIS), (15.0, Y_AXIS)],
    'AsymmetricInertia': [(20.0, X_AXIS), (15.0, Y_AXIS)],
    'TriangleInertia': [(20.0, X_AXIS), (15.0, Y_AXIS)],
}


@pytest.mark.parametrize("scheduleEffector", [True, False])
@pytest.mark.parametrize("chain, shouldRaise", [
    ('Valid', False),
    ('MasslessJoint', False),
    ('MasslessChain', False),
    ('MasslessOutermost', True),
    ('CollinearMassless', True),
    ('CoplanarMassless', True),
    ('NoBodies', True),
    ('SkewedDCM', True),
    ('AsymmetricInertia', True),
    ('TriangleInertia', True),
])
def test_translatingBodyConfigurationValidation(chain, shouldRaise, scheduleEffector):
    """
    Verify that initialization rejects a chain the equations of motion cannot represent, and that a
    massless body used to build a multiple degree of freedom joint is not one of them.

    The joint mass matrix sums the masses outboard of each axis. A body may be left massless so that
    several single axis bodies compose one multi-axis joint, but the matrix is singular whenever some
    nonzero combination of axis rates leaves every body carrying mass at rest. That covers a massless
    outermost body and a massless body sharing its axis with the body outboard of it, and it is a
    collective condition rather than a pairwise one: massless stages along x and y followed by a
    massive stage along x + y have pairwise independent axes yet span only two dimensions, so they
    are rejected as well.
    Inverting a singular matrix fills the spacecraft state with NaN rather than raising, which is why
    this is asserted as an error at initialization instead of as a tolerance on a trajectory. The
    axes are fixed in their parents and a translating body does not rotate, so the matrix never
    changes during the integration. The rotation matrix and inertia tensor checks match those the
    spinning body effectors already apply, including skipping the inertia check for a massless body,
    whose inertia tensor is legitimately zero.

    An accepted chain is integrated as well, because initializing without error would not show that
    a massless body carries the correct dynamics. Every damper is zero, so the rotational energy and
    the rotational angular momentum about the vehicle center of mass must both be conserved.

    **Test Parameters:**

    - chain: [string]
        translating body configuration to initialize
    - shouldRaise: [bool]
        whether initialization must reject the configuration
    - scheduleEffector: [bool]
        whether the effector is added to the task in addition to the spacecraft

    The scheduling parameter matters because the checks must not depend on it. Spacecraft
    initialization calls ``registerStates()`` on every attached state effector but never calls
    ``Reset()``, which runs only for a module added to a task. The module user guide adds the
    effector to the spacecraft alone, so validation reached from ``Reset()`` would miss the
    documented setup and let a singular chain integrate to NaN.
    """
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg*m^2]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.2], [0.3]]  # [rad/s]

    effector = linearTranslationNDOFStateEffector.LinearTranslationNDOFStateEffector()
    for mass, fHat_P in VALIDATION_CHAINS[chain]:
        body = linearTranslationNDOFStateEffector.TranslatingBody()
        if mass is not None:
            body.setMass(mass)  # [kg]
        else:
            body.setIPntFc_F([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])  # [kg*m^2]
        if chain == 'SkewedDCM':
            body.setDCM_FP([[1.0, 0.1, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        if chain == 'AsymmetricInertia':
            body.setIPntFc_F([[50.0, 3.0, 0.0], [0.0, 80.0, 0.0], [0.0, 0.0, 60.0]])  # [kg*m^2]
        if chain == 'TriangleInertia':
            # positive definite, but the principal moments violate the triangle inequality
            body.setIPntFc_F([[10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 90.0]])  # [kg*m^2]
        body.setR_FcF_F([[0.5], [0.2], [-0.3]])  # [m]
        body.setR_F0P_P([[1.0], [0.5], [0.25]])  # [m]
        body.setFHat_P(fHat_P)
        body.setRhoInit(0.1)  # [m]
        body.setRhoDotInit(0.05)  # [m/s]
        body.setK(10.0)  # [N/m]
        effector.addTranslatingBody(body)
    scObject.addStateEffector(effector)

    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.SetProgressBar(False)
    unitTestSim.CreateNewProcess("TestProcess").addTask(
        unitTestSim.CreateNewTask("unitTask", macros.sec2nano(0.001)))
    if scheduleEffector:
        unitTestSim.AddModelToTask("unitTask", effector)
    unitTestSim.AddModelToTask("unitTask", scObject)
    conservationLog = scObject.logger(["totRotEnergy", "totRotAngMomPntC_N"])
    unitTestSim.AddModelToTask("unitTask", conservationLog)

    if shouldRaise:
        with pytest.raises(BasiliskError):
            unitTestSim.InitializeSimulation()
        return

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(1.0))
    unitTestSim.ExecuteSimulation()

    accuracy = 1e-10
    rotEnergy = np.array(conservationLog.totRotEnergy)
    rotAngMom = np.array(conservationLog.totRotAngMomPntC_N)
    np.testing.assert_allclose(rotEnergy, rotEnergy[0], rtol=accuracy,
                               err_msg="Rotational energy is not constant.")
    np.testing.assert_allclose(rotAngMom, np.broadcast_to(rotAngMom[0], rotAngMom.shape),
                               rtol=accuracy,
                               err_msg="Rotational angular momentum is not constant.")


def translatingBodyNoInput(show_plots):
    r"""
    This test does not use any input messages or lock flags, so the links are free to move.
    """
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create four translating rigid bodies
    translatingBodyEffector = linearTranslationNDOFStateEffector.LinearTranslationNDOFStateEffector()
    translatingBodyEffector.ModelTag = "translatingBodyEffector"

    # define properties
    translatingBody1 = linearTranslationNDOFStateEffector.TranslatingBody()
    translatingBody1.setMass(np.random.uniform(5.0, 50.0))
    translatingBody1.setIPntFc_F(randomValidInertia())
    translatingBody1.setDCM_FP([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    translatingBody1.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                [np.random.uniform(-1.0, 1.0)],
                                [np.random.uniform(-1.0, 1.0)]])
    translatingBody1.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                [np.random.uniform(-1.0, 1.0)],
                                [np.random.uniform(-1.0, 1.0)]])
    translatingBody1.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody1.setRhoInit(np.random.uniform(-5.0, 10.0))
    translatingBody1.setRhoDotInit(0.05)
    translatingBody1.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody1)

    translatingBody2 = linearTranslationNDOFStateEffector.TranslatingBody()
    translatingBody2.setMass(np.random.uniform(5.0, 50.0))
    translatingBody2.setIPntFc_F(randomValidInertia())
    translatingBody2.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody2.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody2.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody2.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody2.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody2.setRhoDotInit(0.05)
    translatingBody2.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody2)

    translatingBody3 = linearTranslationNDOFStateEffector.TranslatingBody()
    translatingBody3.setMass(np.random.uniform(5.0, 50.0))
    translatingBody3.setIPntFc_F(randomValidInertia())
    translatingBody3.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody3.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody3.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody3.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody3.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody3.setRhoDotInit(0.05)
    translatingBody3.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody3)

    translatingBody4 = linearTranslationNDOFStateEffector.TranslatingBody()
    translatingBody4.setMass(np.random.uniform(5.0, 50.0))
    translatingBody4.setIPntFc_F(randomValidInertia())
    translatingBody4.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody4.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody4.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody4.setFHat_P([[0.0], [0.0], [1.0]])
    translatingBody4.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody4.setRhoDotInit(0.05)
    translatingBody4.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody4)

    # Add body to spacecraft
    scObject.addStateEffector(translatingBodyEffector)

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, translatingBodyEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Log the spacecraft state message
    datLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    # Add states to log
    rho1Data = translatingBodyEffector.translatingBodyOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho1Data)
    rho2Data = translatingBodyEffector.translatingBodyOutMsgs[1].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho2Data)
    rho3Data = translatingBodyEffector.translatingBodyOutMsgs[2].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho3Data)
    rho4Data = translatingBodyEffector.translatingBodyOutMsgs[3].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho4Data)

    # Setup and run the simulation
    stopTime = 5000 * testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    rotEnergy = scObjectLog.totRotEnergy
    orbEnergy = scObjectLog.totOrbEnergy
    rho1 = rho1Data.rho
    rho1Dot = rho1Data.rhoDot
    rho2 = rho2Data.rho
    rho2Dot = rho2Data.rhoDot
    rho3 = rho3Data.rho
    rho3Dot = rho3Data.rhoDot
    rho4 = rho4Data.rho
    rho4Dot = rho4Data.rhoDot

    # Set up the conservation quantities
    timeSec = scObjectLog.times() * 1e-9
    initialOrbAngMom_N = [orbAngMom_N[0, 0], orbAngMom_N[0, 1], orbAngMom_N[0, 2]]
    finalOrbAngMom = orbAngMom_N[-1]
    initialRotAngMom_N = [rotAngMom_N[0, 0], rotAngMom_N[0, 1], rotAngMom_N[0, 2]]
    finalRotAngMom = rotAngMom_N[-1]
    initialOrbEnergy = orbEnergy[0]
    finalOrbEnergy = orbEnergy[-1]
    initialRotEnergy = rotEnergy[0]
    finalRotEnergy = rotEnergy[-1]

    # Plotting
    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbAngMom_N[:, 0] - initialOrbAngMom_N[0]) / initialOrbAngMom_N[0],
             timeSec, (orbAngMom_N[:, 1] - initialOrbAngMom_N[1]) / initialOrbAngMom_N[1],
             timeSec, (orbAngMom_N[:, 2] - initialOrbAngMom_N[2]) / initialOrbAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbEnergy - initialOrbEnergy) / initialOrbEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Energy')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotAngMom_N[:, 0] - initialRotAngMom_N[0]) / initialRotAngMom_N[0],
             timeSec, (rotAngMom_N[:, 1] - initialRotAngMom_N[1]) / initialRotAngMom_N[1],
             timeSec, (rotAngMom_N[:, 2] - initialRotAngMom_N[2]) / initialRotAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotEnergy - initialRotEnergy) / initialRotEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Energy')

    plt.figure()
    plt.clf()
    plt.plot(rho1Data.times() * 1e-9, rho1, label=r'$\rho_1$')
    plt.plot(rho2Data.times() * 1e-9, rho2, label=r'$\rho_2$')
    plt.plot(rho3Data.times() * 1e-9, rho3, label=r'$\rho_3$')
    plt.plot(rho4Data.times() * 1e-9, rho4, label=r'$\rho_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Displacement')

    plt.figure()
    plt.clf()
    plt.plot(rho1Data.times() * 1e-9, rho1Dot, label=r'$\dot{\rho}_1$')
    plt.plot(rho2Data.times() * 1e-9, rho2Dot, label=r'$\dot{\rho}_2$')
    plt.plot(rho3Data.times() * 1e-9, rho3Dot, label=r'$\dot{\rho}_3$')
    plt.plot(rho4Data.times() * 1e-9, rho4Dot, label=r'$\dot{\rho}_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Displacement Rate')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-13

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy, err_msg="Orbital energy is not constant.")
    np.testing.assert_allclose(finalRotEnergy, initialRotEnergy, rtol=accuracy,
                               err_msg="Rotational energy is not constant.")
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy,
                                   err_msg="Orbital angular momentum is not constant.")
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy,
                                   err_msg="Rotational angular momentum is not constant.")


def translatingBodyLockAxis(show_plots):
    r"""
    This test locks the axis, so the displacement is kept constant throughout the simulation.
    """
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create four translating rigid bodies
    translatingBodyEffector = linearTranslationNDOFStateEffector.LinearTranslationNDOFStateEffector()
    translatingBodyEffector.ModelTag = "translatingBodyEffector"

    # define properties
    translatingBody1 = linearTranslationNDOFStateEffector.TranslatingBody()
    translatingBody1.setMass(np.random.uniform(5.0, 50.0))
    translatingBody1.setIPntFc_F(randomValidInertia())
    translatingBody1.setDCM_FP([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    translatingBody1.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody1.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody1.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody1.setRhoInit(np.random.uniform(-5.0, 10.0))
    translatingBody1.setRhoDotInit(0.05)
    translatingBody1.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody1)

    translatingBody2 = linearTranslationNDOFStateEffector.TranslatingBody()
    translatingBody2.setMass(np.random.uniform(5.0, 50.0))
    translatingBody2.setIPntFc_F(randomValidInertia())
    translatingBody2.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody2.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody2.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody2.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody2.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody2.setRhoDotInit(0.05)
    translatingBody2.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody2)

    translatingBody3 = linearTranslationNDOFStateEffector.TranslatingBody()
    translatingBody3.setMass(np.random.uniform(5.0, 50.0))
    translatingBody3.setIPntFc_F(randomValidInertia())
    translatingBody3.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody3.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody3.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody3.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody3.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody3.setRhoDotInit(0.05)
    translatingBody3.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody3)

    translatingBody4 = linearTranslationNDOFStateEffector.TranslatingBody()
    translatingBody4.setMass(np.random.uniform(5.0, 50.0))
    translatingBody4.setIPntFc_F(randomValidInertia())
    translatingBody4.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody4.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody4.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody4.setFHat_P([[0.0], [0.0], [1.0]])
    translatingBody4.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody4.setRhoDotInit(0.05)
    translatingBody4.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody4)

    # Add body to spacecraft
    scObject.addStateEffector(translatingBodyEffector)

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, translatingBodyEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # create lock message
    lockArray = messaging.ArrayEffectorLockMsgPayload()
    lockArray.effectorLockFlag = [1, 0, 0, 1]
    lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
    translatingBodyEffector.motorLockInMsg.subscribeTo(lockMsg)

    # Log the spacecraft state message
    datLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    # Add states to log
    rho1Data = translatingBodyEffector.translatingBodyOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho1Data)
    rho2Data = translatingBodyEffector.translatingBodyOutMsgs[1].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho2Data)
    rho3Data = translatingBodyEffector.translatingBodyOutMsgs[2].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho3Data)
    rho4Data = translatingBodyEffector.translatingBodyOutMsgs[3].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho4Data)

    # Setup and run the simulation
    stopTime = 5000 * testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    rotEnergy = scObjectLog.totRotEnergy
    orbEnergy = scObjectLog.totOrbEnergy
    rho1 = rho1Data.rho
    rho1Dot = rho1Data.rhoDot
    rho2 = rho2Data.rho
    rho2Dot = rho2Data.rhoDot
    rho3 = rho3Data.rho
    rho3Dot = rho3Data.rhoDot
    rho4 = rho4Data.rho
    rho4Dot = rho4Data.rhoDot

    # Set up the conservation quantities
    timeSec = scObjectLog.times() * 1e-9
    initialOrbAngMom_N = [orbAngMom_N[0, 0], orbAngMom_N[0, 1], orbAngMom_N[0, 2]]
    finalOrbAngMom = orbAngMom_N[-1]
    initialRotAngMom_N = [rotAngMom_N[0, 0], rotAngMom_N[0, 1], rotAngMom_N[0, 2]]
    finalRotAngMom = rotAngMom_N[-1]
    initialOrbEnergy = orbEnergy[0]
    finalOrbEnergy = orbEnergy[-1]
    initialRotEnergy = rotEnergy[0]
    finalRotEnergy = rotEnergy[-1]

    # Plotting
    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbAngMom_N[:, 0] - initialOrbAngMom_N[0]) / initialOrbAngMom_N[0],
             timeSec, (orbAngMom_N[:, 1] - initialOrbAngMom_N[1]) / initialOrbAngMom_N[1],
             timeSec, (orbAngMom_N[:, 2] - initialOrbAngMom_N[2]) / initialOrbAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbEnergy - initialOrbEnergy) / initialOrbEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Energy')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotAngMom_N[:, 0] - initialRotAngMom_N[0]) / initialRotAngMom_N[0],
             timeSec, (rotAngMom_N[:, 1] - initialRotAngMom_N[1]) / initialRotAngMom_N[1],
             timeSec, (rotAngMom_N[:, 2] - initialRotAngMom_N[2]) / initialRotAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotEnergy - initialRotEnergy) / initialRotEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Energy')

    plt.figure()
    plt.clf()
    plt.plot(rho1Data.times() * 1e-9, rho1, label=r'$\rho_1$')
    plt.plot(rho2Data.times() * 1e-9, rho2, label=r'$\rho_2$')
    plt.plot(rho3Data.times() * 1e-9, rho3, label=r'$\rho_3$')
    plt.plot(rho4Data.times() * 1e-9, rho4, label=r'$\rho_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Displacement')

    plt.figure()
    plt.clf()
    plt.plot(rho1Data.times() * 1e-9, rho1Dot, label=r'$\dot{\rho}_1$')
    plt.plot(rho2Data.times() * 1e-9, rho2Dot, label=r'$\dot{\rho}_2$')
    plt.plot(rho3Data.times() * 1e-9, rho3Dot, label=r'$\dot{\rho}_3$')
    plt.plot(rho4Data.times() * 1e-9, rho4Dot, label=r'$\dot{\rho}_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Displacement Rate')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-13

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy,
                               err_msg="Orbital energy is not constant.")
    np.testing.assert_allclose(finalRotEnergy, initialRotEnergy, rtol=accuracy,
                               err_msg="Rotational energy is not constant.")
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy,
                                   err_msg="Orbital angular momentum is not constant.")
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy,
                                   err_msg="Rotational angular momentum is not constant.")


def translatingBodyCommandedForce(show_plots):
    r"""
    This test includes a commanded force to the link, so energy is not conserved.
    """
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.001)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create four translating rigid bodies
    translatingBodyEffector = linearTranslationNDOFStateEffector.LinearTranslationNDOFStateEffector()
    translatingBodyEffector.ModelTag = "translatingBodyEffector"

    # define properties
    translatingBody1 = linearTranslationNDOFStateEffector.TranslatingBody()
    translatingBody1.setMass(np.random.uniform(5.0, 50.0))
    translatingBody1.setIPntFc_F(randomValidInertia())
    translatingBody1.setDCM_FP([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
    translatingBody1.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody1.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody1.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody1.setRhoInit(np.random.uniform(-5.0, 10.0))
    translatingBody1.setRhoDotInit(0.05)
    translatingBody1.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody1)

    translatingBody2 = linearTranslationNDOFStateEffector.TranslatingBody()
    translatingBody2.setMass(np.random.uniform(5.0, 50.0))
    translatingBody2.setIPntFc_F(randomValidInertia())
    translatingBody2.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody2.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody2.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody2.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody2.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody2.setRhoDotInit(0.05)
    translatingBody2.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody2)

    translatingBody3 = linearTranslationNDOFStateEffector.TranslatingBody()
    translatingBody3.setMass(np.random.uniform(5.0, 50.0))
    translatingBody3.setIPntFc_F(randomValidInertia())
    translatingBody3.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody3.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody3.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody3.setFHat_P([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody3.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody3.setRhoDotInit(0.05)
    translatingBody3.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody3)

    translatingBody4 = linearTranslationNDOFStateEffector.TranslatingBody()
    translatingBody4.setMass(np.random.uniform(5.0, 50.0))
    translatingBody4.setIPntFc_F(randomValidInertia())
    translatingBody4.setDCM_FP([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    translatingBody4.setR_FcF_F([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody4.setR_F0P_P([[np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)],
                                 [np.random.uniform(-1.0, 1.0)]])
    translatingBody4.setFHat_P([[0.0], [0.0], [1.0]])
    translatingBody4.setRhoInit(np.random.uniform(-5.0, 5.0))
    translatingBody4.setRhoDotInit(0.05)
    translatingBody4.setK(np.random.random())
    translatingBodyEffector.addTranslatingBody(translatingBody4)

    # Add body to spacecraft
    scObject.addStateEffector(translatingBodyEffector)

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [1.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.1], [-0.1], [0.1]]

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, translatingBodyEffector)
    unitTestSim.AddModelToTask(unitTaskName, scObject)

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # meters!
    earthGravBody.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Create the force message
    cmdArray = messaging.ArrayMotorForceMsgPayload()
    cmdArray.motorForce = [0.1, -0.2, 0.3, -0.15]  # [Nm]
    cmdMsg = messaging.ArrayMotorForceMsg().write(cmdArray)
    translatingBodyEffector.motorForceInMsg.subscribeTo(cmdMsg)

    # Log the spacecraft state message
    datLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    # Add states to log
    rho1Data = translatingBodyEffector.translatingBodyOutMsgs[0].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho1Data)
    rho2Data = translatingBodyEffector.translatingBodyOutMsgs[1].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho2Data)
    rho3Data = translatingBodyEffector.translatingBodyOutMsgs[2].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho3Data)
    rho4Data = translatingBodyEffector.translatingBodyOutMsgs[3].recorder()
    unitTestSim.AddModelToTask(unitTaskName, rho4Data)

    # Setup and run the simulation
    stopTime = 5000 * testProcessRate
    unitTestSim.ConfigureStopTime(stopTime)
    unitTestSim.ExecuteSimulation()

    # Extract the logged variables
    orbAngMom_N = scObjectLog.totOrbAngMomPntN_N
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N
    rotEnergy = scObjectLog.totRotEnergy
    orbEnergy = scObjectLog.totOrbEnergy
    rho1 = rho1Data.rho
    rho1Dot = rho1Data.rhoDot
    rho2 = rho2Data.rho
    rho2Dot = rho2Data.rhoDot
    rho3 = rho3Data.rho
    rho3Dot = rho3Data.rhoDot
    rho4 = rho4Data.rho
    rho4Dot = rho4Data.rhoDot

    # Set up the conservation quantities
    timeSec = scObjectLog.times() * 1e-9
    initialOrbAngMom_N = [orbAngMom_N[0, 0], orbAngMom_N[0, 1], orbAngMom_N[0, 2]]
    finalOrbAngMom = orbAngMom_N[-1]
    initialRotAngMom_N = [rotAngMom_N[0, 0], rotAngMom_N[0, 1], rotAngMom_N[0, 2]]
    finalRotAngMom = rotAngMom_N[-1]
    initialOrbEnergy = orbEnergy[0]
    finalOrbEnergy = orbEnergy[-1]
    initialRotEnergy = rotEnergy[0]

    # Plotting
    plt.close("all")
    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbAngMom_N[:, 0] - initialOrbAngMom_N[0]) / initialOrbAngMom_N[0],
             timeSec, (orbAngMom_N[:, 1] - initialOrbAngMom_N[1]) / initialOrbAngMom_N[1],
             timeSec, (orbAngMom_N[:, 2] - initialOrbAngMom_N[2]) / initialOrbAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (orbEnergy - initialOrbEnergy) / initialOrbEnergy)
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Orbital Energy')

    plt.figure()
    plt.clf()
    plt.plot(timeSec, (rotAngMom_N[:, 0] - initialRotAngMom_N[0]) / initialRotAngMom_N[0],
             timeSec, (rotAngMom_N[:, 1] - initialRotAngMom_N[1]) / initialRotAngMom_N[1],
             timeSec, (rotAngMom_N[:, 2] - initialRotAngMom_N[2]) / initialRotAngMom_N[2])
    plt.xlabel('time (s)')
    plt.ylabel('Relative Difference')
    plt.title('Rotational Angular Momentum')

    plt.figure()
    plt.clf()
    plt.plot(rho1Data.times() * 1e-9, rho1, label=r'$\rho_1$')
    plt.plot(rho2Data.times() * 1e-9, rho2, label=r'$\rho_2$')
    plt.plot(rho3Data.times() * 1e-9, rho3, label=r'$\rho_3$')
    plt.plot(rho4Data.times() * 1e-9, rho4, label=r'$\rho_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Displacement')

    plt.figure()
    plt.clf()
    plt.plot(rho1Data.times() * 1e-9, rho1Dot, label=r'$\dot{\rho}_1$')
    plt.plot(rho2Data.times() * 1e-9, rho2Dot, label=r'$\dot{\rho}_2$')
    plt.plot(rho3Data.times() * 1e-9, rho3Dot, label=r'$\dot{\rho}_3$')
    plt.plot(rho4Data.times() * 1e-9, rho4Dot, label=r'$\dot{\rho}_4$')
    plt.legend(loc='best')
    plt.xlabel('time (s)')
    plt.ylabel('Displacement Rate')

    if show_plots:
        plt.show()
    plt.close("all")

    # Testing setup
    accuracy = 1e-13

    np.testing.assert_allclose(finalOrbEnergy, initialOrbEnergy, rtol=accuracy,
                               err_msg="Orbital energy is not constant.")
    for i in range(3):
        np.testing.assert_allclose(finalOrbAngMom, initialOrbAngMom_N, rtol=accuracy,
                                   err_msg="Orbital angular momentum is not constant.")
        np.testing.assert_allclose(finalRotAngMom, initialRotAngMom_N, rtol=accuracy,
                                   err_msg="Rotational angular momentum is not constant.")


if __name__ == "__main__":
    translatingBodyNoInput(True)
    # translatingBodyLockAxis(True)
    # translatingBodyCommandedForce(True)
