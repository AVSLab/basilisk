# ISC License
#
# Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
#   Integrated Test Script
#   Purpose:            Test effector branching functionality
#   Author:             Andrew Morell
#   Creation Date:      September 6, 2025
#

import inspect
import os
import matplotlib.pyplot as plt
import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass, macros, simIncludeThruster
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.architecture import sim_model
from Basilisk.simulation import spacecraft, svIntegrators, gravityEffector
from Basilisk.simulation import ( # state effectors
    hingedRigidBodyStateEffector,
    dualHingedRigidBodyStateEffector,
    nHingedRigidBodyStateEffector,
    spinningBodyOneDOFStateEffector,
    spinningBodyTwoDOFStateEffector,
    spinningBodyNDOFStateEffector,
    linearTranslationOneDOFStateEffector,
    linearTranslationNDOFStateEffector,
    linearSpringMassDamper,
    sphericalPendulum,
    prescribedMotionStateEffector,
    reactionWheelStateEffector,
    vscmgStateEffector,
    thrusterStateEffector,
    fuelTank,
)
from Basilisk.simulation import ( # dynamic effectors
    extForceTorque,
    ExtPulsedTorque,
    thrusterDynamicEffector,
    constraintDynamicEffector,
    dragDynamicEffector,
    radiationPressure,
    facetSRPDynamicEffector,
    MtbEffector,
)
from Basilisk.architecture import messaging

# uncomment this line if this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail()

# Note: effectors commented out as True are expected to be added in the future, effectors
#       commented out as False are not expected to be added in the future
@pytest.mark.parametrize("stateEffector, isParent", [
    ("hingedRigidBodies",             True),
    # ("dualHingedRigidBodies",           True),
    # ("nHingedRigidBodies",              True),
    ("spinningBodiesOneDOF",          True),
    ("spinningBodiesTwoDOF",          True),
    ("spinningBodiesNDOF",            True),
    ("linearTranslationBodiesOneDOF", True),
    # ("linearTranslationBodiesNDOF",     True),
    ("linearSpringMassDamper",          False),
    # ("sphericalPendulum",               False),
    # ("prescribedMotion",                False),
    # ("reactionWheels",                  False),
    # ("VSCMGs",                          False),
    # ("thrusterStateEffector",           False),
    # ("fuelTank",                        False),
])
@pytest.mark.parametrize("dynamicEffector, isChild", [
    ("extForceTorque",            True),
    ("extPulseTorque",              False),
    ("thrusterDynamicEffector",   True),
    ("constraintEffectorOneHub",  True),
    ("constraintEffectorNoHubs",  True),
    # ("dragEffector",                False),
    # ("radiationPressure",           False),
    # ("facetSRPDynamicEffector",     False),
    # ("MtbEffector",                 False),
    ("multiEffector",             True),
])

def test_effectorBranchingIntegratedTest(show_plots, stateEffector, isParent, dynamicEffector, isChild):
    r"""
    **Validation Test Description**

    This integrated test sets up combinations of dynamic effector attached to a state effector.

    **Description of Variables Being Tested**

    In this file we are checking first for branching compatibility, as not every state effector is
    set up to host dependent effectors (isParent), and not every dynamic effector is set up to be
    attached to a state effector (isChild). The state effector compatibility is checked when the
    addDynamicEffector() method is called. The dynamic effector compatibility is checked at
    simulation initialization when linkInProperties() is called.

    A summary of which state effectors and dynamic effectors are expected to be able to act as a
    parent or child effector to another is summarized in :ref:`bskPrinciples-11`.

    Note that the constraint effector is tested in two configurations: once where one vehicle's
    state effector is attached to the hub on another vehicle (``constraintEffectorOneHub``), and
    once where both vehicles' state effectors are attached to each other
    (``constraintEffectorNoHubs``). Additionally, a test with multiple dynamic effectors attached
    to a single state effector is included (``multiEffector``). Finally, at least one non-compatible
    state effector and one non-compatible dynamic effector are included to check that the error
    handling catches as expected.

    Note that the center of mass of the hub is adjusted to balance the shift in total vehicle COM
    due to the addition of the state effector's mass. This adjustment is calculated such that at
    simulation initialization the total center of mass of the vehicle is coincident with the hub's
    body frame B (r_BN_N = r_CN_N). This greatly simplifies the setup of the constraint effector
    branching scenarios. This calculation is performed by calculating the parameter mr_PcB_B in each
    state effector's setup method which for the state effector's series of :math:`i` bodies is:

    .. math::

        \sum_i m_{P_i} {}^{\mathcal{B}}\mathbf{r}_{Pc_i/B}

    Where P is the state effector's designation as "parent", with each of its :math:`i` segments
    having segment body fixed frame :math:`\mathcal{P}_i`, origin :math:`P_i`, and segment center of
    mass :math:`Pc_i`. Then used to compute the hub center of mass offset scObject.hub.r_BcB_B as:

    .. math::

        {}^{\mathcal{B}}\mathbf{r}_{Bc/B} =
        \frac{- \sum_i m_{P_i} {}^{\mathcal{B}}\mathbf{r}_{Pc_i/B}} {\sum_i m_{Bc}}

    In the case of a permissible combination, we then check that properties are being handed
    correctly from the state effector to dynamic effector.

    These variables include:

    - ``inertialPositionProperty``
    - ``inertialVelocityProperty``
    - ``inertialAttitudeProperty``
    - ``inertialAngVelocityProperty``

    Finally, we simultaneously check that a) the applied force and torque are being handed correctly
    from the dynamic effector to state effector, and b) that these forces and torques are
    implemented correctly in the state effector's equations of motion. We do this by isolating each
    case where a state effector has the :ref:`extForceTorque` effector attached to it. Using the
    explicitly defined ``forceExternal_B`` and ``torqueExternalPntB_B`` and the logged inertial
    position and attitude properties of the state effector, we manually compute the total external
    force on the vehicle about the combined center of mass and the accumulated delta V of the
    vehicle's combined center of mass. The torque is then integrated using a trapezoid rule and
    compared against the spacecraft's internally computed angular momentum.

    .. math::

        {}^{\mathcal{N}}\!\Delta\mathbf{H}_{C}
        & = \int_{t_0}^{t} {}^{\mathcal{N}}\!\boldsymbol{\tau}_{\text{ext},C}(t)\,dt \\
        & = \int_{t_0}^{t} \text{Pure Torque + Force Relative to COM} \\
        & = \int_{t_0}^{t} [\mathcal{NP}_j] {}^{\mathcal{P}_j}\!\boldsymbol{\tau}_{\text{ext},P_j}
          + \left( {}^{\mathcal{N}}\mathbf{r}_{Pc_j/N}
          - [\mathcal{NP}_j] {}^{\mathcal{P}_j}\mathbf{r}_{Pc_j/P_j}
          - {}^{\mathcal{N}}\mathbf{r}_{C/N} \right)
          \times \left( [\mathcal{NP}_j] {}^{\mathcal{P}_j}\mathbf{F}_{P_j} \right)

    Where :math:`j` is the segment that the dynamic effector is attached to. The sim 'truth'
    :math:`{}^{\mathcal{N}}\!\Delta\mathbf{H}_{C}` (scObject.totOrbAngMomPntN_N) and
    :math:`{}^{\mathcal{N}}\mathbf{r}_{C/N}` are logged from the spacecraft module. Exerted
    :math:`{}^{\mathcal{P}_j}\!\boldsymbol{\tau}_{\text{ext},P_j}` and
    :math:`{}^{\mathcal{P}_j}\mathbf{F}_{P_j}` are from the extForceTorque effector module.
    :math:`{}^{\mathcal{N}}\mathbf{r}_{Pc_j/N}`, :math:`[\mathcal{NP}_j]`, and
    :math:`{}^{\mathcal{P}_j}\mathbf{r}_{Pc_j/P_j}` come from the state effector module.

    The accumulated delta V is compared against the internally computed delta V of the spacecraft
    center of mass.

    .. math::

        {}^{\mathcal{N}}\!\Delta v_{accum,C} = \int_{t_0}^{t} \frac{[\mathcal{NP}_j]
        {}^{\mathcal{P}_j}\mathbf{F}_{P_j}} {m_{Bc} + \sum_i m_{P_i}} dt

    where again :math:`j` is the segment that the dynamic effector is attached to among all
    :math:`i` segments. The sim 'truth' :math:`{}^{\mathcal{N}}\!\Delta v_{accum,C}` is logged from
    the spacecraft module.
    """

    effectorBranchingIntegratedTest(show_plots, stateEffector, isParent, dynamicEffector, isChild)

def effectorBranchingIntegratedTest(show_plots, stateEffector, isParent, dynamicEffector, isChild):
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.SetProgressBar(True)

    # Create test thread
    timestep = 0.001
    testProcessRate = macros.sec2nano(timestep)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))

    # Create the spacecraft object
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"

    # Set the integrator to RKF45
    integratorObject = svIntegrators.svIntegratorRKF45(scObject)
    scObject.setIntegrator(integratorObject)

    # Define mass properties of the rigid hub of the spacecraft
    scObject.hub.mHub = 750.0
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]
    scObject.hub.omega_BN_BInit = [[0.1], [0.1], [0.1]]

    # Add Earth gravity to the simulation
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15
    earthGravBody.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    # Create the state effector of interest
    if stateEffector == "spinningBodiesOneDOF":
        stateEff, stateEffProps = setup_spinningBodiesOneDOF()
        segment = 1
    elif stateEffector == "spinningBodiesTwoDOF":
        stateEff, stateEffProps = setup_spinningBodiesTwoDOF()
        segment = 2
    elif stateEffector == "spinningBodiesNDOF":
        stateEff, stateEffProps = setup_spinningBodiesNDOF()
        segment = 4
    elif stateEffector == "hingedRigidBodies":
        stateEff, stateEffProps = setup_hingedRigidBodyStateEffector()
        segment = 1
    elif stateEffector == "linearTranslationBodiesOneDOF":
        stateEff, stateEffProps = setup_translatingBodiesOneDOF()
        segment = 1
    elif stateEffector == "linearSpringMassDamper":
        stateEff, stateEffProps = setup_linearSpringMassDamper()
        segment = 1
    else:
        pytest.fail("ERROR: Effector branching integrated test using unrecognized state effector input.")

    # Compute r_BcB_B such that point B and initial total COM coincide
    scObject.hub.r_BcB_B = stateEffProps.mr_PcB_B / scObject.hub.mHub

    # Create the dynamic effector of interest
    if dynamicEffector == "extForceTorque":
        dynamicEff = setup_extForceTorque()
    elif dynamicEffector == "extPulseTorque":
        dynamicEff = setup_extPulseTorque()
    elif dynamicEffector == "thrusterDynamicEffector":
        dynamicEff, thFactory = setup_thrusterDynamicEffector()
    elif dynamicEffector == "constraintEffectorOneHub":
        dynamicEff, scObjectx = setup_constraintEffectorOneHub(scObject, stateEffProps)
        unitTestSim.AddModelToTask("unitTask", scObjectx)
    elif dynamicEffector == "constraintEffectorNoHubs":
        dynamicEff, scObjectx, stateEffx = setup_constraintEffectorNoHubs(scObject, stateEffProps)
        unitTestSim.AddModelToTask("unitTask", scObjectx)
        unitTestSim.AddModelToTask(unitTaskName, stateEffx)
    elif dynamicEffector == "multiEffector":
        dynamicEff = [setup_extForceTorque(), setup_extForceTorque()]
    else:
        pytest.fail("ERROR: Effector branching integrated test using unrecognized dynamic effector input.")

    # Add dynamic effector to state effector
    try:
        if dynamicEffector == "thrusterDynamicEffector": # if thruster, then use thruster factory
            thFactory.addToSpacecraftSubcomponent("dynamicThruster", dynamicEff, stateEff, segment)
        elif dynamicEffector == "multiEffector": # if multiple effectors, loop over all to add
            for dynEff in dynamicEff: stateEff.addDynamicEffector(dynEff, segment)
        else:
            stateEff.addDynamicEffector(dynamicEff, segment)
    except BasiliskError:
        # check if error was meant to happen
        assert not isParent, "FAILED: attempted attaching to a compatible state effector, but errored"
        return
    else:
        # check if error wasn't meant to happen
        assert isParent, "FAILED: attached to an incompatible state effector without erroring"

    # Add state effector to spacecraft
    scObject.addStateEffector(stateEff)

    # Add test module to runtime call list
    unitTestSim.AddModelToTask(unitTaskName, stateEff)
    unitTestSim.AddModelToTask(unitTaskName, scObject)
    if dynamicEffector == "multiEffector":
        for dynEff in dynamicEff: unitTestSim.AddModelToTask(unitTaskName, dynEff)
    else:
        unitTestSim.AddModelToTask(unitTaskName, dynamicEff)

    # Log the spacecraft state message
    datLog = scObject.scStateOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, datLog)

    # Log the effector's inertial properties
    if segment == 1:
        inertialPropLog = getattr(stateEff, f"{stateEffProps.inertialPropLogName}").recorder()
    else:
        inertialPropLog = getattr(stateEff, f"{stateEffProps.inertialPropLogName}")[segment-1].recorder()
    unitTestSim.AddModelToTask(unitTaskName, inertialPropLog)

    # Add energy and momentum variables to log
    scObjectLog = scObject.logger(["totOrbAngMomPntN_N", "totRotAngMomPntC_N", "totOrbEnergy", "totRotEnergy"])
    unitTestSim.AddModelToTask(unitTaskName, scObjectLog)

    try:
        unitTestSim.InitializeSimulation()
    except BasiliskError:
        # check if error was meant to happen
        assert not isChild, "FAILED: attempted to attach a compatible dynamic effector, but errored"
        return
    else:
        # check if error wasn't meant to happen
        assert isChild, "FAILED: attached an incompatible dynamic effector without erroring"

    # Check that properties are being handed correctly from state effector to dynamic effector
    if (stateEffector == "spinningBodiesNDOF" or stateEffector == "linearTranslationBodiesOneDOF"
        or stateEffector == "linearTranslationBodiesNDOF"):
        # Newer effector classes keep all variables private and so we check with the dynParamManager
        positionName = getModernStateEffInertialPropName(scObject, segment, stateEff, "Position")
        velocityName = getModernStateEffInertialPropName(scObject, segment, stateEff, "Velocity")
        attitudeName = getModernStateEffInertialPropName(scObject, segment, stateEff, "Attitude")
        angvelocityName = getModernStateEffInertialPropName(scObject, segment, stateEff, "AngVelocity")
    else:
        # older effector classes have public variable names that are simply checked directly
        positionName = getStateEffInertialPropName(segment, stateEff, "Position")
        velocityName = getStateEffInertialPropName(segment, stateEff, "Velocity")
        attitudeName = getStateEffInertialPropName(segment, stateEff, "Attitude")
        angvelocityName = getStateEffInertialPropName(segment, stateEff, "AngVelocity")

    assert getDynEffInertialPropName(dynamicEffector, dynamicEff, "Position") == positionName, (
        "FAILED: inertialPositionProperty not handed correctly between state and dynamic effectors")
    assert getDynEffInertialPropName(dynamicEffector, dynamicEff, "Velocity") == velocityName, (
        "FAILED: inertialVelocityProperty not handed correctly between state and dynamic effectors")
    assert getDynEffInertialPropName(dynamicEffector, dynamicEff, "Attitude") == attitudeName, (
        "FAILED: inertialAttitudeProperty not handed correctly between state and dynamic effectors")
    assert getDynEffInertialPropName(dynamicEffector, dynamicEff, "AngVelocity") == angvelocityName, (
        "FAILED: inertialAngVelocityProperty not handed correctly between state and dynamic effectors")

    # Run the sim for a few timesteps to confirm execution without error
    stopTime = 1
    unitTestSim.ConfigureStopTime(macros.sec2nano(stopTime))
    unitTestSim.ExecuteSimulation()

    # Continue to check state effector EOMs using pure force & torque
    if dynamicEffector != "extForceTorque":
        return

    # Grab conservation quantities to compare against
    rotAngMom_N = scObjectLog.totRotAngMomPntC_N  # total rotational angular momentum about the total vehicle COM
    totAccumDV_N = datLog.TotalAccumDV_CN_N # total accumulated deltaV of the total vehicle COM

    # Grab effector's inertial position
    if stateEffector in ["hingedRigidBodies", "dualHingedRigidBodies", "nHingedRigidBodies"]:
        r_ScN_N_log = np.zeros_like(inertialPropLog.r_BN_N)
        for i in range(len(inertialPropLog.sigma_BN)):
            sigmai_SN = inertialPropLog.sigma_BN[i, :] # MRP at timestep i and S is the parent frame not B
            dcm_NS = np.transpose(rbk.MRP2C(sigmai_SN))
            r_ScN_N_log[i, :] = inertialPropLog.r_BN_N[i, :] + (dcm_NS @ stateEffProps.r_PcP_P).flatten()
    else:
        r_ScN_N_log = inertialPropLog.r_BN_N

    # Grab effector's attitude properties
    sigma_SN_log = inertialPropLog.sigma_BN

    # Compute conservation quantities using the state and dynamic effector's logged properties
    n = rotAngMom_N.shape[0]-1 # length of log minus 1 as the inertial property log lags by a timestep
    extTorque = np.empty((n,3))
    dV = np.empty((n,3))
    for idx in range(n):
        dcm_NS = np.transpose(rbk.MRP2C(sigma_SN_log[idx,:]))
        # Compute the total accumulated deltaV
        if idx == 0:
            dV[idx,:] = [0.0, 0.0, 0.0]
        else:
            dV[idx,:] = (dV[idx-1,:] + (dcm_NS @ np.array(dynamicEff.extForce_B).flatten())
                       / (scObject.hub.mHub + stateEffProps.totalMass) * timestep)
        # Compute the total external torque on the vehicle
        if stateEffector == "linearTranslationBodiesOneDOF" or stateEffector == "linearTranslationBodiesNDOF":
            extTorque[idx,:] = (dcm_NS @ np.array(dynamicEff.extTorquePntB_B).flatten()
                                + np.cross(r_ScN_N_log[idx+1,:] - dcm_NS
                                @ np.array(stateEffProps.r_PcP_P).flatten()
                                - datLog.r_CN_N[idx,:], dcm_NS
                                @ np.array(dynamicEff.extForce_B).flatten()))
        else:
            extTorque[idx,:] = (dcm_NS @ np.array(dynamicEff.extTorquePntB_B).flatten()
                                + np.cross(r_ScN_N_log[idx+1,:] - dcm_NS
                                @ np.array(stateEffProps.r_PcP_P).flatten()
                                - datLog.r_CN_N[idx,:], dcm_NS
                                @ np.array(dynamicEff.extForce_B).flatten()))

    # Integrate the torque to find accumulated change in angular momentum
    dx = np.ones(n-1)*timestep
    y_avg = 0.5 * (extTorque[1:] + extTorque[:-1])
    integral = np.cumsum(y_avg * dx[:, None], axis=0)
    dH = np.vstack((np.zeros((1, 3)), integral))

    # Plotting
    plt.close("all")
    plt.figure()
    for idx in range(3):
        plt.plot(scObjectLog.times() * macros.NANO2SEC, rotAngMom_N[:,idx]-rotAngMom_N[0,idx], label='$dH_{truth,' + str(idx) + '}$')
        plt.plot(scObjectLog.times()[:-1] * macros.NANO2SEC, dH[:,idx], '--', label='$dH_{test,' + str(idx) + '}$')
    plt.plot(scObjectLog.times() * macros.NANO2SEC, np.linalg.norm(rotAngMom_N-rotAngMom_N[0,:], axis=1), linewidth=3, label='$dH_{truth, magnitude}$')
    plt.plot(scObjectLog.times()[:-1] * macros.NANO2SEC, np.linalg.norm(dH, axis=1), '--', linewidth=3, label='$dH_{test, magnitude}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Relative Difference $\Delta$H')
    plt.title('Total Rotational Angular Momentum')

    plt.figure()
    for idx in range(3):
        plt.plot(datLog.times() * macros.NANO2SEC, totAccumDV_N[:,idx], label='$dV_{truth,' + str(idx) + '}$')
        plt.plot(datLog.times()[:-1] * macros.NANO2SEC, dV[:,idx], "--", label='$dV_{test,' + str(idx) + '}$')
    plt.plot(datLog.times() * macros.NANO2SEC, np.linalg.norm(totAccumDV_N, axis=1), linewidth=3, label='$dV_{truth, magnitude}$')
    plt.plot(datLog.times()[:-1] * macros.NANO2SEC, np.linalg.norm(dV, axis=1), "--", linewidth=3, label='$dV_{test, magnitude}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Accumulated $\Delta$V')
    plt.title('Total COM DeltaV')

    plt.figure()
    for idx in range(3):
        plt.plot(scObjectLog.times()[:-1] * macros.NANO2SEC, rotAngMom_N[:-1,idx]-rotAngMom_N[0,idx] - dH[:,idx], label=str(idx))
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Truth - Test $\Delta$H')
    plt.title('Total Rotational Angular Momentum Change')

    plt.figure()
    for idx in range(3):
        plt.plot(datLog.times()[:-1] * macros.NANO2SEC, totAccumDV_N[:-1,idx] - dV[:,idx], label=str(idx))
    plt.legend(loc='lower right')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'Truth - Test $\Delta$V')
    plt.title('Total COM DeltaV Accumulated')

    if show_plots:
        plt.show()
    plt.close("all")

    # Check angular momentum difference against sim truth
    angMom_accuracy = 1e-3
    # np.testing.assert_allclose(np.linalg.norm(rotAngMom_N[:-1,:]-rotAngMom_N[0,:] - dH, axis=1), 0, atol=angMom_accuracy,
    #                            err_msg="angular momentum difference beyond accuracy limits")
    np.testing.assert_allclose(rotAngMom_N[:-1,:]-rotAngMom_N[0,:], dH, atol=angMom_accuracy,
                               err_msg="angular momentum difference beyond accuracy limits")

    # Check deltaV difference against sim truth
    deltaV_accuracy = 1e-6
    # np.testing.assert_allclose(np.linalg.norm(totAccumDV_N[:-1,:] - dV, axis=1), 0, atol=deltaV_accuracy,
    #                            err_msg="deltaV difference beyond accuracy limits")
    np.testing.assert_allclose(totAccumDV_N[:-1,:], dV, 0, atol=deltaV_accuracy,
                               err_msg="deltaV difference beyond accuracy limits")

    return

def getDynEffInertialPropName(dynamicEffector, dynamicEff, propType):
    if dynamicEffector == "multiEffector":
        return getattr(dynamicEff[1], f"getPropName_inertial{propType}")()
    elif dynamicEffector == "constraintEffectorOneHub" or dynamicEffector == "constraintEffectorNoHubs":
        propList = getattr(dynamicEff, f"getPropName_inertial{propType}")()
        return propList[0]
    else:
        return getattr(dynamicEff, f"getPropName_inertial{propType}")()

def getStateEffInertialPropName(segment, stateEff, propType):
    if segment == 1:
        return getattr(stateEff, f"nameOfInertial{propType}Property")
    elif segment == 2:
        return getattr(stateEff, f"nameOfInertial{propType}Property2")
    elif segment == 4:
        try:
            propName = stateEff.ModelTag + "Inertial" + propType + "1_4"
            scObject.dynManager.getPropertyReference(propName)
        except BasiliskError:
            return "notHandedCorrectly"
        return propName

def getModernStateEffInertialPropName(scObject, segment, stateEff, propType):
    try:
        if segment == 1:
            propName = stateEff.ModelTag + "Inertial" + propType + "1"
            scObject.dynManager.getPropertyReference(propName)
        elif segment == 4:
            propName = stateEff.ModelTag + "Inertial" + propType + "1_4"
            scObject.dynManager.getPropertyReference(propName)
    except BasiliskError:
        return "notHandedCorrectly"
    return propName

def setup_extForceTorque():
    extFT = extForceTorque.ExtForceTorque()
    extFT.extForce_B = [[1.0], [1.0], [1.0]]
    extFT.extTorquePntB_B = [[1.0], [1.0], [1.0]]
    extFT.ModelTag = "extForceTorque"

    return(extFT)

def setup_extPulseTorque():
    extPT = ExtPulsedTorque.ExtPulsedTorque()
    extPT.countOnPulse = 1
    extPT.countOff = 1
    extPT.pulsedTorqueExternalPntB_B = [[1], [1], [1]]
    extPT.ModelTag = "extPulseTorque"

    return(extPT)

def setup_thrusterDynamicEffector():
    thruster = thrusterDynamicEffector.ThrusterDynamicEffector()
    thFactory = simIncludeThruster.thrusterFactory()
    thFactory.create('MOOG_Monarc_22_6', [0, 0, 0], [0, -1.5, 0])

    thrMsgData = messaging.THRArrayOnTimeCmdMsgPayload(OnTimeRequest=[0, 0, 0])
    thrMsg = messaging.THRArrayOnTimeCmdMsg()
    thrMsg.write(thrMsgData)
    thruster.cmdsInMsg.subscribeTo(thrMsg)

    return(thruster, thFactory)

def setup_constraintEffector(scObject1):
    scObject2 = spacecraft.Spacecraft()
    scObject2.ModelTag = "spacecraftBody2"

    # Sync dynamics integration across both spacecraft
    scObject1.syncDynamicsIntegration(scObject2)

    scObject2.hub.mHub = 750.0
    scObject2.hub.IHubPntBc_B = [[600.0, 0.0, 0.0], [0.0, 600.0, 0.0], [0.0, 0.0, 600.0]]

    scObject2.gravField.gravBodies = scObject1.gravField.gravBodies

    return scObject2

def setup_constraintEffectorOneHub(scObjecty, stateEffProps):
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()
    constraintEffector.ModelTag = "constraintEffectorOneHub"

    scObjectx = setup_constraintEffector(scObjecty)

    # Attached to the state effector of spacecraft y and the hub of spacecraft x
    r_P1Bx_Bx = [[1.0], [0.0], [0.0]]  # attachment point on spacecraft x's hub Bx
    r_P2S_S = [[-1.0], [0.0], [0.0]] # attachment point on spacecraft y's state effector frame S
    r_P2P1_BxInit = [[1.0], [0.0], [0.0]]  # connect arm between attachment points, in the S frame

    # assume r_BcB_B for spacecraft y is set s.t. r_CN_N = r_BN_N and all frames start aligned
    r_BxN_N_0 = np.array(scObjecty.hub.r_CN_NInit) + stateEffProps.r_PB_B + r_P2S_S - r_P2P1_BxInit - r_P1Bx_Bx

    # let C be the frame at the combined COM of the two vehicles
    r_CN_N = (np.array(scObjecty.hub.r_CN_NInit) * (scObjecty.hub.mHub + stateEffProps.totalMass) + r_BxN_N_0
              * scObjectx.hub.mHub) / (scObjecty.hub.mHub + stateEffProps.totalMass
              + scObjectx.hub.mHub)
    r_ByC_N = scObjecty.hub.r_CN_NInit - r_CN_N
    r_BxC_N = r_BxN_N_0 - r_CN_N

    # Set the initial values for spacecraft states, augmenting angular velocity
    scObjectx.hub.r_CN_NInit = r_BxN_N_0
    scObjectx.hub.v_CN_NInit = np.array(scObjecty.hub.v_CN_NInit).flatten() + np.cross(np.array(scObjecty.hub.omega_BN_BInit).flatten(),r_BxC_N.flatten())

    scObjectx.hub.omega_BN_BInit = scObjecty.hub.omega_BN_BInit
    scObjecty.hub.v_CN_NInit = np.array(scObjecty.hub.v_CN_NInit).flatten() + np.cross(np.array(scObjecty.hub.omega_BN_BInit).flatten(),r_ByC_N.flatten())

    # Create the constraint effector module
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()
    # Set up the constraint effector
    constraintEffector.ModelTag = "constraintEffector"
    constraintEffector.setR_P1B1_B1(r_P1Bx_Bx)
    constraintEffector.setR_P2B2_B2(r_P2S_S)
    constraintEffector.setR_P2P1_B1Init(r_P2P1_BxInit)
    constraintEffector.setAlpha(1E1)
    constraintEffector.setBeta(1e1)

    # Add constraints to both spacecraft
    scObjectx.addDynamicEffector(constraintEffector)

    return (constraintEffector, scObjectx)

def setup_constraintEffectorNoHubs(scObjecty, stateEffPropsy):
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()
    constraintEffector.ModelTag = "constraintEffectorNoHub"

    scObjectx = setup_constraintEffector(scObjecty)

    stateEffx, stateEffPropsx = setup_spinningBodiesOneDOF()
    scObjectx.addStateEffector(stateEffx)

    # Attached to the state effector of spacecraft y and the state effector of spacecraft x
    r_P1Sx_Sx = [[1.0], [0.0], [0.0]]  # attachment point on spacecraft x's state effector frame Sx
    r_P2Sy_Sy = [[-1.0], [0.0], [0.0]] # attachment point on spacecraft y's state effector frame Sy
    r_P2P1_SxInit = [[1.0], [0.0], [0.0]]  # connect arm between attachment points, in the Sx frame

    # assume r_BcB_B for spacecraft x & y are set s.t. r_CN_N = r_BN_N and all frames start aligned
    r_BxN_N_0 = np.array(scObjecty.hub.r_CN_NInit) + stateEffPropsy.r_PB_B + r_P2Sy_Sy - r_P2P1_SxInit - r_P1Sx_Sx - stateEffPropsx.r_PB_B

    # let C be the frame at the combined COM of the two vehicles
    r_CN_N = (np.array(scObjecty.hub.r_CN_NInit) * (scObjecty.hub.mHub + stateEffPropsy.totalMass)
              + r_BxN_N_0 * (scObjectx.hub.mHub + stateEffPropsx.totalMass)) / (scObjecty.hub.mHub
              + stateEffPropsy.totalMass + scObjectx.hub.mHub + stateEffPropsx.totalMass)
    r_ByC_N = scObjecty.hub.r_CN_NInit - r_CN_N
    r_BxC_N = r_BxN_N_0 - r_CN_N

    # Set the initial values for spacecraft states, augmenting angular velocity
    scObjectx.hub.r_BcB_B = stateEffPropsx.mr_PcB_B / scObjectx.hub.mHub
    scObjectx.hub.r_CN_NInit = r_BxN_N_0
    scObjectx.hub.v_CN_NInit = np.array(scObjecty.hub.v_CN_NInit).flatten() + np.cross(np.array(scObjecty.hub.omega_BN_BInit).flatten(),r_BxC_N.flatten())

    scObjectx.hub.omega_BN_BInit = scObjecty.hub.omega_BN_BInit
    scObjecty.hub.v_CN_NInit = np.array(scObjecty.hub.v_CN_NInit).flatten() + np.cross(np.array(scObjecty.hub.omega_BN_BInit).flatten(),r_ByC_N.flatten())

    # Create the constraint effector module
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()
    # Set up the constraint effector
    constraintEffector.ModelTag = "constraintEffector"
    constraintEffector.setR_P1B1_B1(r_P1Sx_Sx)
    constraintEffector.setR_P2B2_B2(r_P2Sy_Sy)
    constraintEffector.setR_P2P1_B1Init(r_P2P1_SxInit)
    constraintEffector.setAlpha(1E1)
    constraintEffector.setBeta(1e1)

    # Add constraints to both spacecraft
    scObjectx.addDynamicEffector(constraintEffector)

    return (constraintEffector, scObjectx, stateEffx)

def setup_spinningBodiesOneDOF():
    spinningBody = spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector()

    # Define properties of spinning body
    spinningBody.mass = 50.0
    spinningBody.IPntSc_S = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]
    spinningBody.dcm_S0B = [[0.0, -1.0, 0.0], [0.0, .0, -1.0], [1.0, 0.0, 0.0]]
    spinningBody.r_ScS_S = [[1.0], [0.0], [-1.0]]
    spinningBody.r_SB_B = [[0.5], [-1.5], [-0.5]]
    spinningBody.sHat_S = [[0], [-1], [0]]
    spinningBody.thetaInit = 5.0 * macros.D2R
    spinningBody.thetaDotInit = -1.0 * macros.D2R
    spinningBody.k = 100.0
    spinningBody.c = 50
    spinningBody.ModelTag = "SpinningBody"

    # Compute COM offset contribution, to be divided by the hub mass
    mr_ScB_B = -(spinningBody.r_SB_B + np.transpose(spinningBody.dcm_S0B) @ spinningBody.r_ScS_S) * spinningBody.mass

    stateEffProps = stateEffectorProperties()
    stateEffProps.totalMass = spinningBody.mass
    stateEffProps.mr_PcB_B = mr_ScB_B
    stateEffProps.r_PB_B = spinningBody.r_SB_B
    stateEffProps.r_PcP_P = spinningBody.r_ScS_S
    stateEffProps.inertialPropLogName = "spinningBodyConfigLogOutMsg"

    return(spinningBody, stateEffProps)

def setup_spinningBodiesTwoDOF():
    spinningBody = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()

    # Define properties of spinning body
    spinningBody.mass1 = 100.0
    spinningBody.mass2 = 50.0
    spinningBody.IS1PntSc1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    spinningBody.IS2PntSc2_S2 = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]
    spinningBody.dcm_S10B = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody.dcm_S20S1 = [[0.0, -1.0, 0.0], [0.0, .0, -1.0], [1.0, 0.0, 0.0]]
    spinningBody.r_Sc1S1_S1 = [[1.0], [-0.5], [0.0]]
    spinningBody.r_Sc2S2_S2 = [[1.0], [0.0], [-1.0]]
    spinningBody.r_S1B_B = [[-1.0], [0.5], [-1.0]]
    spinningBody.r_S2S1_S1 = [[0.5], [-0.5], [-0.5]]
    spinningBody.s1Hat_S1 = [[0], [0], [1]]
    spinningBody.s2Hat_S2 = [[0], [-1], [0]]
    spinningBody.theta1DotInit = 1.0 * macros.D2R
    spinningBody.theta2DotInit = 1.0 * macros.D2R
    spinningBody.k1 = 1000.0
    spinningBody.k2 = 500.0
    spinningBody.c1 = 500
    spinningBody.c2 = 200
    spinningBody.ModelTag = "SpinningBody"

    # Compute COM offset contribution, to be divided by the hub mass
    mr_ScB_B = -( (spinningBody.r_S1B_B + np.transpose(spinningBody.dcm_S10B) @
                   spinningBody.r_Sc1S1_S1) * spinningBody.mass1 + (spinningBody.r_S1B_B +
                   np.transpose(spinningBody.dcm_S10B) @ (spinningBody.r_S2S1_S1 +
                   np.transpose(spinningBody.dcm_S20S1) @ spinningBody.r_Sc2S2_S2) )
                   * spinningBody.mass2)

    stateEffProps = stateEffectorProperties()
    stateEffProps.totalMass = spinningBody.mass1 + spinningBody.mass2
    stateEffProps.mr_PcB_B = mr_ScB_B
    stateEffProps.r_PB_B = spinningBody.r_S1B_B + np.transpose(spinningBody.dcm_S10B) @ spinningBody.r_S2S1_S1
    stateEffProps.r_PcP_P = spinningBody.r_Sc2S2_S2
    stateEffProps.inertialPropLogName = "spinningBodyConfigLogOutMsgs"

    return(spinningBody, stateEffProps)

def setup_spinningBodiesNDOF():
    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    numberOfSegments = 3 # 3 segments of 2DOF joints is really 6 spinning bodies here
    massSubPanel = 100.0 / numberOfSegments
    lengthSubPanel = 18.0 / numberOfSegments
    widthSubPanel =  3.0
    thicknessSubPanel = 0.3
    r_ScB_B = np.array([[0.0], [0.0], [0.0]])
    dcm_SB = np.array([[1.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0],
                       [0.0, 0.0, 1.0]])
    mr_ScB_B = 0.0

    for idx in range(numberOfSegments):
        spinningBody = spinningBodyNDOFStateEffector.SpinningBody()
        spinningBody.setMass(0.0)
        spinningBody.setISPntSc_S([[0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0]])
        spinningBody.setDCM_S0P([[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0],
                                 [0.0, 0.0, 1.0]])
        spinningBody.setR_ScS_S([[0.0], [lengthSubPanel / 2], [0.0]])
        if idx == 0:
            spinningBody.setR_SP_P([[0.0], [3 / 2], [3 / 2 - thicknessSubPanel / 2]])
        else:
            spinningBody.setR_SP_P([[0.0], [lengthSubPanel], 0.0])
        spinningBody.setSHat_S([[1], [0], [0]])
        spinningBody.setThetaInit(2.0 * macros.D2R)
        spinningBody.setThetaDotInit(-0.5 * macros.D2R)
        spinningBody.setK(10)
        spinningBody.setC(8)
        spinningBodyEffector.addSpinningBody(spinningBody)
        r_ScB_B += dcm_SB.transpose() @ spinningBody.getR_SP_P()
        dcm_SB = rbk.PRV2C(spinningBody.getThetaInit() * np.array(spinningBody.getSHat_S())) @ spinningBody.getDCM_S0P() @ dcm_SB

        spinningBody = spinningBodyNDOFStateEffector.SpinningBody()
        spinningBody.setMass(massSubPanel)
        spinningBody.setISPntSc_S([[massSubPanel / 12 * (lengthSubPanel ** 2 + thicknessSubPanel ** 2), 0.0, 0.0],
                                   [0.0, massSubPanel / 12 * (widthSubPanel ** 2 + thicknessSubPanel ** 2), 0.0],
                                   [0.0, 0.0, massSubPanel / 12 * (widthSubPanel ** 2 + lengthSubPanel ** 2)]])
        spinningBody.setDCM_S0P([[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0],
                                 [0.0, 0.0, 1.0]])
        spinningBody.setR_ScS_S([[0.0], [lengthSubPanel / 2], [0.0]])
        spinningBody.setR_SP_P([[0.0], [0.0], [0.0]])
        spinningBody.setSHat_S([[0], [1], [0]])
        spinningBody.setThetaInit(2.0 * macros.D2R)
        spinningBody.setThetaDotInit(-0.5 * macros.D2R)
        spinningBody.setK(1)
        spinningBody.setC(0.8)
        spinningBodyEffector.addSpinningBody(spinningBody)
        dcm_SB = rbk.PRV2C(spinningBody.getThetaInit() * np.array(spinningBody.getSHat_S())) @ spinningBody.getDCM_S0P() @ dcm_SB

        # Compute COM offset contribution, to be divided by the hub mass
        mr_ScB_B -= spinningBody.getMass() * (r_ScB_B + dcm_SB.transpose() @ spinningBody.getR_ScS_S())

    spinningBodyEffector.ModelTag = "spinningBody"

    stateEffProps = stateEffectorProperties()
    stateEffProps.totalMass = massSubPanel * numberOfSegments
    stateEffProps.mr_PcB_B = mr_ScB_B
    stateEffProps.r_PB_B = r_ScB_B - dcm_SB.transpose() @ spinningBody.getR_ScS_S()
    stateEffProps.r_PcP_P = spinningBody.getR_ScS_S()
    stateEffProps.inertialPropLogName = "spinningBodyConfigLogOutMsgs"

    return(spinningBodyEffector, stateEffProps)

def setup_hingedRigidBodyStateEffector():
    hingedBody = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define properties of HRB
    hingedBody.mass = 50.0
    hingedBody.IPntS_S = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]
    hingedBody.d = 1.0
    hingedBody.k = 100.0
    hingedBody.c = 50.0
    hingedBody.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    hingedBody.r_HB_B = [[0.5], [-1.5], [-0.5]]
    hingedBody.thetaInit = 5 * macros.D2R
    hingedBody.thetaDotInit = -1 * macros.D2R
    hingedBody.ModelTag = "HingedRigidBody"

    # Compute COM offset contribution, to be divided by the hub mass
    dcm_SH = rbk.euler2(hingedBody.thetaInit)
    s1_hat = np.array([[-1.0],[0.0],[0.0]])
    mr_ScB_B = -(hingedBody.r_HB_B + np.transpose(hingedBody.dcm_HB) @
                 (np.transpose(dcm_SH) @ (hingedBody.d * s1_hat))) * hingedBody.mass

    stateEffProps = stateEffectorProperties()
    stateEffProps.totalMass = hingedBody.mass
    stateEffProps.mr_PcB_B = mr_ScB_B
    stateEffProps.r_PB_B = hingedBody.r_HB_B
    stateEffProps.r_PcP_P = hingedBody.d * s1_hat
    stateEffProps.inertialPropLogName = "hingedRigidBodyConfigLogOutMsg"

    return(hingedBody, stateEffProps)

def setup_translatingBodiesOneDOF():
    translatingBody = linearTranslationOneDOFStateEffector.LinearTranslationOneDOFStateEffector()

    # Define properties of translating body
    translatingBody.setMass(20.0)
    translatingBody.setK(100.0)
    translatingBody.setC(50.0)
    translatingBody.setRhoInit(1.0)
    translatingBody.setRhoDotInit(0.05)
    translatingBody.setFHat_B([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody.setR_FcF_F([[-1.0], [1.0], [0.0]])
    translatingBody.setR_F0B_B([[-1.0], [1.0], [0.0]])
    translatingBody.setIPntFc_F([[50.0, 0.0, 0.0],
                                 [0.0, 80.0, 0.0],
                                 [0.0, 0.0, 60.0]])
    translatingBody.setDCM_FB([[0.0, -1.0, 0.0],
                               [0.0, 0.0, -1.0],
                               [1.0, 0.0, 0.0]])
    translatingBody.ModelTag = "linearTranslation"

    mr_ScB_B = -(translatingBody.getR_F0B_B() + np.transpose(translatingBody.getDCM_FB()) @
                 (translatingBody.getR_FcF_F() + translatingBody.getRhoInit() *
                  np.array(translatingBody.getFHat_B()))) * translatingBody.getMass()

    stateEffProps = stateEffectorProperties()
    stateEffProps.totalMass = translatingBody.getMass()
    stateEffProps.mr_PcB_B = mr_ScB_B
    stateEffProps.r_PB_B = translatingBody.getR_F0B_B()
    stateEffProps.r_PcP_P = translatingBody.getR_FcF_F()
    stateEffProps.inertialPropLogName = "translatingBodyConfigLogOutMsg"

    return(translatingBody, stateEffProps)

def setup_linearSpringMassDamper():
    linearSpring = linearSpringMassDamper.LinearSpringMassDamper()
    linearSpring.massInit = 50.0
    linearSpring.k = 100.0
    linearSpring.c = 50.0
    linearSpring.r_PB_B = [[1.0], [0.0], [0.0]]
    linearSpring.pHat_B = [[1.0], [0.0], [0.0]]
    linearSpring.rhoInit = 0.0
    linearSpring.rhoDotInit = 0.5
    linearSpring.ModelTag = "linearSpringMassDamper"

    # Compute COM offset contribution, to be divided by the hub mass
    mr_ScB_B = -(linearSpring.r_PB_B + linearSpring.rhoInit * np.array(linearSpring.pHat_B)) * linearSpring.massInit

    stateEffProps = stateEffectorProperties()
    stateEffProps.totalMass = linearSpring.massInit
    stateEffProps.mr_PcB_B = mr_ScB_B
    stateEffProps.r_PB_B = linearSpring.r_PB_B

    return(linearSpring, stateEffProps)

class stateEffectorProperties:
    # to be used in joint COM calculation
    totalMass = 0.0 # total mass of the effector (sum of all linkages)
    mr_PcB_B = [[0.0], [0.0], [0.0]] # sum(m_i * r_SiB_B) for i linkages, see rst documentation
    r_PB_B = [[0.0], [0.0], [0.0]] # frame origin for linkage that dynEff will be attached to
    # to be used in checking equations of motion
    inertialPropLogName = "" # name of inertial property output log message
    r_PcP_P = [[0.0], [0.0], [0.0]] # individual COM for linkage that dynEff will be attached to

if __name__ == "__main__":
    effectorBranchingIntegratedTest(True, "hingedRigidBodies", True, "extForceTorque", True)
