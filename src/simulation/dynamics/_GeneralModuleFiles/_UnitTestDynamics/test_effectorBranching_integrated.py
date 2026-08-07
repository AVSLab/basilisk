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
import re
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
    facetDragDynamicEffector,
    radiationPressure,
    facetSRPDynamicEffector,
    MtbEffector,
)
from Basilisk.architecture import messaging

COARSE_TIMESTEP = 0.001  # [s]
FINE_TIMESTEP = 0.0005  # [s]

# uncomment this line if this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail()

# Note: effectors commented out as True are expected to be added in the future, effectors
#       commented out as False are not expected to be added in the future
@pytest.mark.parametrize("stateEffector, isParent", [
    ("hingedRigidBodies",             True),
    ("dualHingedRigidBodies",           True),
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
    ("facetDragDynamicEffector",  True),
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

    Neither of those checks can see an error inside the parent's own equations of motion. The
    parent's back-substitution stays internally consistent even when one of its terms is wrong, so
    the momentum it hands the hub still matches the momentum its own coordinates lose. Energy is
    not blind in the same way: a wrong generalized force does work that the applied load does not
    account for. All state effector dampers are therefore set to zero, and the change in the
    vehicle's rotational energy is compared against the work the child does about the vehicle
    center of mass,

    .. math::

        \Delta T_{\text{rot}} = \int_{t_0}^{t} \left[
        [\mathcal{NP}_j] {}^{\mathcal{P}_j}\!\boldsymbol{\tau}_{\text{ext},P_j}
        \cdot {}^{\mathcal{N}}\boldsymbol{\omega}_{\mathcal{P}_j/\mathcal{N}}
        + [\mathcal{NP}_j] {}^{\mathcal{P}_j}\mathbf{F}_{P_j} \cdot \left(
        {}^{\mathcal{N}}\mathbf{v}_{P_j/N} - {}^{\mathcal{N}}\mathbf{v}_{C/N}
        \right) \right] dt

    where the attachment point velocity is recovered from the logged segment center of mass,
    :math:`\mathbf{v}_{P_j/N} = \mathbf{v}_{Pc_j/N} + \boldsymbol{\omega}_{\mathcal{P}_j/\mathcal{N}}
    \times \left( - [\mathcal{NP}_j] {}^{\mathcal{P}_j}\mathbf{r}_{Pc_j/P_j} \right)`.

    Asserting only that this residual is small would be weak: the residual also carries
    discretization error, so the tolerance has a floor and any defect smaller than that floor is
    invisible unless the applied loads are raised until it clears. The test therefore asserts
    convergence instead. The residual is dominated by the trapezoid rule used for the work integral
    above, which is second order, so halving the step must drop it to roughly a quarter. A wrong
    term in the equations of motion contributes a residual that does not shrink with the step at
    all, and the ratio moves from a quarter towards one. The ratio is therefore required to sit
    near 0.25 rather than merely to decrease. The lower bound matters too: a ratio well under 0.25
    means the residual is falling faster than a trapezoid allows, which in practice means it has
    reached a floor and the check has stopped measuring the equations of motion. This criterion is
    independent of the load magnitude, so it does not have to be re-tuned when the loads or the
    step change.

    The accumulated delta V is compared against the internally computed delta V of the spacecraft
    center of mass.

    .. math::

        {}^{\mathcal{N}}\!\Delta v_{accum,C} = \int_{t_0}^{t} \frac{[\mathcal{NP}_j]
        {}^{\mathcal{P}_j}\mathbf{F}_{P_j}} {m_{Bc} + \sum_i m_{P_i}} dt

    where again :math:`j` is the segment that the dynamic effector is attached to among all
    :math:`i` segments. The sim 'truth' :math:`{}^{\mathcal{N}}\!\Delta v_{accum,C}` is logged from
    the spacecraft module.
    """

    coarseResidual = effectorBranchingIntegratedTest(show_plots, stateEffector, isParent,
                                                    dynamicEffector, isChild)
    if coarseResidual is None:
        return

    fineResidual = effectorBranchingIntegratedTest(False, stateEffector, isParent,
                                                  dynamicEffector, isChild, timestep=FINE_TIMESTEP)
    ratio = fineResidual / coarseResidual
    assert 0.20 < ratio < 0.35, (
        "branching energy work balance does not converge at second order: "
        "%.4e J at 1 ms and %.4e J at 0.5 ms, ratio %.3f rather than 0.25"
        % (coarseResidual, fineResidual, ratio))


@pytest.mark.parametrize("stateEffector", [
    "spinningBodiesOneDOF",
    "spinningBodiesTwoDOF",
    "spinningBodiesNDOF",
    "linearTranslationBodiesOneDOF",
])
def test_parentPublishesCurrentVelocityToChild(stateEffector):
    r"""
    **Validation Test Description**

    This test checks that a branching parent's published inertial velocity is current at the
    integrator substep on which its child evaluates, rather than at the previous task step.

    **Description of Variables Being Tested**

    The conservation checks in the test above cannot see an error here. They drive the parent with
    :ref:`extForceTorque`, whose load is constant, so a wrong published velocity reaches neither
    the trajectory nor the work integral. This test therefore attaches a child whose load is a
    function of that velocity, namely a :ref:`constraintDynamicEffector` between the parent's first
    segment and the hub of the same spacecraft, with its stiffness suppressed so that only the
    velocity feedback term contributes,

    .. math::

        \boldsymbol{\psi}' = {}^{\mathcal{N}}\dot{\mathbf{r}}_{P_2/P_1}
        - \boldsymbol{\omega}_{\mathcal{P}_1/\mathcal{N}} \times \mathbf{r}_{P_2/P_1}

    The parent's first segment is locked and started at rest, which makes that segment and the hub
    one rigid body. Both attachment points are then fixed in that body and
    :math:`\boldsymbol{\omega}_{\mathcal{P}_1/\mathcal{N}}` equals the hub rate, so the two terms
    cancel identically and the constraint force is zero for any hub attitude and rate. A published
    velocity that lags the current substep leaves the cancellation incomplete, and the residual
    appears as a force of order :math:`|\boldsymbol{\omega}_{\mathcal{B}/\mathcal{N}}|
    \, |\mathbf{r}_{P_1/B}|`. The assertion is therefore against zero rather than against a
    tolerance scaled to the step size or to the applied load.

    The hinged rigid bodies are absent because they expose no lock, so their panel always rotates
    relative to the hub and no such rigid configuration exists.
    """
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.SetProgressBar(False)
    testProc = unitTestSim.CreateNewProcess("TestProcess")
    testProc.addTask(unitTestSim.CreateNewTask("unitTask", macros.sec2nano(COARSE_TIMESTEP)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    integratorObject = svIntegrators.svIntegratorRKF45(scObject)
    scObject.setIntegrator(integratorObject)
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg*m^2]
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]  # [m]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]  # [m/s]
    scObject.hub.omega_BN_BInit = [[0.1], [0.1], [0.1]]  # [rad/s]

    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15  # [m^3/s^2]
    earthGravBody.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    setupByName = {
        "spinningBodiesOneDOF": setup_spinningBodiesOneDOF,
        "spinningBodiesTwoDOF": setup_spinningBodiesTwoDOF,
        "spinningBodiesNDOF": setup_spinningBodiesNDOF,
        "linearTranslationBodiesOneDOF": setup_translatingBodiesOneDOF,
    }
    stateEff, stateEffProps = setupByName[stateEffector]()
    weldParentSegmentToHub(stateEffector, stateEff)
    scObject.hub.r_BcB_B = stateEffProps.mr_PcB_B / scObject.hub.mHub  # [m]

    constraintEffector = setup_velocityConstraintEffector()
    # attach to the parent first so the constraint links its first property set to that segment
    stateEff.addDynamicEffector(constraintEffector, 1)
    scObject.addDynamicEffector(constraintEffector)
    scObject.addStateEffector(stateEff)

    unitTestSim.AddModelToTask("unitTask", stateEff)
    unitTestSim.AddModelToTask("unitTask", scObject)
    unitTestSim.AddModelToTask("unitTask", constraintEffector)
    constraintLog = constraintEffector.constraintElements.recorder()
    unitTestSim.AddModelToTask("unitTask", constraintLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(macros.sec2nano(10 * COARSE_TIMESTEP))
    unitTestSim.ExecuteSimulation()

    forceAccuracy = 1e-7  # [N]
    np.testing.assert_allclose(
        constraintLog.Fc_N, 0.0, atol=forceAccuracy,
        err_msg="child evaluated its load against a stale parent velocity property")


def effectorBranchingIntegratedTest(show_plots, stateEffector, isParent, dynamicEffector, isChild,
                                    timestep=COARSE_TIMESTEP):
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    #   Create a sim module as an empty container
    unitTestSim = SimulationBaseClass.SimBaseClass()
    unitTestSim.SetProgressBar(True)

    # Create test thread
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
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg*m^2]

    # Set the initial values for the states
    scObject.hub.r_CN_NInit = [[-4020338.690396649], [7490566.741852513], [5248299.211589362]]  # [m]
    scObject.hub.v_CN_NInit = [[-5199.77710904224], [-3436.681645356935], [1041.576797498721]]  # [m/s]
    scObject.hub.omega_BN_BInit = [[0.1], [0.1], [0.1]]  # [rad/s]

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
    elif stateEffector == "dualHingedRigidBodies":
        stateEff, stateEffProps = setup_dualHingedRigidBodies()
        segment = 2
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
    scObject.hub.r_BcB_B = stateEffProps.mr_PcB_B / scObject.hub.mHub  # [m]

    # Create the dynamic effector of interest
    if dynamicEffector == "extForceTorque":
        dynamicEff = setup_extForceTorque()
    elif dynamicEffector == "extPulseTorque":
        dynamicEff = setup_extPulseTorque()
    elif dynamicEffector == "thrusterDynamicEffector":
        dynamicEff, thFactory = setup_thrusterDynamicEffector()
    elif dynamicEffector == "facetDragDynamicEffector":
        dynamicEff = setup_facetDragDynamicEffector()
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
        # newer effector classes keep their names private, so validate the name handed to the child
        positionName = getModernStateEffInertialPropName(
            scObject, segment, "Position", getDynEffInertialPropName(dynamicEffector, dynamicEff, "Position"))
        velocityName = getModernStateEffInertialPropName(
            scObject, segment, "Velocity", getDynEffInertialPropName(dynamicEffector, dynamicEff, "Velocity"))
        attitudeName = getModernStateEffInertialPropName(
            scObject, segment, "Attitude", getDynEffInertialPropName(dynamicEffector, dynamicEff, "Attitude"))
        angvelocityName = getModernStateEffInertialPropName(
            scObject, segment, "AngVelocity", getDynEffInertialPropName(dynamicEffector, dynamicEff, "AngVelocity"))
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

    r_ScN_N_log = inertialPropLog.r_BN_N
    v_ScN_N_log = inertialPropLog.v_BN_N
    omega_SN_log = inertialPropLog.omega_BN_B
    rotEnergy = scObjectLog.totRotEnergy

    # Grab effector's attitude properties
    sigma_SN_log = inertialPropLog.sigma_BN

    # Compute conservation quantities using the state and dynamic effector's logged properties
    n = rotAngMom_N.shape[0]-1  # length of log minus one so the trapezoid integrals line up
    extTorque = np.empty((n,3))
    dV = np.empty((n,3))
    for idx in range(n):
        dcm_NS = np.transpose(rbk.MRP2C(sigma_SN_log[idx,:]))
        # Compute the total accumulated deltaV
        if idx == 0:
            dV[idx,:] = [0.0, 0.0, 0.0]
        else:
            dV[idx,:] = (dV[idx-1,:] + (dcm_NS @ np.array(dynamicEff.extForce_B).flatten()
                        + np.array(dynamicEff.extForce_N).flatten())
                       / (scObject.hub.mHub + stateEffProps.totalMass) * timestep)
        # Compute the total external torque on the vehicle
        extTorque[idx,:] = (dcm_NS @ np.array(dynamicEff.extTorquePntB_B).flatten()
                            + np.cross(r_ScN_N_log[idx,:] - dcm_NS
                            @ np.array(stateEffProps.r_PcP_P).flatten()
                            - datLog.r_CN_N[idx,:], dcm_NS
                            @ np.array(dynamicEff.extForce_B).flatten()
                            + np.array(dynamicEff.extForce_N).flatten()))

    rotPower = np.empty(n)
    for idx in range(n):
        dcm_NS = np.transpose(rbk.MRP2C(sigma_SN_log[idx,:]))
        F_N = (dcm_NS @ np.array(dynamicEff.extForce_B).flatten()
               + np.array(dynamicEff.extForce_N).flatten())
        tau_N = dcm_NS @ np.array(dynamicEff.extTorquePntB_B).flatten()
        omega_SN_N = dcm_NS @ omega_SN_log[idx,:]
        # the load acts at the parent frame origin, r_PcP_P from the segment center of mass
        v_PN_N = (v_ScN_N_log[idx,:]
                  + np.cross(omega_SN_N, -dcm_NS @ np.array(stateEffProps.r_PcP_P).flatten()))
        rotPower[idx] = tau_N.dot(omega_SN_N) + F_N.dot(v_PN_N - datLog.v_CN_N[idx,:])
    dErot = np.concatenate(([0.0], np.cumsum(0.5*(rotPower[1:] + rotPower[:-1])*timestep)))

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
    angMom_accuracy = 1e-5  # [kg*m^2/s]
    np.testing.assert_allclose(rotAngMom_N[:-1,:]-rotAngMom_N[0,:], dH, 0, atol=angMom_accuracy,
                               err_msg="angular momentum difference beyond accuracy limits")

    rotEnergy_accuracy = 5e-2  # [J]
    np.testing.assert_allclose((rotEnergy[:n] - rotEnergy[0]), dErot, atol=rotEnergy_accuracy,
                               err_msg="branching energy work balance beyond accuracy limits")
    energyResidual = abs((rotEnergy[n-1] - rotEnergy[0]) - dErot[-1])

    # Check deltaV difference against sim truth
    deltaV_accuracy = 1e-6  # [m/s]
    np.testing.assert_allclose(totAccumDV_N[:-1,:], dV, 0, atol=deltaV_accuracy,
                               err_msg="deltaV difference beyond accuracy limits")

    return energyResidual

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

def getModernStateEffInertialPropName(scObject, segment, propType, handedPropName):
    # a generated name ends in a process-global counter, so validate the handed name instead of rebuilding it
    if segment == 1:
        pattern = "linearTranslationInertial" + propType + r"[0-9]+"
    else:
        pattern = "spinningBodyInertial" + propType + r"[0-9]+_" + str(segment)
    try:
        scObject.dynManager.getPropertyReference(handedPropName)
    except BasiliskError:
        return "notHandedCorrectly"
    if re.fullmatch(pattern, handedPropName) is None:
        return "notHandedCorrectly"
    return handedPropName

def setup_extForceTorque():
    extFT = extForceTorque.ExtForceTorque()
    extFT.extForce_B = [[1.0], [1.0], [1.0]]  # [N]
    extFT.extForce_N = [[1.0], [1.0], [1.0]]  # [N]
    extFT.extTorquePntB_B = [[1.0], [1.0], [1.0]]  # [N*m]
    extFT.ModelTag = "extForceTorque"

    return(extFT)

def setup_extPulseTorque():
    extPT = ExtPulsedTorque.ExtPulsedTorque()
    extPT.countOnPulse = 1
    extPT.countOff = 1
    extPT.pulsedTorqueExternalPntB_B = [[1], [1], [1]]  # [N*m]
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

def setup_facetDragDynamicEffector():
    facetDrag = facetDragDynamicEffector.FacetDragDynamicEffector()
    facetDrag.ModelTag = "facetDragDynamicEffector"

    # facet geometry is expressed in the parent frame, not the hub body frame
    panelArea = 10.0  # [m^2]
    panelCd = 5.0  # [-]
    r_FP_P = np.array([0.0, 0.0, 0.3])  # [m] both faces share the panel centroid
    for panelNormal_P in [np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, -1.0])]:
        facetDrag.addFacet(panelArea, panelCd, panelNormal_P, r_FP_P)

    # the orbit shared by every case here sits above any atmosphere table, so feed the effector a
    # fixed density rather than an atmosphere model or the drag load is zero
    atmoMsgData = messaging.AtmoPropsMsgPayload()
    atmoMsgData.neutralDensity = 4.2e-10  # [kg/m^3] nominal 200 km density
    atmoMsg = messaging.AtmoPropsMsg()
    atmoMsg.write(atmoMsgData)
    facetDrag.atmoDensInMsg.subscribeTo(atmoMsg)

    return(facetDrag)

def setup_constraintEffector(scObject1):
    scObject2 = spacecraft.Spacecraft()
    scObject2.ModelTag = "spacecraftBody2"

    # Sync dynamics integration across both spacecraft
    scObject1.syncDynamicsIntegration(scObject2)

    scObject2.hub.mHub = 750.0  # [kg]
    scObject2.hub.IHubPntBc_B = [[600.0, 0.0, 0.0], [0.0, 600.0, 0.0], [0.0, 0.0, 600.0]]  # [kg*m^2]

    scObject2.gravField.gravBodies = scObject1.gravField.gravBodies

    return scObject2


def weldParentSegmentToHub(stateEffector, stateEff):
    # start the first segment at rest and lock it, so it and the hub move as one rigid body
    if stateEffector == "spinningBodiesOneDOF":
        stateEff.thetaDotInit = 0.0  # [rad/s]
        numberOfDegreesOfFreedom = 1
    elif stateEffector == "spinningBodiesTwoDOF":
        stateEff.theta1DotInit = 0.0  # [rad/s]
        numberOfDegreesOfFreedom = 2
    elif stateEffector == "spinningBodiesNDOF":
        numberOfDegreesOfFreedom = len(stateEff.spinningBodyOutMsgs)
        stateEff.getSpinningBody(0).setThetaDotInit(0.0)  # [rad/s]
    elif stateEffector == "linearTranslationBodiesOneDOF":
        stateEff.setRhoDotInit(0.0)  # [m/s]
        numberOfDegreesOfFreedom = 1
    else:
        pytest.fail("Weld requested for a state effector that exposes no lock.")

    lockArray = messaging.ArrayEffectorLockMsgPayload()
    lockArray.effectorLockFlag = [1] + [0] * (numberOfDegreesOfFreedom - 1)
    lockMsg = messaging.ArrayEffectorLockMsg().write(lockArray)
    stateEff.motorLockInMsg.subscribeTo(lockMsg)


def setup_velocityConstraintEffector():
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()
    constraintEffector.ModelTag = "velocityConstraintEffector"
    # body 1 is the parent segment and body 2 is the hub, each attached at its own frame origin
    constraintEffector.setR_P1B1_B1(np.zeros(3))  # [m]
    constraintEffector.setR_P2B2_B2(np.zeros(3))  # [m]
    constraintEffector.setR_P2P1_B1Init(np.zeros(3))  # [m]
    # stiffness suppressed so that only the velocity feedback term reaches the reported force
    constraintEffector.setK_d(1e-12)  # [N/m]
    constraintEffector.setC_d(1.0)  # [N*s/m]

    return constraintEffector


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
    spinningBody.mass = 50.0  # [kg]
    spinningBody.IPntSc_S = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]  # [kg*m^2]
    spinningBody.dcm_S0B = [[0.0, -1.0, 0.0], [0.0, .0, -1.0], [1.0, 0.0, 0.0]]
    spinningBody.r_ScS_S = [[1.0], [0.0], [-1.0]]  # [m]
    spinningBody.r_SB_B = [[0.5], [-1.5], [-0.5]]  # [m]
    spinningBody.sHat_S = [[0], [-1], [0]]
    spinningBody.thetaInit = 5.0 * macros.D2R  # [rad]
    spinningBody.thetaDotInit = -1.0 * macros.D2R  # [rad/s]
    spinningBody.k = 100.0  # [N*m/rad]
    spinningBody.c = 0.0  # [N*m*s/rad]
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
    spinningBody.mass1 = 100.0  # [kg]
    spinningBody.mass2 = 50.0  # [kg]
    spinningBody.IS1PntSc1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]  # [kg*m^2]
    spinningBody.IS2PntSc2_S2 = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]  # [kg*m^2]
    spinningBody.dcm_S10B = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    spinningBody.dcm_S20S1 = [[0.0, -1.0, 0.0], [0.0, .0, -1.0], [1.0, 0.0, 0.0]]
    spinningBody.r_Sc1S1_S1 = [[1.0], [-0.5], [0.0]]  # [m]
    spinningBody.r_Sc2S2_S2 = [[1.0], [0.0], [-1.0]]  # [m]
    spinningBody.r_S1B_B = [[-1.0], [0.5], [-1.0]]  # [m]
    spinningBody.r_S2S1_S1 = [[0.5], [-0.5], [-0.5]]  # [m]
    spinningBody.s1Hat_S1 = [[0], [0], [1]]
    spinningBody.s2Hat_S2 = [[0], [-1], [0]]
    spinningBody.theta1DotInit = 1.0 * macros.D2R  # [rad/s]
    spinningBody.theta2DotInit = 1.0 * macros.D2R  # [rad/s]
    spinningBody.k1 = 1000.0  # [N*m/rad]
    spinningBody.k2 = 500.0  # [N*m/rad]
    spinningBody.c1 = 0.0  # [N*m*s/rad]
    spinningBody.c2 = 0.0  # [N*m*s/rad]
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
    massSubPanel = 100.0 / numberOfSegments  # [kg]
    lengthSubPanel = 18.0 / numberOfSegments  # [m]
    widthSubPanel =  3.0  # [m]
    thicknessSubPanel = 0.3  # [m]
    r_ScB_B = np.array([[0.0], [0.0], [0.0]])
    dcm_SB = np.array([[1.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0],
                       [0.0, 0.0, 1.0]])
    mr_ScB_B = 0.0

    for idx in range(numberOfSegments):
        spinningBody = spinningBodyNDOFStateEffector.SpinningBody()
        spinningBody.setMass(0.0)  # [kg]
        spinningBody.setISPntSc_S([[0.0, 0.0, 0.0],  # [kg*m^2]
                                   [0.0, 0.0, 0.0],
                                   [0.0, 0.0, 0.0]])
        spinningBody.setDCM_S0P([[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0],
                                 [0.0, 0.0, 1.0]])
        spinningBody.setR_ScS_S([[0.0], [lengthSubPanel / 2], [0.0]])
        if idx == 0:
            spinningBody.setR_SP_P([[0.0], [3 / 2], [3 / 2 - thicknessSubPanel / 2]])  # [m]
        else:
            spinningBody.setR_SP_P([[0.0], [lengthSubPanel], 0.0])  # [m]
        spinningBody.setSHat_S([[1], [0], [0]])
        spinningBody.setThetaInit(2.0 * macros.D2R)
        spinningBody.setThetaDotInit(-0.5 * macros.D2R)
        spinningBody.setK(10)  # [N*m/rad]
        spinningBody.setC(0.0)  # [N*m*s/rad]
        spinningBodyEffector.addSpinningBody(spinningBody)
        r_ScB_B += dcm_SB.transpose() @ spinningBody.getR_SP_P()
        dcm_SB = rbk.PRV2C(spinningBody.getThetaInit() * np.array(spinningBody.getSHat_S())) @ spinningBody.getDCM_S0P() @ dcm_SB

        spinningBody = spinningBodyNDOFStateEffector.SpinningBody()
        spinningBody.setMass(massSubPanel)
        spinningBody.setISPntSc_S([[massSubPanel / 12 * (lengthSubPanel ** 2 + thicknessSubPanel ** 2), 0.0, 0.0],  # [kg*m^2]
                                   [0.0, massSubPanel / 12 * (widthSubPanel ** 2 + thicknessSubPanel ** 2), 0.0],
                                   [0.0, 0.0, massSubPanel / 12 * (widthSubPanel ** 2 + lengthSubPanel ** 2)]])
        spinningBody.setDCM_S0P([[1.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0],
                                 [0.0, 0.0, 1.0]])
        spinningBody.setR_ScS_S([[0.0], [lengthSubPanel / 2], [0.0]])
        spinningBody.setR_SP_P([[0.0], [0.0], [0.0]])  # [m]
        spinningBody.setSHat_S([[0], [1], [0]])
        spinningBody.setThetaInit(2.0 * macros.D2R)
        spinningBody.setThetaDotInit(-0.5 * macros.D2R)
        spinningBody.setK(1)  # [N*m/rad]
        spinningBody.setC(0.0)  # [N*m*s/rad]
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
    hingedBody.mass = 50.0  # [kg]
    hingedBody.IPntS_S = [[50.0, 0.0, 0.0], [0.0, 30.0, 0.0], [0.0, 0.0, 40.0]]  # [kg*m^2]
    hingedBody.d = 1.0  # [m]
    hingedBody.k = 100.0  # [N*m/rad]
    hingedBody.c = 0.0  # [N*m*s/rad]
    hingedBody.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    hingedBody.r_HB_B = [[0.5], [-1.5], [-0.5]]  # [m]
    hingedBody.thetaInit = 5 * macros.D2R  # [rad]
    hingedBody.thetaDotInit = -1 * macros.D2R  # [rad/s]
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

def setup_dualHingedRigidBodies():
    hingedBody = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()

    # Define properties of HRB
    hingedBody.mass1 = 200  # [kg]
    hingedBody.IPntS1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]  # [kg*m^2]
    hingedBody.d1 = 0.75  # [m]
    hingedBody.l1 = 1.5  # [m]
    hingedBody.k1 = 100  # [N*m/rad]
    hingedBody.c1 = 0.0  # [N*m*s/rad]
    hingedBody.dcm_H1B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    hingedBody.r_H1B_B = [[0.5], [-1.5], [-0.5]]  # [m]
    hingedBody.theta1Init = 5 * macros.D2R  # [rad]
    hingedBody.theta1DotInit = -1 * macros.D2R  # [rad/s]
    hingedBody.mass2 = 200  # [kg]
    hingedBody.IPntS2_S2 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]  # [kg*m^2]
    hingedBody.d2 = 1  # [m]
    hingedBody.k2 = 100  # [N*m/rad]
    hingedBody.c2 = 0.0  # [N*m*s/rad]
    hingedBody.theta2Init = -2 * macros.D2R  # [rad]
    hingedBody.theta2DotInit = 0.0  # [rad/s]
    hingedBody.ModelTag = "HingedRigidBody2DOF"

    # Compute COM offset contribution, to be divided by the hub mass
    dcm_S1H1 = rbk.euler2(hingedBody.theta1Init)
    dcm_S2H1 = rbk.euler2(hingedBody.theta2Init + hingedBody.thetaH2S1) @ dcm_S1H1
    s1_hat = np.array([[-1.0],[0.0],[0.0]])
    r_H2B_B = (hingedBody.r_H1B_B + np.transpose(hingedBody.dcm_H1B)
               @ np.transpose(dcm_S1H1) @ (hingedBody.l1 * s1_hat))
    r_S1B_B = (hingedBody.r_H1B_B + np.transpose(hingedBody.dcm_H1B)
               @ np.transpose(dcm_S1H1) @ (hingedBody.d1 * s1_hat))
    r_S2B_B = (r_H2B_B + np.transpose(hingedBody.dcm_H1B)
               @ np.transpose(dcm_S2H1) @ (hingedBody.d2 * s1_hat))
    mr_ScB_B = -(r_S1B_B * hingedBody.mass1 + r_S2B_B * hingedBody.mass2)

    stateEffProps = stateEffectorProperties()
    stateEffProps.totalMass = hingedBody.mass1 + hingedBody.mass2
    stateEffProps.mr_PcB_B = mr_ScB_B
    stateEffProps.r_PB_B = r_H2B_B  # [m]
    stateEffProps.r_PcP_P = hingedBody.d2 * s1_hat
    stateEffProps.inertialPropLogName = "dualHingedRigidBodyConfigLogOutMsgs"

    return(hingedBody, stateEffProps)

def setup_translatingBodiesOneDOF():
    translatingBody = linearTranslationOneDOFStateEffector.LinearTranslationOneDOFStateEffector()

    # Define properties of translating body
    translatingBody.setMass(20.0)  # [kg]
    translatingBody.setK(100.0)  # [N/m]
    translatingBody.setC(0.0)  # [N*s/m]
    translatingBody.setRhoInit(1.0)  # [m]
    translatingBody.setRhoDotInit(0.05)  # [m/s]
    translatingBody.setFHat_B([[3.0 / 5.0], [4.0 / 5.0], [0.0]])
    translatingBody.setR_FcF_F([[-1.0], [1.0], [0.0]])  # [m]
    translatingBody.setR_F0B_B([[-1.0], [1.0], [0.0]])  # [m]
    translatingBody.setIPntFc_F([[50.0, 0.0, 0.0],  # [kg*m^2]
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
    stateEffProps.r_PB_B = translatingBody.getR_F0B_B()  # [m]
    stateEffProps.r_PcP_P = translatingBody.getR_FcF_F()
    stateEffProps.inertialPropLogName = "translatingBodyConfigLogOutMsg"

    return(translatingBody, stateEffProps)

def setup_linearSpringMassDamper():
    linearSpring = linearSpringMassDamper.LinearSpringMassDamper()
    linearSpring.massInit = 50.0  # [kg]
    linearSpring.k = 100.0  # [N/m]
    linearSpring.c = 0.0  # [N*s/m]
    linearSpring.r_PB_B = [[1.0], [0.0], [0.0]]  # [m]
    linearSpring.pHat_B = [[1.0], [0.0], [0.0]]
    linearSpring.rhoInit = 0.0  # [m]
    linearSpring.rhoDotInit = 0.5  # [m/s]
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
