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

r"""
Overview
--------

This scenario demonstrates effector branching through a docking and berthing maneuver. See
:ref:`bskPrinciples-11` for the conceptual background on attaching dynamic effectors onto state
effectors. A chaser spacecraft carries a :ref:`linearTranslationOneDOFStateEffector` arm; a
:ref:`constraintDynamicEffector` is attached as a dynamic effector of the arm (branched
attachment) and to a separate target spacecraft, modeling a rigid grapple at the arm tip.

The scenario runs in three phases:

1. **Free-flight approach**: the chaser starts one meter from the target docking point and
   drifts toward it with the grappling constraint disabled. A Basilisk event monitors the
   tip-to-target separation and engages the constraint when the grapple enters the capture
   tolerance.
2. **Capture / settle**: the arm is extended at its initial length with the constraint engaged.
   The two vehicles relax to a constraint-consistent state.
3. **Berthing retraction**: the arm reference length is driven to zero through a prescribed
   motion profile. The arm retracts and pulls the target spacecraft in along with it via the
   branched constraint, producing a hard-docked configuration.

Throughout the simulation both spacecraft are integrated simultaneously with synchronized RKF45
integrators. The default ``run()`` invocation uses ``dynRateSeconds = 1e-4`` (the journal-paper
configuration); the ``run_all_tests.py`` harness passes 1e-3 for a ~10× speedup at the cost of
a small constraint-violation transient on the retraction step.

The script is found in the folder ``basilisk/examples`` and executed by::

    python3 scenarioRoboticGrappling.py

Note that the :ref:`constraintDynamicEffector` requires gain tuning, addressed in
:ref:`scenarioConstrainedDynamicsManeuverAnalysis`.

Illustration of Simulation Results
----------------------------------

The hub-separation plot tracks the inertial distance between the two vehicles, dropping from the
initial three-meter spacing to the docked two-meter separation as the arm retracts. The arm
extension plot shows the smoothed bang-bang retraction profile commanded to the
:ref:`prescribedLinearTranslation` profiler. The position-constraint plot shows the constraint
effector pulling the violation down to numerical zero on engagement; the post-capture zoom
isolates the engaged-constraint behavior on its own axis. The attitude-constraint plot stays at
numerical zero throughout because the two vehicles start aligned and no torques perturb them.

.. image:: /_images/Scenarios/scenarioRoboticGrapplingHubSeparation.svg
   :align: center

.. image:: /_images/Scenarios/scenarioRoboticGrapplingArmExtension.svg
   :align: center

.. image:: /_images/Scenarios/scenarioRoboticGrapplingPositionConstraint.svg
   :align: center

.. image:: /_images/Scenarios/scenarioRoboticGrapplingPositionConstraintPostCapture.svg
   :align: center

.. image:: /_images/Scenarios/scenarioRoboticGrapplingAttitudeConstraint.svg
   :align: center
"""

#
#   Basilisk Scenario Script
#
#   Purpose:            Demonstrate effector branching, docking, and berthing.
#   Author:             Andrew Morell
#   Creation Date:      Nov 14, 2025
#

import os
import time

import matplotlib.pyplot as plt
import numpy as np

from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import (constraintDynamicEffector, gravityEffector,
                                 linearTranslationOneDOFStateEffector,
                                 prescribedLinearTranslation,
                                 spacecraft, svIntegrators)
from Basilisk.utilities import (RigidBodyKinematics, SimulationBaseClass,
                                macros, vizSupport)

from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Phase durations
defaultDynRate = 1e-4  # [s]
settleDuration = 30.0  # [s]
berthDuration = 90.0  # [s]
retractionDuration = 30.0  # [s]
approachTimeout = 80.0  # [s]
vizRate = 0.05  # [s]
captureEventRate = 0.05  # [s]

# Arm geometry (canonical setup: alongtrack-ahead, no central body)
rArmHub = 1.0   # [m] chaser body offset to arm root
rArmInit = 1.0  # [m] initial arm extension (rho)
rArmFinal = 0.0  # [m] commanded arm extension at end of berthing
rTargetTip = 1.0  # [m] target hub to bsk-Sat -Z (Marmon ring) face
marmonRingRadius = 0.75  # [m] grapple offset from bsk-Sat prism axis to ring rim
marmonRingHeight = 0.1  # [m] how far the ring protrudes from the -Z face
approachGap = 1.0  # [m]
approachSpeed = 0.02  # [m/s]
captureTolerance = 5e-3  # [m]
armMass = 20.0  # [kg]
armDiameter = 0.08  # [m]
retractionSmoothingDuration = 1.0  # [s]
postCaptureSkipSeconds = 1.0  # [s] zoom plot start offset, lets the constraint settle from engage transient

# Vizard geometry: chaser bus sized to read at the same scale as Vizard's
# bskSat target (~2 m tall hex prism, ~2.3 m wide). bskSat itself is a
# fixed-size built-in model so the visual match comes from the chaser side.
servicerBusLength = 1.5  # [m]
servicerBusWidth = 1.5  # [m]
servicerBusHeight = 1.5  # [m]
dockingPortDiameter = 0.22  # [m]
dockingPortDepth = 0.08  # [m]
grappleMarkerDiameter = 0.16  # [m]
armVizSegmentCount = 16
armVizSegmentLength = rArmInit / armVizSegmentCount  # [m]
armBaseLength = rArmHub - servicerBusLength / 2.0  # [m]


class SimBaseClass(SimulationBaseClass.SimBaseClass):
    def __init__(self, dynRate=defaultDynRate):
        self.dynRateSec = dynRate  # [s]
        self.dynRateNanos = macros.sec2nano(dynRate)  # [ns]
        SimulationBaseClass.SimBaseClass.__init__(self)

        self.simTaskName = "simTask"
        self.vizTaskName = "vizTask"
        simProcessName = "simProcess"
        self.dynProcess = self.CreateNewProcess(simProcessName)
        self.dynProcess.addTask(self.CreateNewTask(self.simTaskName, self.dynRateNanos))
        self.dynProcess.addTask(self.CreateNewTask(self.vizTaskName, macros.sec2nano(vizRate)))


class GrappleVizGeometry(sysModel.SysModel):
    """
    Write Vizard-only states for the docking ports and telescoping arm links.

    The dynamics model exposes the arm tip as a translating body.  This helper
    converts the physical root-tip geometry into short visual links so Vizard
    displays a retracting arm instead of a fixed-length model attached to the
    moving tip.
    """

    def __init__(self, armSegmentCount, r_F0B_B, r_P2B2_B2, rHat):
        super().__init__()
        self.ModelTag = "grappleVizGeometry"
        self.armSegmentCount = armSegmentCount
        self.r_F0B_B = np.array(r_F0B_B)  # [m]
        self.r_P2B2_B2 = np.array(r_P2B2_B2)  # [m]
        self.rHat = np.array(rHat)

        self.chaserStateInMsg = messaging.SCStatesMsgReader()
        self.targetStateInMsg = messaging.SCStatesMsgReader()
        self.armTipStateInMsg = messaging.SCStatesMsgReader()

        self.armBaseOutMsg = messaging.SCStatesMsg()
        self.chaserPortOutMsg = messaging.SCStatesMsg()
        self.targetPortOutMsg = messaging.SCStatesMsg()
        self.grappleTipOutMsg = messaging.SCStatesMsg()
        self.armSegmentOutMsgs = [
            messaging.SCStatesMsg() for _ in range(self.armSegmentCount)
        ]

    def Reset(self, CurrentSimNanos):
        """Clear all visualization states before the first dynamics update."""
        zeroState = messaging.SCStatesMsgPayload()
        self.armBaseOutMsg.write(zeroState, CurrentSimNanos, self.moduleID)
        self.chaserPortOutMsg.write(zeroState, CurrentSimNanos, self.moduleID)
        self.targetPortOutMsg.write(zeroState, CurrentSimNanos, self.moduleID)
        self.grappleTipOutMsg.write(zeroState, CurrentSimNanos, self.moduleID)
        for segmentOutMsg in self.armSegmentOutMsgs:
            segmentOutMsg.write(zeroState, CurrentSimNanos, self.moduleID)

    def UpdateState(self, CurrentSimNanos):
        """Update the visualization-only arm and docking-port body states."""
        chaserState = self.chaserStateInMsg()
        targetState = self.targetStateInMsg()
        armTipState = self.armTipStateInMsg()

        rB1N_N = np.array(chaserState.r_BN_N)  # [m]
        rTip_N = np.array(armTipState.r_BN_N)  # [m]
        sigmaB1N = np.array(chaserState.sigma_BN)
        sigmaB2N = np.array(targetState.sigma_BN)
        dcm_NB1 = RigidBodyKinematics.MRP2C(sigmaB1N).transpose()
        dcm_NB2 = RigidBodyKinematics.MRP2C(sigmaB2N).transpose()

        rRoot_N = rB1N_N + dcm_NB1 @ self.r_F0B_B  # [m]
        rTargetDock_N = (np.array(targetState.r_BN_N)
                         + dcm_NB2 @ self.r_P2B2_B2)  # [m]
        rArmBase_N = (rB1N_N + dcm_NB1 @
                      ((servicerBusLength / 2.0 + armBaseLength / 2.0)
                       * self.rHat))  # [m]

        self.writeState(self.armBaseOutMsg, rArmBase_N, sigmaB1N,
                        CurrentSimNanos)
        self.writeState(self.chaserPortOutMsg, rRoot_N, sigmaB1N,
                        CurrentSimNanos)
        self.writeState(self.targetPortOutMsg, rTargetDock_N, sigmaB2N,
                        CurrentSimNanos)
        self.writeState(self.grappleTipOutMsg, rTip_N, sigmaB1N,
                        CurrentSimNanos)

        for idx, segmentOutMsg in enumerate(self.armSegmentOutMsgs):
            segmentFraction = (idx + 0.5) / self.armSegmentCount
            rSegment_N = rRoot_N + segmentFraction * (rTip_N - rRoot_N)  # [m]
            self.writeState(segmentOutMsg, rSegment_N, sigmaB1N,
                            CurrentSimNanos)

    def writeState(self, outMsg, r_BN_N, sigma_BN, CurrentSimNanos):
        """Write one inertial visualization state message."""
        state = messaging.SCStatesMsgPayload()
        state.r_BN_N = r_BN_N.tolist()
        state.sigma_BN = sigma_BN.tolist()
        outMsg.write(state, CurrentSimNanos, self.moduleID)


def run(show_plots, gain,
        dynRateSeconds=defaultDynRate,
        approachGapMeters=approachGap,
        settleDurationSeconds=settleDuration,
        berthDurationSeconds=berthDuration):
    """
    Args:
        show_plots (bool): Whether to display plots interactively.
        gain (float): Constraint effector Baumgarte gain (alpha = beta).
        dynRateSeconds (float): integration step. Default ``defaultDynRate``
            (1e-4 s) is the journal-paper configuration. The ``run_all_tests.py``
            harness passes 1e-3 s for a ~10× speedup at the cost of a slight
            constraint-violation transient at the start of the retraction phase.
        approachGapMeters (float): free-flight separation at simulation start.
            Default 1.0 m matches the journal-paper configuration. The test
            harness passes a smaller value to compress phase 1 wall time.
        settleDurationSeconds (float): hold time after capture before commanding
            arm retraction. Default 30 s.
        berthDurationSeconds (float): time after the retraction command before
            simulation stop. Default 90 s gives ample post-retraction settle;
            the test harness passes a shorter value covering only the active
            retraction segment plus a brief hold.
    """
    scSim = SimBaseClass(dynRate=dynRateSeconds)
    scSim.approachGapMeters = approachGapMeters
    scSim.settleDurationSeconds = settleDurationSeconds
    scSim.berthDurationSeconds = berthDurationSeconds
    createSpacecraft(scSim)
    defineInitialConditions(scSim)
    setUpTranslationEffector(scSim)
    setUpConstraintEffector(scSim, gain)
    logData(scSim)
    setUpCaptureEvent(scSim)

    if vizSupport.vizFound:
        setUpVizard(scSim)

    runSimulation(scSim)
    processData(scSim)
    return plotting(scSim, show_plots)


def createSpacecraft(scSim):
    scSim.scObject1 = spacecraft.Spacecraft()
    scSim.scObject1.ModelTag = "chaser"
    scSim.scObject2 = spacecraft.Spacecraft()
    scSim.scObject2.ModelTag = "target"

    scSim.AddModelToTask(scSim.simTaskName, scSim.scObject1)
    scSim.AddModelToTask(scSim.simTaskName, scSim.scObject2)

    # Use RKF45 on the chaser; sync target integration to it
    integratorObject = svIntegrators.svIntegratorRKF45(scSim.scObject1)
    scSim.scObject1.setIntegrator(integratorObject)
    integratorObject.this.disown()
    scSim.scObject1.syncDynamicsIntegration(scSim.scObject2)

    for sc in (scSim.scObject1, scSim.scObject2):
        sc.hub.mHub = 750.0
        sc.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
        sc.hub.IHubPntBc_B = [[900.0, 0.0, 0.0],
                              [0.0, 800.0, 0.0],
                              [0.0, 0.0, 600.0]]


def defineInitialConditions(scSim):
    """Place the vehicles collinear with a one-meter free-flight capture gap."""
    scSim.rHat = np.array([1.0, 0.0, 0.0])
    scSim.r_F0B_B = rArmHub * scSim.rHat       # arm root location in chaser B1 frame
    scSim.fHat_B = scSim.rHat                  # arm extension direction in chaser B1 frame
    # Target B2 is rotated so the bsk-Sat's -Z face (the visible Marmon
    # ring) points along inertial -X toward the chaser. With this MRP the
    # body -Z axis maps to inertial -X, so P2 (at -rTargetTip along body Z)
    # sits exactly at the ring center on the chaser-facing face.
    scSim.sigmaB2NInit = np.array([0.0, np.tan(np.pi / 8.0), 0.0])
    scSim.dcm_NB2_init = RigidBodyKinematics.MRP2C(scSim.sigmaB2NInit).transpose()
    # Grapple point on the Marmon ring rim (offset perpendicular to the
    # prism axis), not the ring center. The +Y body offset puts the chaser
    # off-axis from the bskSat as it approaches. The extra ring-height
    # term in -Z accounts for the ring standing proud of the face, so the
    # arm tip contacts the outer rim rather than sinking into the face.
    scSim.r_P2B2_B2 = np.array([0.0, marmonRingRadius,
                                -(rTargetTip + marmonRingHeight)])
    armTipInB1Init = scSim.r_F0B_B + rArmInit * scSim.fHat_B
    chaserMass = scSim.scObject1.hub.mHub + armMass  # [kg]
    targetMass = scSim.scObject2.hub.mHub  # [kg]
    chaserComFromHub = armMass * armTipInB1Init / chaserMass  # [m]
    # Push r_P2B2 into inertial via the target attitude before using it to
    # place the target hub. Keeps the inertial geometry (arm tip 1 m short
    # of the Marmon-ring face at sim start) independent of B2 orientation.
    r_P2B2_N_init = scSim.dcm_NB2_init @ scSim.r_P2B2_B2  # [m]
    r_C2C1 = (armTipInB1Init - r_P2B2_N_init - chaserComFromHub
              + scSim.approachGapMeters * scSim.rHat)  # [m]

    # Place the system COM at the inertial origin using total spacecraft masses.
    totalMass = chaserMass + targetMass  # [kg]
    scSim.r_C1C = -(targetMass * r_C2C1) / totalMass  # [m]
    scSim.r_C2C = scSim.r_C1C + r_C2C1  # [m]
    scSim.v_C1N = approachSpeed * scSim.rHat  # [m/s]

    scSim.scObject1.hub.r_CN_NInit = scSim.r_C1C
    scSim.scObject1.hub.v_CN_NInit = scSim.v_C1N
    scSim.scObject1.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scSim.scObject1.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]
    scSim.scObject2.hub.r_CN_NInit = scSim.r_C2C
    scSim.scObject2.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]
    scSim.scObject2.hub.sigma_BNInit = scSim.sigmaB2NInit.reshape(3, 1).tolist()
    scSim.scObject2.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]


def setUpTranslationEffector(scSim):
    arm = linearTranslationOneDOFStateEffector.LinearTranslationOneDOFStateEffector()
    arm.ModelTag = "arm"

    arm.setMass(armMass)
    arm.setK(100.0)
    arm.setC(50.0)
    arm.setRhoInit(rArmInit)
    arm.setRhoDotInit(0.0)
    arm.setFHat_B(scSim.rHat)
    arm.setR_FcF_F([0.0, 0.0, 0.0])
    arm.setR_F0B_B(rArmHub * scSim.rHat)
    arm.setIPntFc_F([[50.0, 0.0, 0.0],
                     [0.0, 80.0, 0.0],
                     [0.0, 0.0, 60.0]])
    arm.setDCM_FB([[1.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0],
                   [0.0, 0.0, 1.0]])

    # Command message: held on scSim so we can re-write it during the berthing phase
    scSim.translationCommandMsg = messaging.LinearTranslationRigidBodyMsg()
    refPayload = messaging.LinearTranslationRigidBodyMsgPayload()
    refPayload.rho = rArmInit
    refPayload.rhoDot = 0.0
    scSim.translationCommandMsg.write(refPayload)

    translationProfiler = prescribedLinearTranslation.PrescribedLinearTranslation()
    translationProfiler.ModelTag = "armRetractProfiler"
    retractionAccel = (4.0 * abs(rArmFinal - rArmInit)
                       / (retractionDuration * retractionDuration))  # [m/s^2]
    translationProfiler.setTransHat_M(scSim.fHat_B)
    translationProfiler.setTransPosInit(rArmInit)
    translationProfiler.setTransAccelMax(retractionAccel)
    translationProfiler.setCoastOptionBangDuration(0.0)
    translationProfiler.setSmoothingDuration(retractionSmoothingDuration)
    translationProfiler.linearTranslationRigidBodyInMsg.subscribeTo(scSim.translationCommandMsg)
    arm.translatingBodyRefInMsg.subscribeTo(translationProfiler.linearTranslationRigidBodyOutMsg)

    scSim.scObject1.addStateEffector(arm)
    # Run the profiler before spacecraft dynamics so the arm reads the current reference.
    scSim.AddModelToTask(scSim.simTaskName, translationProfiler, ModelPriority=100)
    scSim.AddModelToTask(scSim.simTaskName, arm)
    scSim.translatingBody = arm
    scSim.translationProfiler = translationProfiler


def setUpConstraintEffector(scSim, gain):
    constraintEffector = constraintDynamicEffector.ConstraintDynamicEffector()
    constraintEffector.ModelTag = "grapple"
    constraintEffector.setR_P1B1_B1([0.0, 0.0, 0.0])
    constraintEffector.setR_P2B2_B2(scSim.r_P2B2_B2)
    constraintEffector.setR_P2P1_B1Init([0.0, 0.0, 0.0])
    # Attitude constraint target: lock B2 relative to B1 at the actual initial
    # relative orientation. The bskSat is rotated +pi/2 about +Y from the
    # chaser so the Marmon ring faces it; without this call the effector
    # would default to identity (B2 == B1) and immediately fight that 90°.
    # Chaser B1 = N initially (sigma_B1N = 0), so sigma_B2B1 = sigma_B2N.
    constraintEffector.setSigma_B2B1Init(scSim.sigmaB2NInit.tolist())
    constraintEffector.setAlpha(gain)
    constraintEffector.setBeta(gain)
    # The alpha/beta defaults make c_d = c_a = 2*beta, which puts the damping
    # ratio at ~0.04 (1/sqrt(m)) for spacecraft-scale masses. That was fine
    # when P2 sat on the bskSat axis (no torque from the constraint force),
    # but with the off-axis ring-rim grapple the lever arm couples the
    # position constraint into the attitude mode, which then rings near zero
    # damping. Override c_d and c_a explicitly for critical damping at the
    # actual reduced mass / inertia of the chaser-target pair.
    m1 = scSim.scObject1.hub.mHub + armMass
    m2 = scSim.scObject2.hub.mHub
    reducedMass = m1 * m2 / (m1 + m2)
    # Worst-case (smallest) target inertia component for the relative attitude.
    I_target_min = min(scSim.scObject2.hub.IHubPntBc_B[0][0],
                       scSim.scObject2.hub.IHubPntBc_B[1][1],
                       scSim.scObject2.hub.IHubPntBc_B[2][2])
    k_d = gain ** 2
    k_a = gain ** 2
    constraintEffector.setC_d(2.0 * np.sqrt(k_d * reducedMass))
    constraintEffector.setC_a(2.0 * np.sqrt(k_a * I_target_min))

    scSim.constraintStatusMsg = messaging.DeviceStatusMsg()
    statusPayload = messaging.DeviceStatusMsgPayload()
    statusPayload.deviceStatus = 0
    scSim.constraintStatusMsg.write(statusPayload)
    constraintEffector.effectorStatusInMsg.subscribeTo(scSim.constraintStatusMsg)

    # Branched attachment: constraint is a dynamic effector of the arm tip on the chaser side
    scSim.translatingBody.addDynamicEffector(constraintEffector)
    scSim.scObject2.addDynamicEffector(constraintEffector)
    scSim.AddModelToTask(scSim.simTaskName, constraintEffector)
    scSim.constraintEffector = constraintEffector


def setUpCaptureEvent(scSim):
    """Create an event that engages the grapple inside the capture tolerance."""
    scSim.captureEventName = "engageGrapple"
    scSim.constraintEngaged = False
    scSim.captureTime = None
    scSim.captureTimeNanos = None
    scSim.captureDistance = None

    scSim.createNewEvent(scSim.captureEventName,
                         eventRate=macros.sec2nano(captureEventRate),
                         eventActive=True,
                         conditionFunction=captureCondition,
                         actionFunction=engageConstraint,
                         terminal=True,
                         exactRateMatch=False)


def captureCondition(scSim):
    """Return ``True`` when the arm tip is close enough to the target dock."""
    return computeDockingDistance(scSim) <= captureTolerance


def engageConstraint(scSim):
    """Enable the grapple constraint and record the capture state."""
    statusPayload = messaging.DeviceStatusMsgPayload()
    statusPayload.deviceStatus = 1
    scSim.constraintStatusMsg.write(statusPayload, scSim.TotalSim.CurrentNanos)
    scSim.constraintEngaged = True
    scSim.captureTime = scSim.TotalSim.CurrentNanos * macros.NANO2SEC  # [s]
    scSim.captureTimeNanos = scSim.TotalSim.CurrentNanos  # [ns]
    scSim.captureDistance = computeDockingDistance(scSim)  # [m]


def computeDockingDistance(scSim):
    """Compute the inertial separation between the arm tip and target dock."""
    tipState = scSim.translatingBody.translatingBodyConfigLogOutMsg.read()
    targetState = scSim.scObject2.scStateOutMsg.read()

    rTip_N = np.array(tipState.r_BN_N)  # [m]
    rB2N_N = np.array(targetState.r_BN_N)  # [m]
    sigmaB2N = np.array(targetState.sigma_BN)
    dcm_NB2 = RigidBodyKinematics.MRP2C(sigmaB2N).transpose()
    rP2N_N = rB2N_N + dcm_NB2 @ scSim.r_P2B2_B2  # [m]

    return np.linalg.norm(rP2N_N - rTip_N)  # [m]


def logData(scSim):
    scSim.datLog1 = scSim.scObject1.scStateOutMsg.recorder()
    scSim.datLog2 = scSim.scObject2.scStateOutMsg.recorder()
    scSim.armLog = scSim.translatingBody.translatingBodyOutMsg.recorder()
    scSim.cnstLog = scSim.constraintEffector.constraintElements.recorder()
    scSim.AddModelToTask(scSim.simTaskName, scSim.datLog1)
    scSim.AddModelToTask(scSim.simTaskName, scSim.datLog2)
    scSim.AddModelToTask(scSim.simTaskName, scSim.armLog)
    scSim.AddModelToTask(scSim.simTaskName, scSim.cnstLog)


def setUpVizard(scSim):
    # Avoid stale custom model overrides when this scenario is rerun interactively.
    del vizSupport.customModelList[:]

    vizGeometry = GrappleVizGeometry(armVizSegmentCount,
                                     scSim.r_F0B_B,
                                     scSim.r_P2B2_B2,
                                     scSim.rHat)
    vizGeometry.chaserStateInMsg.subscribeTo(scSim.scObject1.scStateOutMsg)
    vizGeometry.targetStateInMsg.subscribeTo(scSim.scObject2.scStateOutMsg)
    vizGeometry.armTipStateInMsg.subscribeTo(
        scSim.translatingBody.translatingBodyConfigLogOutMsg
    )
    scSim.AddModelToTask(scSim.simTaskName, vizGeometry, ModelPriority=-100)
    scSim.vizGeometry = vizGeometry

    armSegmentBodies = [
        ["armLink" + str(idx + 1), vizGeometry.armSegmentOutMsgs[idx]]
        for idx in range(armVizSegmentCount)
    ]
    scBodyList = [scSim.scObject1,
                  ["armBase", vizGeometry.armBaseOutMsg],
                  ["chaserDockPort", vizGeometry.chaserPortOutMsg]]
    scBodyList.extend(armSegmentBodies)
    scBodyList.extend([
        ["grappleTip", vizGeometry.grappleTipOutMsg],
        scSim.scObject2,
    ])

    viz = vizSupport.enableUnityVisualization(scSim, scSim.vizTaskName,
                                              scBodyList,
                                              saveFile=fileName)
    vizSupport.createCustomModel(viz,
                                 simBodiesToModify=[scSim.scObject1.ModelTag],
                                 modelPath='CUBE',
                                 scale=(servicerBusLength,
                                        servicerBusWidth,
                                        servicerBusHeight),
                                 color=vizSupport.toRGBA255("gray"))
    vizSupport.createCustomModel(viz,
                                 simBodiesToModify=["armBase"],
                                 modelPath='CUBE',
                                 scale=(armBaseLength,
                                        1.4 * armDiameter,
                                        1.4 * armDiameter),
                                 color=vizSupport.toRGBA255("dimgray"))
    vizSupport.createCustomModel(viz,
                                 simBodiesToModify=["chaserDockPort"],
                                 modelPath='CYLINDER',
                                 rotation=(0.0, np.pi / 2.0, 0.0),
                                 scale=(dockingPortDiameter,
                                        dockingPortDiameter,
                                        dockingPortDepth / 2.0),
                                 color=vizSupport.toRGBA255("darkgray"))
    for idx in range(armVizSegmentCount):
        vizSupport.createCustomModel(viz,
                                     simBodiesToModify=["armLink" + str(idx + 1)],
                                     modelPath='CUBE',
                                     scale=(armVizSegmentLength,
                                            armDiameter,
                                            armDiameter),
                                     color=vizSupport.toRGBA255("gold"))
    vizSupport.createCustomModel(viz,
                                 simBodiesToModify=["grappleTip"],
                                 modelPath='SPHERE',
                                 scale=(grappleMarkerDiameter,
                                        grappleMarkerDiameter,
                                        grappleMarkerDiameter),
                                 color=vizSupport.toRGBA255("blue"))
    # Target spacecraft uses Vizard's default bsk-Sat model. The B2 frame
    # is rotated so the -Z face (Marmon ring) points toward the chaser and
    # the arm contacts the ring center.
    viz.settings.orbitLinesOn = -1
    viz.settings.showSpacecraftLabels = 1
    # Snap the main camera to the chaser at startup so playback isn't
    # left in free-fly inertial mode. In deep space (no celestial body)
    # this is the only hook Vizard's main camera has for following the
    # action. Double-click-snap relies on a celestial-body-relative
    # scene that this scenario doesn't have.
    viz.settings.mainCameraTarget = scSim.scObject1.ModelTag


def runSimulation(scSim):
    t0 = time.time()
    scSim.SetProgressBar(True)
    scSim.InitializeSimulation()

    # Phase 1: free-flight approach until the event engages the grapple.
    scSim.ConfigureStopTime(macros.sec2nano(approachTimeout))
    scSim.ExecuteSimulation()
    if not scSim.constraintEngaged:
        raise RuntimeError("Grapple capture event did not occur before approach timeout.")

    # Phase 2: settle at initial arm extension after capture.
    settleStopTimeNanos = scSim.captureTimeNanos + macros.sec2nano(scSim.settleDurationSeconds)  # [ns]
    scSim.ConfigureStopTime(settleStopTimeNanos)
    scSim.ExecuteSimulation()

    # Phase 3: command arm retraction, constraint draws target in.
    refPayload = messaging.LinearTranslationRigidBodyMsgPayload()
    refPayload.rho = rArmFinal
    refPayload.rhoDot = 0.0
    scSim.translationCommandMsg.write(refPayload, settleStopTimeNanos)

    berthStopTimeNanos = settleStopTimeNanos + macros.sec2nano(scSim.berthDurationSeconds)  # [ns]
    scSim.ConfigureStopTime(berthStopTimeNanos)
    scSim.SetProgressBar(True)
    scSim.ExecuteSimulation()

    scSim.runtime = time.time() - t0


def processData(scSim):
    scSim.timeData = scSim.datLog1.times() * macros.NANO2SEC
    nSteps = len(scSim.timeData)

    rB1N_N = scSim.datLog1.r_BN_N
    rB2N_N = scSim.datLog2.r_BN_N
    scSim.sigmaB1N = scSim.datLog1.sigma_BN
    scSim.sigmaB2N = scSim.datLog2.sigma_BN

    # Constraint diagnostics
    scSim.psi_N = scSim.cnstLog.psi_N

    # Berthing diagnostics: hub separation in inertial frame and arm extension
    scSim.hubSeparation = np.linalg.norm(rB2N_N - rB1N_N, axis=1)
    scSim.rho = scSim.armLog.rho

    # Constraint violation in B1 frame uses the *current* arm tip location, not the initial
    psiB1 = np.empty(rB1N_N.shape)
    sigmaB2B1 = np.empty(scSim.sigmaB1N.shape)
    # Constraint target for relative attitude. Chaser starts at identity so
    # sigma_B2B1Init equals sigma_B2NInit.
    sigmaB2B1Init = scSim.sigmaB2NInit
    for i in range(nSteps):
        dcm_B1N = RigidBodyKinematics.MRP2C(scSim.sigmaB1N[i, :])
        dcm_NB2 = RigidBodyKinematics.MRP2C(scSim.sigmaB2N[i, :]).transpose()
        rB1N_B1 = dcm_B1N @ rB1N_N[i, :]
        rB2N_B1 = dcm_B1N @ rB2N_N[i, :]
        rP2B2_B1 = dcm_B1N @ dcm_NB2 @ scSim.r_P2B2_B2
        armTipInB1 = scSim.r_F0B_B + scSim.rho[i] * scSim.fHat_B
        psiB1[i, :] = rB1N_B1 + armTipInB1 - (rB2N_B1 + rP2B2_B1)
        # Deviation from the constraint's prescribed relative attitude, not the
        # absolute relative attitude. With B2 deliberately rotated 90° from B1
        # at init, the absolute would sit flat at ~90° even when the constraint
        # is perfectly satisfied.
        sigmaB2B1Current = RigidBodyKinematics.subMRP(scSim.sigmaB2N[i, :],
                                                     scSim.sigmaB1N[i, :])
        sigmaB2B1[i, :] = RigidBodyKinematics.subMRP(sigmaB2B1Current,
                                                    sigmaB2B1Init)
    scSim.psi_B1 = psiB1
    scSim.sigmaB2B1 = sigmaB2B1


def applyFormalPlotStyle():
    largerSize = 20
    smallerSize = 18
    fontdict = {'family': 'serif', 'weight': 'normal', 'size': largerSize}
    plt.rc('font', **fontdict)
    plt.rc('axes', labelsize=largerSize)
    plt.rc('xtick', labelsize=smallerSize)
    plt.rc('ytick', labelsize=smallerSize)
    plt.rc('legend', fontsize=smallerSize)
    plt.rcParams['figure.figsize'] = (7, 6)


def annotateEventLine(ax, xVal, label, color, linestyle):
    """Draw vertical event line with rotated text label at bottom-right."""
    ax.axvline(xVal, color=color, linestyle=linestyle, linewidth=1)
    trans = ax.get_xaxis_transform()  # x in data coords, y in axes fraction
    ax.text(xVal, 0.03, ' ' + label, rotation=90,
            verticalalignment='bottom', horizontalalignment='left',
            transform=trans, fontsize=18)


def annotateEvents(ax, captureTimeMin, retractionStartMin):
    """Draw the grapple and berth event lines with vertical labels."""
    annotateEventLine(ax, captureTimeMin, 'grapple', '0.5', '--')
    annotateEventLine(ax, retractionStartMin, 'berth', 'k', ':')


def plotting(scSim, show_plots):
    applyFormalPlotStyle()
    plt.close("all")

    figureList = {}
    timeMin = scSim.timeData / 60.0  # [min]
    captureTimeMin = scSim.captureTime / 60.0  # [min]
    retractionStartMin = (scSim.captureTime + scSim.settleDurationSeconds) / 60.0  # [min]
    logFloor = np.finfo(float).eps

    # Hub separation (the headline berthing plot)
    plt.figure()
    plt.plot(timeMin, scSim.hubSeparation)
    plt.xlabel('time [min]')
    plt.ylabel('Hub separation ' r'$\|r_{B_2/B_1}\|$ [m]')
    plt.axis("tight")
    annotateEvents(plt.gca(), captureTimeMin, retractionStartMin)
    figureList[fileName + "HubSeparation"] = plt.gcf()

    # Arm extension
    plt.figure()
    plt.plot(timeMin, scSim.rho)
    plt.xlabel('time [min]')
    plt.ylabel(r'Arm extension $\rho$ [m]')
    plt.axis("tight")
    annotateEvents(plt.gca(), captureTimeMin, retractionStartMin)
    figureList[fileName + "ArmExtension"] = plt.gcf()

    # Constraint position violation (B1 frame), full timeline
    posConstraintViolation = np.linalg.norm(scSim.psi_B1, axis=1)
    plt.figure()
    plt.semilogy(timeMin, np.maximum(posConstraintViolation, logFloor))
    plt.xlabel('time [min]')
    plt.ylabel(r'Position constraint violation $\psi$ [m]')
    plt.axis("tight")
    annotateEvents(plt.gca(), captureTimeMin, retractionStartMin)
    figureList[fileName + "PositionConstraint"] = plt.gcf()

    # Post-capture position constraint: starts at grapple so the engage
    # transient, steady-state hold, and retraction event are all in frame.
    postMask = scSim.timeData >= scSim.captureTime
    plt.figure()
    plt.semilogy(timeMin[postMask],
                 np.maximum(posConstraintViolation[postMask], logFloor))
    plt.xlabel('time [min]')
    plt.ylabel(r'Position constraint violation $\psi$ [m]')
    plt.axis("tight")
    annotateEvents(plt.gca(), captureTimeMin, retractionStartMin)
    postCaptureXlim = plt.gca().get_xlim()
    figureList[fileName + "PositionConstraintPostCapture"] = plt.gcf()

    # Relative attitude constraint: same post-grapple window as the
    # post-capture position plot so the x-axes line up exactly.
    plt.figure()
    attConstraintViolation = np.linalg.norm(4 * np.arctan(scSim.sigmaB2B1) * macros.R2D,
                                            axis=1)
    plt.semilogy(timeMin[postMask],
                 np.maximum(attConstraintViolation[postMask], logFloor))
    plt.xlabel('time [min]')
    plt.ylabel(r'Attitude constraint violation $\phi$ [deg]')
    plt.xlim(postCaptureXlim)
    annotateEvents(plt.gca(), captureTimeMin, retractionStartMin)
    figureList[fileName + "AttitudeConstraint"] = plt.gcf()

    if show_plots:
        plt.show()

    plt.close("all")
    return figureList


if __name__ == "__main__":
    run(
        True,   # show_plots
        1e4,    # constraint Baumgarte gain
    )
