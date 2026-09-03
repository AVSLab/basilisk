#
#  ISC License
#
#  Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

r"""
Overview
--------

This scenario demonstrates *effector branching* for a thruster mounted on a two-degree-of-freedom
articulated arm. See :ref:`bskPrinciples-11` for the conceptual background on attaching dynamic
effectors onto state effectors. The arm is modeled with a single
:ref:`spinningBodyTwoDOFStateEffector`, attached to a rigid hub with stiffness and damping at
each axis. A :ref:`thrusterDynamicEffector` is rigidly mounted at the tip of the outer arm
segment.

The same simulation is executed twice with identical initial conditions and thruster command.
The two cases differ only in where the thruster's force and torque are applied to the multibody
system:

1. **Effector-branched (correct)**: the thruster is attached as a dynamic effector of the arm
   tip segment via ``addToSpacecraftSubcomponent``. The thrust force and torque are accumulated
   into the arm's equations of motion and propagate through the joints into the hub.

2. **Hub-direct (naive)**: the thruster is attached as a dynamic effector of the hub via
   ``addToSpacecraft``, located at the initial tip position and pointed along the initial tip
   thrust direction expressed in the body frame. This modeling approximation ignores the
   multibody coupling. The thruster does not move with the arm and applies its wrench at a
   fixed B-frame location and direction throughout the burn.

Comparing the hub state and joint-angle responses between the two cases isolates the
contribution of effector branching to the predicted dynamics.

The script is found in the folder ``basilisk/examples`` and executed by::

    python3 scenarioThrusterArm.py

Illustration of Simulation Results
----------------------------------

The arm is initialized with both joint angles at zero. Joint angles and rates are plotted with
both cases overlaid (solid for effector-branched, dashed for hub-direct). Hub inertial velocity
and body-frame angular velocity are plotted as the difference between the two cases, isolating
the modeling error that would be incurred by applying the thruster wrench directly to the hub.
Both cases also write a Vizard ``.bin`` (suffixed ``_branched`` and ``_direct``) for
side-by-side 3D playback.

.. image:: /_images/Scenarios/scenarioThrusterArmtheta.svg
   :align: center

.. image:: /_images/Scenarios/scenarioThrusterArmthetaDot.svg
   :align: center

.. image:: /_images/Scenarios/scenarioThrusterArmvelocityDiff.svg
   :align: center

.. image:: /_images/Scenarios/scenarioThrusterArmangularVelocityDiff.svg
   :align: center
"""

#
#   Basilisk Scenario Script
#
#   Purpose:            Demonstrate effector branching and articulated thrusters.
#   Author:             Andrew Morell
#   Creation Date:      Nov 14, 2025
#

import os
import matplotlib.pyplot as plt
import numpy as np

from Basilisk.utilities import SimulationBaseClass, vizSupport, simIncludeGravBody
from Basilisk.simulation import spacecraft, spinningBodyTwoDOFStateEffector, thrusterDynamicEffector, gravityEffector
from Basilisk.utilities import macros, orbitalMotion, simHelpers, simIncludeThruster
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.architecture import messaging

from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# Geometry/inertia constants shared across both cases
massSC = 400
diameter = 2
height = 4

armMass = 20
armLength = 1.5 * diameter
armWidth = 0.1
armThickness = 0.1

thrusterModel = 'MOOG_Monarc_5'
thrusterOnTime = 30.0  # s
simDuration = macros.min2nano(1)
simulationTimeStep = macros.sec2nano(0.01)


def setupSpinningBody():
    """Configure the 2DOF arm with the same parameters in both cases."""
    arm = spinningBodyTwoDOFStateEffector.SpinningBodyTwoDOFStateEffector()
    arm.ModelTag = "arm"

    segmentLength = armLength / 2.0
    armI = [[armMass / 12 * (segmentLength ** 2 + armThickness ** 2), 0.0, 0.0],
            [0.0, armMass / 12 * (armThickness ** 2 + armWidth ** 2), 0.0],
            [0.0, 0.0, armMass / 12 * (segmentLength ** 2 + armWidth ** 2)]]

    arm.mass1 = armMass
    arm.mass2 = armMass
    arm.IS1PntSc1_S1 = armI
    arm.IS2PntSc2_S2 = armI
    arm.dcm_S10B = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    arm.dcm_S20S1 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    arm.r_Sc1S1_S1 = [[0.0], [armLength / 4], [0.0]]
    arm.r_Sc2S2_S2 = [[0.0], [armLength / 4], [0.0]]
    arm.r_S1B_B = [[0.0], [diameter / 2], [-height / 2 + armThickness / 2]]
    arm.r_S2S1_S1 = [[0.0], [armLength / 2], [0.0]]
    arm.s1Hat_S1 = [[1], [0], [0]]
    arm.s2Hat_S2 = [[1], [0], [0]]
    arm.k1 = 50.0
    arm.k2 = 50.0
    arm.c1 = 30.0
    arm.c2 = 30.0
    arm.theta1Init = 0.0 * macros.D2R
    arm.theta2Init = 0.0 * macros.D2R
    arm.theta1DotInit = 0.0 * macros.D2R
    arm.theta2DotInit = 0.0 * macros.D2R
    return arm


def initialTipFrameInB(arm):
    """Compute initial outer-segment tip position and S2-frame z-axis direction in B coords.

    Used to mount the hub-direct thruster at the same physical pose the
    effector-branched thruster has at t = 0.
    """
    s1Hat = np.array(arm.s1Hat_S1).flatten()
    s2Hat = np.array(arm.s2Hat_S2).flatten()
    dcm_S1S10 = rbk.PRV2C(arm.theta1Init * s1Hat)
    dcm_S2S20 = rbk.PRV2C(arm.theta2Init * s2Hat)
    dcm_S1B = dcm_S1S10 @ np.array(arm.dcm_S10B)
    dcm_S2S1 = dcm_S2S20 @ np.array(arm.dcm_S20S1)
    dcm_S2B = dcm_S2S1 @ dcm_S1B

    r_S1B_B = np.array(arm.r_S1B_B).flatten()
    r_S2S1_S1 = np.array(arm.r_S2S1_S1).flatten()
    r_T2S2_S2 = np.array([0.0, armLength / 2.0, 0.0])  # [m]

    r_PcB_B = r_S1B_B + dcm_S1B.T @ r_S2S1_S1 + dcm_S2B.T @ r_T2S2_S2  # [m]
    dir_B = dcm_S2B.T @ np.array([0.0, 0.0, 1.0])
    return r_PcB_B, dir_B


def runOneCase(useEffectorBranching):
    """Run a single thruster-arm simulation. Returns a dict of logged signals."""
    simTaskName = "simTask"
    simProcessName = "simProcess"

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess(simProcessName)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Hub
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "hub"
    scObject.hub.mHub = massSC
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[massSC / 16 * diameter ** 2 + massSC / 12 * height ** 2, 0.0, 0.0],
                                [0.0, massSC / 16 * diameter ** 2 + massSC / 12 * height ** 2, 0.0],
                                [0.0, 0.0, massSC / 8 * diameter ** 2]]

    # Gravity
    earthGravBody = gravityEffector.GravBodyData()
    earthGravBody.planetName = "earth_planet_data"
    earthGravBody.mu = 0.3986004415E+15
    earthGravBody.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGravBody])

    oe = orbitalMotion.ClassicElements()
    oe.a = earthGravBody.radEquator + 7500e3
    oe.e = 0.01
    oe.i = 30.0 * macros.D2R
    oe.Omega = 60.0 * macros.D2R
    oe.omega = 15.0 * macros.D2R
    oe.f = 90.0 * macros.D2R
    r_CN, rDot_CN = orbitalMotion.elem2rv(earthGravBody.mu, oe)

    scObject.hub.r_CN_NInit = r_CN
    scObject.hub.v_CN_NInit = rDot_CN
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.005], [-0.005], [0.005]]

    # 2DOF arm
    arm = setupSpinningBody()

    angle1Ref = messaging.HingedRigidBodyMsgPayload()
    angle1Ref.theta = 0.0
    angle1Ref.thetaDot = 0.0
    angle1RefMsg = messaging.HingedRigidBodyMsg().write(angle1Ref)
    arm.spinningBodyRefInMsgs[0].subscribeTo(angle1RefMsg)

    angle2Ref = messaging.HingedRigidBodyMsgPayload()
    angle2Ref.theta = 0.0 * macros.D2R
    angle2Ref.thetaDot = 0.0
    angle2RefMsg = messaging.HingedRigidBodyMsg().write(angle2Ref)
    arm.spinningBodyRefInMsgs[1].subscribeTo(angle2RefMsg)

    scObject.addStateEffector(arm)

    # Thruster
    thruster = thrusterDynamicEffector.ThrusterDynamicEffector()
    thFactory = simIncludeThruster.thrusterFactory()

    if useEffectorBranching:
        # Mount on the arm's tip segment; thrust direction in the S2 (parent) frame.
        thrLoc_parent = [0.0, armLength / 2.0, 0.0]  # [m]
        thrDir_parent = [0.0, 0.0, 1.0]
        thFactory.create(thrusterModel, thrLoc_parent, thrDir_parent)
        thFactory.addToSpacecraftSubcomponent(
            "armThruster", thruster, arm, 2, r_PcP_P=arm.r_Sc2S2_S2
        )
    else:
        # Mount on the hub at the arm tip's initial pose; direction frozen in B.
        r_PcB_B, dir_B = initialTipFrameInB(arm)
        thFactory.create(thrusterModel, r_PcB_B.tolist(), dir_B.tolist())
        thFactory.addToSpacecraft("armThruster", thruster, scObject)

    thrCmd = messaging.THRArrayOnTimeCmdMsgPayload()
    thrCmd.OnTimeRequest = [thrusterOnTime]
    thrCmdMsg = messaging.THRArrayOnTimeCmdMsg().write(thrCmd)
    thruster.cmdsInMsg.subscribeTo(thrCmdMsg)

    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, arm)
    scSim.AddModelToTask(simTaskName, thruster)

    # Recorders
    theta1Rec = arm.spinningBodyOutMsgs[0].recorder()
    theta2Rec = arm.spinningBodyOutMsgs[1].recorder()
    scStateRec = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, theta1Rec)
    scSim.AddModelToTask(simTaskName, theta2Rec)
    scSim.AddModelToTask(simTaskName, scStateRec)

    # Save a separate Vizard .bin per case for side-by-side playback
    if vizSupport.vizFound:
        caseSuffix = "_branched" if useEffectorBranching else "_direct"
        # Visual-only scaling: enlarge the arm cross-section so bending reads
        # clearly against the hub. Dynamics are unaffected.
        armVisualWidth = 0.3
        armVisualThickness = 0.3
        scBodyList = [scObject,
                      ["armSeg1", arm.spinningBodyConfigLogOutMsgs[0]],
                      ["armSeg2", arm.spinningBodyConfigLogOutMsgs[1]]]
        # In the branched case the thruster rides on arm segment 2 (index 2);
        # in the hub-direct case it is rigidly attached to the hub (index 0).
        thrList = [None] * 3
        thrList[2 if useEffectorBranching else 0] = [thruster]
        viz = vizSupport.enableUnityVisualization(
            scSim, simTaskName, scBodyList,
            thrEffectorList=thrList,
            saveFile=fileName + caseSuffix,
        )
        viz.settings.defaultThrusterPlumeLifeScalar = 0.25
        vizSupport.createCustomModel(viz,
                                     simBodiesToModify=[scObject.ModelTag],
                                     modelPath="CYLINDER",
                                     scale=[diameter, diameter, height / 2],
                                     color=vizSupport.toRGBA255("gray"))
        for tag in ("armSeg1", "armSeg2"):
            vizSupport.createCustomModel(viz,
                                         simBodiesToModify=[tag],
                                         modelPath="CUBE",
                                         scale=[armVisualWidth, armLength / 2, armVisualThickness],
                                         color=vizSupport.toRGBA255("gold"))
        # Thruster nozzle cone, drawn via the keep-out cone primitive (no
        # ghost body / no .obj asset). In the branched case it rides on
        # armSeg2 at the arm tip; in the hub-direct case it sits on the hub
        # at the fixed initial tip pose.
        if useEffectorBranching:
            r_PcS2_S2 = np.array(arm.r_Sc2S2_S2).flatten()
            coneFromBody = "armSeg2"
            conePosition_B = [0.0, armLength / 2.0 - r_PcS2_S2[1], 0.0]
        else:
            r_TipB_B, _ = initialTipFrameInB(arm)
            coneFromBody = scObject.ModelTag
            conePosition_B = r_TipB_B.tolist()
        # toBodyName is chosen as a body that geometrically cannot enter the
        # cone (the inner arm segment is on the +y side of the cone apex,
        # well outside the ~20° half-angle in -z). This keeps the cone in
        # its non-impinged opaque rendering throughout the burn.
        vizSupport.createConeInOut(viz,
                                   fromBodyName=coneFromBody,
                                   toBodyName="armSeg1",
                                   coneColor=vizSupport.toRGBA255("blue"),
                                   isKeepIn=False,
                                   position_B=conePosition_B,
                                   normalVector_B=[0.0, 0.0, -1.0],
                                   incidenceAngle=0.35,
                                   coneHeight=0.5,
                                   coneName="thrusterCone")
        # Main camera initially targets the hub; final framing is set
        # interactively in Vizard playback for high-resolution recording.
        viz.settings.mainCameraTarget = scObject.ModelTag
        viz.settings.orbitLinesOn = -1
        # Load the full .bin into memory at playback start. Without this the
        # cone's impingement state can flip when streamed chunks roll over.
        viz.settings.messageBufferSize = -1

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simDuration)
    scSim.ExecuteSimulation()

    return {
        'time': theta1Rec.times() * macros.NANO2SEC,
        'theta1': theta1Rec.theta,
        'theta1Dot': theta1Rec.thetaDot,
        'theta2': theta2Rec.theta,
        'theta2Dot': theta2Rec.thetaDot,
        'v_BN_N': scStateRec.v_BN_N,
        'omega_BN_B': scStateRec.omega_BN_B,
    }


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


def run(show_plots):
    """Execute the branched and hub-direct cases and overlay the results."""
    branched = runOneCase(useEffectorBranching=True)
    direct = runOneCase(useEffectorBranching=False)

    applyFormalPlotStyle()
    figureList = {}
    plt.close("all")

    # Joint angles
    plt.figure(1)
    plt.clf()
    plt.plot(branched['time'], macros.R2D * branched['theta1'], 'C0-', label=r'$\theta_1$ branched')
    plt.plot(branched['time'], macros.R2D * branched['theta2'], 'C1-', label=r'$\theta_2$ branched')
    plt.plot(direct['time'], macros.R2D * direct['theta1'], 'C0--', label=r'$\theta_1$ hub-direct')
    plt.plot(direct['time'], macros.R2D * direct['theta2'], 'C1--', label=r'$\theta_2$ hub-direct')
    plt.legend(loc='upper right')
    plt.xlabel('time [s]')
    plt.ylabel(r'$\theta$ [deg]')
    figureList[fileName + "theta"] = plt.figure(1)

    # Joint rates
    plt.figure(2)
    plt.clf()
    plt.plot(branched['time'], macros.R2D * branched['theta1Dot'], 'C0-', label=r'$\dot{\theta}_1$ branched')
    plt.plot(branched['time'], macros.R2D * branched['theta2Dot'], 'C1-', label=r'$\dot{\theta}_2$ branched')
    plt.plot(direct['time'], macros.R2D * direct['theta1Dot'], 'C0--', label=r'$\dot{\theta}_1$ hub-direct')
    plt.plot(direct['time'], macros.R2D * direct['theta2Dot'], 'C1--', label=r'$\dot{\theta}_2$ hub-direct')
    plt.legend()
    plt.xlabel('time [s]')
    plt.ylabel(r'$\dot{\theta}$ [deg/s]')
    figureList[fileName + "thetaDot"] = plt.figure(2)

    # Hub inertial velocity: modeling-error difference (branched - hub-direct)
    dv = branched['v_BN_N'] - direct['v_BN_N']
    plt.figure(3)
    plt.clf()
    for idx in range(3):
        plt.plot(branched['time'], dv[:, idx],
                 color=simHelpers.getLineColor(idx, 3),
                 label=r'$\Delta v_{BN,' + str(idx) + '}$')
    plt.legend()
    plt.xlabel('time [s]')
    plt.ylabel(r'$v_{BN}^{\rm branched} - v_{BN}^{\rm direct}$ [m/s]')
    figureList[fileName + "velocityDiff"] = plt.figure(3)

    # Hub body-frame angular velocity: modeling-error difference
    dw = branched['omega_BN_B'] - direct['omega_BN_B']
    plt.figure(4)
    plt.clf()
    for idx in range(3):
        plt.plot(branched['time'], dw[:, idx],
                 color=simHelpers.getLineColor(idx, 3),
                 label=r'$\Delta \omega_{BN,' + str(idx) + '}$')
    plt.legend()
    plt.xlabel('time [s]')
    plt.ylabel(r'$\omega_{BN}^{\rm branched} - \omega_{BN}^{\rm direct}$ [rad/s]')
    figureList[fileName + "angularVelocityDiff"] = plt.figure(4)

    if show_plots:
        plt.show()

    plt.close("all")
    return figureList


if __name__ == "__main__":
    run(True)
