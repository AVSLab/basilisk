#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
Review these prerequisite examples first:

#. :ref:`scenarioFlexiblePanel` for the classic flexible-panel model.
#. :ref:`scenarioHingedRigidBodyMuJoCo` for MuJoCo hinge and force setup.
#. :ref:`scenarioAttitudeFeedbackRWMuJoCo` for attitude control with MuJoCo.

This script demonstrates how to model a flexible, multi-segment solar panel
using MuJoCo dynamics via :ref:`MJScene<MJScene>` instead of the traditional
hub-centric Basilisk :ref:`spacecraft` dynamics. This scenario is a translation
of the classic Basilisk :ref:`scenarioFlexiblePanel` example.

Running the Example
-------------------

Use a Basilisk installation with MuJoCo support and the example dependencies
described in :ref:`bskInstall`. When building from source, enable
``--mujoco True``. With that Python environment active, run from the repository
root:

.. code-block:: console

   python examples/mujoco/scenarioFlexiblePanelMuJoCo.py

The script displays the result plots. When importing the scenario from
``examples/mujoco``, call ``run(showPlots=False)`` to run without displaying
windows; it still returns a dictionary of Matplotlib figures for inspection
or saving.

Model and Control Setup
-----------------------

The multi-body system is created programmatically as a MuJoCo XML string.
It consists of a free-floating spacecraft bus ("hub") with a single flexible
panel discretized into ``numberOfSegments`` rigid sub-panel bodies
("subPanel1", "subPanel2", ...), connected end-to-end. Each sub-panel is
connected to its neighbor by two colocated hinge joints: a "bend" joint
(bending degree of freedom, or DOF) and a "twist" joint (torsional DOF). The
panel's continuous flexibility is approximated by a discretized series of
rigid links. As such, this system has 6 + 2 * ``numberOfSegments`` DOFs
(3 translational, 3 rotational, 2 DOFs per panel segment).

A torsional spring-damper torque is applied at every bend and twist joint to
emulate the panel's structural stiffness and damping:

#. :ref:`MJJointPIDController <JointPIDController>` computes a restoring torque
   from a joint's angle and angular rate about a reference equilibrium angle,
   and writes the result as
   a ``SingleActuatorMsg`` command. One instance is attached to each bend and
   twist joint, using separate bending and torsional stiffness/damping
   coefficients.

A standard Basilisk flight software (FSW) stack is used to point the hub at a
fixed inertial attitude while the flexible panel dynamically responds to
the resulting motion:

#. :ref:`simpleNav` provides the spacecraft's navigation attitude solution.
#. :ref:`inertial3D` generates a fixed inertial attitude reference.
#. :ref:`attTrackingError` computes the attitude and rate tracking errors.
#. :ref:`mrpFeedback` computes the commanded body torque, using an inertia
   tensor for the hub plus panel computed via the parallel axis theorem.

A small adapter module bridges the FSW torque command to the MuJoCo torque
actuator:

#. :ref:`cmdTorqueBodyToTorqueAtSite` relays the commanded body-frame torque as a
   ``TorqueAtSiteMsg`` (site and body frames are aligned, so the default
   identity ``dcm_SB`` is used), consumed by a torque actuator at the hub site.

Earth gravity is configured using :ref:`NBodyGravity<NBodyGravity>` with a
:ref:`pointMassGravityModel<pointMassGravityModel>` as the central body.
Gravity targets are registered manually for the hub and every sub-panel body.

The spacecraft is placed on an elliptical orbit with the panel initially
undeformed and stationary relative to the hub (all bend/twist angles and
rates initialized to zero) while the attitude controller maneuvers the hub
to the commanded reference orientation. The simulation runs for 10 minutes.

Bending angles, torsional angles, and their rates are plotted for every
sub-panel segment, along with the attitude error and attitude error rate
from the FSW control loop.

Illustration of Simulation Results
----------------------------------

.. image:: /_images/Scenarios/scenarioFlexiblePanelMuJoCo_thetas.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFlexiblePanelMuJoCo_betas.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFlexiblePanelMuJoCo_thetaDots.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFlexiblePanelMuJoCo_betaDots.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFlexiblePanelMuJoCo_sigma_BR.svg
   :align: center

.. image:: /_images/Scenarios/scenarioFlexiblePanelMuJoCo_omega_BR_B.svg
   :align: center
"""

import os
import matplotlib.pyplot as plt
import numpy as np

from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, RigidBodyKinematics as rbk, simHelpers
from Basilisk.simulation import NBodyGravity, mujoco, pointMassGravityModel, simpleNav, MJJointPIDController, cmdTorqueBodyToTorqueAtSite
from Basilisk.fswAlgorithms import mrpFeedback, inertial3D, attTrackingError
from Basilisk.architecture import messaging

from Basilisk import __path__

# Used to tag saved figs with name of file
fileName = os.path.basename(os.path.splitext(__file__)[0])

# -------------------------------------------------------------------------
# PLOTTING FUNCTIONS
# -------------------------------------------------------------------------
def plotBendingAngles(timeAxis: np.ndarray, theta: list, numberOfSegments: int) -> plt.Figure:
    """Plots bending angles for each segment"""
    fig = plt.figure(num = 1, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(numberOfSegments):
        plt.plot(timeAxis * macros.NANO2MIN, macros.R2D * theta[idx],
                 color = simHelpers.getLineColor(idx, numberOfSegments),
                 label = r'$\theta_' + str(idx + 1) + '$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'$\theta$ [deg]')
    plt.title("Bending Angles")

    return fig


def plotTorsionalAngles(timeAxis: np.ndarray, beta: list, numberOfSegments: int) -> plt.Figure:
    """Plots torsional angles for each segment"""
    fig = plt.figure(num = 2, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(numberOfSegments):
        plt.plot(timeAxis * macros.NANO2MIN, macros.R2D * beta[idx],
                 color = simHelpers.getLineColor(idx, numberOfSegments),
                 label = r'$\beta_' + str(idx + 1) + '$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'$\beta$ [deg]')
    plt.title("Torsional Angles")

    return fig


def plotBendingAngleRates(timeAxis: np.ndarray, thetaDot: list, numberOfSegments: int) -> plt.Figure:
    """Plots bending angle rates for each segment"""
    fig = plt.figure(num = 3, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(numberOfSegments):
        plt.plot(timeAxis * macros.NANO2MIN, macros.R2D * thetaDot[idx],
                 color = simHelpers.getLineColor(idx, numberOfSegments),
                 label = r'$\dot{\theta}_' + str(idx + 1) + '$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'$\dot{\theta}$ [deg/s]')
    plt.title("Bending Angle Rates")

    return fig


def plotTorsionalAngleRates(timeAxis: np.ndarray, betaDot: list, numberOfSegments: int) -> plt.Figure:
    """Plots torsional angle rates for each segment"""
    fig = plt.figure(num = 4, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(numberOfSegments):
        plt.plot(timeAxis * macros.NANO2MIN, macros.R2D * betaDot[idx],
                 color = simHelpers.getLineColor(idx, numberOfSegments),
                 label = r'$\dot{\beta}_' + str(idx + 1) + '$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'$\dot{\beta}$ [deg/s]')
    plt.title("Torsional Angle Rates")

    return fig


def plotAttitudeError(timeAxis: np.ndarray, sigma_BR: np.ndarray) -> plt.Figure:
    """Plots attitude error MRP components"""
    fig = plt.figure(num = 5, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, sigma_BR[:, idx],
                 color = simHelpers.getLineColor(idx, 3),
                 label = r'$\sigma_' + str(idx) + '$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'$\sigma_{B/R}$')
    plt.title("Attitude Error")

    return fig


def plotAttitudeErrorRate(timeAxis: np.ndarray, omega_BR_B: np.ndarray) -> plt.Figure:
    """Plots attitude error rate components"""
    fig = plt.figure(num = 6, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, omega_BR_B[:, idx],
                 color = simHelpers.getLineColor(idx, 3),
                 label = r'$\omega_' + str(idx) + '$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'$\omega_{B/R}$ [rad/s]')
    plt.title("Attitude Error Rate")

    return fig


class geometryClass:
    """Store hub and panel dimensions and derive equal-sized panel segments.

    :param numberOfSegments: Positive number of rigid segments used to
        approximate the flexible panel. The total panel length and mass
        are divided equally among these segments.
    """
    massHub = 1000  # [kg]
    lengthHub = 3  # [m]
    widthHub = 3  # [m]
    heightHub = 6  # [m]
    lengthPanel = 18.0  # [m]
    widthPanel = 3.0  # [m]
    thicknessPanel = 0.3  # [m]
    massPanel = 100.0  # [kg]

    def __init__(self, numberOfSegments):
        """Derive the mass and dimensions of each panel segment."""
        self.numberOfSegments = numberOfSegments
        self.massSubPanel = self.massPanel / self.numberOfSegments
        self.lengthSubPanel = self.lengthPanel / self.numberOfSegments
        self.widthSubPanel = self.widthPanel
        self.thicknessSubPanel = self.thicknessPanel


def panelChainGen(scGeometry: geometryClass, baseIndent: int):
    """Build the nested MuJoCo body elements for a flexible panel.

    :param scGeometry: Hub and panel geometry, including the number of segments,
        segment dimensions [m], and segment mass [kg].
    :param baseIndent: Number of leading indentation tabs for the first segment.
    :returns: XML defining the nested segment bodies, bending and twisting
        joints, inertial properties, and visual geometry.
    :rtype: str
    """

    openTags = []
    closeTags = []

    for idx in range(scGeometry.numberOfSegments):
        n = idx + 1
        # Sub-panels connected to previous, denoted through an extra indent following prev. sub-panel
        pad = "\t" * (baseIndent + 2 * idx)

        if idx == 0:
            # Starting sub-panel bending position, located at top corner of hub, centered in thickness of panel
            bendingPos = f"0 {scGeometry.lengthHub / 2} {scGeometry.heightHub / 2 - scGeometry.thicknessSubPanel / 2}"
        else:
            # Else, sub-panel bending position defined at end of previous sub-panel
            bendingPos = f"0 {scGeometry.lengthSubPanel} 0"

        # CoM offset to correctly account for CoM of sub-panel on full flexible panel
        COM_offset = f"0 {scGeometry.lengthSubPanel / 2} 0"

        # Inertia matrix diagonal values for each sub-panel
        ixx = round(scGeometry.massSubPanel / 12 * (scGeometry.lengthSubPanel**2 + scGeometry.thicknessSubPanel**2), 6)
        iyy = round(scGeometry.massSubPanel / 12 * (scGeometry.widthSubPanel**2 + scGeometry.thicknessSubPanel**2), 6)
        izz = round(scGeometry.massSubPanel / 12 * (scGeometry.widthSubPanel**2 + scGeometry.lengthSubPanel**2), 6)

        # XML string defining sub-panel body from previously calculated values
        openTags.append(
f"""{pad}<body name = "subPanel{n}" pos = "{bendingPos}">
{pad}   <joint name = "bendJoint{n}" pos = "0 0 0" axis = "1 0 0" ref = "0"/>
{pad}   <joint name = "twistJoint{n}" pos = "0 0 0" axis = "0 1 0" ref = "0"/>
{pad}   <inertial pos = "{COM_offset}" mass = "{scGeometry.massSubPanel}" diaginertia = "{ixx} {iyy} {izz}"/>
{pad}   <geom name = "subPanel{n}Geom" class = "subpanel_geom" pos = "{COM_offset}"/>""")

        # Correctly closing off bodies in XML string
        closeTags.append(f"{pad}</body>")

    return "\n".join(openTags) + "\n" + "\n".join(reversed(closeTags))


def makeMjXmlString(scGeometry: geometryClass):
    """Build MuJoCo XML for a spacecraft hub with a flexible panel.

    :param scGeometry: Hub and panel geometry, including dimensions [m],
        masses [kg], and the number of panel segments.
    :returns: Complete MuJoCo model XML containing the hub and panel chain.
    :rtype: str
    """

    # Inertia matrix diagonal values for hub
    ixx = scGeometry.massHub / 12 * (scGeometry.lengthHub**2 + scGeometry.heightHub**2)
    iyy = scGeometry.massHub / 12 * (scGeometry.widthHub**2 + scGeometry.heightHub**2)
    izz = scGeometry.massHub / 12 * (scGeometry.lengthHub**2 + scGeometry.widthHub**2)

    # Generating panel chain
    panelChain = panelChainGen(scGeometry, baseIndent = 3)

    return f"""<mujoco model = "busWithFlexiblePanel">
    <compiler angle = "radian" meshdir = ""/>
    <default>
        <default class = "subpanel_geom">
            <geom type = "box" size = "{scGeometry.widthSubPanel / 2} {scGeometry.lengthSubPanel / 2} {scGeometry.thicknessSubPanel / 2}"
            contype = "0" conaffinity = "1"/>
        </default>
    </default>

    <worldbody>
        <body name = "hub" pos = "0 0 0">
            <freejoint name = "busFree"/>
            <inertial pos = "0 0 0" mass = "{scGeometry.massHub}" diaginertia = "{ixx} {iyy} {izz}"/>
            <geom name = "hubVisual" type = "box" size = "{scGeometry.widthHub / 2} {scGeometry.lengthHub / 2} {scGeometry.heightHub / 2}" rgba = "1 1 1 1"/>
            <site name = "hubSite" pos = "0 0 0"/>
{panelChain}
        </body>
    </worldbody>
</mujoco>"""


def run(showPlots: bool = False):
    """Run 10 minutes of attitude control with a flexible solar panel.

    :param showPlots: Display the Matplotlib figures and wait for the plot
        windows to close when True. Defaults to False; figures are still
        generated and returned in either case.
    :returns: Dictionary mapping scenario-prefixed names to six Matplotlib
        figures: bending and twisting angles, their rates, attitude error,
        and angular-rate tracking error. No files are saved by this function.
    :rtype: dict
    """
    # -------------------------------------------------------------------------
    # 1) Simulation configuration and MJScene dynamics model
    # -------------------------------------------------------------------------
    simTaskName = "simTask"
    simProcessName = "simProcess"
    fswTaskName = "fswTask"
    fswProcessName = "fswProcess"

    # Initializing simulation time/time-steps for dynamics/fsw task
    simulationTime = macros.min2nano(10.0)  # [ns]
    timeStep = macros.sec2nano(0.5)  # [ns]
    fswTimeStep = macros.sec2nano(1.0)  # [ns]

    sim = SimulationBaseClass.SimBaseClass()
    dynProcess = sim.CreateNewProcess(simProcessName)
    dynProcess.addTask(sim.CreateNewTask(simTaskName, timeStep))
    fswProcess = sim.CreateNewProcess(fswProcessName)
    fswProcess.addTask(sim.CreateNewTask(fswTaskName, fswTimeStep))

    # Constructing MJ XML string and loaded into MJScene dynamics model
    numberOfSegments = 5 # specifies number of discretized sub-panels (CHANGE THIS FOR SIM)
    scGeometry = geometryClass(numberOfSegments)
    xmlString = makeMjXmlString(scGeometry)
    scene = mujoco.MJScene(xmlString)
    scene.ModelTag = "mujocoScene"
    sim.AddModelToTask(simTaskName, scene)

    # -------------------------------------------------------------------------
    # 2) Retrieve spacecraft components
    # -------------------------------------------------------------------------
    # Pull handles of hub/sub-panel bodies
    busBody = scene.getBody("hub")
    subPanels = [scene.getBody(f"subPanel{i + 1}") for i in range(numberOfSegments)]

    # Retrieving bend/twist joints that define flexible nature of panel
    bendJoints = [subPanels[i].getScalarJoint(f"bendJoint{i + 1}") for i in range(numberOfSegments)]
    twistJoints = [subPanels[i].getScalarJoint(f"twistJoint{i + 1}") for i in range(numberOfSegments)]

    # -------------------------------------------------------------------------
    # 3) Adding damping/stiffness to subpanels (bend & twist)
    # -------------------------------------------------------------------------
    # Initializing stiffness/damping coefficients of bending/twisting DOFs
    kBend, cBend = 10.0, 8.0  # stiffness [N m/rad], damping [N m s/rad]
    kTwist, cTwist = 1.0, 0.8  # stiffness [N m/rad], damping [N m s/rad]
    thetaRef = 0.0  # [rad]

    # Keep all springDamper/refMsgs in list so they remain in scope for whole sim
    springDampers = []
    refMsgs = []
    for i in range(numberOfSegments):
            # Loops through each bend and twist joint to apply proper torsional torquing
            for jointName, k, c in [(f"bendJoint{i + 1}", kBend, cBend), (f"twistJoint{i + 1}", kTwist, cTwist)]:
                # Retrieving joints and adding actuators to mimic damping/stiffness effects
                actuator = scene.addJointSingleActuator(f"{jointName}Actuator", jointName)
                joint = subPanels[i].getScalarJoint(jointName)

                # Using the C++ JointPIDController
                springDamper = MJJointPIDController.JointPIDController()
                springDamper.ModelTag = f"{jointName}SpringDamper"
                springDamper.setProportionalGain(k)
                springDamper.setDerivativeGain(c)

                # Generating reference pos/vel messages
                refPosMsgPayload = messaging.ScalarJointStateMsgPayload()
                refPosMsgPayload.state = thetaRef
                refPosMsg = messaging.ScalarJointStateMsg().write(refPosMsgPayload)

                refVelMsgPayload = messaging.ScalarJointStateMsgPayload()
                refVelMsgPayload.state = 0.0  # [rad/s]
                refVelMsg = messaging.ScalarJointStateMsg().write(refVelMsgPayload)

                refMsgs.extend([refPosMsg, refVelMsg])

                # Connecting reference/current states to desired/measured positions and velocities
                springDamper.desiredPosInMsg.subscribeTo(refPosMsg)
                springDamper.desiredVelInMsg.subscribeTo(refVelMsg)
                springDamper.measuredPosInMsg.subscribeTo(joint.stateOutMsg)
                springDamper.measuredVelInMsg.subscribeTo(joint.stateDotOutMsg)

                # Connecting actuator to commanded torque from PID
                actuator.actuatorInMsg.subscribeTo(springDamper.outputOutMsg)

                # Add each model to dynamics task and append to springDampers
                scene.AddModelToDynamicsTask(springDamper)
                springDampers.append(springDamper)

    # -------------------------------------------------------------------------
    # 4) Add gravity and set up orbital elements
    # -------------------------------------------------------------------------
    oe = orbitalMotion.ClassicElements()
    oe.a = 8e6  # meters
    oe.e = 0.1  # [-]
    oe.i = 0.0 * macros.D2R  # [rad]
    oe.Omega = 0.0 * macros.D2R  # [rad]
    oe.omega = 0.0 * macros.D2R  # [rad]
    oe.f = 0.0 * macros.D2R  # [rad]
    muEarth = 0.3986004415e15  # [m^3/s^2]
    rN, vN = orbitalMotion.elem2rv(muEarth, oe)

    # Adding N-Body gravity model into MJScene
    gravity = NBodyGravity.NBodyGravity()
    gravity.ModelTag = "gravity"
    scene.AddModelToDynamicsTask(gravity)

    # Applying Earth point mass gravity effects to model, make central body
    earthPm = pointMassGravityModel.PointMassGravityModel()
    earthPm.muBody = muEarth
    gravity.addGravitySource("earth", earthPm, isCentralBody = True)

    # Gravity effects added to each body in scene (hub + all sub-panels)
    gravity.addGravityTarget("hub", busBody)
    for i in range(numberOfSegments):
        gravity.addGravityTarget(f"subPanel{i + 1}", subPanels[i])

    # -------------------------------------------------------------------------
    # 5) Navigation and attitude control
    # -------------------------------------------------------------------------
    # Reading s/c state and publishing standard navigation output
    simpleNavObj = simpleNav.SimpleNav()
    simpleNavObj.ModelTag = "simpleNav"
    simpleNavObj.scStateInMsg.subscribeTo(busBody.getCenterOfMass().stateOutMsg)
    sim.AddModelToTask(simTaskName, simpleNavObj)

    # Define the desired inertial attitude using modified Rodrigues parameters.
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    inertial3DObj.sigma_R0N = [0.3, 0.4, 0.5]  # [-]
    sim.AddModelToTask(fswTaskName, inertial3DObj)

    # Tracks attitude error of s/c
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attTrackingError"
    attError.attNavInMsg.subscribeTo(simpleNavObj.attOutMsg) # feed in the current navigation attitude
    attError.attRefInMsg.subscribeTo(inertial3DObj.attRefOutMsg) # feed in inertial frame (reference attitude)
    sim.AddModelToTask(fswTaskName, attError)

    # Hub inertia about own center of mass
    IHubPntBc_B = np.array([[scGeometry.massHub / 12 * (scGeometry.lengthHub**2 + scGeometry.heightHub**2), 0.0, 0.0],
                            [0.0, scGeometry.massHub / 12 * (scGeometry.widthHub**2 + scGeometry.heightHub**2), 0.0],
                            [0.0, 0.0, scGeometry.massHub / 12 * (scGeometry.lengthHub**2 + scGeometry.widthHub**2)]])
    # FULL panel inertia about its own center of mass
    IPanelPntSc_B = np.array([[scGeometry.massPanel / 12 * (scGeometry.lengthPanel**2 + scGeometry.thicknessPanel**2), 0.0, 0.0,],
                              [0.0, scGeometry.massPanel / 12 * (scGeometry.widthPanel**2 + scGeometry.thicknessPanel**2), 0.0],
                              [0.0, 0.0, scGeometry.massPanel / 12 * (scGeometry.widthPanel**2 + scGeometry.lengthPanel**2)]])
    # Position of panel's CoM relative to hub point B, in body frame
    r_ScB_B = [0.0, scGeometry.lengthHub/2 + scGeometry.lengthPanel/2,
           scGeometry.heightHub/2 - scGeometry.thicknessSubPanel/2]
    # Combine into a single inertia tensor about hub reference point B (parallel axis theorem).
    IHubPntB_B =  IHubPntBc_B + IPanelPntSc_B - scGeometry.massPanel * np.array(rbk.v3Tilde(r_ScB_B)) @ np.array(rbk.v3Tilde(r_ScB_B))

    # Apply modified Rodrigues parameter (MRP) attitude control to the spacecraft.
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    decayTime = 50  # [s]
    xi = 0.9  # [-] Damping ratio.
    mrpControl.P = 2 * np.max(IHubPntB_B) / decayTime
    mrpControl.K = (mrpControl.P / xi) ** 2 / np.max(IHubPntB_B)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)

    # Inertia tensor passed into config message, vehicle config created
    configData = messaging.VehicleConfigMsgPayload(ISCPntB_B = list(IHubPntB_B.flatten()))
    configDataMsg = messaging.VehicleConfigMsg()
    configDataMsg.write(configData)
    mrpControl.vehConfigInMsg.subscribeTo(configDataMsg) # so MRP controller knows mass properties
    sim.AddModelToTask(fswTaskName, mrpControl)

    # Torque actuator at hub site for FSW-commanded control torques
    torqueActuator = scene.addTorqueActuator("hubTorqueAct", "hubSite")

    # Library adapter converts body-frame torque into a TorqueAtSite message.
    torqueBridge = cmdTorqueBodyToTorqueAtSite.CmdTorqueBodyToTorqueAtSite()
    torqueBridge.ModelTag = "torqueBridge"
    torqueBridge.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    scene.AddModelToDynamicsTask(torqueBridge)

    torqueActuator.torqueInMsg.subscribeTo(torqueBridge.torqueOutMsg)

    # -------------------------------------------------------------------------
    # 6) Setup data recording
    # -------------------------------------------------------------------------
    # Hub position/velocity
    dataLog = busBody.getCenterOfMass().stateOutMsg.recorder()
    sim.AddModelToTask(simTaskName, dataLog)

    # FSW attitude error (control performance)
    attErrorLog = attError.attGuidOutMsg.recorder()
    sim.AddModelToTask(fswTaskName, attErrorLog)

    # Hinge angle/rate recorders at each sub panel, bend and twist combined into single recorder
    posData, rateData = [], []
    for i in range(numberOfSegments):
        for joint in (bendJoints[i], twistJoints[i]):
            posLog = joint.stateOutMsg.recorder()
            rateLog = joint.stateDotOutMsg.recorder()
            sim.AddModelToTask(simTaskName, posLog)
            sim.AddModelToTask(simTaskName, rateLog)
            posData.append(posLog)
            rateData.append(rateLog)

    # -------------------------------------------------------------------------
    # 7) Initialize simulation, orbit, & joint angles
    # -------------------------------------------------------------------------
    sim.InitializeSimulation()

    # Setting initial conditions
    busFree = busBody.getFreeJoint()
    busBody.setPosition(rN)
    busFree.setVelocity(vN)

    thetaInit = 0.0  # [rad]
    for i in range(numberOfSegments):
        bendJoints[i].setPosition(thetaInit)
        twistJoints[i].setPosition(thetaInit)

    # -------------------------------------------------------------------------
    # 8) Execute simulation
    # -------------------------------------------------------------------------
    sim.ConfigureStopTime(simulationTime)
    sim.ExecuteSimulation()

    # -------------------------------------------------------------------------
    # 9) Post processing and plotting
    # -------------------------------------------------------------------------
    theta, thetaDot = [], []
    betas, betaDot = [], []
    for idx in range(numberOfSegments):
        # Need to unwind bend/twist data for proper logging
        theta.append(posData[2 * idx].state)
        thetaDot.append(rateData[2 * idx].state)
        betas.append(posData[2 * idx + 1].state)
        betaDot.append(rateData[2 * idx + 1].state)

    timeAxis = posData[0].times() # dyn-task time data
    timeAxisFSW = attErrorLog.times() # fsw-task time data

    # Generating plots
    plt.close("all")
    figureList = {}
    figureList[fileName + "_thetas"] = plotBendingAngles(timeAxis, theta, numberOfSegments)
    figureList[fileName + "_betas"] = plotTorsionalAngles(timeAxis, betas, numberOfSegments)
    figureList[fileName + "_thetaDots"] = plotBendingAngleRates(timeAxis, thetaDot, numberOfSegments)
    figureList[fileName + "_betaDots"] = plotTorsionalAngleRates(timeAxis, betaDot, numberOfSegments)
    figureList[fileName + "_sigma_BR"] = plotAttitudeError(timeAxisFSW, attErrorLog.sigma_BR)
    figureList[fileName + "_omega_BR_B"] = plotAttitudeErrorRate(timeAxisFSW, attErrorLog.omega_BR_B)

    if showPlots:
        plt.show()

    return figureList


if __name__ == "__main__":
    run(showPlots = True)
