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
It's recommended to review the following scenario(s) first (and any
recommended scenario(s) that they may have):

#. ``examples/scenarioFlexiblePanel.py``
#. ``examples/mujoco/scenarioHingedRigidBody.py``
#. ``examples/mujoco/scenarioAttitudeFeedbackRW.py``

This script demonstrates how to model a flexible, multi-segment solar panel
using MuJoCo dynamics via :ref:`MJScene<MJScene>` instead of the traditional
hub-centric Basilisk :ref:`spacecraft` dynamics. This scenario is a translation
of the classic Basilisk ``scenarioFlexiblePanel.py`` example.

The multi-body system is created programmatically as a MuJoCo XML string.
It consists of a free-floating spacecraft bus ("hub") with a single flexible
panel discretized into ``numberOfSegments`` rigid sub-panel bodies
("subPanel1", "subPanel2", ...), connected end-to-end. Each sub-panel is
connected to its neighbor by two identically-located hinge joints: a "bend" joint
(bending DOF) and a "twist" joint (torsional DOF), so that the
panel's continuous flexibility is approximated by a discretized series of
rigid links. As such, this system has 6 + 2 * ``numberOfSegments`` DOFs
(3 translational, 3 rotational, 2 DOFs per discretized panel)

A torsional spring-damper torque is applied at every bend and twist joint to
emulate the panel's structural stiffness and damping:

#. ``JointSpringDamper`` computes a restoring torque from a joint's angle and
   angular rate about a reference equilibrium angle, and writes the result as
   a ``SingleActuatorMsg`` command. One instance is attached to each bend and
   twist joint, using separate bending and torsional stiffness/damping
   coefficients.

A standard Basilisk FSW stack is used to point the hub at a fixed inertial
attitude while the flexible panel dynamically responds to the resulting
motion:

#. ``simpleNav`` provides the spacecraft's navigation attitude solution.
#. ``inertial3D`` generates a fixed inertial attitude reference.
#. ``attTrackingError`` computes the attitude and rate tracking errors.
#. ``mrpFeedback`` computes the commanded body torque, using an inertia
   tensor for the hub plus panel computed via the parallel axis theorem.

A small adapter module bridges the FSW torque command to the MuJoCo torque
actuator:

#. ``CmdTorqueToSiteActuator`` relays the commanded body-frame torque
   directly as a ``TorqueAtSiteMsg`` (site and body frames are coincident),
   consumed by a torque actuator at the hub site.

Earth gravity is configured using :ref:`NBodyGravity<NBodyGravity>` with a
:ref:`pointMassGravityModel<pointMassGravityModel>` as the central body.
Gravity targets are registered manually for the hub and every sub-panel body.

The spacecraft is placed on an elliptical orbit and released from rest (all
bend/twist angles initialized to zero) while the attitude controller
maneuvers the hub to the commanded reference orientation. The simulation
runs for 10 minutes.

Bending angles, torsional angles, and their rates are plotted for every
sub-panel segment, along with the attitude error and attitude error rate
from the FSW control loop.
"""

import os
import matplotlib.pyplot as plt
import numpy as np

from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, RigidBodyKinematics as rbk, simHelpers
from Basilisk.simulation import NBodyGravity, mujoco, pointMassGravityModel, simpleNav
from Basilisk.fswAlgorithms import mrpFeedback, inertial3D, attTrackingError
from Basilisk.architecture import messaging, sysModel

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
    plt.title("Bending Angles", fontsize="22")

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
    plt.title("Torsional Angles", fontsize="22")

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
    plt.title("Bending Angle Rates", fontsize="22")

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
    plt.title("Torsional Angle Rates", fontsize="22")

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
    plt.title("Attitude Error", fontsize="22")

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
    plt.ylabel(r'$\omega_{B/R}$')
    plt.title("Attitude Error Rate", fontsize="22")

    return fig


# Specifies geometry of hub and flexible panel, minor calcs for sub-panel geometry as well
class geometryClass:
    massHub = 1000
    lengthHub = 3
    widthHub = 3
    heightHub = 6
    lengthPanel = 18.0
    widthPanel = 3.0
    thicknessPanel = 0.3
    massPanel = 100.0

    def __init__(self, numberOfSegments):
        self.numberOfSegments = numberOfSegments
        self.massSubPanel = self.massPanel / self.numberOfSegments
        self.lengthSubPanel = self.lengthPanel / self.numberOfSegments
        self.widthSubPanel = self.widthPanel
        self.thicknessSubPanel = self.thicknessPanel


# panelChainGen: supporter function for XML constructor function to generate sub-panel bodies
#   that make up the flexible panel.
#   INPUTS:
#       - scGeometry: geometryClass instance
#       - baseIndent: scalar value defining initial number of indents (tabs) necessary for proper XML formatting
def panelChainGen(scGeometry: geometryClass, baseIndent: int):
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



# makeMjXmlString: MuJoCo string constuctor, creates MJ model with the following inputs:
# - scGeometry: geometryClass instance, provides all necessary measurements
def makeMjXmlString(scGeometry: geometryClass):
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
    # -------------------------------------------------------------------------
    # 1) Simulation configuration and MJScene dynamics model
    # -------------------------------------------------------------------------
    simTaskName = "simTask"
    simProcessName = "simProcess"
    fswTaskName = "fswTask"
    fswProcessName = "fswProcess"

    # Initializing simulation time/time-steps for dynamics/fsw task
    simulationTime = macros.min2nano(10.0)
    timeStep = macros.sec2nano(0.1)
    fswTimeStep = macros.sec2nano(0.5)

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
    # 2) Retrieve spacecraft componenets
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
    kBend, cBend = 10.0, 8.0
    kTwist, cTwist = 1.0, 0.8

    springDampers = []
    for i in range(numberOfSegments):
            # Loops through each bend and twist joint to apply proper torsional torquing
            for jointName, k, c in [(f"bendJoint{i + 1}", kBend, cBend), (f"twistJoint{i + 1}", kTwist, cTwist)]:
                # Retrieving joints and adding actuators to mimic damping/stiffness effects
                actuator = scene.addJointSingleActuator(f"{jointName}Actuator", jointName)
                joint = subPanels[i].getScalarJoint(jointName)

                # Custom spring damper sys model application (see associated function), computes 
                # restoring force based on torsional damping/stiffness coefficients
                sd = JointSpringDamper(k = k, c = c, thetaRef = 0.0)
                sd.ModelTag = f"{jointName}SpringDamper"
                sd.jointPosInMsg.subscribeTo(joint.stateOutMsg) # feed in joint angle
                sd.jointVelInMsg.subscribeTo(joint.stateDotOutMsg) # feed in joint angular velocity
                actuator.actuatorInMsg.subscribeTo(sd.actuatorOutMsg) # apply computed torque to specified actuator

                scene.AddModelToDynamicsTask(sd)
                springDampers.append(sd) 

    # *** JointSpringDamper function identical to that in scenarioHingedRigidBodyMuJoCo.py ***

    # -------------------------------------------------------------------------
    # 4) Add gravity and set up orbital elements
    # -------------------------------------------------------------------------
    oe = orbitalMotion.ClassicElements()
    oe.a = 8e6  # meters
    oe.e = 0.1
    oe.i = 0.0 * macros.D2R
    oe.Omega = 0.0 * macros.D2R
    oe.omega = 0.0 * macros.D2R
    oe.f = 0.0 * macros.D2R
    muEarth = 0.3986004415e15  # [m^3/s^2]
    rN, vN = orbitalMotion.elem2rv(muEarth, oe)

    # Adding N-Body gravity model into MJscene
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

    # Identifies inertial frame of s/c for attitude determination
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    inertial3DObj.sigma_R0N = [0.3, 0.4, 0.5]
    sim.AddModelToTask(fswTaskName, inertial3DObj)

    # Tracks attitude error of s/c
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attTrackingError"
    attError.attNavInMsg.subscribeTo(simpleNavObj.attOutMsg) # feed in navigation commands (current attitude)
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
    # Combining into single inertia tensor abour hub reference point B (parallel axis thm)
    IHubPntB_B =  IHubPntBc_B + IPanelPntSc_B - scGeometry.massPanel * np.array(rbk.v3Tilde(r_ScB_B)) @ np.array(rbk.v3Tilde(r_ScB_B))

    # MRP (Modified Rodriguez Parameter) attitude control applied to s/c
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    decayTime = 50
    xi = 0.9
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

    # Adapter module (defined below) to forward body frame torque to TorqueAtSite msg
    torqueBridge = CmdTorqueToSiteActuator()
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

    thetaInit = 0.0
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

    return


# -------------------------------------------------------------------------
# JointSpringDamper: custom sys model to incorporate torsional spring effects
#   torque for a hinge joint in MuJoCo
#   INPUTS:
#       - k : stiffness coefficient
#       - c : damping coefficient
#       - thetaRef : reference angle to equilibrium
# -------------------------------------------------------------------------
class JointSpringDamper(sysModel.SysModel):
    def __init__(self, k: float, c: float, thetaRef: float):
        super().__init__()
        self.k = k # stiffness [Nm / rad]
        self.c = c # damping [Nms / rad]
        self.thetaRef = thetaRef # equilibrium theta [rad]

        self.jointPosInMsg = messaging.ScalarJointStateMsgReader() # current hinge angle
        self.jointVelInMsg = messaging.ScalarJointStateMsgReader() # current hinge angular vel
        self.actuatorOutMsg = messaging.SingleActuatorMsg() # outpur torque command

    def UpdateState(self, CurrentSimNanos):
        """Computes simple linear spring-damper restoring torque at each simulation step"""
        theta = self.jointPosInMsg().state
        thetaDot = self.jointVelInMsg().state

        # Classic spring damper law
        torque = -self.k * (theta - self.thetaRef) - self.c * thetaDot

        payload = messaging.SingleActuatorMsgPayload()
        payload.input = torque
        self.actuatorOutMsg.write(payload, self.moduleID, CurrentSimNanos)


# -------------------------------------------------------------------------
# CmdTorqueToSiteActuator: custom sys model to relay commanded body-frame torque
#   to TorqueAtSite message for actuator to consume
# -------------------------------------------------------------------------
class CmdTorqueToSiteActuator(sysModel.SysModel):
    def __init__(self):
        super().__init__()
        self.cmdTorqueInMsg = messaging.CmdTorqueBodyMsgReader() # FSW command torque
        self.torqueOutMsg = messaging.TorqueAtSiteMsg() # torque expressed at site

    def UpdateState(self, CurrentSimNanos):
        cmd = self.cmdTorqueInMsg()
        # Site frame == body frame
        payload = messaging.TorqueAtSiteMsgPayload(torque_S = cmd.torqueRequestBody)
        self.torqueOutMsg.write(payload, self.moduleID, CurrentSimNanos)


if __name__ == "__main__":
    # --- XML GENERATION TESTING ---
    numSegments = 5 # specifies number of discretized sub-panels (only for testing)
    scGeometry = geometryClass(numSegments)
    xmlString = makeMjXmlString(scGeometry, hubMass = 1000.0)

    # Writes XML to file for confirmation of proper string gen
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "sat_flexiblePanel.xml"), "w") as f:
        f.write(xmlString)

    run(showPlots = True)