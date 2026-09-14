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

#. ``examples/scenarioHingedRigidBody.py``
#. ``examples/mujoco/scenarioReactionWheel.py``

This script demonstrates how to run the classic Basilisk
``scenarioHingedRigidBody.py`` example using MuJoCo dynamics via
:ref:`MJScene<MJScene>` instead of the traditional hub-centric Basilisk
:ref:`spacecraft` dynamics.

The multi-body system is created programmatically as a MuJoCo XML string.
It consists of a free-floating spacecraft bus ("hub") with two solar panel
rigid bodies ("panel1", "panel2") attached via hinge joints ("hinge1",
"hinge2"), giving the system 8 total degrees of freedom (3 translational,
3 rotational, and 2 panel hinge DOFs).

Two small custom system models are added directly to the MuJoCo dynamics task:

#. ``JointSpringDamper`` computes a torsional spring-damper restoring
   torque for a hinge joint from its angle and angular rate, and writes
   the result as a ``SingleActuatorMsg`` command. One instance is
   attached to each panel hinge to emulate panel stiffness and damping.
#. ``InertialForceToSiteActuator`` converts a fixed inertial-frame thrust
   force into a body-frame force at the hub's thruster site, applying it
   only during a specified burn time window.

Earth gravity is configured using :ref:`NBodyGravity<NBodyGravity>` with a
:ref:`pointMassGravityModel<pointMassGravityModel>` as the central body.
Gravity targets are registered manually for the hub and both panel bodies.

The spacecraft is placed on a near-circular LEO orbit. The simulation runs
for a short coast phase followed by a finite-duration translational burn,
during which the panels respond dynamically to the resulting body motion
through their spring-damper hinges.

Inertial position, orbital radius, and the two panel hinge angular
displacements are plotted at the end.
"""

from typing import Tuple
import os

import matplotlib.pyplot as plt
import numpy as np

from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import NBodyGravity, mujoco, pointMassGravityModel
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simHelpers, RigidBodyKinematics

# Used to tag saved figs with name of file
fileName = os.path.basename(os.path.splitext(__file__)[0])

# -------------------------------------------------------------------------
# PLOTTING FUNCTIONS
# -------------------------------------------------------------------------
def plotInertialPos(timeAxis: np.ndarray, posData: np.ndarray) -> plt.Figure:
    """Plots inertial position vector componenets"""
    fig = plt.figure(num = 1, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2MIN, posData[:, idx] / 1000.,
                 color = simHelpers.getLineColor(idx, 3),
                 label = '$r_{BN,' + str(idx) + '}$')
    plt.legend(loc = 'lower right')
    plt.xlabel('Time [h]')
    plt.ylabel('Inertial Position [km]')

    return fig


def plotOrbitalMotion(timeAxis: np.ndarray, posData: np.ndarray, velData: np.ndarray, mu: float) -> plt.Figure:
    """Plots orbital motion of spacecraft"""
    fig = plt.figure(num = 2, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    rData = []
    for idx in range(0, len(posData)):
        rVec = np.array(posData[idx]).flatten()
        vVec = np.array(velData[idx]).flatten()
        
        oeData = orbitalMotion.rv2elem(mu, rVec, vVec)
        rData.append(oeData.rmag / 1000.)
        
    plt.plot(timeAxis * macros.NANO2MIN, rData, color='#aa0000')
    plt.xlabel('Time [min]')
    plt.ylabel('Radius [km]')

    return fig


def plotAngDisp(timeAxis: np.ndarray, panel1thetaLog: np.ndarray, panel2thetaLog: np.ndarray) -> plt.Figure:
    """Plots angular displacements of panels"""
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex = True, num = 3, clear = True)

    ax1.plot(timeAxis * macros.NANO2MIN, panel1thetaLog)
    ax1.set_xlabel("Time [min]")
    ax1.set_ylabel('Panel 1 Angular Displacement [r]')

    ax2.plot(timeAxis * macros.NANO2MIN, panel2thetaLog)
    ax2.set_xlabel("Time [min]")
    ax2.set_ylabel('Panel 2 Angular Displacement [r]')

    fig.tight_layout()

    return fig

# makeMjXmlString: MuJoCo string constuctor, creates MJ model with the following inputs:
# - hubMass: mass of the s/c hub
# - busIDiag: Tuple list representing diagonal of inertia matrix I (assuming symmetric)
def makeMjXmlString(hubMass: float = 800.0, busIDiag: Tuple[float, float, float] = (900.0, 800.0, 600.0)):
    
    ixx, iyy, izz = busIDiag
    
    return f"""
    <mujoco model = "busWith2Panels">
        <compiler angle = "radian" meshdir = ""/>
        
        <default>
            <default class = "panel_geom">
                <geom type = "box" size = "1 1 0.01"
                contype = "0" conaffinity = "1"/>
            </default>
        </default>

        <worldbody>
            <body name = "hub" pos = "0 0 0">
                <freejoint name = "busFree"/>
                
                <inertial pos = "0 0 0" mass = "{hubMass}" diaginertia = "{ixx} {iyy} {izz}"/>
                <geom name = "hubVisual" type = "box" size = "1 1 1" rgba = "1 1 1 1"/>

                <site name = "thrustSite" pos = "0 0 0"/>

                <body name = "panel1" pos = "0.5 0.0 1.0">
                    <joint name = "hinge1" pos = "0 0 0" axis = "0 -1 0" ref = "0"/>
                    <inertial pos = "1.5 0 0" mass = "100.0" diaginertia = "100.0 50.0 50.0"/>
                    <geom name = "panel1_geom" class = "panel_geom" pos = "1.5 0 0"/>
                </body>

                <body name = "panel2" pos = "-0.5 0.0 1.0">
                    <joint name = "hinge2" pos = "0 0 0" axis = "0 1 0" ref = "0"/>
                    <inertial pos = "-1.5 0 0" mass = "100.0" diaginertia = "100.0 50.0 50.0"/>
                    <geom name = "panel2_geom" class = "panel_geom" pos = "-1.5 0 0"/>
                </body>
            </body>
        </worldbody>
    </mujoco>
    """
    

def run(showPlots: bool = False):
    # -------------------------------------------------------------------------
    # 1) Simulation configuration and MJScene dynamics model
    # -------------------------------------------------------------------------
    simTaskName = "simTask"
    simProcessName = "simProcess"

    timeStep = macros.sec2nano(0.1)

    sim = SimulationBaseClass.SimBaseClass()
    dynProcess = sim.CreateNewProcess(simProcessName)
    dynProcess.addTask(sim.CreateNewTask(simTaskName, timeStep))

    # Constructing MJ XML string and loaded into MJScene dynamics model
    xmlString = makeMjXmlString()
    scene = mujoco.MJScene(xmlString)
    scene.ModelTag = "mujocoScene"
    sim.AddModelToTask(simTaskName, scene)

    # Actuator added to site in MJScene, allows for thruster force
    thrustActuator = scene.addForceActuator("thrustForce", "thrustSite")

    # -------------------------------------------------------------------------
    # 2) Retrieve spacecraft componenets
    # -------------------------------------------------------------------------
    # Pull handles of hub/panel bodies from XML
    busBody = scene.getBody("hub")
    panelBodies = [scene.getBody(name) for name in ("panel1", "panel2")]
    numPanels = len(panelBodies)

    # Pull scalar joints connnecting panels
    hinge1 = panelBodies[0].getScalarJoint("hinge1")
    hinge2 = panelBodies[1].getScalarJoint("hinge2")

    # -------------------------------------------------------------------------
    # 3) Adding damping/stiffness to panel hinges
    # ------------------------------------------------------------------------- 
    # Damping/stiffness values initizalized
    k = 1000.0
    c = 0.0

    springDampers = []
    for jointName, body in [("hinge1", panelBodies[0]), ("hinge2", panelBodies[1])]:
        # Retrieving joints and adding actuators to mimic damping/stiffness effects
        actuator = scene.addJointSingleActuator(f"{jointName}Actuator", jointName)
        joint = body.getScalarJoint(jointName)

        # Custom spring damper sys model application (see associated function), computes 
        # restoring force based on torsional damping/stiffness coefficients
        sd = JointSpringDamper(k = k, c = c, thetaRef = 0.0)
        sd.ModelTag = f"{jointName}SpringDamper"
        sd.jointPosInMsg.subscribeTo(joint.stateOutMsg) # feed in hinge angle
        sd.jointVelInMsg.subscribeTo(joint.stateDotOutMsg) # feed in hinge angular rate
        actuator.actuatorInMsg.subscribeTo(sd.actuatorOutMsg) # apply computed torque to actuator

        scene.AddModelToDynamicsTask(sd)
        springDampers.append(sd) 

    # -------------------------------------------------------------------------
    # 4) Add gravity and set up orbital elements
    # -------------------------------------------------------------------------
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000  # meters
    oe.a = rLEO
    oe.e = 0.0001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
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

    # Gravity effects added to each body in scene
    gravity.addGravityTarget("hub", busBody)
    for i in range(numPanels):
        gravity.addGravityTarget(f"panel{i + 1}", panelBodies[i])

    # -------------------------------------------------------------------------
    # 5) Applying thrust
    # -------------------------------------------------------------------------
    # Setting simulation time
    n = np.sqrt(muEarth / oe.a / oe.a / oe.a) # mean motion [rad/s]
    P = 2. * np.pi / n # orbital period [s] 
    simulationTimeFactor = 0.01
    simulationTime = macros.sec2nano(simulationTimeFactor * P)

    T2 = macros.sec2nano(935.) # time it takes to achieve correct deltaV (see original file)
    burnStart = simulationTime
    burnEnd = simulationTime + T2

    # Correctly applies external forcing to site. This custom sys model is required to transform 
    # the provided inertial force to a force at site needed for thurst actuator
    forceConverter = InertialForceToSiteActuator()
    forceConverter.scStateInMsg.subscribeTo(busBody.getCenterOfMass().stateOutMsg)
    forceConverter.setForce_N([-2050.0, -1430.0, -0.00076]) # desired inertial-frame thrust vector [N]
    forceConverter.setBurnWindow(burnStartNanos = burnStart, burnEndNanos = burnEnd)
    forceConverter.ModelTag = "forceConverter"
    scene.AddModelToDynamicsTask(forceConverter)

    # Wiring converted force to thrust actuator
    thrustActuator.forceInMsg.subscribeTo(forceConverter.forceOutMsg)

    # -------------------------------------------------------------------------
    # 6) Setup data recording
    # -------------------------------------------------------------------------
    numDataPoints = 100 # sampling rate based on this
    samplingTime = simHelpers.samplingTime(simulationTime, timeStep, numDataPoints)

    dataLog = busBody.getCenterOfMass().stateOutMsg.recorder(samplingTime)
    pl1Log = hinge1.stateOutMsg.recorder(samplingTime) # data log for panel 1 (recording at hinge)
    pl2Log = hinge2.stateOutMsg.recorder(samplingTime) # data log for panel 2 (recording at hinge)

    sim.AddModelToTask(simTaskName, dataLog)
    sim.AddModelToTask(simTaskName, pl1Log)
    sim.AddModelToTask(simTaskName, pl2Log)

    # -------------------------------------------------------------------------
    # 7) Setup orbit / initialize spacecraft state
    # -------------------------------------------------------------------------
    sim.InitializeSimulation()

    # Setting initial conditions
    busFree = busBody.getFreeJoint()
    busBody.setPosition(rN)
    busFree.setVelocity(vN)

    thetaInit = 5.0 * np.pi / 180.0
    hinge1.setPosition(thetaInit)
    hinge2.setPosition(thetaInit)

    # -------------------------------------------------------------------------
    # 8) Execute simulation (passive orbit + burn)
    # -------------------------------------------------------------------------
    sim.ConfigureStopTime(burnEnd)
    sim.ExecuteSimulation()

    # -------------------------------------------------------------------------
    # 9) Post processing and plotting
    # -------------------------------------------------------------------------
    # Retrieving relevant data from logs
    posData = dataLog.r_BN_N # hub inertial position
    velData = dataLog.v_BN_N # hub inertial velocity
    panel1data = pl1Log.state # panel 1 angle
    panel2data = pl2Log.state # panel 2 angle
    timeAxis = dataLog.times() # time data

    # Generating plots
    plt.close("all")
    figureList = {}
    figureList[fileName + "1"] = plotInertialPos(timeAxis, posData)
    figureList[fileName + "2"] = plotOrbitalMotion(timeAxis, posData, velData, muEarth)
    figureList[fileName + "3"] = plotAngDisp(timeAxis, panel1data, panel2data)

    if showPlots:
        plt.show()

    return figureList


# -------------------------------------------------------------------------
# InertialForceToSiteActuator: custom sys model to convert fixed inertial-frame
#   thrust force into body-frame force at specified site
# -------------------------------------------------------------------------
class InertialForceToSiteActuator(sysModel.SysModel):
    def __init__(self):
        super().__init__()
        self.force_N = np.zeros(3) # desired thrust force (inertial)
        self.burnStartNanos = 0
        self.burnEndNanos = 0

        self.scStateInMsg = messaging.SCStatesMsgReader() # s/c state for current attitude
        self.forceOutMsg = messaging.ForceAtSiteMsg() # output force at actuator site

    def setForce_N(self, force_N):
        """Sets desired thrust force vector (inertial)"""
        self.force_N = np.array(force_N)

    def setBurnWindow(self, burnStartNanos, burnEndNanos):
        """Sets simulation time window during burn"""
        self.burnStartNanos = burnStartNanos
        self.burnEndNanos = burnEndNanos

    def UpdateState(self, CurrentSimNanos):
        """Called at each simulation step, computes output force message"""
        if self.burnStartNanos <= CurrentSimNanos <= self.burnEndNanos:
            state = self.scStateInMsg()
            dcm_BN = RigidBodyKinematics.MRP2C(np.array(state.sigma_BN))
            force_B = dcm_BN @ self.force_N # inertial thrust converted to body during burn
        else:
            force_B = np.zeros(3) # no thrust outside burn

        # NOTE: frames S and B in this scenario are identical, thus force_S = force_B
        self.forceOutMsg.write(messaging.ForceAtSiteMsgPayload(force_S = force_B), self.moduleID, CurrentSimNanos)

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


if __name__ == "__main__":
    run(showPlots = True)