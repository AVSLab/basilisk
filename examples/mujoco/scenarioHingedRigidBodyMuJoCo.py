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

#. :ref:`scenarioHingedRigidBody` for the classic hinged-panel model.
#. :ref:`scenarioReactionWheel` for MuJoCo bodies, joints, and actuators.

This script demonstrates how to run the classic Basilisk
:ref:`scenarioHingedRigidBody` example using MuJoCo dynamics via
:ref:`MJScene<MJScene>` instead of the traditional hub-centric Basilisk
:ref:`spacecraft` dynamics.

Running the Example
-------------------

Use a Basilisk installation with MuJoCo support and the example dependencies
described in :ref:`bskInstall`. When building from source, enable
``--mujoco True``. With that Python environment active, run from the repository
root:

.. code-block:: console

   python examples/mujoco/scenarioHingedRigidBodyMuJoCo.py

The script displays the result plots. When importing the scenario from
``examples/mujoco``, call ``run(showPlots=False)`` to run without displaying
windows; it still returns a dictionary of Matplotlib figures for inspection
or saving.

Model and Force Setup
---------------------

The multi-body system is created programmatically as a MuJoCo XML string.
It consists of a free-floating spacecraft bus ("hub") with two solar panel
rigid bodies ("panel1", "panel2") attached via hinge joints ("hinge1",
"hinge2"), giving the system 8 total degrees of freedom (3 translational,
3 rotational, and 2 panel hinge DOFs).

Two existing library modules and a scenario-specific burn command are added
to the MuJoCo dynamics task:

#. :ref:`MJJointPIDController <JointPIDController>` computes a torsional
   spring-damper restoring torque for a hinge joint from its angle and angular
   rate, and writes
   the result as a ``SingleActuatorMsg`` command. One instance is
   attached to each panel hinge to emulate panel stiffness and damping.
#. ``BurnWindowForceCommand``, defined in this script, publishes a fixed force
   in inertial frame N during the inclusive interval ``[burnStart, burnEnd]``
   and zero force outside it.
#. :ref:`MJCmdForceInertialToForceAtSite <cmdForceInertialToForceAtSite>` converts
   that command from inertial frame N into the thruster site frame S using
   the site's current attitude. The site is fixed to the hub and aligned
   with body frame B in this example. The converter handles the rotation;
   the burn-command module determines when thrust is requested.

The force message chain is ``BurnWindowForceCommand`` ->
``MJCmdForceInertialToForceAtSite`` -> the MuJoCo force actuator at
``thrustSite``. This keeps the requested force direction fixed in inertial
space even as the spacecraft rotates.

Earth gravity is configured using :ref:`NBodyGravity<NBodyGravity>` with a
:ref:`pointMassGravityModel<pointMassGravityModel>` as the central body.
Gravity targets are registered manually for the hub and both panel bodies.

The spacecraft is placed on a near-circular low Earth orbit (LEO). The
simulation runs for a coast phase lasting 1% of an orbit followed by a
935-second translational burn, during which the panels respond dynamically
to the resulting body motion through their spring-damper hinges.

Inertial position, orbital radius, and the two panel hinge angular
displacements are plotted at the end.

Illustration of Simulation Results
----------------------------------

.. image:: /_images/Scenarios/scenarioHingedRigidBodyMuJoCo_inertPos.svg
   :align: center

.. image:: /_images/Scenarios/scenarioHingedRigidBodyMuJoCo_orbitalMotion.svg
   :align: center

.. image:: /_images/Scenarios/scenarioHingedRigidBodyMuJoCo_angDisp.svg
   :align: center
"""

from typing import Tuple
import os

import matplotlib.pyplot as plt
import numpy as np

from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import NBodyGravity, mujoco, pointMassGravityModel, MJJointPIDController, MJCmdForceInertialToForceAtSite
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simHelpers

# Used to tag saved figs with name of file
fileName = os.path.basename(os.path.splitext(__file__)[0])

# -------------------------------------------------------------------------
# PLOTTING FUNCTIONS
# -------------------------------------------------------------------------
def plotInertialPos(timeAxis: np.ndarray, posData: np.ndarray) -> plt.Figure:
    """Plot the inertial position vector components."""
    fig = plt.figure(num = 1, clear = True)
    ax = fig.gca()
    ax.ticklabel_format(useOffset = False, style = 'plain')
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2HOUR, posData[:, idx] / 1000.,
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
    ax1.set_ylabel(r'$\theta$ [rad]')
    ax1.set_title('Panel 1 Angular Displacement')

    ax2.plot(timeAxis * macros.NANO2MIN, panel2thetaLog)
    ax2.set_xlabel("Time [min]")
    ax2.set_ylabel(r'$\theta$ [rad]')
    ax2.set_title('Panel 2 Angular Displacement')

    fig.tight_layout()

    return fig


def makeMjXmlString(hubMass: float = 800.0, busIDiag: Tuple[float, float, float] = (900.0, 800.0, 600.0)):
    """Build MuJoCo XML for a spacecraft hub with two hinged panels.

    :param hubMass: Hub mass [kg]. Defaults to 800.0 kg.
    :param busIDiag: Principal moments of inertia about the hub center of mass,
        ordered along the hub x, y, and z axes [kg m^2]. Defaults to
        ``(900.0, 800.0, 600.0)`` kg m^2.
    :returns: XML defining the hub, panel bodies, hinge joints, and thrust site.
    :rtype: str
    """
    ixx, iyy, izz = busIDiag

    # XML lengths are [m], masses [kg], inertia components [kg m^2], and angles [rad].
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
    """Run a short orbital coast followed by a 935-second burn.

    :param showPlots: Display the Matplotlib figures and wait for the plot
        windows to close when True. Defaults to False; figures are still
        generated and returned in either case.
    :returns: Dictionary mapping scenario-prefixed names to three Matplotlib
        figures: inertial position, orbital radius, and panel hinge angles.
        No files are saved by this function.
    :rtype: dict
    """
    # -------------------------------------------------------------------------
    # 1) Simulation configuration and MJScene dynamics model
    # -------------------------------------------------------------------------
    simTaskName = "simTask"
    simProcessName = "simProcess"

    timeStep = macros.sec2nano(0.1)  # [ns]

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
    # 2) Retrieve spacecraft components
    # -------------------------------------------------------------------------
    # Pull handles of hub/panel bodies from XML
    busBody = scene.getBody("hub")
    panelBodies = [scene.getBody(name) for name in ("panel1", "panel2")]
    numPanels = len(panelBodies)

    # Pull scalar joints connecting panels
    hinge1 = panelBodies[0].getScalarJoint("hinge1")
    hinge2 = panelBodies[1].getScalarJoint("hinge2")

    # -------------------------------------------------------------------------
    # 3) Adding damping/stiffness to panel hinges
    # -------------------------------------------------------------------------
    # Initialize damping and stiffness values.
    k = 1000.0 # Nm/rad
    c = 0.0 # Nms/rad
    thetaRef = 0.0  # [rad]

    # Keep all springDamper models in list so they remain in scope for whole sim
    springDampers = []
    for jointName, body in [("hinge1", panelBodies[0]), ("hinge2", panelBodies[1])]:
        # Retrieving joints and adding actuators to mimic damping/stiffness effects
        actuator = scene.addJointSingleActuator(f"{jointName}Actuator", jointName)
        joint = body.getScalarJoint(jointName)

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
    rLEO = 7000. * 1000  # meters
    oe.a = rLEO
    oe.e = 0.0001  # [-]
    oe.i = 0.0 * macros.D2R  # [rad]
    oe.Omega = 48.2 * macros.D2R  # [rad]
    oe.omega = 347.8 * macros.D2R  # [rad]
    oe.f = 85.3 * macros.D2R  # [rad]
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
    simulationTimeFactor = 0.01  # [-] Fraction of one orbit spent coasting.
    simulationTime = macros.sec2nano(simulationTimeFactor * P)

    T2 = macros.sec2nano(935.)  # [ns] Burn duration from scenarioHingedRigidBody.
    burnStart = simulationTime
    burnEnd = simulationTime + T2

    # Scenario-owned burn schedule: publishes the desired inertial force inside burn window
    burnCommand = BurnWindowForceCommand([-2050.0, -1430.0, -0.00076], burnStart, burnEnd)  # force [N]
    burnCommand.ModelTag = "burnCommand"
    scene.AddModelToDynamicsTask(burnCommand)

    # Site frame conversion
    forceConverter = MJCmdForceInertialToForceAtSite.CmdForceInertialToForceAtSite()
    forceConverter.ModelTag = "forceConverter"
    forceConverter.cmdForceInertialInMsg.subscribeTo(burnCommand.cmdForceOutMsg)
    forceConverter.siteFrameStateInMsg.subscribeTo(busBody.getSite("thrustSite").stateOutMsg)
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
    figureList[fileName + "_inertPos"] = plotInertialPos(timeAxis, posData)
    figureList[fileName + "_orbitalMotion"] = plotOrbitalMotion(timeAxis, posData, velData, muEarth)
    figureList[fileName + "_angDisp"] = plotAngDisp(timeAxis, panel1data, panel2data)

    if showPlots:
        plt.show()

    return figureList


class BurnWindowForceCommand(sysModel.SysModel):
    """Publish an inertial force during a burn window and zero outside it.

    :param force_N: Three-component force expressed in inertial frame N [N].
    :param burnStartNanos: Inclusive burn start time measured from simulation
        start [ns].
    :param burnEndNanos: Inclusive burn end time measured from simulation
        start [ns].
    :ivar cmdForceOutMsg: Commanded inertial force consumed by
        ``MJCmdForceInertialToForceAtSite`` before reaching the actuator.
    """

    def __init__(self, force_N, burnStartNanos, burnEndNanos):
        super().__init__()
        self.force_N = [float(f) for f in force_N]
        self.burnStartNanos = burnStartNanos
        self.burnEndNanos = burnEndNanos
        self.cmdForceOutMsg = messaging.CmdForceInertialMsg()

    def UpdateState(self, CurrentSimNanos):
        payload = messaging.CmdForceInertialMsgPayload()
        if self.burnStartNanos <= CurrentSimNanos <= self.burnEndNanos:
            payload.forceRequestInertial = self.force_N
        else:
            payload.forceRequestInertial = [0.0, 0.0, 0.0]
        self.cmdForceOutMsg.write(payload, CurrentSimNanos, self.moduleID)


if __name__ == "__main__":
    run(showPlots = True)
