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

"""
It's recommended to review the following scenario(s) first (and any
recommended scenario(s) that they may have):

#. ``examples/mujoco/scenarioReactionWheel.py``

This script demonstrates how to run the classic Basilisk
``scenarioAttitudeFeedbackRW.py`` example using MuJoCo dynamics via
:ref:`MJScene<MJScene>` instead of the traditional hub-centric Basilisk
:ref:`spacecraft` dynamics.

The multi-body system is created programmatically as a MuJoCo XML string.
It consists of a free-floating spacecraft bus ("bus") with three reaction
wheel rigid bodies ("rw1", "rw2", "rw3") attached via hinge joints. Each
hinge joint is driven by a MuJoCo single-input actuator, which applies a
motor torque directly to the wheel spin DOF.

A standard Basilisk FSW stack is used:

#. ``inertial3D`` generates an inertial attitude reference.
#. ``attTrackingError`` computes the attitude and rate tracking errors.
#. ``mrpFeedback`` computes the commanded body torque.
#. ``rwMotorTorque`` maps the commanded body torque into individual wheel
   motor torque commands.

Three small "adapter" modules bridge Basilisk messaging to MuJoCo objects:

#. ``scalarJointStatesToRWSpeed`` converts MuJoCo wheel joint rate states into an
   ``RWSpeedMsg`` so the controller can compensate for wheel momentum.
#. ``arrayMotorTorqueToSingleActuators`` converts the ``ArrayMotorTorqueMsg``
   into three ``SingleActuatorMsg`` messages (one per MuJoCo motor).
#. ``saturationSingleActuator`` optionally clamps each ``SingleActuatorMsg`` to
   emulate actuator torque limits (for example, reaction wheel ``uMax``).

The simulation runs for 10 minutes. The attitude error, rate error, wheel
motor torques, and wheel speeds are plotted at the end.

Illustration of Simulation Results
----------------------------------

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRWMuJoCo1.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAttitudeFeedbackRWMuJoCo2.svg
   :align: center

"""

from typing import Tuple
import os

import matplotlib.pyplot as plt
import numpy as np

from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import attTrackingError, inertial3D, mrpFeedback, rwMotorTorque
from Basilisk.simulation import NBodyGravity
from Basilisk.simulation import arrayMotorTorqueToSingleActuators
from Basilisk.simulation import mujoco
from Basilisk.simulation import pointMassGravityModel
from Basilisk.simulation import saturationSingleActuator
from Basilisk.simulation import scalarJointStatesToRWSpeed
from Basilisk.simulation import simpleNav
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, unitTestSupport

fileName = os.path.basename(os.path.splitext(__file__)[0])


def plotAttitudeAndRateErrors(timeMin: np.ndarray, sigmaBR: np.ndarray, omegaBRB: np.ndarray):
    """Plot attitude error (MRPs) and rate error side-by-side with a shared x-axis."""
    fig, (ax1, ax2) = plt.subplots(1, 2, sharex=True, num=1, clear=True)

    for i in range(3):
        ax1.plot(timeMin, sigmaBR[:, i], color=unitTestSupport.getLineColor(i, 3), label=rf"$\sigma_{i}$")
    ax1.set_xlabel("Time [min]")
    ax1.set_ylabel(r"Attitude Error $[\mathbf{\sigma}]$")
    ax1.legend(loc="lower right")

    for i in range(3):
        ax2.plot(timeMin, omegaBRB[:, i], color=unitTestSupport.getLineColor(i, 3), label=rf"$\omega_{{BR,{i}}}$")
    ax2.set_xlabel("Time [min]")
    ax2.set_ylabel("Rate Tracking Error [rad/s]")
    ax2.legend(loc="lower right")

    fig.tight_layout()

    return fig


def plotCmdTorquesAndRwSpeeds(timeMin: np.ndarray, motorTorque: np.ndarray, rwOmega: np.ndarray, numRw: int):
    """Plot commanded RW motor torques and RW spin rates side-by-side with a shared x-axis."""
    fig, (ax1, ax2) = plt.subplots(1, 2, sharex=True, num=2, clear=True)

    for i in range(numRw):
        ax1.plot(timeMin, motorTorque[:, i], color=unitTestSupport.getLineColor(i, 3), label=rf"$\hat u_{{s,{i}}}$")
    ax1.set_xlabel("Time [min]")
    ax1.set_ylabel("Commanded RW Motor Torque [Nm]")
    ax1.legend(loc="lower right")

    for i in range(numRw):
        ax2.plot(timeMin, rwOmega[:, i] / macros.RPM, color=unitTestSupport.getLineColor(i, 3), label=rf"$\Omega_{i}$")
    ax2.set_xlabel("Time [min]")
    ax2.set_ylabel("RW Speed [RPM]")
    ax2.legend(loc="lower right")

    fig.tight_layout()

    return fig


def getHr16SmallWheelParams():
    """Return a simple set of wheel parameters based on Honeywell HR16 small configuration (maxMomentum = 50 Nms)."""
    maxMomentum = 50.0  # [Nms]
    rwMass = 9.0  # [kg]
    rwUMax = 0.200  # [Nm]
    rwOmegaMax = 6000.0 * macros.RPM  # [rad/s]

    rwJs = maxMomentum / rwOmegaMax
    rwJt = 0.5 * rwJs

    return rwMass, rwUMax, rwOmegaMax, rwJs, rwJt


def makeMjXmlString(busMass: float, busIDiag: Tuple[float, float, float], rwMass: float, rwJs: float, rwJt: float):
    """
    Create an MJCF XML string for a free-flying bus with three reaction wheels.

    Notes:
    - Contact is disabled because this scenario does not involve collisions.
    - Actuators are created in Python (via scene.addJointSingleActuator), not in XML.
    """
    ixx, iyy, izz = busIDiag
    return f"""
<mujoco model="busWith3Rws">
  <option>
    <flag contact="disable"/>
  </option>

  <worldbody>
    <body name="bus" pos="0 0 0">
      <freejoint name="busFree"/>

      <inertial pos="0 0 0" mass="{busMass}"
               diaginertia="{ixx} {iyy} {izz}"/>

      <geom type="box" size="0.6 0.5 0.4" rgba="0.6 0.6 0.6 1"/>

      <body name="rw1" pos="0 0 0">
        <joint name="rw1Spin" type="hinge" axis="1 0 0" limited="false"/>
        <inertial pos="0 0 0" mass="{rwMass}"
                 diaginertia="{rwJs} {rwJt} {rwJt}"/>
        <geom type="cylinder" size="0.08 0.03" rgba="0.2 0.2 0.8 1" euler="0 90 0"/>
      </body>

      <body name="rw2" pos="0 0 0">
        <joint name="rw2Spin" type="hinge" axis="0 1 0" limited="false"/>
        <inertial pos="0 0 0" mass="{rwMass}"
                 diaginertia="{rwJt} {rwJs} {rwJt}"/>
        <geom type="cylinder" size="0.08 0.03" rgba="0.2 0.8 0.2 1" euler="90 0 0"/>
      </body>

      <body name="rw3" pos="0.5 0.5 0.5">
        <joint name="rw3Spin" type="hinge" axis="0 0 1" limited="false"/>
        <inertial pos="0 0 0" mass="{rwMass}"
                 diaginertia="{rwJt} {rwJt} {rwJs}"/>
        <geom type="cylinder" size="0.08 0.03" rgba="0.8 0.2 0.2 1"/>
      </body>

    </body>
  </worldbody>
</mujoco>
"""


def run(showPlots: bool = False):
    """Build and run the MJScene attitude feedback simulation."""
    # -------------------------------------------------------------------------
    # 1) Simulation timing configuration
    # -------------------------------------------------------------------------
    gncTaskName = "simTask"
    simProcessName = "simProcess"

    simulationTime = macros.min2nano(10.0)
    timeStep = macros.sec2nano(0.1)

    # -------------------------------------------------------------------------
    # 2) Physical parameters (match the classic scenarioAttitudeFeedbackRW)
    # -------------------------------------------------------------------------
    busMass = 750.0
    busIDiag = (900.0, 800.0, 600.0)

    rwMass, rwUMax, rwOmegaMax, rwJs, rwJt = getHr16SmallWheelParams()
    numRw = 3

    # -------------------------------------------------------------------------
    # 3) Create the Basilisk simulation and add the MJScene dynamics model
    # -------------------------------------------------------------------------
    sim = SimulationBaseClass.SimBaseClass()
    dynProcess = sim.CreateNewProcess(simProcessName)
    dynProcess.addTask(sim.CreateNewTask(gncTaskName, timeStep))

    # MJCF is constructed as a string and loaded in the scene
    xmlString = makeMjXmlString(busMass, busIDiag, rwMass, rwJs, rwJt)
    scene = mujoco.MJScene(xmlString)
    scene.ModelTag = "mujocoScene"
    sim.AddModelToTask(gncTaskName, scene)

    # -------------------------------------------------------------------------
    # 4) Retrieve MuJoCo bodies and joints, and create joint actuators for each wheel
    # -------------------------------------------------------------------------
    busBody = scene.getBody("bus")
    rwBodies = [scene.getBody(name) for name in ("rw1", "rw2", "rw3")]
    rwJoints = [rwBodies[i].getScalarJoint(f"rw{i+1}Spin") for i in range(numRw)]

    # MuJoCo actuators accept a SingleActuatorMsg scalar command.
    rwActs = [scene.addJointSingleActuator(f"rw{i+1}Act", rwJoints[i]) for i in range(numRw)]

    # -------------------------------------------------------------------------
    # 5) Add gravity to the special MuJoCo dynamics task (evaluated at integrator substeps)
    # -------------------------------------------------------------------------
    gravity = NBodyGravity.NBodyGravity()
    gravity.ModelTag = "gravity"
    scene.AddModelToDynamicsTask(gravity)

    muEarth = 0.3986004415e15  # [m^3/s^2]
    earthPm = pointMassGravityModel.PointMassGravityModel()
    earthPm.muBody = muEarth
    gravity.addGravitySource("earth", earthPm, isCentralBody=True)

    # Apply gravity to all bodies to allow consistent orbital motion.
    gravity.addGravityTarget("bus", busBody)
    for i in range(numRw):
        gravity.addGravityTarget(f"rw{i+1}", rwBodies[i])

    # -------------------------------------------------------------------------
    # 6) Navigation: read the bus state from MJScene and publish standard nav outputs
    # -------------------------------------------------------------------------
    simpleNavObj = simpleNav.SimpleNav()
    simpleNavObj.ModelTag = "simpleNav"
    simpleNavObj.scStateInMsg.subscribeTo(busBody.getCenterOfMass().stateOutMsg)
    sim.AddModelToTask(gncTaskName, simpleNavObj)

    # -------------------------------------------------------------------------
    # 7) FSW stack: inertial reference -> tracking error -> MRP feedback -> wheel torques
    # -------------------------------------------------------------------------
    inertialGuid = inertial3D.inertial3D()
    inertialGuid.ModelTag = "inertial3D"
    inertialGuid.sigma_R0N = [0.0, 0.0, 0.0]
    sim.AddModelToTask(gncTaskName, inertialGuid)

    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attTrackingError"
    attError.attNavInMsg.subscribeTo(simpleNavObj.attOutMsg)
    attError.attRefInMsg.subscribeTo(inertialGuid.attRefOutMsg)
    sim.AddModelToTask(gncTaskName, attError)

    # Reaction wheel configuration (axes, spin inertias, and uMax limits).
    rwConfigPayload = messaging.RWArrayConfigMsgPayload()
    rwConfigPayload.numRW = numRw
    rwConfigPayload.GsMatrix_B = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]
    rwConfigPayload.JsList = [rwJs, rwJs, rwJs]
    rwConfigPayload.uMax = [rwUMax, rwUMax, rwUMax]
    rwConfigMsg = messaging.RWArrayConfigMsg().write(rwConfigPayload)

    # Convert joint rate states into a single RWSpeedMsg for momentum compensation.
    rwSpeedsAdapter = scalarJointStatesToRWSpeed.ScalarJointStatesToRWSpeed()
    rwSpeedsAdapter.ModelTag = "scalarJointStatesToRWSpeed"
    rwSpeedsAdapter.setNumJoints(numRw)
    for i in range(numRw):
        rwSpeedsAdapter.jointStateInMsgs[i].subscribeTo(rwJoints[i].stateDotOutMsg)
    sim.AddModelToTask(gncTaskName, rwSpeedsAdapter)

    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    mrpControl.K = 3.5
    mrpControl.Ki = -1
    mrpControl.P = 30.0
    mrpControl.integralLimit = 0
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    mrpControl.rwParamsInMsg.subscribeTo(rwConfigMsg)
    mrpControl.rwSpeedsInMsg.subscribeTo(rwSpeedsAdapter.rwSpeedOutMsg)
    sim.AddModelToTask(gncTaskName, mrpControl)

    # Vehicle inertia (used internally by mrpFeedback).
    ISCPntB_B = [
        busIDiag[0], 0.0, 0.0,
        0.0, busIDiag[1], 0.0,
        0.0, 0.0, busIDiag[2],
    ]
    vehicleConfigPayload = messaging.VehicleConfigMsgPayload(ISCPntB_B=ISCPntB_B)
    vehicleConfigMsg = messaging.VehicleConfigMsg().write(vehicleConfigPayload)
    mrpControl.vehConfigInMsg.subscribeTo(vehicleConfigMsg)

    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(rwConfigMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)
    rwMotorTorqueObj.controlAxes_B = [
        1, 0, 0,
        0, 1, 0,
        0, 0, 1,
    ]
    sim.AddModelToTask(gncTaskName, rwMotorTorqueObj)

    # -------------------------------------------------------------------------
    # 8) Adapters: ArrayMotorTorqueMsg -> SingleActuatorMsg (per wheel), then clamp each command
    # -------------------------------------------------------------------------
    arrayToSingles = arrayMotorTorqueToSingleActuators.ArrayMotorTorqueToSingleActuators()
    arrayToSingles.ModelTag = "arrayMotorTorqueToSingleActuators"
    arrayToSingles.setNumActuators(numRw)
    arrayToSingles.torqueInMsg.subscribeTo(rwMotorTorqueObj.rwMotorTorqueOutMsg)
    sim.AddModelToTask(gncTaskName, arrayToSingles)

    # Create one saturation module per wheel so each motor command is independently limited.
    # Keep the limiter objects in a list so they remain in scope for the entire simulation.
    torqueLimiters = []
    for i in range(numRw):
        limiter = saturationSingleActuator.SaturationSingleActuator()
        limiter.ModelTag = f"rw{i+1}TorqueLimiter"
        limiter.setMinInput(-rwUMax)
        limiter.setMaxInput(rwUMax)

        # Wire: array element i -> limiter -> MuJoCo actuator i
        limiter.actuatorInMsg.subscribeTo(arrayToSingles.actuatorOutMsgs[i])
        sim.AddModelToTask(gncTaskName, limiter)

        rwActs[i].actuatorInMsg.subscribeTo(limiter.actuatorOutMsg)
        torqueLimiters.append(limiter)

    # -------------------------------------------------------------------------
    # 9) Data logging setup
    # -------------------------------------------------------------------------
    numDataPoints = 100
    samplingTime = unitTestSupport.samplingTime(simulationTime, timeStep, numDataPoints)

    attErrorLog = attError.attGuidOutMsg.recorder(samplingTime)

    # Log wheel rates directly from MuJoCo joints.
    rwOmegaLogs = [rwJoints[i].stateDotOutMsg.recorder(samplingTime) for i in range(numRw)]

    # Log the final actuator commands that MuJoCo receives (after saturation).
    rwCmdLogs = [rwActs[i].actuatorInMsg.recorder(samplingTime) for i in range(numRw)]

    sim.AddModelToTask(gncTaskName, attErrorLog)
    for i in range(numRw):
        sim.AddModelToTask(gncTaskName, rwCmdLogs[i])
        sim.AddModelToTask(gncTaskName, rwOmegaLogs[i])

    # -------------------------------------------------------------------------
    # 10) Initialize sim and set initial conditions (orbit, attitude, wheel speeds)
    # -------------------------------------------------------------------------
    sim.InitializeSimulation()

    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(muEarth, oe)

    busBody.setPosition(rN)
    busBody.setVelocity(vN)
    busBody.setAttitude([0.1, 0.2, -0.3])
    busBody.setAttitudeRate([0.001, -0.01, 0.03])

    # Initial wheel speeds (rad/s), matching the classic example initial RPMs.
    initRwRpm = [100.0, 200.0, 300.0]
    for i in range(numRw):
        rwJoints[i].setVelocity(initRwRpm[i] * macros.RPM)

    # -------------------------------------------------------------------------
    # 11) Run the simulation
    # -------------------------------------------------------------------------
    sim.ConfigureStopTime(simulationTime)
    sim.ExecuteSimulation()

    # -------------------------------------------------------------------------
    # 12) Post-process and plot
    # -------------------------------------------------------------------------
    timeMin = attErrorLog.times() * macros.NANO2MIN
    sigmaBR = attErrorLog.sigma_BR
    omegaBRB = attErrorLog.omega_BR_B

    cmdTorques = np.column_stack([rwCmdLogs[i].input for i in range(numRw)])
    rwOmega = np.column_stack([rwOmegaLogs[i].state for i in range(numRw)])

    plt.close("all")
    figureList = {}
    figureList[fileName + "1"] = plotAttitudeAndRateErrors(timeMin, sigmaBR, omegaBRB)
    figureList[fileName + "2"] = plotCmdTorquesAndRwSpeeds(timeMin, cmdTorques, rwOmega, numRw)

    if showPlots:
        plt.show()

    plt.close("all")

    return figureList

if __name__ == "__main__":
    run(showPlots=True)
