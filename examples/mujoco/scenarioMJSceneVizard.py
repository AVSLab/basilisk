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
This scenario builds on :ref:`scenarioDeployPanels` to show how multiple
:ref:`MJScene<MJScene>` instances and a regular :ref:`spacecraft` object
coexist in one simulation and feed :ref:`Vizard <vizard>` through a single
``enableUnityVisualization`` call:

* **Primary hub** (MJScene, red, 6 panels): loaded from
  ``sat_w_deployable_panels.xml`` - the same XML used by
  ``scenarioDeployPanels.py``.  Circular 500 km LEO around Earth, PID
  deployment, and a four-unit CSS pyramid on the hub.
* **Companion hub** (MJScene, blue, 2 panels): loaded from
  ``sat_w_companion.xml``.  Same circular LEO, 15 m ahead along-track,
  with a Vizard star tracker on the hub.
* **Trailer** (regular Basilisk :ref:`spacecraft`): point-mass body 15 m
  behind the primary along-track, gravity wired through the standard
  ``gravFactory.addBodiesTo`` plumbing, with a Vizard transceiver
  (downlink antenna).

Illustration of Simulation Results
----------------------------------

This scenario does not generate plots; instead it writes to
``examples/mujoco/_VizFiles/scenarioMJSceneVizard_UnityViz.bin`` if the user uncomments
the ``saveFile=__file__`` line.
"""

import os
from typing import Any, List

import numpy as np

from Basilisk.architecture import messaging
from Basilisk.simulation import coarseSunSensor
from Basilisk.simulation import mujoco
from Basilisk.simulation import pointMassGravityModel
from Basilisk.simulation import spacecraft
from Basilisk.simulation import StatefulSysModel
from Basilisk.simulation import svIntegrators
try:
    from Basilisk.simulation import vizInterface
except ImportError:
    pass
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import vizSupport

CURRENT_FOLDER = os.path.dirname(__file__)
PRIMARY_XML_PATH = os.path.join(CURRENT_FOLDER, "sat_w_deployable_panels.xml")
COMPANION_XML_PATH = os.path.join(CURRENT_FOLDER, "sat_w_companion.xml")

# Along-track separation between adjacent spacecraft.
SC_SEPARATION_M = 15.0  # [m]

# Same profile as scenarioDeployPanels.py
JOINT_START_END = [(np.pi / 2, 0), (np.pi, 0), (np.pi, 0)]  # [rad] indexed by tier
MAX_PROFILE_VEL = np.deg2rad(0.05)  # [deg/s] -> [rad/s]
MAX_PROFILE_ACCEL = np.deg2rad(0.0001)  # [deg/s^2] -> [rad/s^2]


# Trapezoidal profile generator
def generateProfiles(
    initialPoint: float, finalPoint: float, vMax: float, aMax: float
) -> List[mujoco.ScalarJointStateInterpolator]:
    distance = abs(finalPoint - initialPoint)
    tAccel = vMax / aMax
    dAccel = 0.5 * aMax * tAccel**2
    if 2 * dAccel > distance:
        tAccel = np.sqrt(distance / aMax)
        tConst = 0.0
    else:
        tConst = (distance - 2 * dAccel) / vMax

    tTotal = 2 * tAccel + tConst
    time = np.linspace(0, tTotal, 100)
    position = np.zeros_like(time)
    velocity = np.zeros_like(time)

    for k, t in enumerate(time):
        if t < tAccel:
            velocity[k] = aMax * t
            position[k] = 0.5 * aMax * t**2
        elif t < tAccel + tConst:
            velocity[k] = vMax
            position[k] = dAccel + vMax * (t - tAccel)
        else:
            tDecel = t - tAccel - tConst
            velocity[k] = vMax - aMax * tDecel
            position[k] = (
                dAccel + vMax * tConst + (vMax * tDecel - 0.5 * aMax * tDecel**2)
            )

    if finalPoint > initialPoint:
        position = initialPoint + position
    else:
        position = initialPoint - position
        velocity = -velocity

    interpolators = []
    for values in (position, velocity):
        pts = np.column_stack([time * 1e9, values])
        interp = mujoco.ScalarJointStateInterpolator()
        interp.setDataPoints(pts, 1)
        interpolators.append(interp)
    return interpolators


# PID controller
class PIDController(StatefulSysModel.StatefulSysModel):
    def __init__(self, *args: Any):
        super().__init__(*args)
        self.K_p = 0.1
        self.K_d = 0.002
        self.K_i = 0.0001
        self.measuredInMsg = messaging.ScalarJointStateMsgReader()
        self.desiredInMsg = messaging.ScalarJointStateMsgReader()
        self.measuredDotInMsg = messaging.ScalarJointStateMsgReader()
        self.desiredDotInMsg = messaging.ScalarJointStateMsgReader()
        self.outputOutMsg = messaging.SingleActuatorMsg()

    def registerStates(self, registerer: StatefulSysModel.DynParamRegisterer):
        self.integralErrorState = registerer.registerState(1, 1, "integralError")
        self.integralErrorState.setState([[0]])

    def UpdateState(self, CurrentSimNanos: int):
        stateError = self.desiredInMsg().state - self.measuredInMsg().state
        stateDotError = self.desiredDotInMsg().state - self.measuredDotInMsg().state
        stateIntegralError = self.integralErrorState.getState()[0][0]
        control_output = (
            self.K_p * stateError
            + self.K_d * stateDotError
            + self.K_i * stateIntegralError
        )
        self.integralErrorState.setDerivative([[stateError]])
        self.outputOutMsg.write(
            messaging.SingleActuatorMsgPayload(input=control_output),
            CurrentSimNanos,
            self.moduleID,
        )


# Helper to attach PID deployment to a set of panel actuators in a scene
def addDeployment(scene, panelIds, bodyPrefix, modelCache):
    for side, i in panelIds:
        actuatorName = f"{bodyPrefix}_{side}{i}_deploy"
        bodyName = f"{bodyPrefix}_{side}{i}"
        act = scene.getSingleActuator(actuatorName)
        joint = scene.getBody(bodyName).getScalarJoint(actuatorName)

        posInterp, velInterp = generateProfiles(
            JOINT_START_END[i - 1][0],
            JOINT_START_END[i - 1][1],
            MAX_PROFILE_VEL,
            MAX_PROFILE_ACCEL,
        )
        pid = PIDController()
        pid.ModelTag = f"{actuatorName}_ctrl"
        pid.desiredInMsg.subscribeTo(posInterp.interpolatedOutMsg)
        pid.desiredDotInMsg.subscribeTo(velInterp.interpolatedOutMsg)
        pid.measuredInMsg.subscribeTo(joint.stateOutMsg)
        pid.measuredDotInMsg.subscribeTo(joint.stateDotOutMsg)
        act.actuatorInMsg.subscribeTo(pid.outputOutMsg)

        scene.AddModelToDynamicsTask(posInterp, 50)
        scene.AddModelToDynamicsTask(velInterp, 49)
        scene.AddModelToDynamicsTask(pid, 25)
        modelCache.extend([posInterp, velInterp, pid])


def setStowed(scene, panelIds, bodyPrefix):
    for side, i in panelIds:
        scene.getBody(f"{bodyPrefix}_{side}{i}").getScalarJoint(
            f"{bodyPrefix}_{side}{i}_deploy"
        ).setPosition(JOINT_START_END[i - 1][0])


# Main scenario
def run(showPlots: bool = False):
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.SetProgressBar(True)
    proc = scSim.CreateNewProcess("simProcess")
    taskPeriodSec = 4.0  # [s]
    proc.addTask(scSim.CreateNewTask("simTask", macros.sec2nano(taskPeriodSec)))

    # Two MJScenes, the primary has 6 panels, and the companion 2 panels.
    # Each is loaded from its own XML and both are running in the same simTask.
    primaryScene = mujoco.MJScene.fromFile(PRIMARY_XML_PATH)
    primaryScene.ModelTag = "primaryScene"
    scSim.AddModelToTask("simTask", primaryScene)

    # MJScene defaults to RK4 so we swap in RKF45 for the adaptive step-size
    # control needed at orbital speeds.
    integ = svIntegrators.svIntegratorRKF45(primaryScene)
    accuracy = 1e-5
    integ.setRelativeTolerance(accuracy)
    integ.setAbsoluteTolerance(accuracy)
    primaryScene.setIntegrator(integ)
    primaryScene.extraEoMCall = True

    companionScene = mujoco.MJScene.fromFile(COMPANION_XML_PATH)
    companionScene.ModelTag = "companionScene"
    scSim.AddModelToTask("simTask", companionScene)
    integ2 = svIntegrators.svIntegratorRKF45(companionScene)
    integ2.setRelativeTolerance(accuracy)
    integ2.setAbsoluteTolerance(accuracy)
    companionScene.setIntegrator(integ2)
    companionScene.extraEoMCall = True

    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    mu = earth.mu
    gravFactory.createMoon()
    gravFactory.createSun()

    spiceObject = gravFactory.createSpiceInterface(
        time="2025 NOVEMBER 15 12:00:00.000",
        epochInMsg=True,
    )
    spiceObject.zeroBase = "Earth"
    scSim.AddModelToTask("simTask", spiceObject)

    # vizSupport dedupes planets by name across scenes.
    primaryScene._vizGravBodies = gravFactory.gravBodies
    companionScene._vizGravBodies = gravFactory.gravBodies

    # Trailer is a regular Basilisk Spacecraft, behind the primary along-track.
    # Uses ``gravFactory.addBodiesTo`` for the standard gravField wiring.
    trailer = spacecraft.Spacecraft()
    trailer.ModelTag = "trailer"
    scSim.AddModelToTask("simTask", trailer)
    gravFactory.addBodiesTo(trailer)

    # Primary hub orbit definition.
    oe = orbitalMotion.ClassicElements()
    oe.a = 6_371_000.0 + 500_000.0  # [m]
    oe.e = 0.0                       # [-]
    oe.i = 28.5 * macros.D2R        # [deg] -> [rad]
    oe.Omega = 48.2 * macros.D2R    # [deg] -> [rad]
    oe.omega = 347.8 * macros.D2R   # [deg] -> [rad]
    oe.f = 85.3 * macros.D2R        # [deg] -> [rad]
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    vHat = np.array(vN) / np.linalg.norm(vN)
    primaryOrbitPeriodSec = 2.0 * np.pi * np.sqrt(oe.a**3 / mu)  # [s]

    # Trailer initial state must be set before InitializeSimulation.
    trailer.hub.r_CN_NInit = list(np.array(rN) - SC_SEPARATION_M * vHat)
    trailer.hub.v_CN_NInit = list(vN)

    # Panel deployment
    primaryPanels = [("p", 1), ("p", 2), ("p", 3), ("n", 1), ("n", 2), ("n", 3)]
    companionPanels = [("p", 1), ("n", 1)]
    models = []
    addDeployment(primaryScene, primaryPanels, "panel", models)
    addDeployment(companionScene, companionPanels, "companion_panel", models)

    # Gravity setup.
    hubBody = primaryScene.getBody("hub")
    companionBody = companionScene.getBody("companion_hub")

    earthGravity = pointMassGravityModel.PointMassGravityModel()
    earthGravity.muBody = mu
    hubBody.addGravitySource(earthGravity)
    companionBody.addGravitySource(earthGravity)
    models.append(earthGravity)

    # CSS sensors mounted on the primary hub. The MJScene body's origin state
    # message subscribes the same way a regular Spacecraft body state message
    # would.  Sun state comes from the SPICE planet outputs.
    hubStateMsg = hubBody.getOrigin().stateOutMsg
    sunStateMsg = spiceObject.planetStateOutMsgs[2]
    cssNormals_B = [
        (1.0, 0.0, 0.0),
        (-1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, -1.0, 0.0),
    ]
    cssUnits = []
    for k, nHat in enumerate(cssNormals_B):
        css = coarseSunSensor.CoarseSunSensor()
        css.ModelTag = f"hub_CSS{k}"
        css.fov = 80.0 * macros.D2R  # [deg] -> [rad]
        css.scaleFactor = 2.0        # [-]
        css.maxOutput = 2.0          # [-]
        css.nHat_B = list(nHat)      # [-]
        css.r_B = [0.0, 0.0, 1.0]   # [m]
        css.sunInMsg.subscribeTo(sunStateMsg)
        css.stateInMsg.subscribeTo(hubStateMsg)
        scSim.AddModelToTask("simTask", css)
        cssUnits.append(css)
    models.extend(cssUnits)

    if vizSupport.vizFound:
        # Star tracker mounted on top of the companion hub, boresight +Z body.
        starTracker = vizInterface.GenericSensor()
        starTracker.r_SB_B = [0.0, 0.0, 1.0]              # [m]
        starTracker.fieldOfView.push_back(8.0 * macros.D2R)  # [deg] -> [rad]
        starTracker.fieldOfView.push_back(8.0 * macros.D2R)  # [deg] -> [rad]
        starTracker.normalVector = [0.0, 0.0, 1.0]          # [-]
        starTracker.size = 1.5                               # [m]
        starTracker.color = vizInterface.IntVector(vizSupport.toRGBA255("yellow"))
        starTracker.label = "STR"
        starTracker.genericSensorCmd = 1

        # Vizard transceiver, boresight +Z body.
        trailerAntenna = vizInterface.Transceiver()
        trailerAntenna.r_SB_B = [0.0, 0.0, 1.0]       # [m]
        trailerAntenna.fieldOfView = 30.0 * macros.D2R  # [deg] -> [rad]
        trailerAntenna.normalVector = [0.0, 0.0, 1.0]  # [-]
        trailerAntenna.color = vizInterface.IntVector(vizSupport.toRGBA255("cyan"))
        trailerAntenna.label = "TX"
        trailerAntenna.animationSpeed = 1

        viz = vizSupport.enableUnityVisualization(
            scSim,
            "simTask",
            [primaryScene, companionScene, trailer],
            # saveFile=__file__,
            cssList=[{"hub": cssUnits}, None, None],
            genericSensorList=[None, {"companion_hub": [starTracker]}, None],
            transceiverList=[None, None, [trailerAntenna]],
        )
        vizSupport.setInstrumentGuiSetting(
            viz,
            spacecraftName="hub",
            viewCSSPanel=True,
            viewCSSCoverage=True,
            viewCSSBoresight=True,
            showCSSLabels=True,
        )
        vizSupport.setInstrumentGuiSetting(
            viz,
            spacecraftName="companion_hub",
            showGenericSensorLabels=True,
        )
        vizSupport.setInstrumentGuiSetting(
            viz,
            spacecraftName="trailer",
            showTransceiverLabels=True,
            showTransceiverFrustum=True,
        )
        viz.settings.orbitLinesOn = 2

    scSim.InitializeSimulation()

    # Panels to stowed
    setStowed(primaryScene, primaryPanels, "panel")
    setStowed(companionScene, companionPanels, "companion_panel")

    # Primary hub: circular LEO
    hubBody.setPosition(rN)
    hubBody.setVelocity(vN)

    # Companion: same orbit, ahead along-track.
    companionBody.setPosition(np.array(rN) + SC_SEPARATION_M * vHat)
    companionBody.setVelocity(vN)

    scSim.ConfigureStopTime(macros.sec2nano(primaryOrbitPeriodSec)*.75)
    scSim.ExecuteSimulation()


if __name__ == "__main__":
    run()
