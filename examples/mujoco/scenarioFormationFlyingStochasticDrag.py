#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab,
#  University of Colorado at Boulder
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

from dataclasses import dataclass
from collections import namedtuple
from typing import Optional, Dict, Any
import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

from Basilisk.architecture import messaging
from Basilisk.simulation import mujoco
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import pointMassGravityModel
from Basilisk.simulation import NBodyGravity
from Basilisk.simulation import MJAerodynamicDrag
from Basilisk.simulation import MJLinearTimeInvariantSystem
from Basilisk.simulation import MJMeanRevertingNoise
from Basilisk.simulation import exponentialAtmosphere
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import MJCmdForceInertialToForceAtSite
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simSetPlanetEnvironment
from Basilisk.fswAlgorithms import orbElemOffset
from Basilisk.simulation import orbElemConvert

"""Two cannon-ball spacecraft with optional drag, feedback control, and feedforward drag cancellation.

The chief and deputy spacecraft are modeled as simple point-mass bodies with sliding joints.
Their inertial properties are set directly, with no sub-bodies or moving parts.
"""

CHIEF_DEPUTY_SCENE_XML = r"""
<mujoco>
    <worldbody>
        <body name="chief">
            <joint name="chief_x" axis="1 0 0" type="slide"/>
            <joint name="chief_y" axis="0 1 0" type="slide"/>
            <joint name="chief_z" axis="0 0 1" type="slide"/>
            <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        </body>

        <body name="deputy">
            <joint name="deputy_x" axis="1 0 0" type="slide"/>
            <joint name="deputy_y" axis="0 1 0" type="slide"/>
            <joint name="deputy_z" axis="0 0 1" type="slide"/>
            <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
        </body>
    </worldbody>
</mujoco>
"""


@dataclass
class ScenarioConfig:
    """Scenario configuration toggles for the chief-deputy simulation."""
    drag: bool = True
    control: bool = True
    feedforward: bool = True
    integralControl: bool = True
    stochastic: bool = False


@dataclass
class PlotConfig:
    """Plot configuration and output control."""
    showPlots: bool = False
    saveFolder: Optional[str] = None

    plotChiefInertial: bool = True
    plotChiefDrag: bool = True
    plotChiefDensity: bool = True
    plotChiefCd: bool = True
    plotControl: bool = True
    plotOeChief: bool = True
    plotOeDeputy: bool = True
    plotOeDiff: bool = True
    plotHillXY: bool = True


@dataclass
class SimulationModels:
    """Handles to all models and messages that must remain in scope."""
    scSim: SimulationBaseClass.SimBaseClass
    integrator: svIntegrators.StateVecIntegrator
    scene: mujoco.MJScene
    chiefBody: mujoco.MJBody
    deputyBody: mujoco.MJBody
    gravity: NBodyGravity.NBodyGravity
    atmo: exponentialAtmosphere.ExponentialAtmosphere

    # Drag models
    dragChief: Optional[MJAerodynamicDrag.AerodynamicDrag] = None
    dragDeputy: Optional[MJAerodynamicDrag.AerodynamicDrag] = None
    stochDragGeomChief: Optional[MJMeanRevertingNoise.StochasticDragCoeff] = None

    # Control models
    orbElemConvertChief: Optional[orbElemConvert.OrbElemConvert] = None
    orbElemConvertDeputy: Optional[orbElemConvert.OrbElemConvert] = None
    orbElemTarget: Optional[orbElemOffset.OrbElemOffset] = None
    oeFeedback: Optional[MJLinearTimeInvariantSystem.OrbitalElementControl] = None
    inertialToSiteForce: Optional[MJCmdForceInertialToForceAtSite.CmdForceInertialToForceAtSite] = None

    # Feedforward drag models
    orbElemConvertTarget: Optional[orbElemConvert.OrbElemConvert] = None
    feedforwardDragTarget: Optional[MJAerodynamicDrag.AerodynamicDrag] = None
    feedforwardDragDeputy: Optional[MJLinearTimeInvariantSystem.ForceAtSiteLTI] = None

    # Force actuators kept alive explicitly
    controlActuatorDeputy: Optional[mujoco.MJForceActuator] = None
    feedforwardDragActuatorTarget: Optional[mujoco.MJForceActuator] = None
    feedforwardDragActuatorDeputy: Optional[mujoco.MJForceActuator] = None

    # Messages that must not be garbage collected
    dragGeomChiefMsg: Optional[messaging.DragGeometryMsg] = None
    dragGeomDeputyMsg: Optional[messaging.DragGeometryMsg] = None
    stochDragGeomChiefMsg: Optional[messaging.DragGeometryMsg] = None
    offsetElementsMsg: Optional[messaging.ClassicElementsMsg] = None


@dataclass
class RecorderHandles:
    """Handles to all recorders used in the simulation."""
    chiefState: Any
    deputyState: Any
    dragChief: Optional[Any] = None
    densityChief: Optional[Any] = None
    cdChief: Optional[Any] = None
    oeFeedbackForce: Optional[Any] = None
    feedforwardDragTarget: Optional[Any] = None
    feedforwardDragDeputy: Optional[Any] = None


@dataclass
class InitialOrbitInfo:
    """Initial orbit geometry for the chief-deputy pair."""
    semiMajorAxis: float
    orbitPeriod: float


@dataclass
class SimulationTimeSeries:
    """All time history data required for plotting and analysis."""
    times: np.ndarray
    posChief: np.ndarray
    velChief: np.ndarray
    posDeputy: np.ndarray
    velDeputy: np.ndarray
    oeChief: np.ndarray
    oeDeputy: np.ndarray
    oeDiff: np.ndarray
    hillPosition: np.ndarray
    hillVelocity: np.ndarray


def run(scenarioConfig: ScenarioConfig = ScenarioConfig(), plotConfig: PlotConfig = PlotConfig(showPlots=False)) -> Dict[str, plt.Figure]:
    """
    Run the chief-deputy relative motion scenario with optional drag, feedback control,
    and drag feedforward cancellation.

    This function builds the simulation, sets initial conditions, runs the dynamics,
    collects time histories, generates figures, and optionally shows and/or saves them.

    Parameters
    ----------
    scenarioConfig
        High level configuration toggles for drag, control, feedforward, etc.
    plotConfig
        Plot visibility and output configuration.

    Returns
    -------
    dict
        Mapping from figure name to matplotlib Figure instance.
    """
    planet = simIncludeGravBody.BODY_DATA["earth"]

    # Proportional and integral gains on classical elements (with mean anomaly)
    kProp = np.array([1e5, 1e5, 0, 0, 10, 10])
    kInt = np.array([100, 0, 0, 0, 0, 0])

    # Controller target delta elements
    targetOeDiffPayload = messaging.ClassicElementsMsgPayload(
        a=0,
        e=0.0005,
        i=0,
        Omega=0,
        omega=0,
        f=0,
    )

    # Time step based on whether stochastic integration is used
    dt = 0.1 if scenarioConfig.stochastic else 10.0

    # Build all simulation models and their connections
    models = createSimulationModels(
        planet=planet,
        scenarioConfig=scenarioConfig,
        targetOeDiffPayload=targetOeDiffPayload,
        kProp=kProp,
        kInt=kInt,
        dt=dt,
    )

    # Attach recorders that sample the main quantities of interest
    recorders = addRecorders(models)

    # Set initial chief and deputy orbits and propagate to Cartesian states
    orbitInfo = setInitialConditions(models, planet)

    # Run the simulation for a fixed number of orbits
    runSimulation(models, orbitInfo)

    # Extract time series from recorders and postprocess into useful coordinates
    timeSeries = extractTimeSeries(planet, recorders)

    # Generate figures, optionally save to disk, and optionally show interactively
    figures = createFigures(
        scenarioConfig=scenarioConfig,
        plotConfig=plotConfig,
        models=models,
        recorders=recorders,
        timeSeries=timeSeries,
        targetOeDiffPayload=targetOeDiffPayload,
        kProp=kProp,
    )

    if plotConfig.showPlots:
        plt.show()

    return figures


def createSimulationModels(
    planet: Any,
    scenarioConfig: ScenarioConfig,
    targetOeDiffPayload: messaging.ClassicElementsMsgPayload,
    kProp: np.ndarray,
    kInt: np.ndarray,
    dt: float,
) -> SimulationModels:
    """Create and connect all simulation models, returning a container with live references."""
    # Create simulation, process, and task
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    task = scSim.CreateNewTask("test", macros.sec2nano(dt))
    process.addTask(task)

    # Create the MuJoCo scene and add it as a dynamic object
    scene = mujoco.MJScene(CHIEF_DEPUTY_SCENE_XML)
    scene.extraEoMCall = True
    scSim.AddModelToTask("test", scene, 1)

    # Select integrator
    if scenarioConfig.stochastic:
        integrator = svIntegrators.svStochIntegratorW2Ito1(scene)
    else:
        integrator = svIntegrators.svIntegratorRKF45(scene)
        integrator.relTol = 1e-10
        integrator.absTol = 1e-10
    scene.setIntegrator(integrator)

    # Get MuJoCo bodies
    chiefBody: mujoco.MJBody = scene.getBody("chief")
    deputyBody: mujoco.MJBody = scene.getBody("deputy")

    # Central body gravity
    gravity = NBodyGravity.NBodyGravity()
    gravity.ModelTag = "gravity"
    scene.AddModelToDynamicsTask(gravity)

    gravityModel = pointMassGravityModel.PointMassGravityModel()
    gravityModel.muBody = planet.mu
    gravity.addGravitySource("earth", gravityModel, True)
    gravity.addGravityTarget("chief", chiefBody)
    gravity.addGravityTarget("deputy", deputyBody)

    # Exponential atmosphere model
    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmo.ModelTag = "ExpAtmo"
    simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
    atmo.addSpacecraftToModel(chiefBody.getCenterOfMass().stateOutMsg)
    atmo.addSpacecraftToModel(deputyBody.getCenterOfMass().stateOutMsg)
    scene.AddModelToDynamicsTask(atmo)

    # Initialize container for all models/messages that must stay in scope
    models = SimulationModels(
        scSim=scSim,
        integrator=integrator,
        scene=scene,
        chiefBody=chiefBody,
        deputyBody=deputyBody,
        gravity=gravity,
        atmo=atmo,
    )

    # Optional drag on chief and deputy
    if scenarioConfig.drag:
        configureDragModels(models, scenarioConfig)

    # Optional feedback control based on orbital elements
    if scenarioConfig.control:
        configureControlModels(models, planet, targetOeDiffPayload, kProp, kInt, scenarioConfig)

    # Optional drag feedforward cancellation
    if scenarioConfig.feedforward:
        configureFeedforwardModels(models, planet)

    return models


def configureDragModels(models: SimulationModels, scenarioConfig: ScenarioConfig) -> None:
    """Attach aerodynamic drag models to chief and deputy, with optional stochastic Cd."""
    scene = models.scene
    atmo = models.atmo
    chiefBody = models.chiefBody
    deputyBody = models.deputyBody

    # Chief drag model
    dragChief = MJAerodynamicDrag.AerodynamicDrag()
    dragChief.ModelTag = "dragChief"
    scene.AddModelToDynamicsTask(dragChief)
    dragChief.applyTo(chiefBody)

    dragGeomChiefMsg = messaging.DragGeometryMsg()
    dragGeomChiefMsg.write(
        messaging.DragGeometryMsgPayload(
            projectedArea=1,
            dragCoeff=30,
            r_CP_S=[0, 0, 0],
        )
    )

    stochDragGeomChief = None
    stochDragGeomChiefMsg = None

    if scenarioConfig.stochastic:
        # Stochastic mean reverting perturbation on Cd for the chief
        stochDragGeomChief = MJMeanRevertingNoise.StochasticDragCoeff()
        stochDragGeomChief.ModelTag = "stochDragGeomChief"
        scene.AddModelToDynamicsTask(stochDragGeomChief)

        stochDragGeomChief.setStationaryStd(0.15)
        stochDragGeomChief.setTimeConstant(1800.0)

        stochDragGeomChief.dragGeomInMsg.subscribeTo(dragGeomChiefMsg)
        dragChief.dragGeometryInMsg.subscribeTo(stochDragGeomChief.dragGeomOutMsg)

        stochDragGeomChiefMsg = dragGeomChiefMsg
    else:
        dragChief.dragGeometryInMsg.subscribeTo(dragGeomChiefMsg)

    dragChief.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])

    # Deputy drag model
    dragDeputy = MJAerodynamicDrag.AerodynamicDrag()
    dragDeputy.ModelTag = "dragDeputy"
    scene.AddModelToDynamicsTask(dragDeputy)
    dragDeputy.applyTo(deputyBody)

    dragGeomDeputyMsg = messaging.DragGeometryMsg()
    dragGeomDeputyMsg.write(
        messaging.DragGeometryMsgPayload(
            projectedArea=1,
            dragCoeff=100,
            r_CP_S=[0, 0, 0],
        )
    )
    dragDeputy.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[1])
    dragDeputy.dragGeometryInMsg.subscribeTo(dragGeomDeputyMsg)

    # Store references in models to prevent garbage collection
    models.dragChief = dragChief
    models.dragDeputy = dragDeputy
    models.stochDragGeomChief = stochDragGeomChief
    models.dragGeomChiefMsg = dragGeomChiefMsg
    models.dragGeomDeputyMsg = dragGeomDeputyMsg
    models.stochDragGeomChiefMsg = stochDragGeomChiefMsg


def configureControlModels(
    models: SimulationModels,
    planet: Any,
    targetOeDiffPayload: messaging.ClassicElementsMsgPayload,
    kProp: np.ndarray,
    kInt: np.ndarray,
    scenarioConfig: ScenarioConfig,
) -> None:
    """Configure orbital element feedback control and force mapping."""
    scene = models.scene
    chiefBody = models.chiefBody
    deputyBody = models.deputyBody

    # Convert SC states to classical orbital elements
    orbElemConvertChief = orbElemConvert.OrbElemConvert()
    orbElemConvertChief.mu = planet.mu
    orbElemConvertChief.scStateInMsg.subscribeTo(chiefBody.getCenterOfMass().stateOutMsg)
    scene.AddModelToDynamicsTask(orbElemConvertChief)

    orbElemConvertDeputy = orbElemConvert.OrbElemConvert()
    orbElemConvertDeputy.mu = planet.mu
    orbElemConvertDeputy.scStateInMsg.subscribeTo(deputyBody.getCenterOfMass().stateOutMsg)
    scene.AddModelToDynamicsTask(orbElemConvertDeputy)

    # Offset elements target (chief plus target delta)
    offsetElementsMsg = messaging.ClassicElementsMsg()
    offsetElementsMsg.write(targetOeDiffPayload)

    orbElemTarget = orbElemOffset.OrbElemOffset()
    orbElemTarget.useMeanAnomalyOffset = True
    orbElemTarget.mainElementsInMsg.subscribeTo(orbElemConvertChief.elemOutMsg)
    orbElemTarget.offsetElementsInMsg.subscribeTo(offsetElementsMsg)
    scene.AddModelToDynamicsTask(orbElemTarget)

    # Orbital element feedback controller
    oeFeedback = MJLinearTimeInvariantSystem.OrbitalElementControl()
    oeFeedback.ModelTag = "oeFeedback"
    oeFeedback.mu = planet.mu
    oeFeedback.setProportionalGain(np.diag(kProp))

    if scenarioConfig.integralControl:
        oeFeedback.setIntegralGain(np.diag(kInt))

    oeFeedback.targetOEInMsg.subscribeTo(orbElemTarget.elementsOutMsg)
    oeFeedback.currentOEInMsg.subscribeTo(orbElemConvertDeputy.elemOutMsg)
    scene.AddModelToDynamicsTask(oeFeedback)

    # Map commanded inertial force to site-fixed force at deputy center of mass
    controlActuatorDeputy: mujoco.MJForceActuator = scene.addForceActuator(
        "deputyControl", deputyBody.getCenterOfMass()
    )

    inertialToSiteForce = MJCmdForceInertialToForceAtSite.CmdForceInertialToForceAtSite()
    inertialToSiteForce.ModelTag = "inertialToSiteForce"
    inertialToSiteForce.siteFrameStateInMsg.subscribeTo(deputyBody.getCenterOfMass().stateOutMsg)
    inertialToSiteForce.cmdForceInertialInMsg.subscribeTo(oeFeedback.forceOutMsg)
    controlActuatorDeputy.forceInMsg.subscribeTo(inertialToSiteForce.forceOutMsg)
    scene.AddModelToDynamicsTask(inertialToSiteForce)

    # Keep references alive
    models.orbElemConvertChief = orbElemConvertChief
    models.orbElemConvertDeputy = orbElemConvertDeputy
    models.orbElemTarget = orbElemTarget
    models.offsetElementsMsg = offsetElementsMsg
    models.oeFeedback = oeFeedback
    models.inertialToSiteForce = inertialToSiteForce
    models.controlActuatorDeputy = controlActuatorDeputy


def configureFeedforwardModels(models: SimulationModels, planet: Any) -> None:
    """Configure drag feedforward models using the deputy drag as a disturbance estimate."""
    scene = models.scene
    atmo = models.atmo
    deputyBody = models.deputyBody
    orbElemTarget = models.orbElemTarget
    dragGeomChiefMsg = models.dragGeomChiefMsg
    dragDeputy = models.dragDeputy

    if orbElemTarget is None:
        raise RuntimeError("Feedforward drag requested but control is not configured.")

    if dragDeputy is None or dragGeomChiefMsg is None:
        raise RuntimeError("Feedforward drag requested but deputy drag model is not configured.")


    # Feedforward drag on desired position, computed at that location and applied to deputy
    orbElemConvertTarget = orbElemConvert.OrbElemConvert()
    orbElemConvertTarget.mu = planet.mu
    orbElemConvertTarget.ModelTag = "orbElemConvertTarget"
    scene.AddModelToDynamicsTask(orbElemConvertTarget)

    orbElemConvertTarget.elemInMsg.subscribeTo( orbElemTarget.elementsOutMsg )

    feedforwardDragTarget = MJAerodynamicDrag.AerodynamicDrag()
    feedforwardDragTarget.ModelTag = "feedforwardDragTarget"
    scene.AddModelToDynamicsTask(feedforwardDragTarget)

    feedforwardDragTarget.dragGeometryInMsg.subscribeTo( dragGeomChiefMsg )
    feedforwardDragTarget.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])
    feedforwardDragTarget.referenceFrameStateInMsg.subscribeTo(orbElemConvertTarget.scStateOutMsg)

    feedforwardDragActuatorTarget: mujoco.MJForceActuator = scene.addForceActuator(
        "feedforwardDragActuatorTarget", deputyBody.getCenterOfMass()
    )
    feedforwardDragActuatorTarget.forceInMsg.subscribeTo(feedforwardDragTarget.forceOutMsg)

    # Map deputy drag into an equal and opposite force on deputy
    feedforwardDragDeputy = MJLinearTimeInvariantSystem.ForceAtSiteLTI()
    feedforwardDragDeputy.ModelTag = "feedforwardDragDeputy"
    scene.AddModelToDynamicsTask(feedforwardDragDeputy)

    feedforwardDragDeputy.setD(-np.eye(3))
    feedforwardDragDeputy.inMsg.subscribeTo(dragDeputy.forceOutMsg)

    feedforwardDragActuatorDeputy: mujoco.MJForceActuator = scene.addForceActuator(
        "feedforwardDragActuatorDeputy", deputyBody.getCenterOfMass()
    )
    feedforwardDragActuatorDeputy.forceInMsg.subscribeTo(feedforwardDragDeputy.outMsg)

    # Store references
    models.orbElemConvertTarget = orbElemConvertTarget
    models.feedforwardDragTarget = feedforwardDragTarget
    models.feedforwardDragDeputy = feedforwardDragDeputy
    models.feedforwardDragActuatorTarget = feedforwardDragActuatorTarget
    models.feedforwardDragActuatorDeputy = feedforwardDragActuatorDeputy


def addRecorders(models: SimulationModels) -> RecorderHandles:
    """Attach all recorders used for postprocessing and keep them in a container."""
    scSim = models.scSim
    chiefBody = models.chiefBody
    deputyBody = models.deputyBody

    recordDt = macros.min2nano(1.0)

    # Inertial states of chief and deputy
    chiefStateRecorder = chiefBody.getCenterOfMass().stateOutMsg.recorder(recordDt)
    deputyStateRecorder = deputyBody.getCenterOfMass().stateOutMsg.recorder(recordDt)

    scSim.AddModelToTask("test", chiefStateRecorder, 0)
    scSim.AddModelToTask("test", deputyStateRecorder, 0)

    dragChiefRecorder = None
    densityChiefRecorder = None
    cdChiefRecorder = None
    oeFeedbackForceRecorder = None
    feedforwardDragTargetRecorder = None
    feedforwardDragDeputyRecorder = None

    # Chief drag and density recorders
    if models.dragChief is not None:
        dragChiefRecorder = models.dragChief.forceOutMsg.recorder(recordDt)
        densityChiefRecorder = models.dragChief.atmoDensInMsg.recorder(recordDt)
        scSim.AddModelToTask("test", dragChiefRecorder, 0)
        scSim.AddModelToTask("test", densityChiefRecorder, 0)

    # Stochastic Cd recorder
    if models.stochDragGeomChief is not None:
        cdChiefRecorder = models.stochDragGeomChief.dragGeomOutMsg.recorder(recordDt)
        scSim.AddModelToTask("test", cdChiefRecorder, 0)

    # Orbital element feedback force
    if models.inertialToSiteForce is not None:
        oeFeedbackForceRecorder = models.inertialToSiteForce.forceOutMsg.recorder(recordDt)
        scSim.AddModelToTask("test", oeFeedbackForceRecorder, 0)

    # Feedforward drag recorders
    if models.feedforwardDragTarget is not None:
        feedforwardDragTargetRecorder = models.feedforwardDragTarget.forceOutMsg.recorder(recordDt)
        scSim.AddModelToTask("test", feedforwardDragTargetRecorder, 0)

    if models.feedforwardDragDeputy is not None:
        feedforwardDragDeputyRecorder = models.feedforwardDragDeputy.outMsg.recorder(recordDt)
        scSim.AddModelToTask("test", feedforwardDragDeputyRecorder, 0)

    return RecorderHandles(
        chiefState=chiefStateRecorder,
        deputyState=deputyStateRecorder,
        dragChief=dragChiefRecorder,
        densityChief=densityChiefRecorder,
        cdChief=cdChiefRecorder,
        oeFeedbackForce=oeFeedbackForceRecorder,
        feedforwardDragTarget=feedforwardDragTargetRecorder,
        feedforwardDragDeputy=feedforwardDragDeputyRecorder,
    )


def setInitialConditions(models: SimulationModels, planet: Any) -> InitialOrbitInfo:
    """Set initial orbital elements for chief and deputy and propagate to Cartesian states."""
    models.scSim.InitializeSimulation()

    chiefBody = models.chiefBody
    deputyBody = models.deputyBody

    # Chief orbit between 250 and 300 km altitude
    initialMinAlt = 250.0  # km
    initialMaxAlt = 300.0  # km
    initialRadiusPeriapsis = planet.radEquator + initialMinAlt * 1000.0
    initialRadiusApoapsis = planet.radEquator + initialMaxAlt * 1000.0
    initialSemiMajorAxis = (initialRadiusPeriapsis + initialRadiusApoapsis) / 2.0

    initialOeChief = orbitalMotion.ClassicElements()
    initialOeChief.a = initialSemiMajorAxis
    initialOeChief.e = 1.0 - initialRadiusPeriapsis / initialSemiMajorAxis
    initialOeChief.i = 33.3 * macros.D2R
    initialOeChief.Omega = 48.2 * macros.D2R
    initialOeChief.omega = 347.8 * macros.D2R
    initialOeChief.f = 85.3 * macros.D2R

    rNChief, vNChief = orbitalMotion.elem2rv(planet.mu, initialOeChief)
    chiefBody.setPosition(rNChief)
    chiefBody.setVelocity(vNChief)

    # Apply small differential elements to deputy
    initialDeltaOe = (0.0001, 0.0002, -0.0003, 0.0005, 0.0005, 0.0006)
    initialOeDeputy = orbitalMotion.ClassicElements()
    initialOeDeputy.a = initialOeChief.a * (1.0 + initialDeltaOe[0])
    initialOeDeputy.e = initialOeChief.e + initialDeltaOe[1]
    initialOeDeputy.i = initialOeChief.i + initialDeltaOe[2]
    initialOeDeputy.Omega = initialOeChief.Omega + initialDeltaOe[3]
    initialOeDeputy.omega = initialOeChief.omega + initialDeltaOe[4]

    initialMeanAnomalyChief = orbitalMotion.E2M(
        orbitalMotion.f2E(initialOeChief.f, initialOeChief.e),
        initialOeChief.e,
    )
    initialMeanAnomalyDeputy = initialMeanAnomalyChief + initialDeltaOe[5]
    initialOeDeputy.f = orbitalMotion.E2f(
        orbitalMotion.M2E(initialMeanAnomalyDeputy, initialOeDeputy.e),
        initialOeDeputy.e,
    )

    rNDeputy, vNDeputy = orbitalMotion.elem2rv(planet.mu, initialOeDeputy)
    deputyBody.setPosition(rNDeputy)
    deputyBody.setVelocity(vNDeputy)

    # Compute orbit period for the initial semi major axis
    initialOrbitPeriod = 2.0 * np.pi / np.sqrt(planet.mu / initialSemiMajorAxis ** 3)

    return InitialOrbitInfo(semiMajorAxis=initialSemiMajorAxis, orbitPeriod=initialOrbitPeriod)


def runSimulation(models: SimulationModels, orbitInfo: InitialOrbitInfo) -> None:
    """Initialize and execute the simulation for a fixed multiple of the orbit period."""
    scSim = models.scSim

    finalTime = 30.0 * orbitInfo.orbitPeriod
    scSim.ConfigureStopTime(macros.sec2nano(finalTime))
    scSim.ExecuteSimulation()


def extractTimeSeries(
    planet: Any,
    recorders: RecorderHandles,
) -> SimulationTimeSeries:
    """Convert raw recorder data into orbital elements, relative coordinates, and time arrays."""
    chiefState = recorders.chiefState
    deputyState = recorders.deputyState

    times = chiefState.times() * macros.NANO2HOUR
    posChief = chiefState.r_BN_N
    velChief = chiefState.v_BN_N
    posDeputy = deputyState.r_BN_N
    velDeputy = deputyState.v_BN_N

    oeChief = rv2elemVector(planet.mu, posChief, velChief)
    oeDeputy = rv2elemVector(planet.mu, posDeputy, velDeputy)

    # Difference in semimajor axis and eccentricity is simple subtraction
    oeDiff = np.copy(oeDeputy)
    oeDiff[:, :2] -= oeChief[:, :2]

    # Angular elements must be wrapped to avoid jumps near 360 deg
    oeDiff[:, 2:] = ((oeDeputy[:, 2:] - oeChief[:, 2:] + 180.0) % 360.0) - 180.0

    hillPosition, hillVelocity = deputyToHillFrame(posChief, velChief, posDeputy, velDeputy)

    return SimulationTimeSeries(
        times=times,
        posChief=posChief,
        velChief=velChief,
        posDeputy=posDeputy,
        velDeputy=velDeputy,
        oeChief=oeChief,
        oeDeputy=oeDeputy,
        oeDiff=oeDiff,
        hillPosition=hillPosition,
        hillVelocity=hillVelocity,
    )

COLORS = namedtuple('Hcset', 'blue yellow red black')('#004488', '#DDAA33', '#BB5566', '#000000')

def createFigures(
    scenarioConfig: ScenarioConfig,
    plotConfig: PlotConfig,
    models: SimulationModels,
    recorders: RecorderHandles,
    timeSeries: SimulationTimeSeries,
    targetOeDiffPayload: messaging.ClassicElementsMsgPayload,
    kProp: np.ndarray,
) -> Dict[str, plt.Figure]:
    """Generate all requested figures, optionally saving them as PDF files."""
    figures: Dict[str, plt.Figure] = {}

    times = timeSeries.times
    posChief = timeSeries.posChief
    oeChief = timeSeries.oeChief
    oeDeputy = timeSeries.oeDeputy
    oeDiff = timeSeries.oeDiff
    hillPosition = timeSeries.hillPosition

    # Ensure save folder exists if requested
    if plotConfig.saveFolder is not None:
        os.makedirs(plotConfig.saveFolder, exist_ok=True)

    def maybeSave(figName: str, fig: plt.Figure) -> None:
        if plotConfig.saveFolder is not None:
            filePath = os.path.join(plotConfig.saveFolder, f"{figName}.pdf")
            fig.savefig(filePath, bbox_inches="tight")

    # Chief inertial trajectory
    if plotConfig.plotChiefInertial:
        fig, ax = plt.subplots(figsize=[5, 5])
        plotGradient(ax, posChief[:, 0] / 1000.0, posChief[:, 1] / 1000.0)
        ax.set_xlabel(r"${}^\mathcal{N} {x}$ [km]")
        ax.set_ylabel(r"${}^\mathcal{N} {y}$ [km]")
        ax.axis("equal")
        figures["chiefInertial"] = fig
        maybeSave("chiefInertial", fig)

    # Chief drag
    if plotConfig.plotChiefDrag and recorders.dragChief is not None:
        fig, ax = plt.subplots(figsize=[8,4])
        chiefDragMagnitude = np.linalg.norm(recorders.dragChief.force_S, axis=1)
        ax.plot(times, chiefDragMagnitude, color=COLORS.blue)
        ax.set_ylabel("Chief Drag [m/s$^2$]")
        ax.set_xlabel("Time [hr]")
        figures["chiefDrag"] = fig
        maybeSave("chiefDrag", fig)

    # Chief density
    if plotConfig.plotChiefDensity and recorders.densityChief is not None:
        fig, ax = plt.subplots(figsize=[8,4])
        ax.plot(times, recorders.densityChief.neutralDensity, color=COLORS.blue)
        ax.set_ylabel("Chief Density [kg/m$^3$]")
        ax.set_xlabel("Time [hr]")
        figures["chiefDensity"] = fig
        maybeSave("chiefDensity", fig)

    # Chief Cd
    if plotConfig.plotChiefCd and recorders.cdChief is not None:
        fig, ax = plt.subplots(figsize=[8,4])
        ax.plot(times, recorders.cdChief.dragCoeff, color=COLORS.blue)
        ax.set_ylabel("$C_D$ [-]")
        ax.set_xlabel("Time [hr]")
        figures["chiefCd"] = fig
        maybeSave("chiefCd", fig)

    # Control forces
    if plotConfig.plotControl and scenarioConfig.control:
        fig, ax = plt.subplots(figsize=[8,4])

        totalControl = np.zeros([len(times), 3])
        plotTotalControl = True

        if recorders.feedforwardDragTarget is not None and recorders.feedforwardDragDeputy is not None:
            feedforwardForceVec = recorders.feedforwardDragTarget.force_S + recorders.feedforwardDragDeputy.force_S
            feedforwardForce = np.linalg.norm(feedforwardForceVec, axis=1)
            ax.plot(times, feedforwardForce, label="Feedforward", color=COLORS.blue)

            totalControl += feedforwardForceVec
        else:
            plotTotalControl = False

        if recorders.oeFeedbackForce is not None:
            feedforwardForceVec = recorders.oeFeedbackForce.force_S
            feedbackForce = np.linalg.norm(feedforwardForceVec, axis=1)
            ax.plot(times, feedbackForce, label="Feedback", color=COLORS.red)

            totalControl += feedforwardForceVec
        else:
            plotTotalControl = False

        if plotTotalControl:
            ax.plot(times, np.linalg.norm(totalControl, axis=1), label="Total", ls="--", color=COLORS.yellow)

        ax.legend(loc="upper right")
        ax.set_xlabel("Time [hr]")
        ax.set_ylabel("Control [m/s$^2$]")  # Deputy mass is 1, so force equals acceleration
        figures["control"] = fig
        maybeSave("control", fig)

    # Orbital element plots
    labels = ("a [m]", "e [-]", "i [deg]", r"$\Omega$ [deg]", r"$\omega$ [deg]", r"$M$ [deg]")
    targetDiffVector = [
        targetOeDiffPayload.a,
        targetOeDiffPayload.e,
        targetOeDiffPayload.i,
        targetOeDiffPayload.Omega,
        targetOeDiffPayload.omega,
        targetOeDiffPayload.f,
    ]
    indicesToPlot = list(range(6))

    def plotOeSet(name: str, oe: np.ndarray, isDiff: bool, enabled: bool) -> None:
        if not enabled:
            return

        n = len(indicesToPlot)
        layout = {1: (1, 1), 2: (1, 2), 3: (1, 3), 4: (2, 2), 5: (2, 3), 6: (2, 3)}
        nrows, ncols = layout[n]
        fig, axs = plt.subplots(nrows=nrows, ncols=ncols, sharex=True, figsize=[10, 6])
        axsArray = np.atleast_1d(axs).flatten()

        for i, idx in enumerate(indicesToPlot):
            ax = axsArray[i]
            ax.plot(times, oe[:, idx], color=COLORS.blue)
            ylabel = (r"$\Delta$" if isDiff else "") + labels[idx]
            ax.set_ylabel(ylabel)
            ax.ticklabel_format(useOffset=False, axis="y")

            # Draw controller target for states that are regulated
            if isDiff and models.oeFeedback is not None and kProp[idx] != 0:
                targetValue = targetDiffVector[idx]
                if idx > 1:
                    targetValue = np.rad2deg(targetValue)
                ax.axhline(targetValue, ls="--", color=COLORS.red)

        # Label shared x axis on bottom row
        for ax in axsArray[-ncols:]:
            ax.set_xlabel("Time [hr]")

        figures[f"oe{name.capitalize()}"] = fig
        maybeSave(f"oe{name.capitalize()}", fig)

    plotOeSet("chief", oeChief, isDiff=False, enabled=plotConfig.plotOeChief)
    plotOeSet("deputy", oeDeputy, isDiff=False, enabled=plotConfig.plotOeDeputy)
    plotOeSet("diff", oeDiff, isDiff=True, enabled=plotConfig.plotOeDiff)

    # Hill frame deputy relative motion
    if plotConfig.plotHillXY:
        fig, ax = plt.subplots(figsize=[4, 4.8])
        plotGradient(ax, hillPosition[:, 0] / 1000.0, hillPosition[:, 1] / 1000.0)
        ax.set_xlabel(r"${}^\mathcal{H} {x}$ [km]")
        ax.set_ylabel(r"${}^\mathcal{H} {y}$ [km]")
        ax.axis("equal")
        figures["hillXY"] = fig
        maybeSave("hillXY", fig)

    return figures


def rv2elemVector(mu: float, rBNN: np.ndarray, vBNN: np.ndarray) -> np.ndarray:
    """Convert arrays of Cartesian states to classical orbital elements with mean anomaly."""
    oe = np.empty((rBNN.shape[0], 6))
    for i in range(rBNN.shape[0]):
        oeI = orbitalMotion.rv2elem(mu, rBNN[i, :], vBNN[i, :])
        meanAnomaly = orbitalMotion.E2M(orbitalMotion.f2E(oeI.f, oeI.e), oeI.e)
        oe[i, :] = (
            oeI.a,
            oeI.e,
            np.rad2deg(oeI.i),
            np.rad2deg(oeI.Omega),
            np.rad2deg(oeI.omega),
            np.rad2deg(meanAnomaly),
        )
    return oe


def deputyToHillFrame(
    nRC: np.ndarray,
    nVC: np.ndarray,
    nRD: np.ndarray,
    nVD: np.ndarray,
) -> (np.ndarray, np.ndarray):
    """
    Transform deputy inertial state to Hill frame relative position and velocity.

    Parameters
    ----------
    nRC
        Chief position in inertial frame, shape (N, 3).
    nVC
        Chief velocity in inertial frame, shape (N, 3).
    nRD
        Deputy position in inertial frame, shape (N, 3).
    nVD
        Deputy velocity in inertial frame, shape (N, 3).

    Returns
    -------
    hillRho
        Deputy relative position in Hill frame, shape (N, 3).
    hillRhoPrime
        Deputy relative velocity in Hill frame, shape (N, 3).
    """
    def rowNorm(x: np.ndarray) -> np.ndarray:
        return np.linalg.norm(x, axis=1, keepdims=True)

    rHat = nRC / rowNorm(nRC)
    hVec = np.cross(nRC, nVC, axis=1)
    hHat = hVec / rowNorm(hVec)
    yHat = np.cross(hHat, rHat, axis=1)

    hillDcmN = np.stack([rHat, yHat, hHat], axis=1)

    hMagnitude = rowNorm(np.cross(nRC, nVC, axis=1))[:, 0]
    radiusMagnitude = rowNorm(nRC)[:, 0]
    trueAnomalyDot = hMagnitude / (radiusMagnitude ** 2)
    omega = np.stack(
        [np.zeros_like(trueAnomalyDot), np.zeros_like(trueAnomalyDot), trueAnomalyDot],
        axis=1,
    )

    hillRho = np.einsum("nij,nj->ni", hillDcmN, (nRD - nRC))
    hillRhoPrime = np.einsum("nij,nj->ni", hillDcmN, (nVD - nVC)) - np.cross(omega, hillRho, axis=1)

    return hillRho, hillRhoPrime


def plotGradient(
    ax: plt.Axes,
    x: np.ndarray,
    y: np.ndarray,
    *,
    cmap: str = "viridis",
    linewidth: float = 2.0,
    arrows: bool = True,
    arrowEvery: int = 40,
    showEndpoints: bool = False,
    **kwargs: Any,
) -> LineCollection:
    """
    Plot a line whose color varies along its length, optionally with arrows and endpoints.

    Parameters
    ----------
    ax
        Matplotlib axes on which to draw.
    x, y
        Coordinate arrays.
    cmap
        Name of the colormap to use.
    linewidth
        Width of the plotted line.
    arrows
        If True, draw arrows along the trajectory.
    arrowEvery
        Spacing between arrows in number of points.
    showEndpoints
        If True, mark start and end points.

    Returns
    -------
    LineCollection
        The line collection added to the axes.
    """
    x = np.asarray(x)
    y = np.asarray(y)

    points = np.column_stack([x, y]).reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    lineCollection = LineCollection(segments, cmap=cmap, linewidth=linewidth, **kwargs)
    lineCollection.set_array(np.linspace(0.0, 1.0, len(x)))
    ax.add_collection(lineCollection)

    if arrows:
        for i in range(0, len(x) - arrowEvery, arrowEvery):
            ax.annotate(
                "",
                xy=(x[i + 1], y[i + 1]),
                xytext=(x[i], y[i]),
                arrowprops=dict(
                    arrowstyle="->",
                    color="k",
                    lw=1,
                    shrinkA=0,
                    shrinkB=0,
                ),
            )

    if showEndpoints:
        ax.scatter(x[0], y[0], color="red", zorder=3)
        ax.scatter(x[-1], y[-1], color="blue", zorder=3)

    ax.autoscale()
    ax.set_aspect("equal")
    return lineCollection

def runMultipleCases():
    cases = {
        "noDragSimpleControl": ScenarioConfig(
            drag=False,
            control=True,
            feedforward=False,
            integralControl=False,
            stochastic=False,
        ),
        "simpleControl": ScenarioConfig(
            drag=True,
            control=True,
            feedforward=False,
            integralControl=False,
            stochastic=False,
        ),
        "withFeedforward": ScenarioConfig(
            drag=True,
            control=True,
            feedforward=True,
            integralControl=False,
            stochastic=False,
        ),
        "withIntegral": ScenarioConfig(
            drag=True,
            control=True,
            feedforward=False,
            integralControl=True,
            stochastic=False,
        ),
        "withFeedforwardAndIntegral": ScenarioConfig(
            drag=True,
            control=True,
            feedforward=True,
            integralControl=True,
            stochastic=False,
        ),
        "stochasticWithFeedforwardAndIntegral": ScenarioConfig(
            drag=True,
            control=True,
            feedforward=True,
            integralControl=True,
            stochastic=True,
        ),
    }
    for folder, config in cases.items():
        run(config, PlotConfig(saveFolder=f"scenarioFormationFlyingStochasticDrag/{folder}"))
        print("Finished case:", folder)
        plt.close("all")

if __name__ == "__main__":
    runMultipleCases()
    scenarioConfig = ScenarioConfig(
        drag=True,
        control=True,
        feedforward=True,
        integralControl=False,
        stochastic=False,
    )
    plotConfig = PlotConfig(
        showPlots=True,
        saveFolder=None,
    )
    # run(scenarioConfig, plotConfig)
