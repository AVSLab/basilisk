# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

.. youtube:: vUzMG_ZaVg4
   :width: 560
   :height: 315

Overview
--------

This scenario demonstrates effector branching for facet drag dynamic effectors attached to two
hinged solar-panel state effectors on an aerobraking spacecraft. See :ref:`bskPrinciples-11` for
the conceptual background on attaching dynamic effectors onto state effectors. The orbit is
strongly elliptical about Earth, with a periapsis that dips into dense atmosphere on each pass.
The configuration takes its cue from NASA's Magellan aerobraking campaign at Venus, which used
asymmetric solar-panel tilting to control drag, rather than reproducing that mission's orbit.

The hub carries six cube facets (one per face) attached directly to it. Each hinged panel
carries two facets (front and back) attached via the branched :ref:`facetDragDynamicEffector`
on the :ref:`hingedRigidBodyStateEffector`. Drag forces and torques therefore depend on each
panel's instantaneous orientation, propagating into the hub equations of motion.

The two panels are mirrored about the hub's velocity-axis plane so that drag torques on the
pair self-cancel about the hub center of mass when the vehicle is aligned with the flow, giving
a passively aerodynamically stable "shuttlecock" configuration. The scenario runs four cases and
overlays them on each plot, a 2×2 sweep over drag model and control:

1. **Branched + control**: panel drag is branched onto each hinged body (orientation-dependent)
   and an MRP-feedback velocity-pointing controller is active.
2. **Branched + free**: same drag model, no controller, which shows the passive aerodynamic
   stabilization.
3. **Legacy + control**: panel facets are baked into the hub at the undeflected (theta = 0)
   panel positions, so drag is independent of panel orientation. This is the modeling baseline
   that branched effectors are meant to improve on.
4. **Legacy + free**: same drag baseline with no controller. This is the control for the
   stabilization comparison, isolating how much of the passive pointing comes from the panels
   being free to feather rather than from the vehicle simply being uncontrolled.

The script is found in the folder ``basilisk/examples`` and executed by::

    python3 scenarioAerobrake.py

Illustration of Simulation Results
----------------------------------

The default ``run()`` invocation reproduces the configuration presented in the companion
journal article (citation pending publication): all four cases at ``dynRateSeconds = 0.2``
over two full orbits. The pytest wrapper in ``src/tests`` drives a stripped-down
configuration (``dynRateSeconds = 0.5`` over 1.2 orbits) to keep wall time low. For best
fidelity drop ``dynRateSeconds`` to 0.1 s, since the chaotic free-attitude case is the most
timestep-sensitive dynamic.

The orbit trace shows two apogees of decreasing altitude across the three atmospheric passes as
drag bleeds energy from each. The legacy cases overlap each other and lie inside the branched
curves. Comparing the two controlled cases, the legacy model overestimates aerobraking by roughly
270 km of apoapsis altitude per pass, because rigid panels keep their full drag area through the
pass whereas the branched panels feather backward and shed projected area.

.. image:: /_images/Scenarios/scenarioAerobrakeOrbitTrace.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAerobrakeAltitude.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAerobrakeApoapsisAltitude.svg
   :align: center

The panel-deflection plot shows the branched panels feathering to about 61° at each perigee,
whereas the legacy panels reach about 48° when controlled. The legacy panels still hinge, driven by
the hub's deceleration, but they feel no aero loads of their own.

The pointing error trace shows the branched free case holding velocity-pointing to within a
hundredth of a degree at every perigee, whereas the legacy free case degrades from about 1° on the
first pass to 19° on the third. Panels free to feather track the flow and supply a restoring
moment over a wider range of arrival attitudes than panels frozen at zero deflection. Between
passes both free vehicles rotate at the rate they left the previous perigee with, whereas the
velocity direction rotates at a rate that varies with true anomaly, so each pass is entered with
an accumulated pointing error. The aerodynamic torque recaptures that error when it is inside
roughly sixty degrees, and the apoapsis is chosen so a free vehicle re-enters inside that basin.

.. image:: /_images/Scenarios/scenarioAerobrakePanelDeflection.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAerobrakePointingError.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAerobrakeDensity.svg
   :align: center
"""

#
#   Basilisk Scenario Script
#
#   Purpose:            Aerobrake demo with branched facet drag on hinged solar panels.
#   Author:             Andrew Morell & Isabella Davila
#   Creation Date:      2026-05-04
#

import os

import matplotlib.pyplot as plt
import numpy as np

from Basilisk.architecture import messaging, sysModel
from Basilisk.fswAlgorithms import (attTrackingError, mrpFeedback, velocityPoint)
from Basilisk.simulation import (exponentialAtmosphere, extForceTorque,
                                 facetDragDynamicEffector, gravityEffector,
                                 hingedRigidBodyStateEffector, simpleNav,
                                 spacecraft, svIntegrators)
from Basilisk.utilities import (RigidBodyKinematics, SimulationBaseClass,
                                macros, orbitalMotion, simSetPlanetEnvironment,
                                vizSupport)

fileName = os.path.basename(os.path.splitext(__file__)[0])


# Hub geometry / inertia
hubMass = 100.0  # [kg]
hubSide = 1.0    # [m]
hubFaceArea = hubSide ** 2  # [m^2]
hubHalf = hubSide / 2.0     # [m]
hubInertia = hubMass / 6.0 * hubSide ** 2  # [kg m^2] cube about its center

# Panel geometry / inertia
panelMass = 10.0     # [kg]
panelLength = 2.0    # [m]   spans from hinge outward
panelWidth = 1.0     # [m]   along the hinge axis
panelArea = panelLength * panelWidth  # [m^2]
panelD = panelLength / 2.0            # [m] hinge-to-CoM
# plate inertia about its CoM in the S frame: length along s1, width along s2
panelI11 = panelMass * panelWidth ** 2 / 12.0  # [kg m^2]
panelI22 = panelMass * panelLength ** 2 / 12.0  # [kg m^2]
panelI33 = panelMass * (panelLength ** 2 + panelWidth ** 2) / 12.0  # [kg m^2]

panelStiffness = 5.0  # [N m / rad]
panelDamping = 8.0    # [N m s / rad], ζ ≈ 0.5 about the hinge axis

# Drag coefficients
dragCoeffPanel = 2.2  # [-]
dragCoeffHub = 2.2  # [-]

# Orbit / atmosphere
earthRadius = 6378.1363e3    # [m] used for altitude reporting
periapsisAltitude = 135e3    # [m]
# sets how far a free vehicle rotates between passes; see the module docstring
apoapsisAltitude = 11000e3   # [m]
orbitInclination = 0.0      # [rad] equatorial, orbit lies in the inertial X-Y plane

# negative starts inbound, so the panels load gradually before perigee
startTrueAnomalyDeg = -60.0  # [deg]

# Simulation cadence: paper-quality defaults (overridable from run())
defaultDynRateSeconds = 0.2  # [s] dt=0.1 reproduced within <1 km apoapsis, <0.1° panel angle
defaultNOrbits = 2

# Default case set: the full 2x2 sweep over drag model and attitude control
allCases = (
    ("branched + ctrl", True,  "branched"),
    ("branched + free", False, "branched"),
    ("legacy + ctrl",   True,  "legacy"),
    ("legacy + free",   False, "legacy"),
)

# Attitude control gains (only used when control is enabled)
attCtrlK = 5.0  # [N m] MRP feedback proportional
attCtrlP = 50.0  # [N m s] MRP feedback rate

# False leaves playback speed to Vizard's own controls
prescribePlaybackSpeed = True

class PlaybackSpeedController(sysModel.SysModel):
    """Per-tick Vizard playback-speed toggle keyed on spacecraft altitude.

    User specifies the desired playback speeds (e.g. ``fastSpeed=700`` for
    700×). Vizard's actual control is the integer ``playbackMultiplier``
    mapped as ``rate = 2^multiplier``, so the controller rounds
    ``log2(speed)`` to the nearest integer.

    Vizard has two playback *modes* with very different effective rates
    for the same multiplier label:

    * ``'realTime'``: literal ``2^N`` sim-sec per wall-sec. Used for the
      slow band so ``slowSpeed=1`` actually produces 1× wall-clock.
    * ``'frameRate'``: show 1 of every ``2^N`` data frames. Effective
      rate is ``renderer_fps × dynRateSeconds × 2^N``, typically ~12×
      higher than the label at 60 fps with ``dynRateSeconds = 0.2``.
      Used for the fast band where real-time mode gets renderer-capped.

    Modes are switched *per altitude band*: real-time below
    ``slowAltitude_m`` (true slow-motion), frame-rate above
    ``fastAltitude_m`` (max effective fast). The transition band uses
    frame-rate mode so the ramp doesn't get capped by the real-time
    renderer. The mode discontinuity sits at ``slowAltitude_m``, which
    is also the visual entry into the atmosphere. The abrupt slow-down
    there reads as "and now we're at perigee" rather than as a glitch.

    The mode is asserted every tick because Vizard's API refuses to act
    on ``playbackMultiplier = 0`` (the protobuf default reads as "no
    change"). Per ``VizardReleaseNotes.rst:54`` the documented workaround
    is to send a mode value alongside. The mode toggle is what lets
    Vizard accept multiplier=0 and snap back to 1×.
    """

    def __init__(self, viz, earthRadius, slowAltitude_m,
                 fastAltitude_m=None,
                 slowSpeed=1.0, fastSpeed=16.0,
                 slowMode='realTime', fastMode='frameRate'):
        super().__init__()
        self.ModelTag = "playbackSpeed"
        self.viz = viz
        self.scStateInMsg = messaging.SCStatesMsgReader()
        self.earthRadius = earthRadius          # [m]
        self.slowAltitude_m = slowAltitude_m    # [m]
        # None steps hard at slowAltitude_m; otherwise the multiplier ramps between the two
        self.fastAltitude_m = (fastAltitude_m if fastAltitude_m is not None
                               else slowAltitude_m)
        if self.fastAltitude_m < slowAltitude_m:
            raise ValueError(
                f"fastAltitude_m ({fastAltitude_m}) must be >= "
                f"slowAltitude_m ({slowAltitude_m})")
        self.slowMultiplier = self._speedToMultiplier(slowSpeed)
        self.fastMultiplier = self._speedToMultiplier(fastSpeed)
        self.slowModeValue = self._modeToValue(slowMode)
        self.fastModeValue = self._modeToValue(fastMode)
        # multipliers are powers of two; report when a request gets rounded
        for requested, multiplier in ((fastSpeed, self.fastMultiplier),
                                      (slowSpeed, self.slowMultiplier)):
            actual = 2.0 ** multiplier
            if abs(actual - requested) > 1e-9:
                print(f"  PlaybackSpeedController: {requested:g}x playback rounded "
                      f"to {actual:g}x")

    @staticmethod
    def _speedToMultiplier(speed):
        """Convert a speed label (e.g. 700×) to the nearest integer
        Vizard playback-multiplier exponent (``rate = 2^multiplier``)."""
        if speed <= 0:
            raise ValueError(
                f"Playback speed must be positive, got {speed}")
        return int(round(np.log2(speed)))

    @staticmethod
    def _modeToValue(mode):
        """Map ``'realTime'``/``'frameRate'`` to Vizard's
        ``playbackInRealTime`` field values (+1 / −1)."""
        if mode == 'realTime':
            return 1
        if mode == 'frameRate':
            return -1
        raise ValueError(
            f"mode must be 'realTime' or 'frameRate', got {mode!r}")

    def Reset(self, CurrentSimNanos):
        # start in the fast band; UpdateState corrects on the first tick
        self.viz.liveSettings.playbackInRealTime = self.fastModeValue
        self.viz.liveSettings.playbackMultiplier = self.fastMultiplier

    def UpdateState(self, CurrentSimNanos):
        scState = self.scStateInMsg()
        rN = np.array(scState.r_BN_N)
        altitude = np.linalg.norm(rN) - self.earthRadius
        if altitude <= self.slowAltitude_m:
            mode = self.slowModeValue
            multiplier = self.slowMultiplier
        elif altitude >= self.fastAltitude_m:
            mode = self.fastModeValue
            multiplier = self.fastMultiplier
        else:
            # frame-rate mode through the band so the ramp is not capped by the real-time renderer
            mode = self.fastModeValue
            frac = ((altitude - self.slowAltitude_m)
                    / (self.fastAltitude_m - self.slowAltitude_m))
            multiplier = int(round(self.slowMultiplier
                                   + frac * (self.fastMultiplier
                                             - self.slowMultiplier)))
        # re-assert both every tick; the mode field is the workaround for multiplier 0 reading as no change
        self.viz.liveSettings.playbackInRealTime = mode
        self.viz.liveSettings.playbackMultiplier = multiplier


def panel1DCM():
    """H frame for the +X panel.

    Panel arm extends in +X (radial out), so sHat1_B = -X (CoM-to-hinge).
    Hinge along +Z (orbit normal): sHat2_B = +Z. sHat3_B = sHat1 x sHat2 = +Y
    so the plate plane is normal to +Y at theta=0, i.e. the panel face is
    perpendicular to the velocity vector.
    """
    return np.array([[-1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0],
                     [0.0, 1.0, 0.0]])


def panel2DCM():
    """H frame for the -X panel: mirrored hinge along -Z so deflection sign matches panel1."""
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, 0.0, -1.0],
                     [0.0, 1.0, 0.0]])


def addCubeFacets(facetDrag):
    """Add six cube faces with outward normals at half-side offsets."""
    for axis in (0, 1, 2):
        for sign in (+1.0, -1.0):
            normal = np.zeros(3)
            normal[axis] = sign
            location = hubHalf * normal
            facetDrag.addFacet(hubFaceArea, dragCoeffHub, normal, location)


def addPanelFacets(facetDrag):
    """Add the two faces of a flat plate panel, both normals along the panel's s3 axis.

    BSK hingedBody convention: the plate plane is the s1-s2 plane and sHat3 is
    perpendicular to the plate, so the panel face normals are ±sHat3_S. A branched
    effector's frame origin is the hinge, and the plate's center of pressure sits at
    its centroid, ``-d`` along sHat1 from there.
    """
    centerOfPressure = np.array([-panelD, 0.0, 0.0])  # [m] in the panel S frame
    facetDrag.addFacet(panelArea, dragCoeffPanel,
                       np.array([0.0, 0.0, 1.0]), centerOfPressure)
    facetDrag.addFacet(panelArea, dragCoeffPanel,
                       np.array([0.0, 0.0, -1.0]), centerOfPressure)


def addRigidPanelFacetsToHub(facetDrag):
    """Bake panel faces into the hub at their theta=0 locations.

    Used by the legacy drag-modeling case: each panel is treated as a
    rigid extension of the hub with no orientation update, so its drag area
    contributes a fixed pair of facets at the panel CoM positions in B.
    """
    for sign in (+1.0, -1.0):
        location = np.array([sign * (hubHalf + panelD), 0.0, 0.0])
        facetDrag.addFacet(panelArea, dragCoeffPanel,
                           np.array([0.0, 1.0, 0.0]), location)
        facetDrag.addFacet(panelArea, dragCoeffPanel,
                           np.array([0.0, -1.0, 0.0]), location)


def buildPanel(modelTag, dcm_HB, r_HB_B):
    panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    panel.ModelTag = modelTag
    panel.mass = panelMass
    panel.IPntS_S = [[panelI11, 0.0, 0.0],
                     [0.0, panelI22, 0.0],
                     [0.0, 0.0, panelI33]]
    panel.d = panelD
    panel.k = panelStiffness
    panel.c = panelDamping
    panel.r_HB_B = [[r_HB_B[0]], [r_HB_B[1]], [r_HB_B[2]]]
    panel.dcm_HB = dcm_HB.tolist()
    panel.thetaInit = 0.0
    panel.thetaDotInit = 0.0
    return panel


def runOneCase(useAttitudeControl, dragModel="branched",
               dynRateSeconds=defaultDynRateSeconds, nOrbitsToRun=defaultNOrbits):
    simTaskName = "simTask"
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("dynProcess")
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, macros.sec2nano(dynRateSeconds)))

    # Hub, default RK4; RKF45 subdivides too aggressively near perigee
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "hub"
    scObject.hub.mHub = hubMass
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[hubInertia, 0.0, 0.0],
                                [0.0, hubInertia, 0.0],
                                [0.0, 0.0, hubInertia]]

    # Earth gravity
    earthGrav = gravityEffector.GravBodyData()
    earthGrav.planetName = "earth_planet_data"
    earthGrav.mu = 0.3986004415e15  # [m^3/s^2]
    earthGrav.radEquator = earthRadius
    earthGrav.isCentralBody = True
    scObject.gravField.gravBodies = spacecraft.GravBodyVector([earthGrav])

    # Orbit elements (perigee start)
    rPeri = earthRadius + periapsisAltitude
    rApo = earthRadius + apoapsisAltitude
    oe = orbitalMotion.ClassicElements()
    oe.a = 0.5 * (rPeri + rApo)  # [m]
    oe.e = (rApo - rPeri) / (rApo + rPeri)  # [-]
    oe.i = orbitInclination
    oe.Omega = 0.0
    oe.omega = 0.0
    oe.f = startTrueAnomalyDeg * np.pi / 180.0  # [rad]
    rN, vN = orbitalMotion.elem2rv(earthGrav.mu, oe)
    scObject.hub.r_CN_NInit = rN.reshape(3, 1)
    scObject.hub.v_CN_NInit = vN.reshape(3, 1)

    # initial attitude in the velocityPoint frame: radial, velocity, orbit normal
    # built from v and h, since r and v are only perpendicular at the apsides
    vHat_N = vN / np.linalg.norm(vN)
    hHat_N = np.cross(rN, vN)
    hHat_N = hHat_N / np.linalg.norm(hHat_N)
    inPlane_N = np.cross(vHat_N, hHat_N)  # radial outward at the apsides
    dcm_BN = np.vstack((inPlane_N, vHat_N, hHat_N))
    sigma_BN = RigidBodyKinematics.C2MRP(dcm_BN)
    scObject.hub.sigma_BNInit = sigma_BN.reshape(3, 1)

    # h/r^2 about the orbit normal; mean motion would be 2.5x off at perigee
    orbitRate = np.linalg.norm(np.cross(rN, vN)) / np.dot(rN, rN)  # [rad/s]
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [orbitRate]]

    # Hinged panels: mirrored on the radial (±X) faces of the hub
    panel1 = buildPanel("panel1", panel1DCM(), [hubHalf, 0.0, 0.0])
    panel2 = buildPanel("panel2", panel2DCM(), [-hubHalf, 0.0, 0.0])
    scObject.addStateEffector(panel1)
    scObject.addStateEffector(panel2)

    # Atmosphere
    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmo.ModelTag = "expAtmo"
    simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
    atmo.addSpacecraftToModel(scObject.scStateOutMsg)

    # Hub drag: always six cube facets attached to the hub
    hubDrag = facetDragDynamicEffector.FacetDragDynamicEffector()
    hubDrag.ModelTag = "hubDrag"
    addCubeFacets(hubDrag)
    if dragModel == "legacy":
        # legacy model: panel facets baked into the hub at their undeflected positions
        addRigidPanelFacetsToHub(hubDrag)
    hubDrag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])
    scObject.addDynamicEffector(hubDrag)

    panel1Drag = panel2Drag = None
    if dragModel == "branched":
        # Branched facet drag on each panel
        panel1Drag = facetDragDynamicEffector.FacetDragDynamicEffector()
        panel1Drag.ModelTag = "panel1Drag"
        addPanelFacets(panel1Drag)
        panel1Drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])
        panel1.addDynamicEffector(panel1Drag)

        panel2Drag = facetDragDynamicEffector.FacetDragDynamicEffector()
        panel2Drag.ModelTag = "panel2Drag"
        addPanelFacets(panel2Drag)
        panel2Drag.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])
        panel2.addDynamicEffector(panel2Drag)
    elif dragModel != "legacy":
        raise ValueError(f"unknown dragModel: {dragModel}")

    scSim.AddModelToTask(simTaskName, atmo)
    scSim.AddModelToTask(simTaskName, scObject)
    scSim.AddModelToTask(simTaskName, panel1)
    scSim.AddModelToTask(simTaskName, panel2)
    scSim.AddModelToTask(simTaskName, hubDrag)
    if panel1Drag is not None:
        scSim.AddModelToTask(simTaskName, panel1Drag)
        scSim.AddModelToTask(simTaskName, panel2Drag)

    # Optional velocity-pointing attitude control
    extFT = None
    if useAttitudeControl:
        sNav = simpleNav.SimpleNav()
        sNav.ModelTag = "sNav"
        sNav.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

        # Earth ephemeris at the inertial origin (velocityPoint references planet-relative velocity)
        earthEphPayload = messaging.EphemerisMsgPayload()
        earthEphMsg = messaging.EphemerisMsg().write(earthEphPayload)

        velPt = velocityPoint.velocityPoint()
        velPt.ModelTag = "velPoint"
        velPt.mu = earthGrav.mu
        velPt.transNavInMsg.subscribeTo(sNav.transOutMsg)
        velPt.celBodyInMsg.subscribeTo(earthEphMsg)

        attErr = attTrackingError.attTrackingError()
        attErr.ModelTag = "attErr"
        attErr.attRefInMsg.subscribeTo(velPt.attRefOutMsg)
        attErr.attNavInMsg.subscribeTo(sNav.attOutMsg)

        vcPayload = messaging.VehicleConfigMsgPayload()
        vcPayload.ISCPntB_B = [hubInertia, 0.0, 0.0,
                               0.0, hubInertia, 0.0,
                               0.0, 0.0, hubInertia]
        vcMsg = messaging.VehicleConfigMsg().write(vcPayload)

        ctrl = mrpFeedback.mrpFeedback()
        ctrl.ModelTag = "mrpFb"
        ctrl.K = attCtrlK
        ctrl.P = attCtrlP
        ctrl.Ki = -1.0
        ctrl.integralLimit = 2.0 / abs(ctrl.Ki) * 0.1  # [N m]
        ctrl.guidInMsg.subscribeTo(attErr.attGuidOutMsg)
        ctrl.vehConfigInMsg.subscribeTo(vcMsg)

        extFT = extForceTorque.ExtForceTorque()
        extFT.ModelTag = "ctrlTorque"
        extFT.cmdTorqueInMsg.subscribeTo(ctrl.cmdTorqueOutMsg)
        scObject.addDynamicEffector(extFT)

        scSim.AddModelToTask(simTaskName, sNav)
        scSim.AddModelToTask(simTaskName, velPt)
        scSim.AddModelToTask(simTaskName, attErr)
        scSim.AddModelToTask(simTaskName, ctrl)
        scSim.AddModelToTask(simTaskName, extFT)

    # Recorders
    samplingTime = macros.sec2nano(dynRateSeconds)
    scStateRec = scObject.scStateOutMsg.recorder(samplingTime)
    panel1Rec = panel1.hingedRigidBodyOutMsg.recorder(samplingTime)
    panel2Rec = panel2.hingedRigidBodyOutMsg.recorder(samplingTime)
    atmoRec = atmo.envOutMsgs[0].recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, scStateRec)
    scSim.AddModelToTask(simTaskName, panel1Rec)
    scSim.AddModelToTask(simTaskName, panel2Rec)
    scSim.AddModelToTask(simTaskName, atmoRec)

    # Vizard hookup (best-effort)
    if vizSupport.vizFound:
        caseSuffix = ("_" + dragModel
                      + ("_ctrl" if useAttitudeControl else "_free"))
        scBodyList = [scObject,
                      ["panel1", panel1.hingedRigidBodyConfigLogOutMsg],
                      ["panel2", panel2.hingedRigidBodyConfigLogOutMsg]]
        viz = vizSupport.enableUnityVisualization(
            scSim, simTaskName, scBodyList
            # , saveFile=fileName + caseSuffix
        )
        vizSupport.createCustomModel(viz,
                                     simBodiesToModify=[scObject.ModelTag],
                                     modelPath="CUBE",
                                     scale=[hubSide, hubSide, hubSide],
                                     color=vizSupport.toRGBA255("blue"))
        for tag in ("panel1", "panel2"):
            vizSupport.createCustomModel(viz,
                                         simBodiesToModify=[tag],
                                         modelPath="CUBE",
                                         scale=[panelLength, panelWidth, 0.05],  # [m] along ±sHat1, ±sHat2, ±sHat3
                                         color=vizSupport.toRGBA255("gold"))
        # slow playback through the atmospheric pass, fast in vacuum
        if prescribePlaybackSpeed:
            playbackCtrl = PlaybackSpeedController(
                viz,
                earthRadius=earthRadius,
                slowAltitude_m=200e3,  # [m]
                fastAltitude_m=1000e3,  # [m]
                slowSpeed=4.0,
                fastSpeed=256.0)
            playbackCtrl.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
            scSim.AddModelToTask(simTaskName, playbackCtrl, ModelPriority=10)
        # load the whole .bin up front so high multipliers are not disk bound
        viz.settings.messageBufferSize = -1

    # Run
    period = 2.0 * np.pi * np.sqrt(oe.a ** 3 / earthGrav.mu)  # [s]
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(nOrbitsToRun * period))
    print(f"running {dragModel} + {'ctrl' if useAttitudeControl else 'free'} case")
    scSim.SetProgressBar(True)
    scSim.ExecuteSimulation()

    # Extract data
    timeData = scStateRec.times() * macros.NANO2SEC
    rN_hist = scStateRec.r_BN_N
    vN_hist = scStateRec.v_BN_N
    sigmaBN_hist = scStateRec.sigma_BN

    altitude = np.linalg.norm(rN_hist, axis=1) - earthRadius
    speed = np.linalg.norm(vN_hist, axis=1)

    # Pointing error: angle between body y-axis (the velocity-pointing axis) and velocity direction
    pointingErrDeg = np.empty(len(timeData))
    for i in range(len(timeData)):
        dcm_BN_i = RigidBodyKinematics.MRP2C(sigmaBN_hist[i, :])
        by_N = dcm_BN_i.T @ np.array([0.0, 1.0, 0.0])
        vHat = vN_hist[i, :] / np.linalg.norm(vN_hist[i, :])
        cosAng = np.clip(np.dot(by_N, vHat), -1.0, 1.0)
        pointingErrDeg[i] = np.degrees(np.arccos(cosAng))

    # Per-orbit apoapsis tracking via instantaneous orbital energy
    energy = 0.5 * speed ** 2 - earthGrav.mu / np.linalg.norm(rN_hist, axis=1)  # [m^2/s^2]
    semiMajor = -earthGrav.mu / (2.0 * energy)
    # Periapsis is approximately preserved on ballistic + drag-only trajectories
    apoapsisAltHist = (2.0 * semiMajor - np.linalg.norm(rN_hist, axis=1).min()) - earthRadius

    return {
        'time': timeData,
        'rN': rN_hist,
        'vN': vN_hist,
        'altitude': altitude,
        'theta1': panel1Rec.theta,
        'theta2': panel2Rec.theta,
        'density': atmoRec.neutralDensity,
        'pointingErrDeg': pointingErrDeg,
        'semiMajor': semiMajor,
        'apoapsisAlt': apoapsisAltHist,
        'period': period,
    }


def run(show_plots,
        dynRateSeconds=defaultDynRateSeconds,
        nOrbitsToRun=defaultNOrbits,
        cases=allCases):
    """Run the requested cases and overlay them.

    Args:
        show_plots: display matplotlib windows interactively if ``True``.
        dynRateSeconds: integration step size. Default 0.2 s; lower to 0.1 s
            for highest fidelity, especially in the chaotic branched + free case.
        nOrbitsToRun: number of orbital periods to integrate. Default 2.
        cases: iterable of (label, useAttitudeControl, dragModel) tuples.
            Defaults to all four (the journal-paper configuration); the test
            wrapper passes a smaller subset to keep the suite fast.
    """
    caseStyles = (("C0", "-"), ("C1", "--"), ("C2", "-."), ("C3", ":"))
    series = []
    for (label, useAttitudeControl, dragModel), (color, ls) in zip(cases, caseStyles):
        result = runOneCase(useAttitudeControl=useAttitudeControl,
                            dragModel=dragModel,
                            dynRateSeconds=dynRateSeconds,
                            nOrbitsToRun=nOrbitsToRun)
        series.append((label, result, color, ls))

    plt.close("all")

    figureList = {}
    minDensFloor = np.finfo(float).tiny

    # Orbit trace in the X-Y inertial plane (equatorial orbit lies here)
    plt.figure()
    for label, data, color, ls in series:
        plt.plot(data['rN'][:, 0] / 1e3, data['rN'][:, 1] / 1e3,
                 color=color, linestyle=ls, label=label)
    plt.gca().add_patch(plt.Circle((0, 0), earthRadius / 1e3,
                                   color='0.7', alpha=0.4, zorder=0))
    plt.xlabel('x [km]')
    plt.ylabel('y [km]')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.legend()
    figureList[fileName + "OrbitTrace"] = plt.gcf()

    # Altitude vs time
    plt.figure()
    for label, data, color, ls in series:
        plt.plot(data['time'] / 60.0, data['altitude'] / 1e3,
                 color=color, linestyle=ls, label=label)
    plt.xlabel('time [min]')
    plt.ylabel('altitude [km]')
    plt.legend()
    figureList[fileName + "Altitude"] = plt.gcf()

    # Apoapsis altitude (the headline aerobraking effect)
    plt.figure()
    for label, data, color, ls in series:
        plt.plot(data['time'] / 60.0, data['apoapsisAlt'] / 1e3,
                 color=color, linestyle=ls, label=label)
    plt.xlabel('time [min]')
    plt.ylabel('apoapsis altitude [km]')
    plt.legend()
    figureList[fileName + "ApoapsisAltitude"] = plt.gcf()

    # Panel deflection: only theta1 is plotted per case (panels are mirrored)
    plt.figure()
    for label, data, color, ls in series:
        plt.plot(data['time'] / 60.0, np.degrees(data['theta1']),
                 color=color, linestyle=ls, label=label)
    plt.xlabel('time [min]')
    plt.ylabel(r'panel 1 deflection $\theta_1$ [deg]')
    plt.legend()
    figureList[fileName + "PanelDeflection"] = plt.gcf()

    # Pointing error (body y-axis vs velocity vector)
    plt.figure()
    for label, data, color, ls in series:
        plt.plot(data['time'] / 60.0, data['pointingErrDeg'],
                 color=color, linestyle=ls, label=label)
    plt.xlabel('time [min]')
    # trailing spaces shift the centered label down so the tight bounding box keeps its top
    plt.ylabel(r'Velocity pointing error $\angle(\hat{b}_2, \hat{v})$ [deg]    ')
    plt.legend()
    figureList[fileName + "PointingError"] = plt.gcf()

    # Atmospheric density
    plt.figure()
    for label, data, color, ls in series:
        plt.semilogy(data['time'] / 60.0,
                     np.maximum(data['density'], minDensFloor),
                     color=color, linestyle=ls, label=label)
    plt.xlabel('time [min]')
    plt.ylabel(r'atmospheric density [kg/m$^3$]')
    plt.legend()
    figureList[fileName + "Density"] = plt.gcf()

    if show_plots:
        plt.show()

    plt.close("all")
    return figureList


if __name__ == "__main__":
    run(True)
