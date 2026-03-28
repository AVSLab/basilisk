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
Overview
--------

This scenario demonstrates sequential strip imaging of three ground strips using the
:ref:`stripLocation` and :ref:`locationPointing` modules. A spacecraft in a low-Earth
equatorial orbit images three strips one after another, each with a different orientation
relative to the orbit track:

1. **Parallel** to the orbit track (constant latitude, varying longitude)
2. **Perpendicular** to the orbit track (varying latitude, constant longitude)
3. **Diagonal** to the orbit track (both latitude and longitude vary)

The script is found in the folder ``basilisk/examples`` and executed by using::

    python3 scenarioStripImaging.py

Each strip is defined by a start and end geodetic coordinate using the :ref:`stripLocation`
module, which propagates a target point along the great-circle arc between the two endpoints
at a configurable acquisition speed. The :ref:`locationPointing` module provides three-degree-of-freedom
attitude guidance: it points the body-fixed boresight axis ``pHat_B`` toward the moving target while
aligning the scanning-line axis ``cHat_B`` orthogonal to the scanning direction.

Before each imaging phase, a pre-imaging period allows the attitude control system to achieve acceptable
attitude accuracy and/or wait for an acceptable viewing angle.

Attitude control is implemented with an MRP steering law (:ref:`mrpSteering`), a rate servo
(:ref:`rateServoFullNonlinear`), and three reaction wheels (:ref:`reactionWheelStateEffector`).

Two imaging constraints are tracked throughout the mission:

- **Minimum elevation angle** (10 deg): the view angle between the spacecraft and the target
  must exceed this threshold.
- **Maximum principal rotation error** (5 deg): the attitude error must remain below this
  threshold during imaging.

Illustration of Simulation Results
----------------------------------

::

    show_plots = True

The first plot shows the principal rotation error over the full mission. Gray-shaded regions
indicate pre-imaging periods. During imaging phases, green shading marks times
when the error is below the 5 deg threshold and red shading marks times when it is exceeded.

.. image:: /_images/Scenarios/scenarioStripImaging1.svg
   :align: center

The second plot shows the view angle (elevation) over the full mission. Green shading during
imaging phases indicates the elevation exceeds the 10 deg minimum, while red indicates it
does not.

.. image:: /_images/Scenarios/scenarioStripImaging2.svg
   :align: center

The third plot shows the norm of the perpendicular velocity component
:math:`\|\mathbf{v}_{\perp}\|`. When this value is small (hatched red region), the boresight
and target velocity are nearly collinear and the :ref:`locationPointing` module falls back to
point-only targeting.

.. image:: /_images/Scenarios/scenarioStripImaging3.svg
   :align: center

"""

# import basic libraries
import os
import time
import matplotlib.pyplot as plt
import numpy as np

# import message declarations
from Basilisk.architecture import messaging
from Basilisk.architecture import astroConstants

# import FSW Algorithm related support
from Basilisk.fswAlgorithms import locationPointing
from Basilisk.fswAlgorithms import attTrackingError
from Basilisk.fswAlgorithms import mrpSteering
from Basilisk.fswAlgorithms import rateServoFullNonlinear
from Basilisk.fswAlgorithms import rwMotorTorque

# import simulation related support
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import stripLocation
from Basilisk.simulation import simpleNav
from Basilisk.simulation import spacecraft

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import fswSetupRW
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simIncludeRW
from Basilisk.utilities import unitTestSupport

# attempt to import vizard
from Basilisk.utilities import vizSupport
try:
    from Basilisk.simulation import vizInterface
except ImportError:
    pass

from Basilisk import __path__
from Basilisk.utilities import RigidBodyKinematics

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


# ---------------------------------------------------------------------------
#  Helper functions
# ---------------------------------------------------------------------------

def get_target_position(t, r_start, r_end, speed=3e-6):
    """Interpolate a point along the great-circle arc from r_start to r_end."""
    R_E = 6378.137e3
    r_start = np.array(r_start, dtype=float).reshape(-1)
    r_end = np.array(r_end, dtype=float).reshape(-1)
    u_start = r_start / np.linalg.norm(r_start)
    u_end = r_end / np.linalg.norm(r_end)
    dot_product = np.clip(np.dot(u_start, u_end), -1.0, 1.0)
    theta = np.arccos(dot_product)
    if theta < 1e-6:
        return r_start
    arc_length = R_E * theta
    t_total = arc_length / speed
    t_frac = np.clip(t / t_total, 0.0, 1.0)
    sin_theta = np.sin(theta)
    u_interp = (np.sin((1 - t_frac) * theta) * u_start +
                np.sin(t_frac * theta) * u_end) / sin_theta
    return (R_E * u_interp).reshape(3, 1)


def compute_extended_start(r_start, r_end, preImagingTime, speed=3e-6):
    """Extend the strip start backwards along the great circle by preImagingTime to get pStartUpdated.
    """
    R_E = 6378.137e3
    pStart = np.array(r_start, dtype=float).flatten()
    pStart = pStart / np.linalg.norm(pStart) * R_E
    pEnd = np.array(r_end, dtype=float).flatten()
    pEnd = pEnd / np.linalg.norm(pEnd) * R_E

    dot_val = np.clip(np.dot(pStart, pEnd) / (R_E * R_E), -1.0, 1.0)
    theta = np.arccos(dot_val)

    if speed <= 0 or preImagingTime <= 0 or theta < 1e-12:
        return pStart

    length = theta * R_E
    line_speed_ratio = length / speed
    t = -preImagingTime / line_speed_ratio  # negative -> backwards

    sin_theta = np.sin(theta)
    coeff1 = np.sin((1 - t) * theta) / sin_theta
    coeff2 = np.sin(t * theta) / sin_theta
    interp = coeff1 * pStart + coeff2 * pEnd
    return interp / np.linalg.norm(interp) * R_E


def lat_lon_to_surface(lat_deg, lon_deg, radius):
    """Convert geodetic lat/lon (deg) to a spherical-Earth surface position."""
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    return np.array([
        radius * np.cos(lat) * np.cos(lon),
        radius * np.cos(lat) * np.sin(lon),
        radius * np.sin(lat),
    ])

def compute_strip_vertices(r_start, r_end, width=10e3):
    """Compute 4 corner vertices of a rectangular strip on the surface."""
    r_start = np.array(r_start).flatten()
    r_end = np.array(r_end).flatten()
    d_vec = r_end - r_start
    d_vec /= np.linalg.norm(d_vec)
    mid_vec = (r_start + r_end) / 2.0
    mid_vec /= np.linalg.norm(mid_vec)
    p_vec = np.cross(mid_vec, d_vec)
    p_vec /= np.linalg.norm(p_vec)
    half_width = width / 2.0
    v1 = r_start + half_width * p_vec
    v2 = r_start - half_width * p_vec
    v3 = r_end + half_width * p_vec
    v4 = r_end - half_width * p_vec
    return list(v1) + list(v3) + list(v4) + list(v2)

def compute_scan_line_extremities(r_s, r_e, width=200e3):
    """Compute left and right extremity positions of the current scan line."""
    r_s = np.array(r_s).flatten()
    r_e = np.array(r_e).flatten()
    r_mid = 0.5 * (r_s + r_e)
    d_vec = r_e - r_s
    d_norm = np.linalg.norm(d_vec)
    if d_norm > 1e-12:
        d_vec /= d_norm
    radial = r_mid / np.linalg.norm(r_mid)
    p_vec = np.cross(radial, d_vec)
    p_norm = np.linalg.norm(p_vec)
    if p_norm > 1e-12:
        p_vec /= p_norm
    half_width = width / 2.0
    r_left = r_mid + half_width * p_vec
    r_right = r_mid - half_width * p_vec
    return r_left.tolist(), r_right.tolist()

def compute_v_perp_norm(att_ref, v_LP_N, pHat_B):
    """
    Compute ||v_perp|| = ||pHat_B x v_TP_B|| at each time step.
    """
    pHat = np.asarray(pHat_B, dtype=float).ravel()
    norms = np.empty(len(att_ref))
    for i in range(len(att_ref)):
        sigma_RN_i = att_ref[i]
        v_i = v_LP_N[i]
        v_mag = np.linalg.norm(v_i)
        if v_mag < 1e-12:
            norms[i] = 0.0
            continue
        v_hat = v_i / v_mag
        dcm_RN = np.array(RigidBodyKinematics.MRP2C(sigma_RN_i))
        v_TP_B = dcm_RN @ v_hat
        v_TP_B_mag = np.linalg.norm(v_TP_B)
        if v_TP_B_mag > 1e-12:
            v_TP_B /= v_TP_B_mag
        v_perp = np.cross(pHat, v_TP_B)
        norms[i] = np.linalg.norm(v_perp)
    return norms

def _add_threshold_shading(ax, imaging_regions_min, threshold, green_below=True):
    """Add green/red threshold shading to each imaging region."""
    y_min, y_max = ax.get_ylim()
    if y_max - y_min < 1e-12:
        return
    for (t0, t1) in imaging_regions_min:
        if green_below:
            # green below threshold (good), red above (bad) – for rotation error
            if y_max <= threshold:
                ax.axvspan(t0, t1, ymin=0, ymax=1, color="green", alpha=0.14, zorder=0)
            elif y_min >= threshold:
                ax.axvspan(t0, t1, ymin=0, ymax=1, color="red", alpha=0.14, zorder=0)
            else:
                frac = (threshold - y_min) / (y_max - y_min)
                ax.axvspan(t0, t1, ymin=0, ymax=frac, color="green", alpha=0.14, zorder=0)
                ax.axvspan(t0, t1, ymin=frac, ymax=1, color="red", alpha=0.14, zorder=0)
        else:
            # green above threshold (good), red below (bad) – for view angle
            if y_min >= threshold:
                ax.axvspan(t0, t1, ymin=0, ymax=1, color="green", alpha=0.14, zorder=0)
            elif y_max <= threshold:
                ax.axvspan(t0, t1, ymin=0, ymax=1, color="red", alpha=0.14, zorder=0)
            else:
                frac = (threshold - y_min) / (y_max - y_min)
                ax.axvspan(t0, t1, ymin=0, ymax=frac, color="red", alpha=0.14, zorder=0)
                ax.axvspan(t0, t1, ymin=frac, ymax=1, color="green", alpha=0.14, zorder=0)


def _plot_segments(ax, time_min, data, seg_indices, strip_colors, strip_labels,
                   gray_indices):
    """Plot per-phase colored segments and transition segments."""
    # Transition segments – use the color of the strip they lead into
    for k, (i0, i1) in enumerate(gray_indices):
        ax.plot(time_min[i0:i1+1], data[i0:i1+1],
                color=strip_colors[k], ls="--", lw=1.0)
    # Strip segments
    for k, (i0, i1) in enumerate(seg_indices):
        ax.plot(time_min[i0:i1+1], data[i0:i1+1],
                color=strip_colors[k], label=strip_labels[k], lw=1.2)

def plot_principal_rotation_error(time_min, dataSigmaBR,
                                  gray_regions_min, imaging_regions_min,
                                  seg_indices, gray_indices,
                                  strip_colors, strip_labels,
                                  maxPrincipalRotationError=5.0):
    """Plot principal rotation error over the entire sequential mission."""
    sigma_norm = np.linalg.norm(dataSigmaBR, axis=1)
    theta_deg = np.degrees(4.0 * np.arctan(sigma_norm))

    fig, ax = plt.subplots(figsize=(8, 4))
    _plot_segments(ax, time_min, theta_deg, seg_indices, strip_colors,
                   strip_labels, gray_indices)

    ax.set_ylim(bottom=0.0)
    ax.set_xlim(time_min[0], time_min[-1])

    # Gray shading for pre-imaging / transitions
    for (t0, t1) in gray_regions_min:
        ax.axvspan(t0, t1, color="gray", alpha=0.15, zorder=0)

    # Green/red threshold shading for imaging periods
    _add_threshold_shading(ax, imaging_regions_min, threshold=maxPrincipalRotationError,
                           green_below=True)

    ax.set_xlabel("Time [min]", fontsize=12)
    ax.set_ylabel("Principal Rotation Error [deg]", fontsize=12)
    ax.grid(True)
    ax.legend(fontsize=10)
    plt.tight_layout()
    return fig


def plot_view_angle(time_min, elevation_rad,
                    gray_regions_min, imaging_regions_min,
                    seg_indices, gray_indices,
                    strip_colors, strip_labels,
                    minimumElevation_deg=10.0):
    """Plot view angle (elevation) over the entire sequential mission."""
    elevation_deg = np.degrees(elevation_rad)

    fig, ax = plt.subplots(figsize=(8, 4))
    _plot_segments(ax, time_min, elevation_deg, seg_indices, strip_colors,
                   strip_labels, gray_indices)

    ax.set_xlim(time_min[0], time_min[-1])

    for (t0, t1) in gray_regions_min:
        ax.axvspan(t0, t1, color="gray", alpha=0.15, zorder=0)

    # Green/red threshold shading for imaging periods
    _add_threshold_shading(ax, imaging_regions_min, threshold=minimumElevation_deg,
                           green_below=False)

    ax.set_xlabel("Time [min]", fontsize=12)
    ax.set_ylabel("View Angle [deg]", fontsize=12)
    ax.grid(True)
    ax.legend(fontsize=10)
    plt.tight_layout()
    return fig


def plot_v_perp_norm(time_min, v_perp_norms,
                     gray_regions_min,
                     seg_indices, gray_indices,
                     strip_colors, strip_labels,
                     threshold=0.1):
    """Plot ||v_perp|| over the entire sequential mission."""
    fig, ax = plt.subplots(figsize=(8, 4))
    _plot_segments(ax, time_min, v_perp_norms, seg_indices, strip_colors,
                   strip_labels, gray_indices)

    ax.set_xlim(time_min[0], time_min[-1])
    ax.set_ylim(bottom=0.0)

    # Hatched area below threshold (point-only targeting region)
    ax.axhspan(0, threshold, facecolor="red", alpha=0.2, hatch="//",
               edgecolor="red", linewidth=0.0, zorder=0,
               label="Point Imaging Region")

    for (t0, t1) in gray_regions_min:
        ax.axvspan(t0, t1, color="gray", alpha=0.15, zorder=0)

    ax.set_xlabel("Time [min]", fontsize=12)
    ax.set_ylabel(r"$\|\mathbf{v}_{\perp}\|$", fontsize=12)
    ax.grid(True)
    ax.legend(fontsize=10)
    plt.tight_layout()
    return fig


# ---------------------------------------------------------------------------
#  Sequential three-strip simulation
# ---------------------------------------------------------------------------

def run(show_plots=True):
    """
    Run a single continuous simulation imaging three strips sequentially:
      1. Parallel to orbit track      (constant latitude, varying longitude)
      2. Perpendicular to orbit track  (varying latitude, constant longitude)
      3. Diagonal to orbit track       (both latitude and longitude vary)
    """

    # ==================================================================
    # Timing
    # ==================================================================
    pre_imaging_s  = 120  # pre-imaging [s]
    imaging_s      = 167   # imaging time per strip (500km strip with acquisition speed of 3km/s) [s]

    pre_imaging_ns = macros.sec2nano(pre_imaging_s)
    imaging_ns     = macros.sec2nano(imaging_s)

    # Phase end-times (cumulative nanoseconds)
    phase1_img_start = pre_imaging_ns
    phase1_end       = phase1_img_start + imaging_ns

    phase2_img_start = phase1_end + pre_imaging_ns
    phase2_end       = phase2_img_start + imaging_ns

    phase3_img_start = phase2_end + pre_imaging_ns
    phase3_end       = phase3_img_start + imaging_ns

    # Strip definitions
    strip_configs = [
        {"label": "Parallel",      "start": (-5, 110),   "end": (-5, 120), "color": "blue"},
        {"label": "Perpendicular", "start": (2, 137),  "end": (12, 137), "color": "magenta"},
        {"label": "Diagonal",      "start": (-1.5, 152),  "end": (1.5, 142), "color": "orangered"},
    ]

    # ==================================================================
    # Simulation setup
    # ==================================================================
    simTaskName    = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()

    dynProcess = scSim.CreateNewProcess(simProcessName)
    simulationTimeStep = macros.sec2nano(0.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # ------------------------------------------------------------------
    # Spacecraft
    # ------------------------------------------------------------------
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    I = [100., 0., 0.,
         0., 100., 0.,
         0., 0., 100.]
    scObject.hub.mHub = 330.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    # ------------------------------------------------------------------
    # Gravity
    # ------------------------------------------------------------------
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    mu = earth.mu
    gravFactory.addBodiesTo(scObject)

    # ------------------------------------------------------------------
    # Reaction wheels
    # ------------------------------------------------------------------
    rwFactory = simIncludeRW.rwFactory()
    for axis, omega in zip([[1,0,0],[0,1,0],[0,0,1]], [500., 500., 500.]):
        rwFactory.create('Honeywell_HR16', axis,
                         maxMomentum=50., Omega=omega, u_max=0.5)
    numRW = rwFactory.getNumOfDevices()

    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)

    scSim.AddModelToTask(simTaskName, rwStateEffector, 2)
    scSim.AddModelToTask(simTaskName, scObject, 1)

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    # ------------------------------------------------------------------
    # Three strip targets (all added to the task from the start)
    # ------------------------------------------------------------------
    # Imaging constraints (used for Vizard tile colouring and plot shading)
    minimumElevation = np.radians(10.0)          # view-angle threshold [rad]
    maxPrincipalRotationError = 5.0              # attitude error threshold [deg]

    strip_target = stripLocation.StripLocation()
    strip_target.ModelTag = "Strip_Imaging_Target"
    strip_target.planetRadius = astroConstants.REQ_EARTH * 1e3

    def _set_strip_attr(obj, camel_name, snake_name, value):
        """Prefer renamed API while keeping compatibility with older Basilisk builds."""
        if hasattr(obj, camel_name):
            setattr(obj, camel_name, value)
        else:
            setattr(obj, snake_name, value)

    _set_strip_attr(strip_target, "acquisitionSpeed", "acquisition_speed", 3e-6)
    _set_strip_attr(strip_target, "preImagingTime", "pre_imaging_time", pre_imaging_ns)
    strip_target.minimumElevation = minimumElevation
    strip_target.maximumRange = 1e9

    def configure_strip_target(cfg):
        strip_target.specifyLocationStart(
            np.radians(cfg["start"][0]), np.radians(cfg["start"][1]), 0)
        strip_target.specifyLocationEnd(
            np.radians(cfg["end"][0]), np.radians(cfg["end"][1]), 0)
        strip_target.isStartPositionUpdated = False
        _set_strip_attr(strip_target, "durationStripImaging", "duration_strip_imaging", 0)

    configure_strip_target(strip_configs[0])
    strip_target.addSpacecraftToModel(scObject.scStateOutMsg)
    scSim.AddModelToTask(simTaskName, strip_target)

    # ------------------------------------------------------------------
    # Guidance – location pointing
    # ------------------------------------------------------------------
    locPoint = locationPointing.locationPointing()
    locPoint.ModelTag = "locPoint"
    locPoint.pHat_B = [0, 0, 1] # pointing vector
    locPoint.cHat_B = [0, 1, 0] # scanning line vector
    locPoint.useBoresightRateDamping = 0 # not needed 3 degrees of control for strip imaging
    scSim.AddModelToTask(simTaskName, locPoint)

    # ------------------------------------------------------------------
    # Attitude tracking error
    # ------------------------------------------------------------------
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)

    # ------------------------------------------------------------------
    # MRP steering controller
    # ------------------------------------------------------------------
    mrpControl = mrpSteering.mrpSteering()
    mrpControl.ModelTag = "MRP_Steering"
    mrpControl.K1 = 10
    mrpControl.K3 = 3
    mrpControl.ignoreOuterLoopFeedforward = False
    mrpControl.omega_max = np.radians(1.8)
    scSim.AddModelToTask(simTaskName, mrpControl)

    # ------------------------------------------------------------------
    # Rate servo
    # ------------------------------------------------------------------
    servo = rateServoFullNonlinear.rateServoFullNonlinear()
    servo.ModelTag = "rate_servo"
    servo.Ki = 5
    servo.P = 500
    servo.integralLimit = 2. / servo.Ki * 0.5
    servo.knownTorquePntB_B = [0., 0., 0.]
    scSim.AddModelToTask(simTaskName, servo)

    # ------------------------------------------------------------------
    # RW motor-torque mapping
    # ------------------------------------------------------------------
    rwMotorTorqueObj = rwMotorTorque.rwMotorTorque()
    rwMotorTorqueObj.ModelTag = "rwMotorTorque"
    rwMotorTorqueObj.controlAxes_B = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    scSim.AddModelToTask(simTaskName, rwMotorTorqueObj)

    # ------------------------------------------------------------------
    # Configuration messages
    # ------------------------------------------------------------------
    vehicleConfigOut = messaging.VehicleConfigMsgPayload()
    vehicleConfigOut.ISCPntB_B = I
    vcMsg = messaging.VehicleConfigMsg().write(vehicleConfigOut)

    fswSetupRW.clearSetup()
    for key, rw in rwFactory.rwList.items():
        fswSetupRW.create(unitTestSupport.EigenVector3d2np(rw.gsHat_B),
                          rw.Js, 0.2)
    fswRwParamMsg = fswSetupRW.writeConfigMessage()
    fswSetupRW.clearSetup()  # release SWIG-owned objects before shutdown

    # ------------------------------------------------------------------
    # Message connections (guidance starts with strip 1)
    # ------------------------------------------------------------------
    sNavObject.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    locPoint.scAttInMsg.subscribeTo(sNavObject.attOutMsg)
    locPoint.scTransInMsg.subscribeTo(sNavObject.transOutMsg)
    locPoint.locationstripInMsg.subscribeTo(
        strip_target.currentStripStateOutMsg)
    attError.attNavInMsg.subscribeTo(sNavObject.attOutMsg)
    attError.attRefInMsg.subscribeTo(locPoint.attRefOutMsg)
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    servo.guidInMsg.subscribeTo(attError.attGuidOutMsg)
    servo.vehConfigInMsg.subscribeTo(vcMsg)
    servo.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    servo.rwSpeedsInMsg.subscribeTo(rwStateEffector.rwSpeedOutMsg)
    servo.rateSteeringInMsg.subscribeTo(mrpControl.rateCmdOutMsg)
    rwMotorTorqueObj.rwParamsInMsg.subscribeTo(fswRwParamMsg)
    rwMotorTorqueObj.vehControlInMsg.subscribeTo(servo.cmdTorqueOutMsg)
    rwStateEffector.rwMotorCmdInMsg.subscribeTo(
        rwMotorTorqueObj.rwMotorTorqueOutMsg)

    # ------------------------------------------------------------------
    # Data logging
    # ------------------------------------------------------------------
    samplingTime = macros.sec2nano(3)

    attErrLog  = attError.attGuidOutMsg.recorder(samplingTime)
    attrefLog  = locPoint.attRefOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, attErrLog)
    scSim.AddModelToTask(simTaskName, attrefLog)

    # Single-strip access and target loggers (record throughout)
    accessLog = strip_target.accessOutMsgs[-1].recorder(samplingTime)
    targetLog = strip_target.currentStripStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, accessLog)
    scSim.AddModelToTask(simTaskName, targetLog)

    # ------------------------------------------------------------------
    # Initial conditions – equatorial orbit
    # ------------------------------------------------------------------
    oe = orbitalMotion.ClassicElements()
    oe.a = (6378 + 520) * 1000.0
    oe.e = 0
    oe.i = 0 * macros.D2R
    oe.Omega = 110 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 0 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN
    scObject.hub.sigma_BNInit = [[0], [-0.5], [0]]
    scObject.hub.omega_BN_BInit = [[0], [0], [0]]

    # ==================================================================
    # Vizard visualisation
    # ==================================================================
    # RGBA colours – default grey for not-yet-scanned tiles
    strip_rgba       = [0, 0, 0, 180]     # black (imaging region, not yet scanned)
    strip_rgba_faint = [0, 0, 0, 60]     # black (pre-imaging region, not yet scanned)
    scan_line_rgba   = [0, 220, 255, 255]   # cyan scanning line

    # Constraint-based tile colours (assigned at runtime)
    TILE_GREEN       = [0, 200, 0, 180]     # both constraints met – imaging
    TILE_GREEN_FAINT = [0, 200, 0, 60]      # both constraints met – pre-imaging
    TILE_RED         = [200, 0, 0, 180]     # constraint violated – imaging
    TILE_RED_FAINT   = [200, 0, 0, 60]      # constraint violated – pre-imaging

    # Phase imaging start/end pairs in nanoseconds for each strip
    strip_phase_ns = [
        (phase1_img_start, phase1_end),
        (phase2_img_start, phase2_end),
        (phase3_img_start, phase3_end),
    ]

    viz = None
    if vizSupport.vizFound:
        viz = vizSupport.enableUnityVisualization(
            scSim, simTaskName, scObject,
            # saveFile=fileName,
            oscOrbitColorList=[[255, 255, 255, 255]])
        viz.quadMaps.clear()

        # -- Draw the three full strips as quad-map tiles ----------------
        dt_viz = macros.sec2nano(5)
        for si, cfg in enumerate(strip_configs):
            id_base = (si + 1) * 100000
            r_start_orig = lat_lon_to_surface(
                cfg["start"][0], cfg["start"][1], astroConstants.REQ_EARTH * 1e3)
            r_end = lat_lon_to_surface(
                cfg["end"][0], cfg["end"][1], astroConstants.REQ_EARTH * 1e3)
            # Extend start backwards to match C++ pStartUpdated
            r_start_ext = compute_extended_start(
                r_start_orig, r_end, pre_imaging_ns, speed=3e-6)

            t_tile = 0
            tile_id = id_base
            img_start_ns, img_end_ns = strip_phase_ns[si]
            while t_tile < (img_end_ns - img_start_ns + pre_imaging_ns):
                r_s = unitTestSupport.EigenVector3d2list(
                    get_target_position(t_tile, r_start_ext, r_end, speed=3e-6))
                r_e = unitTestSupport.EigenVector3d2list(
                    get_target_position(t_tile + dt_viz, r_start_ext, r_end, speed=3e-6))
                verts = compute_strip_vertices(r_s, r_e, width=200e3)
                # Default grey – will be updated green/red during simulation
                if t_tile < pre_imaging_ns:
                    clr = strip_rgba_faint
                else:
                    clr = strip_rgba
                vizSupport.addQuadMap(viz, ID=tile_id,
                                     parentBodyName="earth",
                                     vertices=verts, color=clr)
                t_tile += dt_viz
                tile_id += 1

        # Add two locations for the left and right extremities of the scan line
        r_s0 = lat_lon_to_surface(
            strip_configs[0]["start"][0], strip_configs[0]["start"][1], astroConstants.REQ_EARTH * 1e3)
        r_e0 = lat_lon_to_surface(
            strip_configs[0]["end"][0], strip_configs[0]["end"][1], astroConstants.REQ_EARTH * 1e3)
        r_s0_ext = compute_extended_start(r_s0, r_e0, pre_imaging_ns, speed=3e-6)
        r_init = unitTestSupport.EigenVector3d2list(
            get_target_position(0, r_s0_ext, r_e0, speed=3e-6))
        r_init_next = unitTestSupport.EigenVector3d2list(
            get_target_position(dt_viz, r_s0_ext, r_e0, speed=3e-6))
        r_left_init, r_right_init = compute_scan_line_extremities(
            r_init, r_init_next, width=200e3)

        vizSupport.addLocation(viz,
            stationName="Left Extremity",
            parentBodyName="earth",
            r_GP_P=r_left_init,
            fieldOfView=np.radians(160.0),
            color=[0, 220, 255, 255],
            range=2000.0 * 1000,
            markerScale=1.0,
            isHidden=False)
        vizSupport.createTargetLine(viz,
            toBodyName="Left Extremity",
            lineColor=[0, 220, 255, 255])

        vizSupport.addLocation(viz,
            stationName="Right Extremity",
            parentBodyName="earth",
            r_GP_P=r_right_init,
            fieldOfView=np.radians(160.0),
            color=[0, 220, 255, 255],
            range=2000.0 * 1000,
            markerScale=1.0,
            isHidden=False)
        vizSupport.createTargetLine(viz,
            toBodyName="Right Extremity",
            lineColor=[0, 220, 255, 255])

        viz.settings.showLocationCommLines = -1
        viz.settings.showLocationCones = -1
        viz.settings.showLocationLabels = -1
        viz.settings.spacecraftSizeMultiplier = 2
        viz.settings.spacecraftCSon = 1
        viz.settings.planetCSon = -1
        viz.settings.celestialBodyOrbitLineWidth = 5
        viz.settings.mainCameraTarget = "spacecraftBody"         # target the satellite
        viz.settings.forceStartAtSpacecraftLocalView = 1   # start in its local view
    # ==================================================================
    # Run the simulation in three phases, switching strips between them
    # ==================================================================
    scSim.InitializeSimulation()
    t_total_start = time.perf_counter()

    # Scanning-line quad-map ID base (must not overlap with pre-created strip tile IDs)
    SCAN_LINE_ID = 900000
    # Tile step size must match the dt_viz used when pre-creating tiles
    dt_tile = macros.sec2nano(5)

    def _step_phase(phase_start_ns, phase_end_ns, strip_idx):
        """Execute a phase in 5-second steps, updating the scanning line."""
        dt_step = macros.sec2nano(5)
        t_cur = phase_start_ns
        st = strip_target
        cfg = strip_configs[strip_idx]
        r_start_orig = lat_lon_to_surface(
            cfg["start"][0], cfg["start"][1], astroConstants.REQ_EARTH * 1e3)
        r_end = lat_lon_to_surface(
            cfg["end"][0], cfg["end"][1], astroConstants.REQ_EARTH * 1e3)
        # Use the same extended start as the C++ module
        r_start_ext = compute_extended_start(
            r_start_orig, r_end, pre_imaging_ns, speed=3e-6)
        img_start_ns = strip_phase_ns[strip_idx][0]
        id_base = (strip_idx + 1) * 100000

        while t_cur < phase_end_ns:
            t_next = min(t_cur + dt_step, phase_end_ns)
            scSim.ConfigureStopTime(t_next)
            scSim.ExecuteSimulation()

            # Update scanning line if Vizard is active
            if viz is not None:
                # Time within the strip's own timeline
                elapsed = t_cur - img_start_ns + pre_imaging_ns
                r_s = unitTestSupport.EigenVector3d2list(
                    get_target_position(elapsed, r_start_ext, r_end, speed=3e-6))
                r_e = unitTestSupport.EigenVector3d2list(
                    get_target_position(elapsed + dt_step, r_start_ext, r_end, speed=3e-6))
                verts = compute_strip_vertices(r_s, r_e, width=200e3)

                # --- Colour the strip tile under the scan line -------
                # Check both imaging constraints
                sigma_BR = attError.attGuidOutMsg.read().sigma_BR
                sigma_norm = np.linalg.norm(sigma_BR)
                pre_deg = np.degrees(4.0 * np.arctan(sigma_norm))
                elev_rad = st.accessOutMsgs[-1].read().elevation
                constraints_met = (pre_deg < maxPrincipalRotationError) and (elev_rad > minimumElevation)
                is_pre_imaging = (elapsed < pre_imaging_ns)

                if constraints_met:
                    tile_clr = TILE_GREEN_FAINT if is_pre_imaging else TILE_GREEN
                else:
                    tile_clr = TILE_RED_FAINT if is_pre_imaging else TILE_RED

                # Determine which pre-created tile ID to update
                tile_index = int(elapsed // dt_tile)
                vizSupport.addQuadMap(viz, ID=id_base + tile_index,
                                     parentBodyName="earth",
                                     vertices=compute_strip_vertices(r_s, r_e, width=200e3),
                                     color=tile_clr)

                # Draw scan line last; nudge vertices ~100 m outward to prevent
                # z-fighting with the tile (both quads share the same surface plane)
                LIFT = 1.0 + 100.0 / 6.371e6
                scan_verts = [c * LIFT for c in verts]
                vizSupport.addQuadMap(viz, ID=SCAN_LINE_ID,
                                     parentBodyName="earth",
                                     vertices=scan_verts,
                                     color=scan_line_rgba)

                # Update extremity location positions
                r_left, r_right = compute_scan_line_extremities(
                    r_s, r_e, width=200e3)
                vizSupport.changeLocation(viz, stationName="Left Extremity", r_GP_P=r_left)
                vizSupport.changeLocation(viz, stationName="Right Extremity", r_GP_P=r_right)
            t_cur = t_next

    # Phase 1: pre-imaging + strip 1 imaging
    print("Phase 1: Pre-imaging + Strip 1 (Parallel) ...")
    t1_start = time.perf_counter()
    _step_phase(0, phase1_end, strip_idx=0)
    t1_elapsed = time.perf_counter() - t1_start
    print(f"  -> Phase 1 done in {t1_elapsed:.2f} s")

    # Switch guidance to strip 2 and reset its imaging clock
    print("Phase 2: Transition + Strip 2 (Perpendicular) ...")
    configure_strip_target(strip_configs[1])
    t2_start = time.perf_counter()
    _step_phase(phase1_end, phase2_end, strip_idx=1)
    t2_elapsed = time.perf_counter() - t2_start
    print(f"  -> Phase 2 done in {t2_elapsed:.2f} s")

    # Switch guidance to strip 3 and reset its imaging clock
    print("Phase 3: Transition + Strip 3 (Diagonal) ...")
    configure_strip_target(strip_configs[2])
    t3_start = time.perf_counter()
    _step_phase(phase2_end, phase3_end, strip_idx=2)
    t3_elapsed = time.perf_counter() - t3_start
    print(f"  -> Phase 3 done in {t3_elapsed:.2f} s")

    t_total = time.perf_counter() - t_total_start
    print("-" * 40)
    print(f"  Phase 1 : {t1_elapsed:6.2f} s  ({100*t1_elapsed/t_total:.1f}%)")
    print(f"  Phase 2 : {t2_elapsed:6.2f} s  ({100*t2_elapsed/t_total:.1f}%)")
    print(f"  Phase 3 : {t3_elapsed:6.2f} s  ({100*t3_elapsed/t_total:.1f}%)")
    print(f"  Total   : {t_total:6.2f} s")
    print("-" * 40)

    # ==================================================================
    # Retrieve and stitch logged data
    # ==================================================================
    times_ns    = attErrLog.times()
    time_min    = times_ns * macros.NANO2MIN
    dataSigmaBR = attErrLog.sigma_BR
    att_ref     = attrefLog.sigma_RN

    N = len(times_ns)
    elevation = np.empty(N)
    v_LP_N    = np.empty((N, 3))

    for i in range(N):
        elevation[i] = accessLog.elevation[i]
        v_LP_N[i] = targetLog.v_LP_N[i]

    v_perp_norms = compute_v_perp_norm(att_ref, v_LP_N, locPoint.pHat_B)

    # ==================================================================
    # Compute phase boundaries in minutes for plotting
    # ==================================================================
    pre_img_min      = pre_imaging_ns * macros.NANO2MIN
    p1_end_min       = phase1_end * macros.NANO2MIN
    p2_img_start_min = phase2_img_start * macros.NANO2MIN
    p2_end_min       = phase2_end * macros.NANO2MIN
    p3_img_start_min = phase3_img_start * macros.NANO2MIN
    p3_end_min       = phase3_end * macros.NANO2MIN

    # Gray (pre-imaging / transition) regions
    gray_regions_min = [
        (time_min[0], pre_img_min),          # initial pre-imaging
        (p1_end_min,  p2_img_start_min),     # transition strip 1 → 2
        (p2_end_min,  p3_img_start_min),     # transition strip 2 → 3
    ]

    # Imaging regions (receive green/red threshold shading)
    imaging_regions_min = [
        (pre_img_min,      p1_end_min),
        (p2_img_start_min, p2_end_min),
        (p3_img_start_min, p3_end_min),
    ]

    # Index ranges for colored curve segments
    def _idx(t_ns):
        return int(np.searchsorted(times_ns, t_ns, side="right")) - 1

    idx_pre_img = _idx(pre_imaging_ns)
    idx_p1_end  = _idx(phase1_end)
    idx_p2_img  = _idx(phase2_img_start)
    idx_p2_end  = _idx(phase2_end)
    idx_p3_img  = _idx(phase3_img_start)
    idx_p3_end  = len(times_ns) - 1

    # Gray segments: pre-imaging, transition 1→2, transition 2→3
    gray_indices = [
        (0, idx_pre_img),
        (idx_p1_end, idx_p2_img),
        (idx_p2_end, idx_p3_img),
    ]
    # Imaging segments (one per strip)
    seg_indices = [
        (idx_pre_img, idx_p1_end),
        (idx_p2_img,  idx_p2_end),
        (idx_p3_img,  idx_p3_end),
    ]
    strip_colors = [cfg["color"] for cfg in strip_configs]
    strip_labels = [cfg["label"] for cfg in strip_configs]

    # ==================================================================
    # Plot
    # ==================================================================
    plt.close("all")

    figureList = {}
    figureList[fileName + "1"] = plot_principal_rotation_error(
        time_min, dataSigmaBR,
        gray_regions_min, imaging_regions_min,
        seg_indices, gray_indices,
        strip_colors, strip_labels,
        maxPrincipalRotationError=maxPrincipalRotationError,
    )

    figureList[fileName + "2"] = plot_view_angle(
        time_min, elevation,
        gray_regions_min, imaging_regions_min,
        seg_indices, gray_indices,
        strip_colors, strip_labels,
        minimumElevation_deg=np.degrees(minimumElevation),
    )

    figureList[fileName + "3"] = plot_v_perp_norm(
        time_min, v_perp_norms,
        gray_regions_min,
        seg_indices, gray_indices,
        strip_colors, strip_labels,
    )

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return figureList


if __name__ == "__main__":
    run(show_plots=True)
