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

r"""
Accuracy-versus-runtime Pareto study on the hub-with-reaction-wheels-and-panels system
(see :ref:`scenarioCompareRwPanels` for the model, and :ref:`scenarioCompareOrbit` for
the introduction to the two dynamics engines).

The earlier comparison scenarios hold the integrator and time step fixed and ask whether
the two engines agree. This scenario asks, for each engine: *what is the cheapest way to
reach a given accuracy?* This is the classical work-precision (Pareto) diagram from the
numerical-integration literature.

We sweep a family of integrators and accuracy controls:

* Fixed-step integrators (``Euler``, RK2, RK4) over a range of time
  steps.
* Adaptive Runge-Kutta-Fehlberg integrators (RKF45, RKF78) over a range of error
  tolerances.

For each configuration and each engine the study measures wall-clock time and final
attitude error: the principal rotation angle of the relative direction cosine matrix (as
in :ref:`scenarioCompareTorque`) with respect to a *reference* solution computed for the
same engine at a tight tolerance. A per-engine reference isolates the integrator's
work-precision behavior from the formulation difference between the engines, which the
other scenarios already characterize.

Plotting error against wall-clock time produces a Pareto front: the lower-left envelope
of points is the set of non-dominated (most efficient) configurations. Higher-order
integrators occupy the high-accuracy end, while low-order or adaptive integrators with
loose tolerances occupy the cheap, low-accuracy end. This system is slow: its fastest
mode is resolved at every step size swept, no stability wall is hit, and the front is
wide and well populated. Its companion
:ref:`scenarioCompareParetoFlexPanels` runs the same sweep on a stability-limited
high-frequency array where that wall dominates.

.. note::

    The :ref:`MJScene<MJScene>` runs enable ``highOrderAttitudeIntegration`` so the
    free-joint attitude quaternion is advanced at the integrator's full order. MuJoCo's
    default attitude step (a single exponential map of the stage-averaged body rate) is
    only second-order accurate on SO(3); that error sits outside the velocity ODE the
    adaptive controller sees, so tightening the tolerance cannot reduce it. High-order
    integration removes that floor: the macro step is no longer an accuracy knob, so
    every adaptive configuration holds it fixed and varies only the tolerance, which
    drives the error down to machine precision.

The script is found in the folder ``basilisk/examples/dynamicsComparison`` and executed
by using::

    python3 scenarioCompareParetoRwPanels.py

Illustration of Simulation Results
----------------------------------

Each engine's work-precision front spans many orders of magnitude in accuracy. The
lower-left envelope is the Pareto-optimal frontier; points above and to the right of it
are dominated.

.. image:: /_images/Scenarios/scenarioCompareParetoRwPanels_pareto.svg
   :align: center

Comparing the non-dominated frontier of each engine shows which one reaches a target
accuracy for the least wall-clock cost.

.. image:: /_images/Scenarios/scenarioCompareParetoRwPanels_frontier.svg
   :align: center

"""

import os
import time

import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeRW
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.simulation import spacecraft
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import hingedRigidBodyStateEffector
from Basilisk.simulation import svIntegrators
from Basilisk.architecture import messaging

try:
    from Basilisk.simulation import mujoco
    couldImportMujoco = True
except Exception:
    couldImportMujoco = False

fileName = os.path.basename(os.path.splitext(__file__)[0])

# Folder this scenario writes its JSON summary into.
resultsPath = os.path.join(os.path.dirname(__file__), "results")

# ---------------------------------------------------------------------------
# Model parameters (same medium-class free-flyer as scenarioCompareRwPanels)
# ---------------------------------------------------------------------------
HUB_MASS = 600.0  # [kg]
HUB_CORE_INERTIA = (400.0, 380.0, 360.0)  # [kg*m^2] bare hub, before wheel inertia folds in
RW_JS = 0.08  # [kg*m^2] wheel axial inertia
RW_JT = 0.04  # [kg*m^2] wheel transverse inertia
RW_MASS = 1.0e-6  # [kg] negligible, matching the massless balanced-wheel idealization
RW_TORQUE = (0.010, -0.015, 0.008)  # [N*m] constant motor torque per wheel
PANEL_MASS = 50.0  # [kg]
PANEL_INERTIA = (30.0, 20.0, 12.0)  # [kg*m^2] about the panel center of mass
PANEL_D = 1.2  # [m] hinge-axis to panel center-of-mass distance
PANEL_K = 150.0  # [N*m/rad] torsional spring stiffness
PANEL_C = 8.0  # [N*m*s/rad] torsional damping
PANEL_HINGE_X = 0.8  # [m] hinge offset from the hub origin along x
PANEL_THETA0 = 12.0*macros.D2R  # [rad] initial panel deflection
OMEGA0_B = (0.010, -0.020, 0.015)  # [rad/s] initial hub rate

SIM_DURATION = 30.0  # [s] propagation horizon for every configuration
TIMING_TRIALS = 5  # number of trials for median/min/std reporting
TIMING_WARMUP_SEC = 1.0  # [s] discarded warmup sim time absorbing one-time process costs

# Macro (task) time step shared by every adaptive-integrator configuration. With
# high-order attitude integration the tolerance, not the macro step, controls the error:
# the adaptive ladders hold this step fixed and vary only the tolerance.
ADAPTIVE_MACRO_DT = 0.5  # [s]

# Reference (truth) configuration: a high-order adaptive integrator at the rounding-limit
# tolerance. With full-order attitude integration the tolerance alone drives the error to
# machine precision, so the reference does not need a small task step.
REFERENCE = {"integrator": "svIntegratorRKF78", "dt": ADAPTIVE_MACRO_DT, "tol": 1.0e-13}

# Estimate of the reference's own error: the same integrator and macro step with the
# tolerance relaxed one notch. The disagreement ``||y(reference) - y(refined)||`` bounds
# the reference error and sets the accuracy floor of the study; reported errors at or
# below it only show agreement with the reference to within its own uncertainty.
REFERENCE_CHECK = {"integrator": "svIntegratorRKF78", "dt": ADAPTIVE_MACRO_DT, "tol": 1.0e-12}

# Full Pareto sweep. Each entry is (integrator, dt [s], tolerance or None). Fixed-step
# integrators (Euler/RK2/RK4) vary the macro time step, their only accuracy knob.
# Adaptive integrators (RKF45/RKF78) hold the macro step at ``ADAPTIVE_MACRO_DT`` and
# vary only the tolerance, covering ~1e-4 rad down to machine precision. This is the
# default sweep and what ``runAllComparisons.py`` uses for the published figures.
FULL_SWEEP_CONFIGS = [
    ("svIntegratorEuler", 0.1, None),
    ("svIntegratorEuler", 0.05, None),
    ("svIntegratorEuler", 0.02, None),
    ("svIntegratorEuler", 0.01, None),
    ("svIntegratorEuler", 0.005, None),
    ("svIntegratorEuler", 0.002, None),
    ("svIntegratorRK2", 0.2, None),
    ("svIntegratorRK2", 0.1, None),
    ("svIntegratorRK2", 0.05, None),
    ("svIntegratorRK2", 0.02, None),
    ("svIntegratorRK2", 0.01, None),
    ("svIntegratorRK2", 0.005, None),
    ("svIntegratorRK4", 0.4, None),
    ("svIntegratorRK4", 0.2, None),
    ("svIntegratorRK4", 0.1, None),
    ("svIntegratorRK4", 0.05, None),
    ("svIntegratorRK4", 0.02, None),
    ("svIntegratorRK4", 0.01, None),
    # Adaptive ladders: fixed macro step, tolerance swept over many decades.
    ("svIntegratorRKF45", ADAPTIVE_MACRO_DT, 1.0e-4),
    ("svIntegratorRKF45", ADAPTIVE_MACRO_DT, 3.0e-5),
    ("svIntegratorRKF45", ADAPTIVE_MACRO_DT, 1.0e-5),
    ("svIntegratorRKF45", ADAPTIVE_MACRO_DT, 3.0e-6),
    ("svIntegratorRKF45", ADAPTIVE_MACRO_DT, 1.0e-6),
    ("svIntegratorRKF45", ADAPTIVE_MACRO_DT, 1.0e-7),
    ("svIntegratorRKF45", ADAPTIVE_MACRO_DT, 1.0e-8),
    ("svIntegratorRKF45", ADAPTIVE_MACRO_DT, 1.0e-9),
    ("svIntegratorRKF45", ADAPTIVE_MACRO_DT, 1.0e-10),
    ("svIntegratorRKF45", ADAPTIVE_MACRO_DT, 1.0e-11),
    ("svIntegratorRKF45", ADAPTIVE_MACRO_DT, 1.0e-12),
    ("svIntegratorRKF78", ADAPTIVE_MACRO_DT, 1.0e-6),
    ("svIntegratorRKF78", ADAPTIVE_MACRO_DT, 1.0e-7),
    ("svIntegratorRKF78", ADAPTIVE_MACRO_DT, 1.0e-8),
    ("svIntegratorRKF78", ADAPTIVE_MACRO_DT, 1.0e-9),
    ("svIntegratorRKF78", ADAPTIVE_MACRO_DT, 1.0e-10),
    ("svIntegratorRKF78", ADAPTIVE_MACRO_DT, 1.0e-11),
    ("svIntegratorRKF78", ADAPTIVE_MACRO_DT, 1.0e-12),
    ("svIntegratorRKF78", ADAPTIVE_MACRO_DT, 1.0e-13),
]

# Default sweep used by ``run()``; the unit test overrides it with a reduced subset.
SWEEP_CONFIGS = FULL_SWEEP_CONFIGS

# Plot style per integrator family (color from the standard palette, distinct marker).
INTEGRATOR_STYLE = {
    "svIntegratorEuler": (unitTestSupport.getLineColor(0, 5), "o", "Euler"),
    "svIntegratorRK2": (unitTestSupport.getLineColor(1, 5), "s", "RK2"),
    "svIntegratorRK4": (unitTestSupport.getLineColor(2, 5), "^", "RK4"),
    "svIntegratorRKF45": (unitTestSupport.getLineColor(3, 5), "D", "RKF45 (adaptive)"),
    "svIntegratorRKF78": (unitTestSupport.getLineColor(4, 5), "v", "RKF78 (adaptive)"),
}


def mujocoModel():
    """Return the MJCF model: hub, three reaction wheels, two hinged panels."""
    ix, iy, iz = HUB_CORE_INERTIA
    ip = PANEL_INERTIA
    return f"""
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" mass="{HUB_MASS}" fullinertia="{ix} {iy} {iz} 0 0 0"/>

      <body name="rw_x" pos="0 0 0">
        <joint name="rw_x" type="hinge" axis="1 0 0"/>
        <inertial pos="0 0 0" mass="{RW_MASS}" fullinertia="{RW_JS} {RW_JT} {RW_JT} 0 0 0"/>
      </body>
      <body name="rw_y" pos="0 0 0">
        <joint name="rw_y" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 0" mass="{RW_MASS}" fullinertia="{RW_JT} {RW_JS} {RW_JT} 0 0 0"/>
      </body>
      <body name="rw_z" pos="0 0 0">
        <joint name="rw_z" type="hinge" axis="0 0 1"/>
        <inertial pos="0 0 0" mass="{RW_MASS}" fullinertia="{RW_JT} {RW_JT} {RW_JS} 0 0 0"/>
      </body>

      <body name="panelP" pos="{PANEL_HINGE_X} 0 0">
        <joint name="panelP" type="hinge" axis="0 1 0" stiffness="{PANEL_K}" damping="{PANEL_C}" springref="0"/>
        <inertial pos="{-PANEL_D} 0 0" mass="{PANEL_MASS}" fullinertia="{ip[0]} {ip[1]} {ip[2]} 0 0 0"/>
      </body>
      <body name="panelM" pos="{-PANEL_HINGE_X} 0 0">
        <joint name="panelM" type="hinge" axis="0 1 0" stiffness="{PANEL_K}" damping="{PANEL_C}" springref="0"/>
        <inertial pos="{-PANEL_D} 0 0" mass="{PANEL_MASS}" fullinertia="{ip[0]} {ip[1]} {ip[2]} 0 0 0"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="rw_x" joint="rw_x"/>
    <motor name="rw_y" joint="rw_y"/>
    <motor name="rw_z" joint="rw_z"/>
  </actuator>
</mujoco>
"""


def hubInertiaBSM():
    """BSM hub inertia: bare hub plus the three folded-in wheel tensors."""
    inertia = np.diag(HUB_CORE_INERTIA).astype(float)
    for axis in range(3):
        diag = [RW_JT, RW_JT, RW_JT]
        diag[axis] = RW_JS
        inertia = inertia + np.diag(diag)
    return inertia


def makeIntegrator(dynObject, integratorName, tol):
    """Create and attach an integrator, applying the tolerance if adaptive.

    Args:
        dynObject: the ``Spacecraft`` or ``MJScene`` to integrate.
        integratorName (str): attribute name in ``svIntegrators``.
        tol (float): relative and absolute tolerance for adaptive integrators, or None.

    Returns:
        the integrator instance (which the caller must keep referenced).
    """
    integrator = getattr(svIntegrators, integratorName)(dynObject)
    if tol is not None:
        integrator.relTol = tol
        integrator.absTol = tol
    dynObject.setIntegrator(integrator)
    return integrator


def buildBSM(integratorName, dt, tol):
    """Build the back-substitution (BSM) hub-wheel-panel simulation with the given integrator.

    Args:
        integratorName (str): integrator attribute in ``svIntegrators``.
        dt (float): task (macro) time step [s].
        tol (float): adaptive tolerance, or None for fixed-step integrators.

    Returns:
        tuple: ``(scSim, recorder, handles)``.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "hub"
    scObject.hub.mHub = HUB_MASS  # [kg]
    scObject.hub.IHubPntBc_B = hubInertiaBSM().tolist()  # [kg*m^2]
    scObject.hub.omega_BN_BInit = [[w] for w in OMEGA0_B]  # [rad/s]
    scSim.AddModelToTask("dynTask", scObject)

    integrator = makeIntegrator(scObject, integratorName, tol)

    rwEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwEffector.ModelTag = "rw"
    rwFactory = simIncludeRW.rwFactory()
    for spinAxis in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):
        rwFactory.create("custom", spinAxis, Js=RW_JS, Omega=0.0, rWB_B=[0., 0., 0.],
                         useMaxTorque=False, useMinTorque=False)
    rwFactory.addToSpacecraft("rw", rwEffector, scObject)
    scSim.AddModelToTask("dynTask", rwEffector)
    rwCmdMsg = messaging.ArrayMotorTorqueMsg()
    rwCmdMsg.write(messaging.ArrayMotorTorqueMsgPayload(
        motorTorque=list(RW_TORQUE) + [0.0]*33))
    rwEffector.rwMotorCmdInMsg.subscribeTo(rwCmdMsg)

    panels = []
    for hingeX in (PANEL_HINGE_X, -PANEL_HINGE_X):
        panel = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
        panel.mass = PANEL_MASS  # [kg]
        panel.IPntS_S = np.diag(PANEL_INERTIA).tolist()  # [kg*m^2]
        panel.d = PANEL_D  # [m]
        panel.k = PANEL_K  # [N*m/rad]
        panel.c = PANEL_C  # [N*m*s/rad]
        panel.r_HB_B = [[hingeX], [0.0], [0.0]]  # [m]
        panel.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        panel.thetaInit = PANEL_THETA0  # [rad]
        panel.thetaDotInit = 0.0  # [rad/s]
        panel.ModelTag = "panel" + ("P" if hingeX > 0 else "M")
        scObject.addStateEffector(panel)
        scSim.AddModelToTask("dynTask", panel)
        panels.append(panel)

    recorder = scObject.scStateOutMsg.recorder(macros.sec2nano(SIM_DURATION))
    scSim.AddModelToTask("dynTask", recorder)

    scSim.InitializeSimulation()
    handles = [scObject, integrator, rwEffector, rwFactory, rwCmdMsg] + panels
    return scSim, recorder, handles


def buildMujoco(integratorName, dt, tol):
    """Build the MuJoCo hub-wheel-panel simulation with the given integrator.

    Args:
        integratorName (str): integrator attribute in ``svIntegrators``.
        dt (float): task (macro) time step [s].
        tol (float): adaptive tolerance, or None for fixed-step integrators.

    Returns:
        tuple: ``(scSim, recorder, handles)``.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scene = mujoco.MJScene(mujocoModel())
    scene.ModelTag = "hubMj"
    scene.extraEoMCall = True
    # Full-order attitude integration: the quaternion error converges at the RK method's
    # order and is controlled by the adaptive tolerance, not the task (macro) time step.
    scene.highOrderAttitudeIntegration = True
    scSim.AddModelToTask("dynTask", scene, 1)

    integrator = makeIntegrator(scene, integratorName, tol)

    hub = scene.getBody("hub")
    actuatorMsgs = []
    for i, name in enumerate(["rw_x", "rw_y", "rw_z"]):
        actuator = scene.getSingleActuator(name)
        cmdMsg = messaging.SingleActuatorMsg()
        cmdMsg.write(messaging.SingleActuatorMsgPayload(input=RW_TORQUE[i]))
        actuator.actuatorInMsg.subscribeTo(cmdMsg)
        actuatorMsgs.append(cmdMsg)

    recorder = hub.getOrigin().stateOutMsg.recorder(macros.sec2nano(SIM_DURATION))
    scSim.AddModelToTask("dynTask", recorder, 0)

    scSim.InitializeSimulation()
    hub.setAttitudeRate(list(OMEGA0_B))
    for name in ["panelP", "panelM"]:
        scene.getBody(name).getScalarJoint(name).setPosition(PANEL_THETA0)
    return scSim, recorder, [scene, integrator, hub] + actuatorMsgs


def finalAttitude(builder, integratorName, dt, tol):
    """Propagate one configuration and return its final attitude MRP.

    Args:
        builder (callable): ``buildBSM`` or ``buildMujoco``.
        integratorName (str): integrator attribute name.
        dt (float): task time step [s].
        tol (float): adaptive tolerance, or None.

    Returns:
        numpy.ndarray: final ``sigma_BN`` (shape ``(3,)``), or None if the run diverged.
    """
    scSim, recorder, _ = builder(integratorName, dt, tol)
    scSim.ConfigureStopTime(macros.sec2nano(SIM_DURATION))
    scSim.ExecuteSimulation()
    sigma = np.array(recorder.sigma_BN)[-1]
    if not np.all(np.isfinite(sigma)):
        return None
    return sigma


def timedRun(builder, integratorName, dt, tol):
    """Wall-clock timing for one configuration: median/min/std over the trials.

    Args:
        builder (callable): ``buildBSM`` or ``buildMujoco``.
        integratorName (str): integrator attribute name.
        dt (float): task time step [s].
        tol (float): adaptive tolerance, or None.

    Returns:
        dict: ``{"median", "min", "std"}`` wall-clock over the trials [s].
    """
    stopNs = macros.sec2nano(SIM_DURATION)
    # Warmup (discarded): absorbs one-time process costs (cold code/data caches, first
    # allocations, bytecode) so the measured trials reflect steady-state cost.
    if TIMING_WARMUP_SEC > 0:
        warm, _, _ = builder(integratorName, dt, tol)
        warm.ConfigureStopTime(macros.sec2nano(min(SIM_DURATION, TIMING_WARMUP_SEC)))
        warm.ExecuteSimulation()
    times = []
    for _ in range(TIMING_TRIALS):
        scSim, _, _ = builder(integratorName, dt, tol)
        scSim.ConfigureStopTime(stopNs)  # set the stop time outside the timed region
        start = time.perf_counter()
        scSim.ExecuteSimulation()
        times.append(time.perf_counter() - start)
    return {"median": float(np.median(times)), "min": float(np.min(times)),
            "std": float(np.std(times))}


def principalAngle(sigmaA, sigmaB):
    """Principal rotation angle between two attitudes given as MRPs [rad].

    The angle is computed from the relative MRP as ``4*atan(|sigma_rel|)``, which stays
    well conditioned down to machine precision, unlike ``arccos((trace(C)-1)/2)``, whose
    argument approaches 1 for small angles and collapses to zero below roughly 1e-8 rad.
    """
    dcmRel = rbk.MRP2C(sigmaA).dot(rbk.MRP2C(sigmaB).T)
    sigmaRel = rbk.C2MRP(dcmRel)
    return float(4.0*np.arctan(np.linalg.norm(sigmaRel)))


def sweepEngine(builder, reference=REFERENCE, referenceCheck=REFERENCE_CHECK):
    """Run the reference and every sweep configuration for one engine.

    Args:
        builder (callable): ``buildBSM`` or ``buildMujoco``.
        reference (dict): reference (truth) configuration with keys ``integrator``,
            ``dt`` and ``tol``.
        referenceCheck (dict): refined configuration used to estimate the reference's
            accuracy floor.

    Returns:
        tuple: ``(rows, referenceFloor)`` where ``rows`` holds one dict per successful
        configuration (integrator, error, wall-clock) and ``referenceFloor`` is the
        estimated accuracy of the reference itself [rad]. Errors at or below the floor
        only show agreement with the reference to within its own uncertainty.
    """
    referenceSigma = finalAttitude(builder, reference["integrator"],
                                   reference["dt"], reference["tol"])
    checkSigma = finalAttitude(builder, referenceCheck["integrator"],
                               referenceCheck["dt"], referenceCheck["tol"])
    referenceFloor = (principalAngle(checkSigma, referenceSigma)
                      if checkSigma is not None else 0.0)
    rows = []
    for integratorName, dt, tol in SWEEP_CONFIGS:
        sigma = finalAttitude(builder, integratorName, dt, tol)
        if sigma is None:
            # Diverged (unstable for this step/tolerance) -> not on the front.
            continue
        error = principalAngle(sigma, referenceSigma)
        wall = timedRun(builder, integratorName, dt, tol)
        rows.append({
            "integrator": integratorName,
            "dt": dt,
            "tol": tol,
            "error": error,
            "wall": wall["median"],
            "wallMin": wall["min"],
            "wallStd": wall["std"],
        })
    return rows, referenceFloor


def paretoFrontier(rows):
    """Return the non-dominated subset (lowest error for a given or lower cost).

    A point is Pareto-optimal if no other point has both lower wall-clock time and lower
    error. The returned list is sorted by increasing wall-clock time.

    Args:
        rows (list): per-configuration dicts with ``error`` and ``wall``.

    Returns:
        list: the non-dominated configurations, sorted by wall-clock time.
    """
    ordered = sorted(rows, key=lambda r: (r["wall"], r["error"]))
    frontier = []
    bestError = np.inf
    for row in ordered:
        if row["error"] < bestError:
            frontier.append(row)
            bestError = row["error"]
    return frontier


def run(showPlots=False, saveJson=False, sweepConfigs=None, reference=None,
        referenceCheck=None):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): if True, plot and show the simulation results.
            Defaults to False.
        saveJson (bool, optional): if True, write the Pareto data to
            ``results/scenarioCompareParetoRwPanels.json``. Defaults to False.
        sweepConfigs (list, optional): override the integrator sweep (the unit test
            passes a reduced set). Defaults to :data:`SWEEP_CONFIGS`.
        reference (dict, optional): override the reference (truth) configuration.
            Defaults to :data:`REFERENCE`.
        referenceCheck (dict, optional): override the reference-accuracy check
            configuration. Defaults to :data:`REFERENCE_CHECK`.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    global SWEEP_CONFIGS
    if sweepConfigs is not None:
        SWEEP_CONFIGS = sweepConfigs
    referenceConfig = reference if reference is not None else REFERENCE
    referenceCheckConfig = referenceCheck if referenceCheck is not None else REFERENCE_CHECK

    bsmRows, bsmFloor = sweepEngine(buildBSM, referenceConfig,
                                            referenceCheckConfig)
    if couldImportMujoco:
        mujocoRows, mujocoFloor = sweepEngine(buildMujoco, referenceConfig,
                                              referenceCheckConfig)
    else:
        mujocoRows, mujocoFloor = [], 0.0
    # The accuracy of the study is limited by the least accurate engine reference.
    referenceFloor = max(bsmFloor, mujocoFloor)

    if saveJson:
        import json
        os.makedirs(resultsPath, exist_ok=True)
        with open(os.path.join(resultsPath, fileName+".json"), "w") as f:
            json.dump({"scenario": fileName, "referenceFloor": referenceFloor,
                       "bsm": bsmRows, "mujoco": mujocoRows}, f, indent=2)

    figureList = plotResults(bsmRows, mujocoRows, referenceFloor)

    if showPlots:
        plt.show()
    plt.close("all")

    return figureList


def plotResults(bsmRows, mujocoRows, referenceFloor=0.0):
    """Build the Pareto and frontier figures.

    Args:
        bsmRows (list): BSM-engine sweep results.
        mujocoRows (list): MuJoCo-engine sweep results (possibly empty).
        referenceFloor (float): estimated reference accuracy [rad] (recorded in the JSON
            summary; not drawn on the figures).

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    figureList = {}

    # --- Figure 1: all configurations, colored by integrator, shaped by engine. ---
    # Legend outside the axes so it does not overlap the data.
    fig, ax = plt.subplots(figsize=(8, 4))
    figureList[fileName+"_pareto"] = fig
    seenLabels = set()
    for rows, engineLabel, facecolor in (
            (bsmRows, "BSM", True), (mujocoRows, "mujoco", False)):
        for row in rows:
            color, marker, intLabel = INTEGRATOR_STYLE[row["integrator"]]
            label = None
            key = (row["integrator"], engineLabel)
            if key not in seenLabels:
                label = f"{intLabel} ({engineLabel})"
                seenLabels.add(key)
            ax.scatter(row["wall"], row["error"],
                       marker=marker, s=45,
                       facecolors=color if facecolor else "none",
                       edgecolors=color, label=label)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Wall-clock time [s]")
    ax.set_ylabel("Final attitude error [rad]")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=7, ncol=1, loc="center left", bbox_to_anchor=(1.02, 0.5))
    fig.tight_layout()

    # --- Figure 2: the non-dominated frontier of each engine. ---
    fig, ax = plt.subplots(figsize=(8, 4))
    figureList[fileName+"_frontier"] = fig
    for rows, engineLabel, color in (
            (bsmRows, "Back-substitution (BSM)", unitTestSupport.getLineColor(0, 3)),
            (mujocoRows, "MuJoCo", unitTestSupport.getLineColor(1, 3))):
        if not rows:
            continue
        frontier = paretoFrontier(rows)
        walls = [r["wall"] for r in frontier]
        errors = [r["error"] for r in frontier]
        ax.plot(walls, errors, "-o", color=color, label=engineLabel)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Wall-clock time [s]")
    ax.set_ylabel("Final attitude error [rad]")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))
    fig.tight_layout()

    return figureList


if __name__ == "__main__":
    run(True, False)
