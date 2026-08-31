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

.. note::

    **What the reference "truth" is, and what it is not.** The reference is *per-engine*:
    each engine's error is measured against its own tight solution, not against the other
    engine or a shared analytic truth. These plots therefore rank each engine's integrator
    work-precision; they are not a statement of absolute cross-engine accuracy (that is
    established separately by the fixed-step agreement scenarios and, for the Keplerian
    case, by the analytic reference in :ref:`scenarioCompareOrbit`).

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

.. note::

    To bound CI time, the automated documentation build generates these figures
    from three representative configurations: one RK4, one Euler, and one RKF45
    run. They exercise the work-precision workflow but do not constitute the
    wide, well-populated default Pareto study described above. Run this script
    directly with its defaults before drawing performance conclusions.

The reduced documentation profile illustrates how the individual configurations
and their lower-left envelope are displayed.

.. image:: /_images/Scenarios/scenarioCompareParetoRwPanels_pareto.svg
   :align: center

The reduced frontier is a plotting demonstration, not a basis for ranking the
engines at unsampled target accuracies.

.. image:: /_images/Scenarios/scenarioCompareParetoRwPanels_frontier.svg
   :align: center

The same work-precision analysis is repeated for the hub inertial position error, giving
a companion view of the translational coupling alongside the attitude result.

.. image:: /_images/Scenarios/scenarioCompareParetoRwPanels_frontierPosition.svg
   :align: center

Next comparison: :ref:`scenarioCompareParetoFlexPanels` repeats the work-precision
study on a stability-limited flexible system.

"""

import os

import numpy as np
import matplotlib.pyplot as plt

import _comparePlots
import _comparisonValidation
import _runtimeTable

from Basilisk import hasBuildFeature
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeRW
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.simulation import spacecraft
from Basilisk.simulation import reactionWheelStateEffector
from Basilisk.simulation import hingedRigidBodyStateEffector
from Basilisk.simulation import svIntegrators
from Basilisk.architecture import messaging

couldImportMujoco = hasBuildFeature("mujoco")
if couldImportMujoco:
    from Basilisk.simulation import mujoco

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


def panelRootDcm(hingeX):
    """Return the root-frame DCM that makes both panels extend away from the hub."""
    if hingeX > 0.0:
        return [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


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
# tolerance relaxed one notch. The disagreement ``||y(reference) - y(check)||`` is a
# conservative reference self-consistency threshold; errors below it only show agreement
# with the selected reference at that scale.
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
]

# Default sweep used by ``run()``; the unit test overrides it with a reduced subset.
SWEEP_CONFIGS = FULL_SWEEP_CONFIGS

# Paul Tol high-contrast colors. Markers distinguish the five integrator families.
TOL_COLORS = ("#004488", "#BB5566", "#DDAA33")


def tolColor(index):
    return TOL_COLORS[index % len(TOL_COLORS)]


INTEGRATOR_STYLE = {
    "svIntegratorEuler": (tolColor(0), "o", "Euler"),
    "svIntegratorRK2": (tolColor(1), "s", "RK2"),
    "svIntegratorRK4": (tolColor(2), "^", "RK4"),
    "svIntegratorRKF45": (tolColor(3), "D", "RKF45 (adaptive)"),
    "svIntegratorRKF78": (tolColor(4), "v", "RKF78 (adaptive)"),
}


def mujocoModel():
    """Return the MJCF model: hub, three reaction wheels, two hinged panels."""
    ix, iy, iz = HUB_CORE_INERTIA
    ip = PANEL_INERTIA
    return f"""
<mujoco>
  <!-- No geoms, equalities, limits, or frictionloss in this model: disable the
       collision and constraint pipelines so each derivative evaluation skips
       broadphase and constraint-assembly bookkeeping (bit-identical results). -->
  <option gravity="0 0 0">
    <flag contact="disable" constraint="disable"/>
  </option>
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

      <body name="panelP" pos="{PANEL_HINGE_X} 0 0" quat="0 0 0 1">
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


def buildBSM(integratorName, dt, tol, record=True):
    """Build the back-substitution (BSM) hub-wheel-panel simulation with the given integrator.

    Args:
        integratorName (str): integrator attribute in ``svIntegrators``.
        dt (float): task (macro) time step [s].
        tol (float): adaptive tolerance, or None for fixed-step integrators.
        record (bool, optional): attach a final-state recorder. Defaults to True.

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
        panel.dcm_HB = panelRootDcm(hingeX)
        panel.thetaInit = PANEL_THETA0  # [rad]
        panel.thetaDotInit = 0.0  # [rad/s]
        panel.ModelTag = "panel" + ("P" if hingeX > 0 else "M")
        scObject.addStateEffector(panel)
        scSim.AddModelToTask("dynTask", panel)
        panels.append(panel)

    recorder = None
    if record:
        recorder = scObject.scStateOutMsg.recorder(macros.sec2nano(dt))
        scSim.AddModelToTask("dynTask", recorder)

    scSim.InitializeSimulation()
    handles = [scObject, integrator, rwEffector, rwFactory, rwCmdMsg] + panels
    return scSim, recorder, handles


def buildMujoco(integratorName, dt, tol, record=True):
    """Build the MuJoCo hub-wheel-panel simulation with the given integrator.

    Args:
        integratorName (str): integrator attribute in ``svIntegrators``.
        dt (float): task (macro) time step [s].
        tol (float): adaptive tolerance, or None for fixed-step integrators.
        record (bool, optional): attach a final-state recorder. Defaults to True.

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

    recorder = None
    if record:
        recorder = hub.getOrigin().stateOutMsg.recorder(macros.sec2nano(dt))
        scSim.AddModelToTask("dynTask", recorder, 0)

    scSim.InitializeSimulation()
    hub.setAttitudeRate(list(OMEGA0_B))
    for name in ["panelP", "panelM"]:
        scene.getBody(name).getScalarJoint(name).setPosition(PANEL_THETA0)
    return scSim, recorder, [scene, integrator, hub] + actuatorMsgs


def finalState(builder, integratorName, dt, tol):
    """Propagate one configuration and return its final hub attitude and position.

    Args:
        builder (callable): ``buildBSM`` or ``buildMujoco``.
        integratorName (str): integrator attribute name.
        dt (float): task time step [s].
        tol (float): adaptive tolerance, or None.

    Returns:
        tuple: ``(sigma_BN, r_BN_N)`` at the final time -- the attitude MRP (shape
        ``(3,)``) and the hub inertial position [m] (shape ``(3,)``) -- or ``(None, None)``
        if the run diverged.
    """
    stopTimeNanos = macros.sec2nano(SIM_DURATION)
    taskStepNanos = macros.sec2nano(dt)
    if stopTimeNanos % taskStepNanos != 0:
        raise ValueError(
            f"dt={dt} s does not divide the {SIM_DURATION} s comparison horizon"
        )
    scSim, recorder, _ = builder(integratorName, dt, tol)
    scSim.ConfigureStopTime(stopTimeNanos)
    scSim.ExecuteSimulation()
    times = np.array(recorder.times())*macros.NANO2SEC  # [s]
    if len(times) == 0 or not np.isclose(
            times[-1], SIM_DURATION, rtol=0.0, atol=0.5*macros.NANO2SEC):
        return None, None
    sigma = np.array(recorder.sigma_BN)[-1]
    r = np.array(recorder.r_BN_N)[-1]  # [m] hub inertial position
    if not (np.all(np.isfinite(sigma)) and np.all(np.isfinite(r))):
        return None, None
    return sigma, r


def snappedStep(dt):
    """Snap a requested step onto the comparison horizon and report the change."""
    snapped = _comparisonValidation.snapStepToHorizon(dt, SIM_DURATION)
    if snapped != dt:
        print(
            f"dt={dt:g} s does not divide the {SIM_DURATION:g} s horizon; "
            f"using dt={snapped:g} s"
        )
    return snapped


def normalizedConfiguration(configuration):
    """Return one named configuration with its executed step recorded."""
    normalized = dict(configuration)
    requestedStep = normalized["dt"]
    normalized["dt"] = snappedStep(requestedStep)
    if normalized["dt"] != requestedStep:
        normalized["requestedDt"] = requestedStep
    return normalized


def normalizedSweepConfigurations(configurations):
    """Return execution tuples and serializable requested/executed step records."""
    execution = []
    records = []
    requestedByExecution = {}
    for integrator, requestedStep, tolerance in configurations:
        executedStep = snappedStep(requestedStep)
        executionKey = (integrator, executedStep, tolerance)
        if executionKey in requestedByExecution:
            previousStep = requestedByExecution[executionKey]
            raise ValueError(
                f"requested steps {previousStep:g} s and {requestedStep:g} s "
                f"both map to the same executed {integrator} configuration "
                f"at dt={executedStep:g} s and tol={tolerance!r}."
            )
        requestedByExecution[executionKey] = requestedStep
        execution.append((integrator, executedStep, tolerance))
        record = {"integrator": integrator, "dt": executedStep, "tol": tolerance}
        if executedStep != requestedStep:
            record["requestedDt"] = requestedStep
        records.append(record)
    return tuple(execution), records


def principalAngle(sigmaA, sigmaB):
    """Principal rotation angle between two attitudes given as MRPs [rad].

    The angle is computed from the relative MRP as ``4*atan(|sigma_rel|)``, which stays
    well conditioned down to machine precision, unlike ``arccos((trace(C)-1)/2)``, whose
    argument approaches 1 for small angles and collapses to zero below roughly 1e-8 rad.
    """
    dcmRel = rbk.MRP2C(sigmaA).dot(rbk.MRP2C(sigmaB).T)
    sigmaRel = rbk.C2MRP(dcmRel)
    return float(4.0*np.arctan(np.linalg.norm(sigmaRel)))


def sweepEngine(builder, reference=REFERENCE, referenceCheck=REFERENCE_CHECK,
                sweepConfigs=None):
    """Run the reference and every sweep configuration for one engine.

    Args:
        builder (callable): ``buildBSM`` or ``buildMujoco``.
        reference (dict): reference (truth) configuration with keys ``integrator``,
            ``dt`` and ``tol``.
        referenceCheck (dict): second configuration used to estimate reference
            self-consistency.
        sweepConfigs (sequence, optional): configurations to evaluate. Defaults to
            :data:`SWEEP_CONFIGS`.

    Returns:
        tuple: ``(rows, attitudeCheck, positionCheck)`` where ``rows`` holds one dict per
        successful configuration (integrator, attitude error, position error, wall-clock)
        and the final values are the attitude [rad] and position [m] disagreements between
        the selected reference and check configurations.
    """
    referenceSigma, referenceR = finalState(
        builder,
        reference["integrator"],
        snappedStep(reference["dt"]),
        reference["tol"],
    )
    checkSigma, checkR = finalState(
        builder,
        referenceCheck["integrator"],
        snappedStep(referenceCheck["dt"]),
        referenceCheck["tol"],
    )
    if referenceSigma is None or checkSigma is None:
        raise RuntimeError("Pareto reference or reference-check propagation failed.")
    attitudeFloor = principalAngle(checkSigma, referenceSigma)
    positionFloor = float(np.linalg.norm(np.asarray(checkR) - np.asarray(referenceR)))
    rows = []
    configurations = SWEEP_CONFIGS if sweepConfigs is None else sweepConfigs
    for integratorName, dt, tol in configurations:
        step = snappedStep(dt)
        sigma, r = finalState(builder, integratorName, step, tol)
        if sigma is None:
            # Diverged (unstable for this step/tolerance) -> not on the front.
            continue
        rows.append({
            "integrator": integratorName,
            "dt": step,
            "tol": tol,
            "error": principalAngle(sigma, referenceSigma),
            "positionError": float(np.linalg.norm(np.asarray(r) - np.asarray(referenceR))),
        })
    return rows, attitudeFloor, positionFloor


def addInterleavedTimings(bsmRows, mujocoRows):
    """Attach recorder-free timing statistics, alternating engine order by trial."""
    rowSets = (("bsm", buildBSM, bsmRows), ("mujoco", buildMujoco, mujocoRows))
    rowMaps = {
        engine: {(row["integrator"], row["dt"], row["tol"]): row for row in rows}
        for engine, _, rows in rowSets
    }
    keys = list(dict.fromkeys(
        (row["integrator"], row["dt"], row["tol"])
        for _, _, rows in rowSets for row in rows
    ))

    def configuredBuilder(builder, configuration, stopTime):
        integratorName, dt, tol = configuration

        def build():
            simulation, recorder, handles = builder(
                integratorName, dt, tol, record=False)
            simulation.ConfigureStopTime(macros.sec2nano(stopTime))
            return simulation, recorder, handles

        return build

    for key in keys:
        measured = []
        warmups = []
        destinations = []
        for engine, builder, _ in rowSets:
            row = rowMaps[engine].get(key)
            if row is None:
                continue
            measured.append(configuredBuilder(builder, key, SIM_DURATION))
            warmups.append(configuredBuilder(
                builder, key, min(SIM_DURATION, TIMING_WARMUP_SEC)))
            destinations.append(row)
        stats = _runtimeTable.interleavedPropagationStats(
            measured, TIMING_TRIALS, warmups)
        for row, values in zip(destinations, stats):
            row["wall"] = values["median"]
            row["wallMin"] = values["min"]
            row["wallStd"] = values["std"]


def paretoFrontier(rows, errorKey="error"):
    """Return the non-dominated subset (lowest error for a given or lower cost).

    A point is Pareto-optimal if no other point has both lower wall-clock time and lower
    error. The returned list is sorted by increasing wall-clock time.

    Args:
        rows (list): per-configuration dicts with ``wall`` and the metric ``errorKey``.
        errorKey (str): row key of the error metric to build the frontier over
            (``"error"`` for attitude, ``"positionError"`` for hub position).

    Returns:
        list: the non-dominated configurations, sorted by wall-clock time.
    """
    ordered = sorted(rows, key=lambda r: (r["wall"], r[errorKey]))
    frontier = []
    bestError = np.inf
    for row in ordered:
        if row[errorKey] < bestError:
            frontier.append(row)
            bestError = row[errorKey]
    return frontier


def run(showPlots=False, saveJson=False, sweepConfigs=None, reference=None,
        referenceCheck=None, resultsDir=None):
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
        resultsDir (str, optional): explicit artifact directory. Defaults to
            the scenario ``results`` folder.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    requestedConfigurations = SWEEP_CONFIGS if sweepConfigs is None else tuple(sweepConfigs)
    configurations, configurationRecords = normalizedSweepConfigurations(
        requestedConfigurations)
    referenceConfig = normalizedConfiguration(
        reference if reference is not None else REFERENCE)
    referenceCheckConfig = normalizedConfiguration(
        referenceCheck if referenceCheck is not None else REFERENCE_CHECK)

    bsmRows, bsmFloor, bsmPosFloor = sweepEngine(
        buildBSM, referenceConfig, referenceCheckConfig, configurations)
    if couldImportMujoco:
        mujocoRows, mujocoFloor, mujocoPosFloor = sweepEngine(
            buildMujoco, referenceConfig, referenceCheckConfig, configurations)
    else:
        mujocoRows, mujocoFloor, mujocoPosFloor = [], 0.0, 0.0
    addInterleavedTimings(bsmRows, mujocoRows)
    referenceThreshold = max(bsmFloor, mujocoFloor)  # [rad]
    positionReferenceThreshold = max(bsmPosFloor, mujocoPosFloor)  # [m]

    if saveJson:
        import json
        targetResults = resultsPath if resultsDir is None else resultsDir
        os.makedirs(targetResults, exist_ok=True)
        with open(os.path.join(targetResults, fileName+".json"), "w") as f:
            json.dump({"scenario": fileName,
                       "configuration": {
                           "simDuration": SIM_DURATION,
                           "reference": referenceConfig,
                           "referenceCheck": referenceCheckConfig,
                           "sweepConfigs": configurationRecords},
                       "referenceSelfConsistency": {
                           "bsm": {"attitude": bsmFloor, "position": bsmPosFloor},
                           "mujoco": {
                               "attitude": mujocoFloor, "position": mujocoPosFloor}},
                       "sharedReferenceThreshold": referenceThreshold,
                       "sharedPositionReferenceThreshold": positionReferenceThreshold,
                       "bsm": bsmRows, "mujoco": mujocoRows}, f, indent=2)

    figureList = plotResults(bsmRows, mujocoRows, referenceThreshold,
                             positionReferenceThreshold)

    _comparePlots.finalizeFigures(figureList)
    if showPlots:
        plt.show()
    plt.close("all")

    return figureList


def _paretoScatter(bsmRows, mujocoRows, errorKey, ylabel):
    """Scatter every configuration: color = integrator, filled = BSM, open = MuJoCo.

    Args:
        bsmRows (list): BSM-engine sweep results.
        mujocoRows (list): MuJoCo-engine sweep results (possibly empty).
        errorKey (str): row key of the error metric to plot on the y-axis.
        ylabel (str): y-axis label.

    Returns:
        matplotlib.figure.Figure: the scatter figure.
    """
    # Legend outside the axes so it does not overlap the data.
    fig, ax = plt.subplots(figsize=(8, 4), layout="constrained")
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
            value = row[errorKey]*macros.R2D if errorKey == "error" else row[errorKey]
            ax.scatter(row["wall"], value,
                       marker=marker, s=45,
                       facecolors=color if facecolor else "none",
                       edgecolors=color, label=label)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Wall-clock time [s]")
    ax.set_ylabel(ylabel)
    ax.grid(True, which="both", alpha=0.3)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(handles, labels, fontsize=7, ncol=1, loc="center left",
                  bbox_to_anchor=(1.02, 0.5))
    return fig


def _paretoFrontierPlot(bsmRows, mujocoRows, errorKey, ylabel):
    """Plot the non-dominated frontier of each engine for one error metric.

    Args:
        bsmRows (list): BSM-engine sweep results.
        mujocoRows (list): MuJoCo-engine sweep results (possibly empty).
        errorKey (str): row key of the error metric to build the frontier over.
        ylabel (str): y-axis label.

    Returns:
        matplotlib.figure.Figure: the frontier figure.
    """
    fig, ax = plt.subplots(figsize=(8, 4), layout="constrained")
    for rows, engineLabel, color in (
            (bsmRows, "Back-substitution (BSM)", tolColor(0)),
            (mujocoRows, "MuJoCo", tolColor(1))):
        if not rows:
            continue
        frontier = paretoFrontier(rows, errorKey)
        values = [
            r[errorKey]*macros.R2D if errorKey == "error" else r[errorKey]
            for r in frontier
        ]
        ax.plot([r["wall"] for r in frontier], values,
                "-o", color=color, label=engineLabel)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Wall-clock time [s]")
    ax.set_ylabel(ylabel)
    ax.grid(True, which="both", alpha=0.3)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(handles, labels, loc="center left", bbox_to_anchor=(1.02, 0.5))
    return fig


def plotResults(bsmRows, mujocoRows, referenceThreshold=0.0,
                positionReferenceThreshold=0.0):
    """Build the Pareto and frontier figures for both attitude and hub-position error.

    Args:
        bsmRows (list): BSM-engine sweep results.
        mujocoRows (list): MuJoCo-engine sweep results (possibly empty).
        referenceThreshold (float): conservative attitude reference self-consistency
            threshold [rad], recorded in JSON but not drawn here.
        positionReferenceThreshold (float): conservative position reference
            self-consistency threshold [m], recorded in JSON but not drawn here.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    attLabel = "Attitude error [deg]"
    posLabel = "Final hub\nposition error [m]"
    return {
        fileName+"_pareto": _paretoScatter(bsmRows, mujocoRows, "error", attLabel),
        fileName+"_frontier": _paretoFrontierPlot(bsmRows, mujocoRows, "error", attLabel),
        fileName+"_paretoPosition": _paretoScatter(
            bsmRows, mujocoRows, "positionError", posLabel),
        fileName+"_frontierPosition": _paretoFrontierPlot(
            bsmRows, mujocoRows, "positionError", posLabel),
    }


if __name__ == "__main__":
    run(True, False)
