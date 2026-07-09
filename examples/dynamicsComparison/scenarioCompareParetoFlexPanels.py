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
Accuracy-versus-runtime Pareto study on a stability-limited, sixteen-segment flexible
solar array (see :ref:`scenarioCompareFlexPanels` for the lumped-mass model and
:ref:`scenarioCompareParetoRwPanels` for the companion study on a slower, well-resolved
system).

This scenario repeats the work-precision (Pareto) sweep of
:ref:`scenarioCompareParetoRwPanels` on a harder problem: each solar array is discretized
into ``N = 16`` rigid segments, giving 70 system degrees of freedom and a wide spread of
vibration frequencies. Damping is light (:math:`\zeta\approx0.01`), so the
highest-frequency mode is oscillatory and sets a hard explicit stability limit on the
step size. The dynamics are stability-limited rather than classically stiff: a stiff
system has widely separated real eigenvalues and rewards implicit methods, whereas here
the fast eigenvalues are lightly damped complex pairs that must be resolved for phase
accuracy. The case stress-tests how each explicit integrator behaves near its stability
boundary.

.. note::

    Basilisk's ``svIntegrator`` system offers only explicit Runge-Kutta methods (Euler,
    RK2, RK4, RKF45, RKF78), and :ref:`MJScene<MJScene>` is driven by the Basilisk
    integrator, so MuJoCo's implicit solvers are bypassed. This study characterizes the
    explicit family only.

The sweep runs fixed-step integrators (RK2, RK4) over a range of time steps and adaptive
integrators (RKF45, RKF78) over a range of tolerances, measures the wall-clock time and
the final attitude error (principal angle of the relative direction cosine matrix
against a per-engine tight reference), and assembles a Pareto front.

Two effects dominate the result:

#. **Fixed-step integrators are stability-bound, not accuracy-bound.** Once the step is
   small enough to be stable, a high-order method is already very accurate, so the usual
   graded accuracy-cost tradeoff collapses; slightly larger steps go unstable. Diverged
   configurations are detected and excluded.
#. **Adaptive integrators give a clean, controllable front.** Because they subdivide the
   step to satisfy the tolerance (and to remain stable), RKF45 and RKF78 trace a smooth
   accuracy-versus-cost curve and are the efficient choice for this stability-limited
   model.

As in :ref:`scenarioCompareParetoRwPanels`, the :ref:`MJScene<MJScene>` runs enable
``highOrderAttitudeIntegration`` so the free-joint attitude quaternion is advanced at the
integrator's full order. The adaptive ladders therefore hold the task (macro) time step
fixed and vary only the tolerance, which controls the error down to machine precision.

.. note::

    **What the reference "truth" is, and what it is not.** The reference is *per-engine*:
    each engine's error is measured against its own tight solution, not against the other
    engine or a shared truth, so these plots rank each engine's integrator work-precision
    rather than absolute cross-engine accuracy.

The script is found in the folder ``basilisk/examples/dynamicsComparison`` and executed
by using::

    python3 scenarioCompareParetoFlexPanels.py

Illustration of Simulation Results
----------------------------------

All sampled configurations, colored by integrator and shaped by engine. Fixed-step
points cluster against the stability limit; adaptive points sweep a clean curve.

.. image:: /_images/Scenarios/scenarioCompareParetoFlexPanels_pareto.svg
   :align: center

The non-dominated frontier of each engine summarizes the cheapest achievable accuracy on
this stability-limited system.

.. image:: /_images/Scenarios/scenarioCompareParetoFlexPanels_frontier.svg
   :align: center

The same work-precision analysis is repeated for the hub inertial position error, giving
a companion view of the translational coupling alongside the attitude result.

.. image:: /_images/Scenarios/scenarioCompareParetoFlexPanels_frontierPosition.svg
   :align: center

"""

import os
import time

import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.simulation import spacecraft
from Basilisk.simulation import nHingedRigidBodyStateEffector
from Basilisk.simulation import svIntegrators

try:
    from Basilisk.simulation import mujoco
    couldImportMujoco = True
except Exception:
    couldImportMujoco = False

fileName = os.path.basename(os.path.splitext(__file__)[0])

# Folder this scenario writes its JSON summary into.
resultsPath = os.path.join(os.path.dirname(__file__), "results")

# ---------------------------------------------------------------------------
# Model parameters: a stability-limited, lightly damped 16-segment high-frequency array.
# ---------------------------------------------------------------------------
N_SEGMENTS = 16  # segments per chain (70 system degrees of freedom)
HUB_MASS = 600.0  # [kg]
HUB_INERTIA = (400.0, 380.0, 360.0)  # [kg*m^2]
PANEL_TOTAL_MASS = 60.0  # [kg] total mass of one array, split across its segments
PANEL_TOTAL_LEN = 3.0  # [m] total length of one array
PANEL_K = 120.0  # [N*m/rad] per-hinge torsional stiffness
PANEL_C = 0.05  # [N*m*s/rad] per-hinge damping (light, so vibration persists)
PANEL_HINGE_X = 0.8  # [m] root-hinge offset from hub origin along x
ROOT_THETA0 = 8.0*macros.D2R  # [rad] initial deflection of the root segment
OMEGA0_B = (0.010, -0.020, 0.015)  # [rad/s] initial hub rate

SIM_DURATION = 10.0  # [s] propagation horizon for every configuration
TIMING_TRIALS = 5  # number of trials for median/min/std reporting
TIMING_WARMUP_SEC = 1.0  # [s] discarded warmup sim time absorbing one-time process costs

# Macro (task) time step shared by every adaptive-integrator configuration. With
# high-order attitude integration the macro step is not a separate accuracy knob, so the
# adaptive ladders hold it fixed and vary only the tolerance. It is small enough that
# RKF78 stays stable at the loosest tolerance; a larger macro step diverges.
ADAPTIVE_MACRO_DT = 0.1  # [s]

# Reference (truth): a high-order adaptive integrator at the tightest tolerance, using
# the same macro step as the adaptive sweep.
REFERENCE = {"integrator": "svIntegratorRKF78", "dt": ADAPTIVE_MACRO_DT, "tol": 1.0e-13}

# Same integrator and macro step with the tolerance relaxed one notch; its disagreement
# with the reference is the study's accuracy floor.
REFERENCE_CHECK = {"integrator": "svIntegratorRKF78", "dt": ADAPTIVE_MACRO_DT, "tol": 1.0e-12}

# Full Pareto sweep. Fixed-step integrators vary dt around the explicit stability limit;
# adaptive integrators hold the macro step at ``ADAPTIVE_MACRO_DT`` and vary only the
# tolerance. A fixed-step configuration is either stable (and then already near the
# metric floor) or it diverges and is dropped at run time, so the lists probe a wide
# range. ``runAllComparisons.py`` uses this sweep for the published figures.
FULL_SWEEP_CONFIGS = [
    ("svIntegratorRK2", 1.0e-3, None),
    ("svIntegratorRK2", 8.0e-4, None),
    ("svIntegratorRK2", 5.0e-4, None),
    ("svIntegratorRK2", 3.0e-4, None),
    ("svIntegratorRK4", 2.5e-3, None),
    ("svIntegratorRK4", 2.0e-3, None),
    ("svIntegratorRK4", 1.0e-3, None),
    ("svIntegratorRK4", 5.0e-4, None),
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

# Default sweep; the unit test overrides this with a reduced set.
SWEEP_CONFIGS = FULL_SWEEP_CONFIGS

INTEGRATOR_STYLE = {
    "svIntegratorEuler": (unitTestSupport.getLineColor(0, 5), "o", "Euler"),
    "svIntegratorRK2": (unitTestSupport.getLineColor(1, 5), "s", "RK2"),
    "svIntegratorRK4": (unitTestSupport.getLineColor(2, 5), "^", "RK4"),
    "svIntegratorRKF45": (unitTestSupport.getLineColor(3, 5), "D", "RKF45 (adaptive)"),
    "svIntegratorRKF78": (unitTestSupport.getLineColor(4, 5), "v", "RKF78 (adaptive)"),
}


def segmentMass():
    """Mass of one chain segment [kg]."""
    return PANEL_TOTAL_MASS/N_SEGMENTS


def segmentHalfLength():
    """Half-length ``d`` of one chain segment [m]."""
    return PANEL_TOTAL_LEN/(2.0*N_SEGMENTS)


def segmentInertia():
    """Slender-bar inertia tensor of one chain segment about its center of mass.

    Returns:
        numpy.ndarray: 3x3 inertia tensor [kg*m^2].
    """
    mass = segmentMass()
    length = 2.0*segmentHalfLength()  # [m]
    iBar = mass*length**2/12.0  # [kg*m^2]
    iAxial = max(iBar*0.05, 1.0e-4)  # [kg*m^2] small, well-posed axial inertia
    return np.diag([iAxial, iBar, iBar])


def mujocoModel():
    """Return the MJCF model: hub with two ``N``-segment hinged chains."""
    ix, iy, iz = HUB_INERTIA
    halfLen = segmentHalfLength()
    mass = segmentMass()
    inertia = segmentInertia()

    def chain(sign, tag):
        body = ""
        for i in reversed(range(N_SEGMENTS)):
            jointName = f"{tag}_{i}"
            pos = f"{sign*PANEL_HINGE_X} 0 0" if i == 0 else f"{-2*halfLen} 0 0"
            body = (
                f'<body name="{jointName}" pos="{pos}">'
                f'<joint name="{jointName}" type="hinge" axis="0 1 0" '
                f'stiffness="{PANEL_K}" damping="{PANEL_C}" springref="0"/>'
                f'<inertial pos="{-halfLen} 0 0" mass="{mass}" '
                f'fullinertia="{inertia[0,0]} {inertia[1,1]} {inertia[2,2]} 0 0 0"/>'
                f'{body}'
                f'</body>'
            )
        return body

    return f"""
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" mass="{HUB_MASS}" fullinertia="{ix} {iy} {iz} 0 0 0"/>
      {chain(+1, "pP")}
      {chain(-1, "pM")}
    </body>
  </worldbody>
</mujoco>
"""


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
    """Build the back-substitution (BSM) 16-segment flexible-array simulation.

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
    scObject.hub.IHubPntBc_B = np.diag(HUB_INERTIA).tolist()  # [kg*m^2]
    scObject.hub.omega_BN_BInit = [[w] for w in OMEGA0_B]  # [rad/s]
    scSim.AddModelToTask("dynTask", scObject)

    integrator = makeIntegrator(scObject, integratorName, tol)

    inertia = segmentInertia().tolist()
    chains = []
    for sign in (+1, -1):
        effector = nHingedRigidBodyStateEffector.NHingedRigidBodyStateEffector()
        effector.ModelTag = f"chain{'P' if sign > 0 else 'M'}"
        effector.r_HB_B = [[sign*PANEL_HINGE_X], [0.0], [0.0]]  # [m]
        effector.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        for i in range(N_SEGMENTS):
            panel = nHingedRigidBodyStateEffector.HingedPanel()
            panel.mass = segmentMass()  # [kg]
            panel.d = segmentHalfLength()  # [m]
            panel.k = PANEL_K  # [N*m/rad]
            panel.c = PANEL_C  # [N*m*s/rad]
            panel.IPntS_S = inertia  # [kg*m^2]
            panel.thetaInit = ROOT_THETA0 if i == 0 else 0.0  # [rad]
            effector.addHingedPanel(panel)
        scObject.addStateEffector(effector)
        scSim.AddModelToTask("dynTask", effector)
        chains.append(effector)

    recorder = scObject.scStateOutMsg.recorder(macros.sec2nano(SIM_DURATION))
    scSim.AddModelToTask("dynTask", recorder)

    scSim.InitializeSimulation()
    return scSim, recorder, [scObject, integrator] + chains


def buildMujoco(integratorName, dt, tol):
    """Build the MuJoCo 16-segment flexible-array simulation.

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
    # Full-order attitude integration so the hub quaternion error converges at the RK
    # method's order and is controlled by the adaptive tolerance, not the macro step.
    scene.highOrderAttitudeIntegration = True
    scSim.AddModelToTask("dynTask", scene, 1)

    integrator = makeIntegrator(scene, integratorName, tol)

    hub = scene.getBody("hub")
    recorder = hub.getOrigin().stateOutMsg.recorder(macros.sec2nano(SIM_DURATION))
    scSim.AddModelToTask("dynTask", recorder, 0)

    scSim.InitializeSimulation()
    hub.setAttitudeRate(list(OMEGA0_B))
    for tag in ("pP_0", "pM_0"):
        scene.getBody(tag).getScalarJoint(tag).setPosition(ROOT_THETA0)
    return scSim, recorder, [scene, integrator, hub]


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
    try:
        scSim, recorder, _ = builder(integratorName, dt, tol)
        scSim.ConfigureStopTime(macros.sec2nano(SIM_DURATION))
        scSim.ExecuteSimulation()
    except Exception:
        # MuJoCo raises on NaN acceleration; treat as divergence.
        return None, None
    sigma = np.array(recorder.sigma_BN)[-1]
    r = np.array(recorder.r_BN_N)[-1]  # [m] hub inertial position
    if not (np.all(np.isfinite(sigma)) and np.all(np.isfinite(r))):
        return None, None
    return sigma, r


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

    Computed from the relative MRP as ``4*atan(|sigma_rel|)``, which stays well
    conditioned down to machine precision, unlike ``arccos((trace(C)-1)/2)``, which loses
    resolution below ~1e-8 rad.
    """
    dcmRel = rbk.MRP2C(sigmaA).dot(rbk.MRP2C(sigmaB).T)
    sigmaRel = rbk.C2MRP(dcmRel)
    return float(4.0*np.arctan(np.linalg.norm(sigmaRel)))


def sweepEngine(builder, reference=REFERENCE, referenceCheck=None):
    """Run the reference and every sweep configuration for one engine.

    Args:
        builder (callable): ``buildBSM`` or ``buildMujoco``.
        reference (dict): reference (truth) configuration with keys ``integrator``,
            ``dt`` and ``tol``.
        referenceCheck (dict): second high-accuracy configuration whose disagreement with
            the reference estimates the study's accuracy floor. Defaults to
            :data:`REFERENCE_CHECK`.

    Returns:
        tuple: ``(rows, attitudeFloor, positionFloor)`` -- one dict per successful
        configuration (attitude error, position error, wall-clock) and the estimated
        reference accuracies in attitude [rad] and hub position [m]. Errors at or below a
        floor only indicate agreement with the reference to round-off.
    """
    if referenceCheck is None:
        referenceCheck = REFERENCE_CHECK
    referenceSigma, referenceR = finalState(builder, reference["integrator"],
                                            reference["dt"], reference["tol"])
    checkSigma, checkR = finalState(builder, referenceCheck["integrator"],
                                    referenceCheck["dt"], referenceCheck["tol"])
    attitudeFloor = (principalAngle(checkSigma, referenceSigma)
                     if checkSigma is not None else 0.0)
    positionFloor = (float(np.linalg.norm(np.asarray(checkR) - np.asarray(referenceR)))
                     if checkR is not None else 0.0)
    rows = []
    for integratorName, dt, tol in SWEEP_CONFIGS:
        sigma, r = finalState(builder, integratorName, dt, tol)
        if sigma is None:
            continue  # diverged: stability limit exceeded
        wall = timedRun(builder, integratorName, dt, tol)
        rows.append({
            "integrator": integratorName,
            "dt": dt,
            "tol": tol,
            "error": principalAngle(sigma, referenceSigma),
            "positionError": float(np.linalg.norm(np.asarray(r) - np.asarray(referenceR))),
            "wall": wall["median"],
            "wallMin": wall["min"],
            "wallStd": wall["std"],
        })
    return rows, attitudeFloor, positionFloor


def paretoFrontier(rows, errorKey="error"):
    """Return the non-dominated subset, sorted by increasing wall-clock time.

    Args:
        rows (list): per-configuration dicts with ``wall`` and the metric ``errorKey``.
        errorKey (str): row key of the error metric to build the frontier over
            (``"error"`` for attitude, ``"positionError"`` for hub position).

    Returns:
        list: the non-dominated configurations.
    """
    ordered = sorted(rows, key=lambda r: (r["wall"], r[errorKey]))
    frontier = []
    bestError = np.inf
    for row in ordered:
        if row[errorKey] < bestError:
            frontier.append(row)
            bestError = row[errorKey]
    return frontier


def run(showPlots=False, saveJson=False, sweepConfigs=None, simDuration=None,
        reference=None, referenceCheck=None):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): if True, plot and show the simulation results.
            Defaults to False.
        saveJson (bool, optional): if True, write the Pareto data to
            ``results/scenarioCompareParetoFlexPanels.json``. Defaults to
            False.
        sweepConfigs (list, optional): override the integrator sweep (the unit test
            passes a reduced set). Defaults to :data:`SWEEP_CONFIGS`.
        simDuration (float, optional): override the propagation horizon [s]. Defaults to
            :data:`SIM_DURATION`.
        reference (dict, optional): override the reference (truth) configuration.
            Defaults to :data:`REFERENCE`.
        referenceCheck (dict, optional): override the reference-error-estimate
            configuration. Defaults to :data:`REFERENCE_CHECK`.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    global SWEEP_CONFIGS, SIM_DURATION
    if sweepConfigs is not None:
        SWEEP_CONFIGS = sweepConfigs
    if simDuration is not None:
        SIM_DURATION = simDuration
    referenceConfig = reference if reference is not None else REFERENCE
    checkConfig = referenceCheck if referenceCheck is not None else REFERENCE_CHECK

    bsmRows, bsmFloor, bsmPosFloor = sweepEngine(buildBSM, referenceConfig, checkConfig)
    if couldImportMujoco:
        mujocoRows, mujocoFloor, mujocoPosFloor = sweepEngine(
            buildMujoco, referenceConfig, checkConfig)
    else:
        mujocoRows, mujocoFloor, mujocoPosFloor = [], 0.0, 0.0
    referenceFloor = max(bsmFloor, mujocoFloor)  # [rad]
    positionReferenceFloor = max(bsmPosFloor, mujocoPosFloor)  # [m]

    if saveJson:
        import json
        os.makedirs(resultsPath, exist_ok=True)
        with open(os.path.join(resultsPath, fileName+".json"), "w") as f:
            json.dump({"scenario": fileName, "nSegments": N_SEGMENTS,
                       "referenceFloor": referenceFloor,
                       "positionReferenceFloor": positionReferenceFloor,
                       "bsm": bsmRows, "mujoco": mujocoRows}, f, indent=2)

    figureList = plotResults(bsmRows, mujocoRows, referenceFloor,
                             positionReferenceFloor)

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
    # Legend outside the axes so it does not overlap the data points.
    fig, ax = plt.subplots(figsize=(8, 4))
    seenLabels = set()
    for rows, engineLabel, filled in (
            (bsmRows, "BSM", True), (mujocoRows, "mujoco", False)):
        for row in rows:
            color, marker, intLabel = INTEGRATOR_STYLE[row["integrator"]]
            label = None
            key = (row["integrator"], engineLabel)
            if key not in seenLabels:
                label = f"{intLabel} ({engineLabel})"
                seenLabels.add(key)
            ax.scatter(row["wall"], row[errorKey],
                       marker=marker, s=45,
                       facecolors=color if filled else "none",
                       edgecolors=color, label=label)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Wall-clock time [s]")
    ax.set_ylabel(ylabel)
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=7, ncol=1, loc="center left", bbox_to_anchor=(1.02, 0.5))
    fig.tight_layout()
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
    fig, ax = plt.subplots(figsize=(8, 4))
    for rows, engineLabel, color in (
            (bsmRows, "Back-substitution (BSM)", unitTestSupport.getLineColor(0, 3)),
            (mujocoRows, "MuJoCo", unitTestSupport.getLineColor(1, 3))):
        if not rows:
            continue
        frontier = paretoFrontier(rows, errorKey)
        ax.plot([r["wall"] for r in frontier], [r[errorKey] for r in frontier],
                "-o", color=color, label=engineLabel)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Wall-clock time [s]")
    ax.set_ylabel(ylabel)
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))
    fig.tight_layout()
    return fig


def plotResults(bsmRows, mujocoRows, referenceFloor=0.0, positionReferenceFloor=0.0):
    """Build the Pareto and frontier figures for both attitude and hub-position error.

    Args:
        bsmRows (list): BSM-engine sweep results.
        mujocoRows (list): MuJoCo-engine sweep results (possibly empty).
        referenceFloor (float): estimated attitude reference accuracy [rad] (recorded in
            the JSON summary; not drawn on the figures).
        positionReferenceFloor (float): estimated position reference accuracy [m] (recorded
            in the JSON summary; not drawn on the figures).

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    attLabel = "Final attitude error [rad]"
    posLabel = "Final hub position error [m]"
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
