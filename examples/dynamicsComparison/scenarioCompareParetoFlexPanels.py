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
Accuracy-versus-runtime Pareto study on a numerically stiff, sixteen-segment flexible
solar array (see :ref:`scenarioCompareFlexPanels` for the lumped-mass model and
:ref:`scenarioCompareParetoRwPanels` for the companion study on a non-stiff system).

This scenario repeats the work-precision (Pareto) sweep of
:ref:`scenarioCompareParetoRwPanels`, but on a much harder problem: each solar array is
discretized into ``N = 16`` rigid segments, giving 70 system degrees of freedom and a
spread of vibration frequencies. The fastest (highest-frequency) mode sets a hard
stability limit for explicit integrators, which makes this a revealing stress test of
how each integrator and each engine behave when the dynamics are stiff.

As before, we sweep fixed-step integrators (Euler, RK2, RK4) over a range of time steps
and adaptive integrators (RKF45, RKF78) over a range of tolerances, measure the
wall-clock time and the final attitude error (principal angle of the relative direction
cosine matrix against a per-engine tight reference), and assemble a Pareto front.

Two effects dominate the result and are worth watching for:

#. **Fixed-step integrators are stability-bound, not accuracy-bound.** Once the step is
   small enough to be stable, a high-order method is already very accurate, so the usual
   graded accuracy-cost tradeoff collapses; steps that are slightly too large do not
   merely lose accuracy but go unstable (and are dropped from the front). Configurations
   that diverge are detected and excluded automatically.
#. **Adaptive integrators give a clean, controllable front.** Because they subdivide the
   step to satisfy the tolerance (and to remain stable), RKF45 and RKF78 trace a smooth
   accuracy-versus-cost curve and are generally the efficient choice for stiff
   multi-body models.

As in :ref:`scenarioCompareParetoRwPanels`, the :ref:`MJScene<MJScene>` runs enable
``highOrderAttitudeIntegration`` so the free-joint attitude quaternion is advanced at the
integrator's full order. The adaptive ladders therefore hold the task (macro) time step
fixed and vary only the tolerance, which controls the error down to machine precision.

The script is found in the folder ``basilisk/examples/dynamicsComparison`` and executed
by using::

    python3 scenarioCompareParetoFlexPanels.py

Illustration of Simulation Results
----------------------------------

All sampled configurations, colored by integrator and shaped by engine. Note how the
fixed-step points cluster against the stability limit while the adaptive points sweep a
clean curve.

.. image:: /_images/Scenarios/scenarioCompareParetoFlexPanels_pareto.svg
   :align: center

The non-dominated frontier of each engine summarises the cheapest achievable accuracy on
this stiff system.

.. image:: /_images/Scenarios/scenarioCompareParetoFlexPanels_frontier.svg
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
# Model parameters: a stiff, lightly damped 16-segment flexible array.
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
TIMING_REPEATS = 1  # this system is expensive; a single timed run is used

# Macro (task) time step shared by every adaptive-integrator configuration. With
# high-order attitude integration the macro step is not a separate accuracy knob, so the
# adaptive ladders hold it fixed and vary only the tolerance. It is chosen small enough
# that even the high-order RKF78 stays stable on this stiff system at the loosest
# tolerance (a larger macro step makes RKF78 cross the stability edge and diverge).
ADAPTIVE_MACRO_DT = 0.1  # [s]

# Reference (truth): a high-order adaptive integrator at the rounding-limit tolerance,
# using the same fixed macro step as the adaptive sweep. With full-order attitude
# integration the tolerance alone drives the error to machine precision.
REFERENCE = {"integrator": "svIntegratorRKF78", "dt": ADAPTIVE_MACRO_DT, "tol": 1.0e-13}

# Estimate of the reference's own error: the same integrator and macro step with the
# tolerance relaxed one notch. Its disagreement with the reference bounds the reference
# error and is the accuracy floor of the study.
REFERENCE_CHECK = {"integrator": "svIntegratorRKF78", "dt": ADAPTIVE_MACRO_DT, "tol": 1.0e-12}

# Full Pareto sweep. Fixed-step integrators vary dt around the stiff stability limit;
# adaptive integrators hold the macro step fixed at ``ADAPTIVE_MACRO_DT`` and vary only
# the tolerance. Because this system is stiff, explicit fixed-step integrators are
# stability-bound: a configuration is either stable (and then already very accurate, near
# the metric floor) or it crosses the stability edge and diverges. Diverged configurations
# are detected at run time and dropped, so the lists deliberately probe a wide range to
# show both the achievable accuracy cluster and the cost spread to reach it. This is the
# default and what ``runAllComparisons.py`` uses for the published figures.
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

# ``run()`` and the unit test use a subset (see the unit test); the full sweep above is
# used directly by the runner script and by ``__main__``.
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
    """Slender-bar inertia tensor of one chain segment about its centre of mass.

    Returns:
        numpy.ndarray: 3x3 inertia tensor [kg*m^2].
    """
    mass = segmentMass()
    length = 2.0*segmentHalfLength()  # [m]
    iBar = mass*length**2/12.0  # [kg*m^2]
    iAxial = max(iBar*0.05, 1.0e-4)  # [kg*m^2] small but well-posed transverse plate width
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


def buildClassic(integratorName, dt, tol):
    """Build the classic 16-segment flexible-array simulation.

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


def finalAttitude(builder, integratorName, dt, tol):
    """Propagate one configuration and return its final attitude MRP.

    Args:
        builder (callable): ``buildClassic`` or ``buildMujoco``.
        integratorName (str): integrator attribute name.
        dt (float): task time step [s].
        tol (float): adaptive tolerance, or None.

    Returns:
        numpy.ndarray: final ``sigma_BN`` (shape ``(3,)``), or None if the run diverged.
    """
    try:
        scSim, recorder, _ = builder(integratorName, dt, tol)
        scSim.ConfigureStopTime(macros.sec2nano(SIM_DURATION))
        scSim.ExecuteSimulation()
    except Exception:
        # MuJoCo raises on a NaN acceleration; treat as divergence for this config.
        return None
    sigma = np.array(recorder.sigma_BN)[-1]
    if not np.all(np.isfinite(sigma)):
        return None
    return sigma


def timedRun(builder, integratorName, dt, tol):
    """Best-of-``TIMING_REPEATS`` wall-clock time for one configuration [s].

    Args:
        builder (callable): ``buildClassic`` or ``buildMujoco``.
        integratorName (str): integrator attribute name.
        dt (float): task time step [s].
        tol (float): adaptive tolerance, or None.

    Returns:
        float: minimum wall-clock time over the repeats [s].
    """
    best = np.inf
    for _ in range(TIMING_REPEATS):
        scSim, _, _ = builder(integratorName, dt, tol)
        start = time.perf_counter()
        scSim.ConfigureStopTime(macros.sec2nano(SIM_DURATION))
        scSim.ExecuteSimulation()
        best = min(best, time.perf_counter()-start)
    return best


def principalAngle(sigmaA, sigmaB):
    """Principal rotation angle between two attitudes given as MRPs [rad].

    The angle is obtained from the relative MRP as ``4*atan(|sigma_rel|)``, which is well
    conditioned down to machine precision, unlike ``arccos((trace(C)-1)/2)`` (whose
    argument approaches 1 for small angles and collapses to zero below ~1e-8 rad).
    """
    dcmRel = rbk.MRP2C(sigmaA).dot(rbk.MRP2C(sigmaB).T)
    sigmaRel = rbk.C2MRP(dcmRel)
    return float(4.0*np.arctan(np.linalg.norm(sigmaRel)))


def sweepEngine(builder, reference=REFERENCE, referenceCheck=None):
    """Run the reference and every sweep configuration for one engine.

    Args:
        builder (callable): ``buildClassic`` or ``buildMujoco``.
        reference (dict): reference (truth) configuration with keys ``integrator``,
            ``dt`` and ``tol``.
        referenceCheck (dict): a second, independent ultra-accurate configuration whose
            disagreement with the reference estimates the reference's own error (the
            study's accuracy floor). Defaults to :data:`REFERENCE_CHECK`.

    Returns:
        tuple: ``(rows, referenceFloor)`` -- one dict per successful configuration and the
        estimated reference accuracy [rad]. Reported errors at or below the floor are not
        meaningful (they only show the configuration matched the reference to round-off).
    """
    if referenceCheck is None:
        referenceCheck = REFERENCE_CHECK
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
            continue  # diverged: stability limit exceeded for this configuration
        error = principalAngle(sigma, referenceSigma)
        wall = timedRun(builder, integratorName, dt, tol)
        rows.append({
            "integrator": integratorName,
            "dt": dt,
            "tol": tol,
            "error": error,
            "wall": wall,
        })
    return rows, referenceFloor


def paretoFrontier(rows):
    """Return the non-dominated subset, sorted by increasing wall-clock time.

    Args:
        rows (list): per-configuration dicts with ``error`` and ``wall``.

    Returns:
        list: the non-dominated configurations.
    """
    ordered = sorted(rows, key=lambda r: (r["wall"], r["error"]))
    frontier = []
    bestError = np.inf
    for row in ordered:
        if row["error"] < bestError:
            frontier.append(row)
            bestError = row["error"]
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
        sweepConfigs (list, optional): override the integrator sweep (used by the unit
            test to run a reduced set). Defaults to the full :data:`SWEEP_CONFIGS`.
        simDuration (float, optional): override the propagation horizon [s] (used by the
            unit test to shorten the run). Defaults to :data:`SIM_DURATION`.
        reference (dict, optional): override the reference (truth) configuration (used by
            the unit test to pick a cheaper reference). Defaults to :data:`REFERENCE`.
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

    classicRows, classicFloor = sweepEngine(buildClassic, referenceConfig, checkConfig)
    if couldImportMujoco:
        mujocoRows, mujocoFloor = sweepEngine(buildMujoco, referenceConfig, checkConfig)
    else:
        mujocoRows, mujocoFloor = [], 0.0
    referenceFloor = max(classicFloor, mujocoFloor)

    if saveJson:
        import json
        os.makedirs(resultsPath, exist_ok=True)
        with open(os.path.join(resultsPath, fileName+".json"), "w") as f:
            json.dump({"scenario": fileName, "nSegments": N_SEGMENTS,
                       "referenceFloor": referenceFloor,
                       "classic": classicRows, "mujoco": mujocoRows}, f, indent=2)

    figureList = plotResults(classicRows, mujocoRows, referenceFloor)

    if showPlots:
        plt.show()
    plt.close("all")

    return figureList


def plotResults(classicRows, mujocoRows, referenceFloor=0.0):
    """Build the Pareto and frontier figures.

    Args:
        classicRows (list): classic-engine sweep results.
        mujocoRows (list): MuJoCo-engine sweep results (possibly empty).
        referenceFloor (float): estimated reference accuracy [rad] (recorded in the JSON
            summary; not drawn on the figures).

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    figureList = {}

    # The legend is placed outside the axes so it never overlaps the data points.
    fig, ax = plt.subplots(figsize=(8, 4))
    figureList[fileName+"_pareto"] = fig
    seenLabels = set()
    for rows, engineLabel, filled in (
            (classicRows, "classic", True), (mujocoRows, "mujoco", False)):
        for row in rows:
            color, marker, intLabel = INTEGRATOR_STYLE[row["integrator"]]
            label = None
            key = (row["integrator"], engineLabel)
            if key not in seenLabels:
                label = f"{intLabel} ({engineLabel})"
                seenLabels.add(key)
            ax.scatter(row["wall"], row["error"],
                       marker=marker, s=45,
                       facecolors=color if filled else "none",
                       edgecolors=color, label=label)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Wall-clock time [s]")
    ax.set_ylabel("Final attitude error [rad]")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=7, ncol=1, loc="center left", bbox_to_anchor=(1.02, 0.5))
    fig.tight_layout()

    fig, ax = plt.subplots(figsize=(8, 4))
    figureList[fileName+"_frontier"] = fig
    for rows, engineLabel, color in (
            (classicRows, "Classic (back-substitution)", unitTestSupport.getLineColor(0, 3)),
            (mujocoRows, "MuJoCo (MJScene)", unitTestSupport.getLineColor(1, 3))):
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
