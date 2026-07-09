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
Fourth and final scenario in the dynamics-engine comparison series (see
:ref:`scenarioCompareOrbit` for the introduction).

This scenario approximates structural flexibility by discretizing each of two solar
arrays into a serial chain of ``N`` rigid segments connected by torsional spring-damper
hinges. Sweeping ``N`` varies the model dimensionality (the state size grows with the
number of hinges) and shows how each engine's wall-clock cost scales with complexity.

On the BSM side each array is a :ref:`nHingedRigidBodyStateEffector` with ``N``
panels added through ``addHingedPanel``. On the MuJoCo side each array is a serial chain
of ``N`` ``hinge``-jointed child bodies. To match the BSM chain geometry, each
segment's center of mass is offset by ``-d`` along its local x-axis and the next hinge
attaches at the segment's far end (``-2d``); the spring stiffness and damping are applied
on each joint.

The default sweep is ``N = 1, 2, 4, 8, 16, 32``. The integration time step scales with
``N`` because segment stiffness grows as the segments shrink, so a fixed step adequate
for a few large segments would under-resolve the many-segment cases. The timing
measurement uses a fixed step budget at each ``N`` so the wall-clock reflects per-step
cost as a function of model size. Pass a smaller ``nSegmentsList`` (as the unit test
does) to keep the runtime short.

The script is found in the folder ``basilisk/examples/dynamicsComparison`` and executed
by using::

    python3 scenarioCompareFlexPanels.py

Illustration of Simulation Results
----------------------------------

The two engines agree to round-off at every dimensionality in the sweep.

.. image:: /_images/Scenarios/scenarioCompareFlexPanels_validity.svg
   :align: center

The wall-clock cost of both engines grows with the number of flexible degrees of
freedom, with the MuJoCo recursive solver scaling more favorably.

.. image:: /_images/Scenarios/scenarioCompareFlexPanels_runtime.svg
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

# Standard Basilisk plotting colors.
COLOR_BSM = unitTestSupport.getLineColor(0, 3)
COLOR_MUJOCO = unitTestSupport.getLineColor(1, 3)

fileName = os.path.basename(os.path.splitext(__file__)[0])

# Folder this scenario writes its JSON summary into.
resultsPath = os.path.join(os.path.dirname(__file__), "results")

HUB_MASS = 600.0  # [kg]
HUB_INERTIA = (400.0, 380.0, 360.0)  # [kg*m^2]
PANEL_TOTAL_MASS = 60.0  # [kg] total mass of one array, split across its segments
PANEL_TOTAL_LEN = 3.0  # [m] total length of one array
PANEL_K = 600.0  # [N*m/rad] per-hinge torsional stiffness
PANEL_C = 5.0  # [N*m*s/rad] per-hinge damping
PANEL_HINGE_X = 0.8  # [m] root-hinge offset from hub origin along x
ROOT_THETA0 = 8.0*macros.D2R  # [rad] initial deflection of the root segment
OMEGA0_B = (0.010, -0.020, 0.015)  # [rad/s] initial hub rate
STEPS_PER_PERIOD = 200.0  # target RK4 steps per stiffest segment oscillation
TIMING_STEPS = 600  # integrator steps used for the wall-clock measurement
TIMING_TRIALS = 5  # number of trials for median/min/std reporting
TIMING_WARMUP_STEPS = 100  # discarded warmup steps that absorb first-time process costs


def segmentMass(nSegments):
    """Mass of one chain segment [kg]."""
    return PANEL_TOTAL_MASS/nSegments


def segmentHalfLength(nSegments):
    """Half-length ``d`` of one chain segment [m]."""
    return PANEL_TOTAL_LEN/(2.0*nSegments)


def segmentInertia(nSegments):
    """Slender-bar inertia tensor of one chain segment about its center of mass.

    Args:
        nSegments (int): number of segments per chain

    Returns:
        numpy.ndarray: 3x3 inertia tensor [kg*m^2].
    """
    mass = segmentMass(nSegments)
    length = 2.0*segmentHalfLength(nSegments)  # [m]
    iBar = mass*length**2/12.0  # [kg*m^2]
    iAxial = max(iBar*0.05, 1.0e-4)  # [kg*m^2] small but nonzero axial inertia for a well-posed tensor
    return np.diag([iAxial, iBar, iBar])


def timeStep(nSegments):
    """Integrator step that resolves the stiffest segment oscillation [s].

    Args:
        nSegments (int): number of segments per chain

    Returns:
        float: integrator time step [s].
    """
    halfLen = segmentHalfLength(nSegments)
    effInertia = segmentInertia(nSegments)[1, 1] + segmentMass(nSegments)*halfLen**2
    naturalPeriod = 2.0*np.pi*np.sqrt(effInertia/PANEL_K)  # [s]
    return min(0.01, naturalPeriod/STEPS_PER_PERIOD)


def mujocoModel(nSegments):
    """Return the MJCF model: hub with two ``N``-segment hinged chains.

    Args:
        nSegments (int): number of segments per chain

    Returns:
        str: MJCF XML string.
    """
    ix, iy, iz = HUB_INERTIA
    halfLen = segmentHalfLength(nSegments)
    mass = segmentMass(nSegments)
    inertia = segmentInertia(nSegments)

    def chain(sign, tag):
        body = ""
        for i in reversed(range(nSegments)):
            jointName = f"{tag}_{i}"
            # The root segment is offset from the hub origin; each subsequent segment
            # attaches at the previous segment's far end (-2d), matching the BSM
            # nHingedRigidBody convention where the center of mass sits at -d.
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


def buildBSM(nSegments, dt, record):
    """Build (and initialize) the back-substitution (BSM) chained-panel simulation.

    Args:
        nSegments (int): number of segments per chain
        dt (float): integrator time step [s]
        record (bool): if True, attach a hub-state recorder

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

    integrator = svIntegrators.svIntegratorRK4(scObject)
    scObject.setIntegrator(integrator)

    inertia = segmentInertia(nSegments).tolist()
    chains = []
    for sign in (+1, -1):
        effector = nHingedRigidBodyStateEffector.NHingedRigidBodyStateEffector()
        effector.ModelTag = f"chain{'P' if sign > 0 else 'M'}"
        effector.r_HB_B = [[sign*PANEL_HINGE_X], [0.0], [0.0]]  # [m]
        effector.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        for i in range(nSegments):
            panel = nHingedRigidBodyStateEffector.HingedPanel()
            panel.mass = segmentMass(nSegments)  # [kg]
            panel.d = segmentHalfLength(nSegments)  # [m]
            panel.k = PANEL_K  # [N*m/rad]
            panel.c = PANEL_C  # [N*m*s/rad]
            panel.IPntS_S = inertia  # [kg*m^2]
            panel.thetaInit = ROOT_THETA0 if i == 0 else 0.0  # [rad]
            effector.addHingedPanel(panel)
        scObject.addStateEffector(effector)
        scSim.AddModelToTask("dynTask", effector)
        chains.append(effector)

    recorder = None
    if record:
        recorder = scObject.scStateOutMsg.recorder(macros.sec2nano(dt))
        scSim.AddModelToTask("dynTask", recorder)

    scSim.InitializeSimulation()
    return scSim, recorder, [scObject, integrator] + chains


def buildMujoco(nSegments, dt, record):
    """Build (and initialize) the MuJoCo chained-panel simulation.

    Args:
        nSegments (int): number of segments per chain
        dt (float): integrator time step [s]
        record (bool): if True, attach a hub-state recorder

    Returns:
        tuple: ``(scSim, recorder, handles)``.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scene = mujoco.MJScene(mujocoModel(nSegments))
    scene.ModelTag = "hubMj"
    scene.extraEoMCall = True
    # Integrate the hub free-joint quaternion at the integrator's full order: panel-driven
    # nutation continually changes the hub rate direction, so MuJoCo's default
    # second-order SO(3) attitude step would dominate the cross-engine difference.
    scene.highOrderAttitudeIntegration = True
    scSim.AddModelToTask("dynTask", scene, 1)

    integrator = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integrator)

    hub = scene.getBody("hub")

    recorder = None
    if record:
        recorder = hub.getOrigin().stateOutMsg.recorder(macros.sec2nano(dt))
        scSim.AddModelToTask("dynTask", recorder, 0)

    scSim.InitializeSimulation()
    hub.setAttitudeRate(list(OMEGA0_B))
    for tag in ("pP_0", "pM_0"):
        scene.getBody(tag).getScalarJoint(tag).setPosition(ROOT_THETA0)
    return scSim, recorder, [scene, integrator, hub]


def relativePrincipalAngle(sigmaBSM, sigmaMujoco):
    """Per-sample principal rotation angle between two MRP attitude histories.

    Args:
        sigmaBSM (numpy.ndarray): BSM MRP history, shape ``(N, 3)``
        sigmaMujoco (numpy.ndarray): MuJoCo MRP history, shape ``(N, 3)``

    Returns:
        numpy.ndarray: principal angle per sample [rad].
    """
    nSamples = min(len(sigmaBSM), len(sigmaMujoco))
    angle = np.empty(nSamples)
    for i in range(nSamples):
        dcmRel = rbk.MRP2C(sigmaBSM[i]).dot(rbk.MRP2C(sigmaMujoco[i]).T)
        # Use 4*atan(|sigma_rel|), which is well conditioned down to machine precision;
        # arccos((trace-1)/2) collapses to zero for angles below ~1e-8 rad.
        angle[i] = 4.0*np.arctan(np.linalg.norm(rbk.C2MRP(dcmRel)))
    return angle


def measureWallClock(buildFunc, nSegments, dt):
    """Time a fixed step budget for one engine at the given dimensionality.

    Args:
        buildFunc (callable): ``buildBSM`` or ``buildMujoco``
        nSegments (int): number of segments per chain
        dt (float): integrator time step [s]

    Returns:
        dict: ``{"median", "min", "std"}`` wall-clock over the trials [s].
    """
    stopNs = macros.sec2nano(TIMING_STEPS*dt)
    # Warmup (discarded): absorbs one-time process costs (cold code/data caches, first
    # allocations, bytecode) so the measured trials reflect steady-state per-step cost.
    if TIMING_WARMUP_STEPS > 0:
        warm, _, _ = buildFunc(nSegments, dt, False)
        warm.ConfigureStopTime(macros.sec2nano(TIMING_WARMUP_STEPS*dt))
        warm.ExecuteSimulation()
    times = []
    for _ in range(TIMING_TRIALS):
        scSim, _, _ = buildFunc(nSegments, dt, False)
        scSim.ConfigureStopTime(stopNs)
        start = time.perf_counter()
        scSim.ExecuteSimulation()
        times.append(time.perf_counter() - start)
    return {"median": float(np.median(times)), "min": float(np.min(times)),
            "std": float(np.std(times))}


def runOne(nSegments, accuracyWindow):
    """Validate accuracy and benchmark a single dimensionality.

    Args:
        nSegments (int): number of segments per chain
        accuracyWindow (float): duration of the accuracy comparison [s]

    Returns:
        dict: metrics for this ``N`` (DOF, attitude error, wall-clock per engine).
    """
    dt = timeStep(nSegments)  # [s]

    bsmSim, bsmRec, _ = buildBSM(nSegments, dt, True)
    bsmSim.ConfigureStopTime(macros.sec2nano(accuracyWindow))
    bsmSim.ExecuteSimulation()
    sigmaBSM = np.array(bsmRec.sigma_BN)

    row = {"nSegments": nSegments, "dof": 6 + 2*2*nSegments, "timeStep": dt}
    stats = measureWallClock(buildBSM, nSegments, dt)
    row["bsmWall"], row["bsmWallMin"], row["bsmWallStd"] = (
        stats["median"], stats["min"], stats["std"])

    if couldImportMujoco:
        mujocoSim, mujocoRec, _ = buildMujoco(nSegments, dt, True)
        mujocoSim.ConfigureStopTime(macros.sec2nano(accuracyWindow))
        mujocoSim.ExecuteSimulation()
        sigmaMujoco = np.array(mujocoRec.sigma_BN)
        attError = relativePrincipalAngle(sigmaBSM, sigmaMujoco)
        row["attitudeErrorMax"] = float(np.max(attError))
        stats = measureWallClock(buildMujoco, nSegments, dt)
        row["mujocoWall"], row["mujocoWallMin"], row["mujocoWallStd"] = (
            stats["median"], stats["min"], stats["std"])
    return row


def run(showPlots=False, saveJson=False, nSegmentsList=(1, 2, 4, 8, 16, 32)):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): if True, plot and show the simulation results.
            Defaults to False.
        saveJson (bool, optional): if True, write the comparison metrics to
            ``results/scenarioCompareFlexPanels.json``. Defaults to False.
        nSegmentsList (tuple, optional): per-chain segment counts to sweep. Defaults to
            ``(1, 2, 4, 8, 16, 32)``. The unit test passes a smaller sweep to keep the
            runtime short.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    accuracyWindow = 8.0  # [s]
    rows = [runOne(nSegments, accuracyWindow) for nSegments in nSegmentsList]

    if saveJson:
        import json
        os.makedirs(resultsPath, exist_ok=True)
        with open(os.path.join(resultsPath, fileName+".json"), "w") as f:
            json.dump({"scenario": fileName, "rows": rows}, f, indent=2)

    figureList = plotResults(rows)

    if showPlots:
        plt.show()
    plt.close("all")

    return figureList


def plotResults(rows):
    """Build the scenario figures.

    Args:
        rows (list): per-``N`` metric dictionaries from :func:`runOne`

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    dof = np.array([row["dof"] for row in rows])
    figureList = {}

    figureList[fileName+"_validity"], ax = plt.subplots()
    if "attitudeErrorMax" in rows[0]:
        attError = np.array([row["attitudeErrorMax"] for row in rows])
        ax.semilogy(dof, np.maximum(attError, 1e-16), "o-", color=COLOR_MUJOCO)
    ax.set_xlabel("System degrees of freedom")
    ax.set_ylabel("Max cross-engine attitude error [rad]")

    # Draw BSM as a thick translucent underlay and MuJoCo as a thin line on top, with
    # distinct markers, so the two stay distinguishable even where the curves coincide.
    figureList[fileName+"_runtime"], ax = plt.subplots()
    ax.loglog(dof, [row["bsmWall"] for row in rows], "o-", lw=4, alpha=0.4, ms=8,
              color=COLOR_BSM, label="Back-substitution (BSM)")
    if "mujocoWall" in rows[0]:
        ax.loglog(dof, [row["mujocoWall"] for row in rows], "s-", lw=1.3, ms=5,
                  color=COLOR_MUJOCO, label="MuJoCo")
    ax.set_xlabel("System degrees of freedom")
    ax.set_ylabel("Wall-clock for fixed step budget [s]")
    ax.legend(loc="best")

    return figureList


if __name__ == "__main__":
    run(True, False)
