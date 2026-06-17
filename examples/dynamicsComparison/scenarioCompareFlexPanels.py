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
hinges. By sweeping ``N`` we vary the model dimensionality (the state size grows
with the number of hinges) and observe how each engine's wall-clock cost scales with
complexity.

On the classic side each array is a :ref:`nHingedRigidBodyStateEffector` with ``N``
panels added through ``addHingedPanel``. On the MuJoCo side each array is a serial chain
of ``N`` ``hinge``-jointed child bodies. To match the classic chain geometry, each
segment's centre of mass is offset by ``-d`` along its local x-axis and the next hinge
attaches at the segment's far end (``-2d``); the spring stiffness and damping are applied
on each joint.

The default sweep is ``N = 1, 2, 4, 8, 16, 32`` and the integration time step is
automatically scaled with ``N`` so that the stiffest segment oscillation is resolved on
both engines (otherwise a fixed step that is fine for a few large segments becomes
under-resolved for many small, stiff ones, and both propagators would diverge). The
timing measurement uses a fixed step budget at each ``N`` so the wall-clock reflects the
per-step cost as a function of model size. The ``nSegmentsList`` argument can be set to a
smaller sweep (as the unit test does) to keep the runtime short.

The script is found in the folder ``basilisk/examples/dynamicsComparison`` and executed
by using::

    python3 scenarioCompareFlexPanels.py

Illustration of Simulation Results
----------------------------------

The two engines agree to round-off at every dimensionality in the sweep.

.. image:: /_images/Scenarios/scenarioCompareFlexPanels_validity.svg
   :align: center

The wall-clock cost of both engines grows with the number of flexible degrees of
freedom, with the MuJoCo recursive solver scaling more favourably.

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

# Consistent palette drawn from the standard Basilisk plotting colors.
COLOR_CLASSIC = unitTestSupport.getLineColor(0, 3)
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


def segmentMass(nSegments):
    """Mass of one chain segment [kg]."""
    return PANEL_TOTAL_MASS/nSegments


def segmentHalfLength(nSegments):
    """Half-length ``d`` of one chain segment [m]."""
    return PANEL_TOTAL_LEN/(2.0*nSegments)


def segmentInertia(nSegments):
    """Slender-bar inertia tensor of one chain segment about its centre of mass.

    Args:
        nSegments (int): number of segments per chain

    Returns:
        numpy.ndarray: 3x3 inertia tensor [kg*m^2].
    """
    mass = segmentMass(nSegments)
    length = 2.0*segmentHalfLength(nSegments)  # [m]
    iBar = mass*length**2/12.0  # [kg*m^2]
    iAxial = max(iBar*0.05, 1.0e-4)  # [kg*m^2] small but well-posed transverse plate width
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
            # attaches at the previous segment's far end (-2d), matching the classic
            # nHingedRigidBody convention where the centre of mass sits at -d.
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


def buildClassic(nSegments, dt, record):
    """Build (and initialize) the classic chained-panel simulation.

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
    # Integrate the hub free-joint quaternion at the integrator's full order. Panel-driven
    # nutation continually changes the hub rate direction, which is exactly the regime
    # where MuJoCo's default second-order SO(3) attitude step would otherwise dominate the
    # cross-engine difference.
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


def relativePrincipalAngle(sigmaClassic, sigmaMujoco):
    """Per-sample principal rotation angle between two MRP attitude histories.

    Args:
        sigmaClassic (numpy.ndarray): classic MRP history, shape ``(N, 3)``
        sigmaMujoco (numpy.ndarray): MuJoCo MRP history, shape ``(N, 3)``

    Returns:
        numpy.ndarray: principal angle per sample [rad].
    """
    nSamples = min(len(sigmaClassic), len(sigmaMujoco))
    angle = np.empty(nSamples)
    for i in range(nSamples):
        dcmRel = rbk.MRP2C(sigmaClassic[i]).dot(rbk.MRP2C(sigmaMujoco[i]).T)
        # Use 4*atan(|sigma_rel|), which is well conditioned down to machine precision;
        # arccos((trace-1)/2) collapses to zero for angles below ~1e-8 rad.
        angle[i] = 4.0*np.arctan(np.linalg.norm(rbk.C2MRP(dcmRel)))
    return angle


def measureWallClock(buildFunc, nSegments, dt):
    """Time a fixed step budget for one engine at the given dimensionality.

    Args:
        buildFunc (callable): ``buildClassic`` or ``buildMujoco``
        nSegments (int): number of segments per chain
        dt (float): integrator time step [s]

    Returns:
        float: best-of-two wall-clock time for ``TIMING_STEPS`` steps [s].
    """
    best = np.inf
    for _ in range(2):
        scSim, _, _ = buildFunc(nSegments, dt, False)
        start = time.perf_counter()
        scSim.ConfigureStopTime(macros.sec2nano(TIMING_STEPS*dt))
        scSim.ExecuteSimulation()
        best = min(best, time.perf_counter()-start)
    return best


def runOne(nSegments, accuracyWindow):
    """Validate accuracy and benchmark a single dimensionality.

    Args:
        nSegments (int): number of segments per chain
        accuracyWindow (float): duration of the accuracy comparison [s]

    Returns:
        dict: metrics for this ``N`` (DOF, attitude error, wall-clock per engine).
    """
    dt = timeStep(nSegments)  # [s]

    classicSim, classicRec, _ = buildClassic(nSegments, dt, True)
    classicSim.ConfigureStopTime(macros.sec2nano(accuracyWindow))
    classicSim.ExecuteSimulation()
    sigmaClassic = np.array(classicRec.sigma_BN)

    row = {"nSegments": nSegments, "dof": 6 + 2*2*nSegments, "timeStep": dt}
    row["classicWall"] = measureWallClock(buildClassic, nSegments, dt)

    if couldImportMujoco:
        mujocoSim, mujocoRec, _ = buildMujoco(nSegments, dt, True)
        mujocoSim.ConfigureStopTime(macros.sec2nano(accuracyWindow))
        mujocoSim.ExecuteSimulation()
        sigmaMujoco = np.array(mujocoRec.sigma_BN)
        attError = relativePrincipalAngle(sigmaClassic, sigmaMujoco)
        row["attitudeErrorMax"] = float(np.max(attError))
        row["mujocoWall"] = measureWallClock(buildMujoco, nSegments, dt)
    return row


def run(showPlots=False, saveJson=False, nSegmentsList=(1, 2, 4, 8, 16, 32)):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): if True, plot and show the simulation results.
            Defaults to False.
        saveJson (bool, optional): if True, write the comparison metrics to
            ``results/scenarioCompareFlexPanels.json``. Defaults to False.
        nSegmentsList (tuple, optional): per-chain segment counts to sweep. Defaults to
            ``(1, 2, 4, 8, 16, 32)``. A smaller sweep can be passed (for example by the
            unit test) to keep the runtime short.

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

    # Draw classic as a thick translucent underlay and MuJoCo as a thin line on top, with
    # distinct markers, so the two stay distinguishable even where the curves coincide.
    figureList[fileName+"_runtime"], ax = plt.subplots()
    ax.loglog(dof, [row["classicWall"] for row in rows], "o-", lw=4, alpha=0.4, ms=8,
              color=COLOR_CLASSIC, label="Classic (back-substitution)")
    if "mujocoWall" in rows[0]:
        ax.loglog(dof, [row["mujocoWall"] for row in rows], "s-", lw=1.3, ms=5,
                  color=COLOR_MUJOCO, label="MuJoCo (MJScene)")
    ax.set_xlabel("System degrees of freedom")
    ax.set_ylabel("Wall-clock for fixed step budget [s]")
    ax.legend(loc="best")

    return figureList


if __name__ == "__main__":
    run(True, False)
