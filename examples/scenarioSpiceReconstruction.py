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
An example of the :ref:`SpiceInterface<spiceInterface>` ephemeris **reconstruction** feature
(see :ref:`spiceReconstruction`), on a cislunar transfer that is targeted for a deep lunar flyby:
the transfer apogee is aimed at the Moon and pulled slightly short, so the spacecraft passes about
18000 km from the Moon, well inside its sphere of influence, and its trajectory is sharply bent by
the encounter. That makes the Moon's position the dominant driver of the orbit, so how it is
supplied to the gravity model is a significant accuracy knob. The scenario shows that reconstruction trades
SPICE queries for accuracy, and that below the integrator's own error the trade is free.

The **reference solution** every error is measured against is exact SPICE queried at every RK4
sub-step, run on an :ref:`MJScene<MJScene>` dynamics task (the most accurate and most expensive
option). Against it the scenario compares:

#. **Reconstruction sweep**: the Moon position reconstructed from a coarse knot grid, sweeping
   ``knotStep`` to trace error versus SPICE query count.
#. **Default spacecraft setup**: a classic :ref:`spacecraft` with :ref:`gravityEffector`, where
   :ref:`spiceInterface` is queried at the task rate and :ref:`gravityEffector` linearly
   extrapolates the Moon across the sub-steps.

Even with a perfect ephemeris the RK4 integrator has its own truncation error at the study step.
The scenario measures it by rerunning the exact-SPICE case at a much finer step (``REFERENCE_DT``),
treating that as converged truth, and taking the difference: whatever error the study-step run still
carries is the integrator's. Reconstruction error below this level is masked by it and does not
change the orbit, which is what makes coarsening the knot grid free down to that point.

Run with::

    python3 scenarioSpiceReconstruction.py

Illustration of Results
-----------------------
The first figure is the trajectory itself, projected onto the plane of the transfer. It plots the
spacecraft path and the Moon's path over the same leg, with the closest-approach points marked. The
spacecraft coasts out from perigee, turns around about 18000 km from the Moon, and its arc is
visibly bent by the encounter. This is the setup for everything that follows: because the flyby
dominates the final state, the Moon's position is the quantity the answer is most sensitive to, so
how often and how accurately it is supplied to the gravity model is what the rest of the study
measures.

.. figure:: /_images/Scenarios/scenarioSpiceReconstruction1.svg
   :align: center
   :scale: 90 %

The second figure is the accuracy-vs-cost tradeoff. The x-axis is the number of SPICE queries the
Moon costs over the whole run (compute cost); the y-axis is the resulting error in the final
spacecraft position, measured against the exact-SPICE reference. Each blue point is one
reconstruction run at a different knot spacing: coarser grids sit to the left (fewer queries, more
error). The horizontal line is the integrator's own error at this step size, and the shaded band
below it is the "free" zone, where reconstruction error is smaller than the integrator error and so
does not change the orbit. The reconstruction curve drops into that band after only a few tens of
queries, so coarsening the grid further buys nothing. For comparison, the exact-SPICE reference sits
at the far right at its full query cost (zero error by definition, on the broken "0" tick), and the
:ref:`spacecraft` + :ref:`gravityEffector` default sits *above* the band: a handful of
reconstruction knots is both cheaper and more accurate than that default behaviour.

.. figure:: /_images/Scenarios/scenarioSpiceReconstruction2.svg
   :align: center
   :scale: 90 %

The third figure shows the position error buildup against time for three cases: the integrator error (the black
curve, the run with a perfect ephemeris, which grows over the leg and jumps at the flyby), one cheap
reconstruction taken from the free zone of the second figure, and the :ref:`spacecraft` default. The shaded
region is again the free zone, now time-varying because the integrator error grows. The
reconstruction stays inside that zone for nearly the whole leg on a handful of Moon queries, while
the :ref:`spacecraft` default climbs above it early in the coast and grows to tens of kilometres.

.. figure:: /_images/Scenarios/scenarioSpiceReconstruction3.svg
   :align: center
   :scale: 90 %

Does it actually save wall-clock time? The scenario prints the runtime of each run. The cleanest
way to read the effect is to hold the dynamics engine fixed and vary only SPICE: the classic
:ref:`spacecraft` + :ref:`gravityEffector` path with exact SPICE (one Moon query per step) versus
the same path with Moon-position reconstruction. On the author's machine, collapsing the Moon
queries from ~15600 to a handful cuts the classic-path runtime by roughly **5%**. On the
:ref:`MJScene<MJScene>` dynamics task the saving is larger, on the order of **10%**, because that
path queries SPICE four times per step (once per RK4 stage) rather than once, so SPICE is a bigger
share of the step.
"""
import os
import time

import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass, macros, simHelpers, simIncludeGravBody
from Basilisk.simulation import mujoco, svIntegrators, spacecraft
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile

fileName = os.path.splitext(os.path.basename(__file__))[0]

# scenario configuration
EPOCH = "2025 NOVEMBER 15 12:00:00.000"
RK4_DT = 30.0                 # [s] integrator / task step for the study
N_STEPS = 15600               # horizon in steps -> TF = 130 h, past the lunar flyby at ~115 h
TF = RK4_DT * N_STEPS
REFERENCE_DT = 2.0            # [s] fine RK4 step whose exact-SPICE run is the integrator-converged truth
# Apogee radius as a fraction of the Moon's distance at arrival. Aiming the transfer apogee at the
# Moon gives a bullseye impact; pulling it slightly short (0.94) makes the spacecraft turn around
# ~18000 km from the Moon, a deep pass inside its ~66000 km sphere of influence that sharply bends
# the trajectory without being captured.
APOGEE_FRACTION = 0.94
# Reconstruction knot spacings to sweep, as integer multiples of the task step. The range is
# chosen so the reconstruction-error curve crosses the integrator error floor: fine grids stay
# exact, coarse grids rise above it.
KNOT_MULTIPLES = [64, 256, 512, 1024, 1536, 2048, 3072, 4096, 6144]

# A minimal free-flying point mass. Its own shape/mass are irrelevant; only the orbit matters.
SC_XML = r"""
<mujoco>
    <worldbody>
        <body name="sc"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>
    </worldbody>
</mujoco>
"""

# standard Basilisk scenario plotting colours (gist_earth map, as the other scenarios use)
C_RECON = simHelpers.getLineColor(0, 3)
C_CLASSIC = simHelpers.getLineColor(1, 3)
C_EXACT = simHelpers.getLineColor(2, 3)


def runMJScene(rN, vN, dt, knotStep=None, record=False):
    """Propagate the cislunar orbit in an MJScene dynamics task with Earth (central) + Moon third
    body at RK4 step ``dt``. ``knotStep`` (nanoseconds), if given, reconstructs the Moon position
    with cubic Hermite on that knot grid; otherwise the Moon is queried exactly at every RK4
    sub-step. Returns (final position, Moon SPICE query count,
    optional [times, spacecraft positions, Moon positions])."""
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("simProcess")
    dynProcess.addTask(scSim.CreateNewTask("simTask", macros.sec2nano(dt)))

    scene = mujoco.MJScene(SC_XML)
    scene.ModelTag = "cislunarScene"
    scSim.AddModelToTask("simTask", scene)
    integratorObject = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integratorObject)
    body = scene.getBody("sc")

    # Earth (central body at the origin) and the Moon are both produced by one SpiceInterface.
    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravFactory.createEarth().isCentralBody = True
    gravFactory.createMoon()
    spiceObject = gravFactory.createSpiceInterface(time=EPOCH)
    spiceObject.zeroBase = "Earth"
    if knotStep is not None:
        spiceObject.setPlanetPositionReconstruction("moon", int(knotStep))

    # Add the SPICE producer to the dynamics task first so planet states are current at every
    # sub-step, then let the factory build the NBodyGravity model: it registers Earth and the Moon
    # as gravity sources (reusing their gravity models and SPICE connections) and applies gravity
    # at the MuJoCo spacecraft. addBodiesTo returns the model, which must stay referenced.
    scene.AddModelToDynamicsTask(spiceObject)
    gravity = gravFactory.addBodiesTo(scene)

    scRec = body.getOrigin().stateOutMsg.recorder() if record else None
    moonRec = spiceObject.planetStateOutMsgs[1].recorder() if record else None
    if record:
        scSim.AddModelToTask("simTask", scRec)
        scSim.AddModelToTask("simTask", moonRec)

    scSim.InitializeSimulation()
    body.setPosition(list(rN)); body.setVelocity(list(vN))
    scSim.ConfigureStopTime(macros.sec2nano(TF))
    scSim.ExecuteSimulation()

    rFinal = np.array(body.getOrigin().stateOutMsg.read().r_BN_N)
    nQ = int(spiceObject.getPlanetSpiceQueryCount("moon")[0])
    gravFactory.unloadSpiceKernels()
    if record:
        return rFinal, nQ, [np.array(scRec.times()) * macros.NANO2SEC, np.array(scRec.r_BN_N),
                            np.array(moonRec.PositionVector)]
    return rFinal, nQ, []


def runClassicSpacecraft(rN, vN, dt, knotStep=None, record=False):
    """Propagate the same orbit with a classic Spacecraft and gravityEffector on a regular task at
    step ``dt``: SpiceInterface runs once per step and gravityEffector linearly extrapolates the
    Moon across the sub-steps. ``knotStep`` (nanoseconds), if given, reconstructs the Moon position
    on that knot grid instead of querying exact SPICE every step (same engine, SPICE the only change,
    for the apples-to-apples runtime comparison). Returns (final position, Moon query count,
    optional [times, spacecraft positions])."""
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("simProcess")
    dynProcess.addTask(scSim.CreateNewTask("simTask", macros.sec2nano(dt)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "cislunarSat"
    scSim.AddModelToTask("simTask", scObject)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    gravFactory.createEarth().isCentralBody = True
    gravFactory.createMoon()
    gravFactory.addBodiesTo(scObject)               # attaches the gravityEffector + both bodies
    spiceObject = gravFactory.createSpiceInterface(time=EPOCH)
    spiceObject.zeroBase = "Earth"
    if knotStep is not None:
        spiceObject.setPlanetPositionReconstruction("moon", int(knotStep))
    scSim.AddModelToTask("simTask", spiceObject)

    scRec = scObject.scStateOutMsg.recorder() if record else None
    if record:
        scSim.AddModelToTask("simTask", scRec)

    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(TF))
    scSim.ExecuteSimulation()

    rFinal = np.array(scObject.scStateOutMsg.read().r_BN_N)
    nQ = int(spiceObject.getPlanetSpiceQueryCount("moon")[0])
    gravFactory.unloadSpiceKernels()
    if record:
        return rFinal, nQ, [np.array(scRec.times()) * macros.NANO2SEC, np.array(scRec.r_BN_N)]
    return rFinal, nQ, []


def moonFlybyInitialState(muEarth, rEq):
    """Initial state of a transfer from a 600 km perigee out to a lunar flyby.

    The transfer apogee is aimed at where the Moon will be when the spacecraft arrives, in the
    Moon's orbit plane, then pulled short by ``APOGEE_FRACTION`` so the spacecraft turns around a
    controlled distance from the Moon rather than hitting it. The Moon state comes straight from
    SPICE (``spkezr_c``); the time of flight is the perigee-to-apogee half period, iterated to
    self-consistency because the apogee radius depends on where the Moon is at arrival. Returns
    the inertial (position, velocity) at perigee."""
    for kernel in (DataFile.EphemerisData.de430, DataFile.EphemerisData.naif0012):
        pyswice.furnsh_c(str(get_path(kernel)))
    etArray = pyswice.new_doubleArray(1)
    pyswice.str2et_c(EPOCH, etArray)
    et0 = pyswice.doubleArray_getitem(etArray, 0)

    def moonState(etSec):
        state = pyswice.new_doubleArray(6); lt = pyswice.new_doubleArray(1)
        pyswice.spkezr_c("MOON", etSec, "J2000", "NONE", "EARTH", state, lt)
        return np.array([pyswice.doubleArray_getitem(state, i) for i in range(6)]) * 1e3  # m, m/s

    rp = rEq + 600e3                                            # 600 km perigee altitude
    tof = 120.0 * 3600.0
    for _ in range(8):
        ra = APOGEE_FRACTION * np.linalg.norm(moonState(et0 + tof)[:3])
        a = (rp + ra) / 2.0
        tof = np.pi * np.sqrt(a ** 3 / muEarth)                 # perigee -> apogee half period

    arrival = moonState(et0 + tof)
    apoHat = arrival[:3] / np.linalg.norm(arrival[:3])          # apogee aimed at the arrival Moon
    hHat = np.cross(arrival[:3], arrival[3:])                   # transfer in the Moon's orbit plane
    hHat /= np.linalg.norm(hHat)
    rN = -rp * apoHat                                           # perigee is opposite the apogee
    vN = np.sqrt(muEarth * (2.0 / rp - 1.0 / a)) * np.cross(hHat, -apoHat)   # prograde at perigee
    pyswice.kclear_c()
    return rN, vN


def run(show_plots=True):
    """Run the reconstruction accuracy-vs-cost study and produce its three figures.

    Args:
        show_plots (bool): display the figures.

    Returns:
        tuple: ``(errClassic, figureList)`` where ``figureList`` maps figure name to Figure.
    """
    # cislunar transfer targeting a deep lunar flyby (perigee 600 km, apogee near the Moon)
    earth = simIncludeGravBody.gravBodyFactory().createEarth()
    rN, vN = moonFlybyInitialState(earth.mu, earth.radEquator)

    # truth and most-expensive reference: exact SPICE queried at every RK4 sub-step (wall-clock
    # timed so the reconstruction speedup can be reported end-to-end). getPlanetSpiceQueryCount
    # reports the real Moon-query count for the exact path too, so nQExact is measured, not assumed
    # (it comes to one spkezr_c per RK4 stage).
    t0 = time.perf_counter()
    rTruth, nQExact, (tTruth, posTruth, moonTruth) = runMJScene(rN, vN, RK4_DT, record=True)
    tExactRun = time.perf_counter() - t0

    # reference: classic Spacecraft + gravityEffector (SpiceInterface once per step)
    t0 = time.perf_counter()
    rClassic, nQClassic, (tClassic, posClassic) = runClassicSpacecraft(rN, vN, RK4_DT, record=True)
    tClassicRun = time.perf_counter() - t0
    errClassic = float(np.linalg.norm(rClassic - rTruth))

    # Integrator error over time: the exact-SPICE study step differenced against exact SPICE at the
    # fine REFERENCE_DT step (the fine run interpolated onto the coarse sample times). This is the
    # error the coarse integrator carries with a perfect ephemeris, so reconstruction error smaller
    # than it does not affect the orbit. It grows over the leg; its final value is the floor used to
    # pick the "free" reconstruction grid and to draw the free zone on the cost figure.
    _, _, (tFine, posFine, _) = runMJScene(rN, vN, REFERENCE_DT, record=True)
    fineAtCoarse = np.vstack([np.interp(tTruth, tFine, posFine[:, i]) for i in range(3)]).T
    integratorFloor_t = np.linalg.norm(posTruth - fineAtCoarse, axis=1)
    integratorFloor = float(integratorFloor_t[-1])

    # reconstruction sweep: coarsen the Moon knot grid and watch error vs. query count
    sweep = []
    for m in KNOT_MULTIPLES:
        r, nQ, _ = runMJScene(rN, vN, RK4_DT, knotStep=macros.sec2nano(m * RK4_DT))
        sweep.append(dict(mult=m, nQ=nQ, err=float(np.linalg.norm(r - rTruth))))

    print(f"[study] cislunar leg, RK4 dt={RK4_DT:g}s, horizon={TF/3600:.1f}h")
    print(f"[study] integrator error floor (exact SPICE, {RK4_DT:g}s vs {REFERENCE_DT:g}s): {integratorFloor:8.1f} m")
    print(f"[study] exact SPICE (per sub-step):  {nQExact:6d} Moon queries  (truth)")
    print(f"[study] classic sc + gravityEffector:{nQClassic:6d} Moon queries  err={errClassic:8.1f} m")
    for s in sweep:
        print(f"[study] reconstruct knot={s['mult']:4d}xdt:  {s['nQ']:6d} Moon queries  err={s['err']:8.1f} m")

    plt.rcParams.update({"font.size": 13, "lines.linewidth": 2.0, "lines.markersize": 8})
    figureList = {}
    figureList[fileName + "1"] = plotTrajectory(posTruth, moonTruth)
    figureList[fileName + "2"] = plotAccuracyVsCost(sweep, errClassic, integratorFloor,
                                                    nQExact, nQClassic)

    # Figure 3 needs one more run: the cheapest grid whose final error still lands below the floor.
    # Fall back to the finest grid swept if none clears the floor (keeps the figure well-defined).
    below = [s for s in sweep if s["err"] < integratorFloor]
    good = max(below, key=lambda s: s["mult"]) if below else min(sweep, key=lambda s: s["mult"])
    goodKnot = macros.sec2nano(good["mult"] * RK4_DT)
    t0 = time.perf_counter()
    _, nQGood, (tR, posR, _) = runMJScene(rN, vN, RK4_DT, knotStep=goodKnot, record=True)
    tReconRun = time.perf_counter() - t0

    # Apples-to-apples: the classic path with the same lossless reconstruction, so only SPICE
    # changes (same engine). This isolates the wall-clock effect of collapsing the Moon queries.
    t0 = time.perf_counter()
    runClassicSpacecraft(rN, vN, RK4_DT, knotStep=goodKnot)
    tClassicReconRun = time.perf_counter() - t0

    def _saving(exact, recon):
        return 100.0 * (1.0 - recon / exact)  # percent runtime removed
    print(f"[study] runtime classic path:  exact {tClassicRun:.2f}s -> reconstruction "
          f"{tClassicReconRun:.2f}s  ({_saving(tClassicRun, tClassicReconRun):.0f}% less, SPICE-only change)")
    print(f"[study] runtime MJScene path:  exact {tExactRun:.2f}s -> reconstruction "
          f"{tReconRun:.2f}s  ({_saving(tExactRun, tReconRun):.0f}% less, 4 SPICE calls/step -> few)")
    figureList[fileName + "3"] = plotErrorOverTime(tTruth, posTruth, integratorFloor_t,
                                                   tR, posR, nQGood,
                                                   tClassic, posClassic, nQClassic, errClassic)

    if show_plots:
        plt.show()
    else:
        plt.close("all")
    return errClassic, figureList


def plotTrajectory(posTruth, moonTruth):
    """Figure 1: the cislunar transfer in 2D, with the Moon that perturbs it.

    Both Earth-centred trajectories are projected onto the plane that best contains the spacecraft
    path (its two dominant principal directions) so the flyby geometry is planar and the deflection
    is easy to see, and the closest-approach point is marked on both paths.
    """
    _, _, principalDirs = np.linalg.svd(posTruth - posTruth.mean(axis=0), full_matrices=False)
    e1, e2 = principalDirs[0], principalDirs[1]
    scXY = np.column_stack([posTruth @ e1, posTruth @ e2]) / 1e3        # km, in-plane
    moonXY = np.column_stack([moonTruth @ e1, moonTruth @ e2]) / 1e3
    ca = int(np.argmin(np.linalg.norm(posTruth - moonTruth, axis=1)))   # closest-approach sample

    fig, ax = plt.subplots(figsize=(8, 7), num=fileName + "1", layout="constrained")
    ax.plot(scXY[:, 0], scXY[:, 1], color=C_RECON, lw=2.0, label="spacecraft trajectory", zorder=4)
    # only the latter half of the Moon's path, the stretch around the encounter
    moonTail = moonXY[len(moonXY) // 2:]
    ax.plot(moonTail[:, 0], moonTail[:, 1], color="0.55", lw=1.5, ls="--", label="Moon trajectory", zorder=3)
    ax.plot(0, 0, "o", color="0.2", ms=12, zorder=5, label="Earth")
    ax.plot(*scXY[0], "o", color=C_RECON, ms=8, zorder=6)              # spacecraft start (perigee)
    ax.plot(*moonXY[ca], "o", color="0.35", ms=16, zorder=6, label="Moon at closest approach")
    ax.plot(*scXY[ca], "o", color=C_EXACT, ms=9, markeredgecolor="k", zorder=7)
    ax.plot([scXY[ca, 0], moonXY[ca, 0]], [scXY[ca, 1], moonXY[ca, 1]], color="k", lw=1.0, ls=":", zorder=5)
    ax.set_xlabel("in-plane distance from Earth  [km]")
    ax.set_ylabel("in-plane distance from Earth  [km]")
    ax.set_aspect("equal")            # same scale on both axes; extents free to fit the data
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower left")
    return fig


def plotAccuracyVsCost(sweep, errClassic, integratorFloor, nQExact, nQClassic):
    """Figure 2: the accuracy-vs-cost tradeoff, with the references + integrator floor marked."""
    # A coarser knot grid never queries SPICE more, so different knot spacings can land on the same
    # query count; keep the lowest-error point per count so the curve is single-valued.
    byQ = {}
    for s in sweep:
        if s["nQ"] not in byQ or s["err"] < byQ[s["nQ"]]:
            byQ[s["nQ"]] = s["err"]
    nq = np.array(sorted(byQ))
    err = np.array([byQ[q] for q in nq])
    top = errClassic * 3.0
    plotFloor = 1e-6  # [m] bottom of the log panel; the finest grids are microns from the reference

    # Broken y-axis: a wide log panel for the error curve, and a thin panel below holding the single
    # "0" tick where the exact-SPICE reference lives (its error is zero by definition, which cannot
    # sit on a log axis).
    fig, (ax, axz) = plt.subplots(2, 1, figsize=(8, 6), num=fileName + "2",
                                  sharex=True, height_ratios=[12, 1], layout="constrained")
    ax.set_yscale("log")
    ax.set_ylim(plotFloor, top)

    # Grey "free" band spanning the whole panel below the black integrator-error floor line:
    # reconstruction error there does not move the orbit, so query cost to shrink it is wasted.
    ax.axhspan(ax.get_ylim()[0], integratorFloor, color="0.85", alpha=0.6, zorder=0)
    ax.axhline(integratorFloor, color="k", ls="-", lw=1.5, zorder=1,
               label=f"integrator error floor ({integratorFloor:.0f} m)")
    ax.loglog(nq, np.maximum(err, plotFloor), "-o", color=C_RECON, zorder=5,
              label="Moon reconstruction (cubic Hermite)")
    ax.plot(nQClassic, errClassic, "s", color=C_CLASSIC, ms=12, zorder=6,
            label="classic spacecraft + gravityEffector")
    # proxy handle so the exact-SPICE reference (drawn in the break panel) appears in the legend
    ax.plot([], [], "D", color=C_EXACT, ms=10, markeredgecolor="k",
            label=f"exact SPICE reference ({nQExact} queries)")

    # The exact-SPICE reference: zero error by definition, drawn in the break panel at the "0" tick.
    # The whole break panel is below the floor, so it continues the grey "free" band.
    axz.set_xscale("log")
    axz.set_ylim(-0.5, 0.5)
    axz.axhspan(-0.5, 0.5, color="0.85", alpha=0.6, zorder=0)
    axz.plot(nQExact, 0.0, "D", color=C_EXACT, ms=12, markeredgecolor="k", zorder=6)
    axz.set_yticks([0.0]); axz.set_yticklabels(["0"])

    # diagonal break marks between the two panels
    ax.spines["bottom"].set_visible(False); ax.tick_params(bottom=False)
    axz.spines["top"].set_visible(False)
    breakKw = dict(marker=[(-1, -0.5), (1, 0.5)], ms=8, ls="none", color="k", mec="k", mew=1, clip_on=False)
    ax.plot([0, 1], [0, 0], transform=ax.transAxes, **breakKw)
    axz.plot([0, 1], [1, 1], transform=axz.transAxes, **breakKw)

    axz.set_xlim(1.5, nQExact * 1.6)
    axz.set_xlabel("Moon SPICE queries over the run  (compute cost)")
    ax.set_ylabel("orbit final-position error vs. exact-SPICE reference  [m]")
    ax.grid(True, which="both", alpha=0.3)
    axz.grid(True, axis="x", which="both", alpha=0.3)
    ax.legend(loc="upper center")
    return fig


def plotErrorOverTime(tTruth, posTruth, integratorFloor_t, tR, posR, nQGood,
                      tClassic, posClassic, nQClassic, errClassic):
    """Figure 3: error over time for a well-chosen reconstruction, the classic default, and the
    integrator error itself (the run with a perfect ephemeris)."""
    errRecon_t = np.linalg.norm(posR - posTruth, axis=1)
    # classic run error over time (both run at the task step, so the truth samples align)
    errClassic_t = np.linalg.norm(posClassic - posTruth, axis=1)

    fig, ax = plt.subplots(figsize=(8, 6), num=fileName + "3", layout="constrained")
    # The integrator error grows over the leg; everything below it is the "free" zone where
    # reconstruction error does not move the orbit.
    floor_t = np.maximum(integratorFloor_t, 1e-6)
    ax.fill_between(tTruth / 3600.0, 1e-6, floor_t, color="0.85", alpha=0.6, zorder=0)
    ax.semilogy(tTruth / 3600.0, floor_t, color="k", ls="-", lw=1.5, zorder=1,
                label="integrator error (perfect ephemeris)")
    ax.semilogy(tR / 3600.0, np.maximum(errRecon_t, 1e-6), color=C_RECON, zorder=5,
                label=f"reconstruction, {nQGood} Moon queries")
    ax.semilogy(tClassic / 3600.0, np.maximum(errClassic_t, 1e-6), color=C_CLASSIC, ls="--", zorder=2,
                label=f"classic spacecraft + gravityEffector ({nQClassic} queries)")
    ax.set_ylim(1e-6, errClassic * 3.0)
    ax.set_xlabel("time  [hr]")
    ax.set_ylabel("position error vs. exact SPICE  [m]")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(loc="lower right")
    return fig


if __name__ == "__main__":
    run(show_plots=True)
