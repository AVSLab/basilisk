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
The companion to :ref:`scenarioSpiceReconstruction`, applying the :ref:`SpiceInterface<spiceInterface>`
reconstruction feature (see :ref:`spiceReconstruction`) to the **orientation** channel instead of
position. It illustrates the "constant rotation vector" case from that documentation page: for a body that spins
about a fixed axis at a constant rate, orientation reconstruction is cheap and *exact*.

A spacecraft orbits the asteroid (4) Vesta at 50 km altitude, deep in Vesta's modeled degree-8
spherical-harmonic gravity field (``VESTA20H.txt``). Because :ref:`NBodyGravity<NBodyGravity>`
rotates that aspherical field into the body-fixed frame using the source orientation ``J20002Pfix``,
the attitude must be supplied continually as the orbit is integrated, and both dynamics paths call
:ref:`spiceInterface` to get it. On an :ref:`MJScene<MJScene>` dynamics task the attitude is queried
every RK4 sub-step; the classic :ref:`spacecraft` + :ref:`gravityEffector` path
queries once per task step and extrapolates within the step (the same distinction as in the
:ref:`companion position scenario<scenarioSpiceReconstruction>`). Either way, a finely stepped
propagation evaluates ``sxform_c`` thousands of times per orbit.

But Vesta's SPICE orientation ``IAU_VESTA`` is a **constant rotation vector**: a fixed spin axis and
a constant rate (its IAU model is a linear ``W = W0 + Wdot*t``). SPICE returns the attitude as a
DCM, and a DCM trajectory is *not* linear in time, so linearly extrapolating the DCM entries would
not be exact. The reconstruction instead lifts the attitude to its **principal rotation vector**
(PRV) and extrapolates *that*: for a fixed axis at a constant rate the PRV is exactly linear in
time, so a first-order extrapolation from a *single* knot, mapped back to a DCM with ``PRV2C``,
reproduces the attitude to machine precision for the whole run. One SPICE query serves the entire
propagation.

.. important::

    This "one query is exact" simplification holds **only** while the body's angular-velocity
    vector is (near) constant, i.e. a fixed spin axis at a fixed rate. That covers nearly every
    catalogued small body and the planetary ``IAU_*`` frames. It does **not** hold for a tumbler
    (a non-principal-axis rotator such as (99942) Apophis), whose spin axis drifts: there the
    attitude has genuine curvature and single-knot extrapolation degrades quickly. Such bodies
    need proper reconstruction with ``interpolate=True`` and ``nKnots >= 2`` (see the "choosing
    knobs by body type" section of :ref:`spiceReconstruction`).

The scenario runs a fine-step exact reference, then two coarse-step runs that differ only in how
the attitude is supplied, and shows their errors against the reference are identical: the
integrator step sets the error, not the SPICE query cost.

Run with::

    python3 scenarioVestaOrientation.py

Illustration of Results
-----------------------
The first figure shows the spacecraft's path around Vesta over three orbits,
with Vesta drawn to scale at the origin. The orbit is low (50 km) and inclined, so it passes over
a range of longitudes and is steered throughout by the aspherical, body-fixed gravity field. That
is why the body's attitude enters the dynamics at all, and it is the run whose final accuracy the
second figure measures.

.. figure:: /_images/Scenarios/scenarioVestaOrientation1.svg
   :align: center
   :scale: 90 %

The second figure shows the position error against time for two coarse-step runs,
each differenced against the same integrator-converged fine-step reference: one queries the attitude
exactly at every sub-step (thousands of SPICE calls), the other velocity-extrapolates from a single
SPICE query for the whole run. The legend gives each run's query count. The two error curves lie
exactly on top of each other; because Vesta is a constant-rate spinner,
the single-query attitude is exact, so the only error left is the integrator's own truncation error,
identical in both runs. The attitude's SPICE cost, thousands of queries versus one, does not change
the result.

Does it actually save wall-clock time? The scenario prints the runtime of each run. As in the
companion :ref:`scenarioSpiceReconstruction`, reconstruction changes only the SPICE cost, and here
that cost is a small share of the step: the degree-8 spherical-harmonic gravity evaluation dominates.
Collapsing the ~1700 attitude queries to one shaves at most a percent or two off the runtime on
either dynamics path (the classic :ref:`spacecraft` + :ref:`gravityEffector`, which queries
``sxform_c`` once per step, and the :ref:`MJScene<MJScene>` dynamics task, which queries it once per
RK4 stage).

.. figure:: /_images/Scenarios/scenarioVestaOrientation2.svg
   :align: center
   :scale: 90 %
"""
import os
import time

import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simHelpers
from Basilisk.simulation import (mujoco, svIntegrators, NBodyGravity,
                                  sphericalHarmonicsGravityModel, spiceInterface)
from Basilisk.topLevelModules import pyswice
from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile

fileName = os.path.splitext(os.path.basename(__file__))[0]

# scenario configuration
EPOCH = "2025 NOVEMBER 15 12:00:00.000"
VESTA_SH_FILE = os.path.join(list(__import__("Basilisk").__path__)[0], "supportData",
                             "LocalGravData", "VESTA20H.txt")
SH_DEGREE = 8
TARGET = "vesta"                   # SPICE body; IAU_VESTA is its constant-rate body-fixed frame
ORBIT_ALT = 50.0e3                 # [m] low orbit -> strong dependence on the aspherical field
ORBIT_INC = 40.0 * macros.D2R      # inclined so the trajectory samples the tesseral terms
COARSE_DT = 60.0                   # [s] RK4 step for the two comparison runs
REFERENCE_DT = 1.0                 # [s] fine RK4 step for the truth reference (integrator-converged)
N_ORBITS = 3.0                     # propagation horizon in orbital periods

# standard Basilisk scenario plotting colours (gist_earth map, as the other scenarios use)
C_EXACT = simHelpers.getLineColor(0, 2)
C_EXTRAP = simHelpers.getLineColor(1, 2)

SC_XML = r"""
<mujoco>
    <worldbody>
        <body name="sc"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>
    </worldbody>
</mujoco>
"""


def runOrbit(rN, vN, tf, dt, mu, rEq, kernels, knotStep=None):
    """Propagate the Vesta orbit at RK4 step ``dt`` and record the spacecraft trajectory.

    A SpiceInterface supplies only Vesta's orientation: Vesta sits at the frame origin as the
    central body, so its position channel is disabled (identically zero, no SPK kernel needed). If
    ``knotStep`` (nanoseconds) is given the orientation is reconstructed by single-knot velocity
    extrapolation on that grid (one SPICE query for the whole run); otherwise it is queried exactly
    at every integrator sub-step. Returns (orientation-query count, times [s], positions [m])."""
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("simProcess")
    dynProcess.addTask(scSim.CreateNewTask("simTask", macros.sec2nano(dt)))

    scene = mujoco.MJScene(SC_XML)
    scene.ModelTag = "vestaScene"
    scSim.AddModelToTask("simTask", scene)
    integratorObject = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integratorObject)
    body = scene.getBody("sc")

    # Attitude producer: a SpiceInterface for Vesta's orientation only, position disabled. Added to
    # the dynamics task BEFORE gravity so the attitude is current at every integrator stage.
    spiceObject = spiceInterface.SpiceInterface(auto_configure_kernels=False)
    spiceObject.ModelTag = "vestaOrient"
    spiceObject.addPlanetNames(spiceInterface.StringVector([TARGET]))
    spiceObject.UTCCalInit = EPOCH
    for k in kernels:
        spiceObject.addKernelPath(k)
    spiceObject.setPlanetPositionDisabled(TARGET)
    if knotStep is not None:
        # (knotStep, nKnots=1, useVelocity=True, interpolate=False) = single-knot velocity extrap.
        spiceObject.setPlanetOrientationReconstruction(TARGET, int(knotStep), 1, True, False)
    scene.AddModelToDynamicsTask(spiceObject)

    # Vesta's real degree-8 spherical-harmonic field, rotated into the body frame by the supplied
    # orientation. This is what makes the acceleration attitude-dependent.
    gravity = NBodyGravity.NBodyGravity()
    shModel = sphericalHarmonicsGravityModel.SphericalHarmonicsGravityModel()
    shModel.loadFromFile(VESTA_SH_FILE, SH_DEGREE)
    shModel.muBody, shModel.radEquator = mu, rEq
    source = gravity.addGravitySource(TARGET, shModel, True)
    source.stateInMsg.subscribeTo(spiceObject.planetStateOutMsgs[0])
    scene.AddModelToDynamicsTask(gravity)
    gravity.addGravityTarget("sc", body)

    scRec = body.getOrigin().stateOutMsg.recorder()
    scSim.AddModelToTask("simTask", scRec)
    keep = [spiceObject, gravity, shModel, integratorObject]     # noqa: F841  hold C++ refs
    scSim.InitializeSimulation()
    body.setPosition(list(rN)); body.setVelocity(list(vN))
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    # Actual number of sxform_c orientation queries over the run, whether exact (once per RK4
    # stage) or reconstruction (a few knot fills). getPlanetSpiceQueryCount counts both.
    nQ = int(spiceObject.getPlanetSpiceQueryCount(TARGET)[1])
    ts = np.array(scRec.times()) * macros.NANO2SEC
    pos = np.array(scRec.r_BN_N)
    pyswice.kclear_c()
    return nQ, ts, pos


def plotTrajectory(pos, rEq):
    """3D trajectory of the spacecraft around Vesta, with the body drawn to scale."""
    fig = plt.figure(figsize=(7, 7), num=fileName + "1", layout="constrained")
    ax = fig.add_subplot(projection="3d")
    p = pos / 1e3  # km
    ax.plot(p[:, 0], p[:, 1], p[:, 2], color=C_EXTRAP, lw=1.5, label="spacecraft trajectory")
    # Vesta drawn as a sphere at the origin (mean radius).
    u, v = np.mgrid[0:2 * np.pi:40j, 0:np.pi:20j]
    r = rEq / 1e3
    ax.plot_surface(r * np.cos(u) * np.sin(v), r * np.sin(u) * np.sin(v), r * np.cos(v),
                    color="0.7", alpha=0.5, linewidth=0)
    ax.set_xlabel("x  [km]"); ax.set_ylabel("y  [km]"); ax.set_zlabel("z  [km]")
    ax.set_box_aspect((1, 1, 1))
    ax.legend(loc="upper left")
    return fig


def plotErrorOverTime(ts, errExact, nQExact, errOne, nQOne):
    """Figure 2: both coarse runs have the SAME error vs. the reference.

    The two curves overlap: the coarse-step error is set by the integrator, not by how (or how
    often) the attitude is queried from SPICE.
    """
    fig, ax = plt.subplots(figsize=(8, 6), num=fileName + "2", layout="constrained")
    ax.plot(ts / 3600.0, errExact, color=C_EXACT, lw=3.5, alpha=0.7, zorder=4,
            label=f"exact SPICE every sub-step ({nQExact} queries)")
    ax.plot(ts / 3600.0, errOne, color=C_EXTRAP, lw=1.8, ls="--", zorder=5,
            label=f"velocity extrapolation ({nQOne} SPICE query)")
    ax.set_xlim(0, ts[-1] / 3600.0)
    ax.set_ylim(bottom=0.0)
    ax.set_xlabel("time  [hr]")
    ax.set_ylabel(f"orbit error vs. {REFERENCE_DT:g} s reference  [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left")
    return fig


def run(show_plots=True):
    """Run the constant-spinner orientation study and produce its figures.

    Args:
        show_plots (bool): display the figures.

    Returns:
        dict: ``figureList`` mapping figure name to matplotlib Figure.
    """
    # Vesta gravitational parameter and reference radius, from the SH file header
    shModel = sphericalHarmonicsGravityModel.SphericalHarmonicsGravityModel()
    shModel.loadFromFile(VESTA_SH_FILE, SH_DEGREE)
    shModel.initializeParameters()
    mu, rEq = shModel.muBody, shModel.radEquator

    # leapsecond + planetary-constants (PCK) kernels; the PCK carries Vesta's IAU rotation
    kernels = [str(get_path(DataFile.EphemerisData.naif0012)),
               str(get_path(DataFile.EphemerisData.pck00010))]

    # low circular orbit initial state (inclined so it passes over varied longitudes)
    oe = orbitalMotion.ClassicElements()
    oe.a = rEq + ORBIT_ALT
    oe.e, oe.i, oe.Omega, oe.omega, oe.f = 0.0, ORBIT_INC, 0.0, 0.0, 0.0
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    rN, vN = np.array(rN), np.array(vN)
    period = 2.0 * np.pi * np.sqrt(oe.a ** 3 / mu)
    tf = N_ORBITS * period

    # Integrator-converged truth: exact SPICE attitude at a fine RK4 step, so any error left in the
    # coarse runs below is the integrator's, not the attitude treatment's.
    _, tRef, posRef = runOrbit(rN, vN, tf, REFERENCE_DT, mu, rEq, kernels)

    # Two coarse-step runs that differ ONLY in how the attitude is supplied, wall-clock timed so
    # the SPICE-query saving can be reported as an end-to-end runtime difference:
    #   - exact SPICE queried at every sub-step (the default);
    #   - single-knot velocity extrapolation, one SPICE query for the whole run.
    t0 = time.perf_counter()
    nQExact, ts, posExact = runOrbit(rN, vN, tf, COARSE_DT, mu, rEq, kernels)
    tExact = time.perf_counter() - t0
    t0 = time.perf_counter()
    nQOne, _, posOne = runOrbit(rN, vN, tf, COARSE_DT, mu, rEq, kernels,
                                knotStep=macros.sec2nano(10.0 * tf))
    tExtrap = time.perf_counter() - t0

    # Error of each coarse run against the fine reference (reference sampled at the coarse times).
    refAtCoarse = np.vstack([np.interp(ts, tRef, posRef[:, i]) for i in range(3)]).T
    errExact = np.linalg.norm(posExact - refAtCoarse, axis=1)
    errOne = np.linalg.norm(posOne - refAtCoarse, axis=1)

    print(f"[study] Vesta {N_ORBITS:g}-orbit run at {ORBIT_ALT/1e3:g} km altitude")
    print(f"[study] reference: exact SPICE at {REFERENCE_DT:g}s step (integrator-converged)")
    print(f"[study] coarse ({COARSE_DT:g}s) exact:  {nQExact} queries, peak err {errExact.max():.2f} m, {tExact*1e3:.0f} ms")
    print(f"[study] coarse ({COARSE_DT:g}s) extrap: {nQOne} query, peak err {errOne.max():.2f} m, {tExtrap*1e3:.0f} ms")
    print(f"[study] runtime: {100.0*(1.0 - tExtrap/tExact):+.0f}% from reconstruction "
          f"(within timing noise on this short run; SH gravity dominates the step, not SPICE)")

    plt.rcParams.update({"font.size": 13, "lines.linewidth": 2.0, "lines.markersize": 8})
    figureList = {}
    figureList[fileName + "1"] = plotTrajectory(posRef, rEq)
    figureList[fileName + "2"] = plotErrorOverTime(ts, errExact, nQExact, errOne, nQOne)

    if show_plots:
        plt.show()
    else:
        plt.close("all")
    return figureList


if __name__ == "__main__":
    run(show_plots=True)
