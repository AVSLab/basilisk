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
Independent ground-truth reference for the stochastic efficiency study.

This module integrates the *same* physical problem as ``stochasticDragModel`` --
a planar cannonball orbit with cannonball drag and an Ornstein-Uhlenbeck (OU)
density correction -- but **entirely outside Basilisk**, using ``scipy.integrate``
at a tight tolerance.  Because it shares none of Basilisk's integrators, state
plumbing, or task scheduling, it is an unbiased arbiter of the "true" moments of
the figure of merit and cannot be fooled by an error common to the two Basilisk
arms.  (During development it was exactly this reference that revealed -- and
then confirmed the fix for -- a density-task-placement bug that had inflated the
profile arm's variance by ~2.6x.)

The physics is kept deliberately faithful to ``stochasticDragModel``:

* point-mass Earth gravity with the same ``mu``,
* the *same* exponential-atmosphere parameters, read directly from Basilisk's
  :ref:`exponentialAtmosphere` model so the base density is identical,
* the *same* OU path generator (:func:`stochasticDragModel.generateOUPath`) on
  the same fixed fine master grid,
* cannonball drag ``a_drag = -0.5 rho Cd A / m * |v| * v`` with the same
  coefficients,
* identical initial conditions (projected into the orbit plane).

The figure of merit is the final semi-major axis ``a(tf)`` -- identical in
meaning to the Basilisk arms.  Realizations are seeded by ``seed`` and use the
*same* seed convention as ``runComparison.py`` so the reference noise stream is
reproducible and independent.

This file caches its samples through the very same machinery as the Basilisk
arms (see :func:`runComparison.runReferenceScipy`), so the reference is
resumable and is loaded by the analysis/plotting exactly like any other config.
"""
from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass

import numpy as np
from scipy.integrate import solve_ivp

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import stochasticDragModel as model  # noqa: E402


@dataclass(frozen=True)
class _AtmoParams:
    """Exponential-atmosphere parameters, read once from Basilisk."""
    baseDensity: float
    scaleHeight: float
    planetRadius: float


def _readAtmoParams() -> _AtmoParams:
    """Pull the exponential-atmosphere parameters from Basilisk's own model.

    Reading them (rather than hard-coding) guarantees the scipy reference uses
    exactly the same base density profile as the Basilisk arms, so any
    difference in the moments is attributable to the integration, not the model.
    """
    from Basilisk.simulation import exponentialAtmosphere
    from Basilisk.utilities import simSetPlanetEnvironment
    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
    return _AtmoParams(atmo.baseDensity, atmo.scaleHeight, atmo.planetRadius)


def runReferenceRealization(
    params: model.ScenarioParams,
    seed: int,
    rtol: float = 1e-10,
    atol: float = 1e-6,
    maxStep: float = 2.0,
    profileGridDt: float = 0.25,
    _atmo: _AtmoParams = None,
) -> model.RealizationResult:
    """Integrate one reference realization with scipy and return its FoM.

    Args:
        params: shared scenario parameters (same dataclass as the Basilisk arms).
        seed: realization seed (same convention as the Basilisk arms).
        rtol, atol, maxStep: scipy ``solve_ivp`` accuracy controls.  Defaults are
            tight enough that the discretization error is far below the
            Monte-Carlo error, so this is effectively the exact moment.
        profileGridDt: master-grid spacing [s] for the prescribed OU path
            (matches the profile arm's master grid).
        _atmo: cached atmosphere parameters (read once by the caller for speed).

    Returns:
        RealizationResult with the final semi-major axis (FoM), altitude, and the
        wall time spent inside ``solve_ivp``.
    """
    planet, oe, rN, vN, orbitPeriod, mu = params.deriveOrbit()
    tf = params.orbits * orbitPeriod
    atmo = _atmo if _atmo is not None else _readAtmoParams()

    Cd, A, m = params.dragCoeff, params.projectedArea, params.mass
    rho0, H, r0 = atmo.baseDensity, atmo.scaleHeight, atmo.planetRadius

    # Prescribed OU density correction on a fixed fine master grid.
    grid = np.linspace(0.0, tf, int(np.ceil(tf / profileGridDt)) + 1)
    xPath = model.generateOUPath(
        grid, params.stationaryStd, params.timeConstant, np.random.default_rng(seed))

    def xOfT(t):
        return np.interp(t, grid, xPath)

    # Build an in-plane 2-D state from the 3-D initial condition.  The two-body +
    # central drag problem stays in the initial orbit plane, so a planar
    # integration is exact for this FoM and much cheaper.
    r_ = np.asarray(rN, dtype=float)
    v_ = np.asarray(vN, dtype=float)
    e1 = r_ / np.linalg.norm(r_)
    hVec = np.cross(r_, v_)
    e3 = hVec / np.linalg.norm(hVec)
    e2 = np.cross(e3, e1)
    y0 = [np.linalg.norm(r_), 0.0, float(np.dot(v_, e1)), float(np.dot(v_, e2))]

    def rhs(t, y):
        rx, ry, vx, vy = y
        r = np.hypot(rx, ry)
        v = np.hypot(vx, vy)
        alt = r - r0
        rho = rho0 * np.exp(-alt / H) * (1.0 + xOfT(t))
        ag = -mu / r**3
        kDrag = -0.5 * rho * Cd * A / m * v
        return [vx, vy, ag * rx + kDrag * vx, ag * ry + kDrag * vy]

    t0 = time.perf_counter()
    sol = solve_ivp(rhs, (0.0, tf), y0, rtol=rtol, atol=atol, max_step=maxStep,
                    dense_output=False)
    wall = time.perf_counter() - t0

    rxf, ryf, vxf, vyf = sol.y[:, -1]
    rmag = np.hypot(rxf, ryf)
    vmag = np.hypot(vxf, vyf)
    energy = vmag**2 / 2.0 - mu / rmag
    a = -mu / (2.0 * energy)
    altitude = (rmag - r0) / 1000.0

    return model.RealizationResult(
        fomSemiMajorAxis=float(a),
        fomAltitude=float(altitude),
        wallSeconds=float(wall),
        nSteps=int(sol.t.size),
    )


def runReferenceTrajectory(
    params: model.ScenarioParams,
    seed: int,
    nOut: int = 2000,
    rtol: float = 1e-10,
    atol: float = 1e-6,
    maxStep: float = 2.0,
    profileGridDt: float = 0.25,
    _atmo: _AtmoParams = None,
) -> dict:
    """Integrate one reference realization and return full time-histories.

    Same dynamics, OU path, and accuracy controls as
    :func:`runReferenceRealization`, but evaluated on a dense time grid so the
    trajectory (position, velocity, density, drag, altitude, semi-major axis) can
    be plotted -- the analogue of ``plotOrbits`` in
    ``examples/mujoco/scenarioStochasticDrag.py``.  Used only for visual
    sanity-checking, never in the Monte-Carlo study.

    Returns a dict of numpy arrays keyed: ``t`` [s], ``pos`` (n,2) [m] in the
    orbit plane, ``vel`` (n,2) [m/s], ``alt`` [km], ``a`` [m] (osculating
    semi-major axis), ``rho`` [kg/m^3] (perturbed density), ``rhoDet``
    [kg/m^3] (deterministic exponential density), ``x`` [-] (OU correction),
    ``dragMag`` [N] (drag force magnitude).
    """
    planet, oe, rN, vN, orbitPeriod, mu = params.deriveOrbit()
    tf = params.orbits * orbitPeriod
    atmo = _atmo if _atmo is not None else _readAtmoParams()
    Cd, A, m = params.dragCoeff, params.projectedArea, params.mass
    rho0, H, r0 = atmo.baseDensity, atmo.scaleHeight, atmo.planetRadius

    grid = np.linspace(0.0, tf, int(np.ceil(tf / profileGridDt)) + 1)
    xPath = model.generateOUPath(
        grid, params.stationaryStd, params.timeConstant, np.random.default_rng(seed))

    def xOfT(t):
        return np.interp(t, grid, xPath)

    r_ = np.asarray(rN, dtype=float)
    v_ = np.asarray(vN, dtype=float)
    e1 = r_ / np.linalg.norm(r_)
    hVec = np.cross(r_, v_)
    e3 = hVec / np.linalg.norm(hVec)
    e2 = np.cross(e3, e1)
    y0 = [np.linalg.norm(r_), 0.0, float(np.dot(v_, e1)), float(np.dot(v_, e2))]

    def rhs(t, y):
        rx, ry, vx, vy = y
        r = np.hypot(rx, ry)
        v = np.hypot(vx, vy)
        rho = rho0 * np.exp(-(r - r0) / H) * (1.0 + xOfT(t))
        ag = -mu / r**3
        kDrag = -0.5 * rho * Cd * A / m * v
        return [vx, vy, ag * rx + kDrag * vx, ag * ry + kDrag * vy]

    tEval = np.linspace(0.0, tf, nOut)
    sol = solve_ivp(rhs, (0.0, tf), y0, rtol=rtol, atol=atol, max_step=maxStep,
                    t_eval=tEval, dense_output=False)

    rx, ry, vx, vy = sol.y
    rmag = np.hypot(rx, ry)
    vmag = np.hypot(vx, vy)
    x = xOfT(sol.t)
    rhoDet = rho0 * np.exp(-(rmag - r0) / H)
    rho = rhoDet * (1.0 + x)
    dragMag = 0.5 * rho * Cd * A / m * vmag**2 * m   # |F| = 0.5 rho Cd A v^2
    energy = vmag**2 / 2.0 - mu / rmag
    aOsc = -mu / (2.0 * energy)

    return {
        "t": sol.t, "pos": np.column_stack([rx, ry]),
        "vel": np.column_stack([vx, vy]),
        "alt": (rmag - r0) / 1000.0, "a": aOsc,
        "rho": rho, "rhoDet": rhoDet, "x": x, "dragMag": dragMag,
        "planetRadius": r0, "mu": mu,
    }


# Convenience for ad-hoc checking
if __name__ == "__main__":
    p = model.ScenarioParams(orbits=2.0, stationaryStd=0.3)
    atmo = _readAtmoParams()
    N = 100
    aa = np.array([runReferenceRealization(p, 20000 + i, _atmo=atmo).fomSemiMajorAxis
                   for i in range(N)])
    print(f"scipy reference: mean(a)={aa.mean():.3f}  std(a)={aa.std(ddof=1):.1f}  (N={N})")
