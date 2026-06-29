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
Shared simulation model for the stochastic-integrator efficiency study.

This module builds the *same physical scenario* as
``examples/mujoco/scenarioStochasticDrag.py`` -- a 250 km cannonball orbit
subject to drag whose atmospheric density carries an Ornstein-Uhlenbeck (OU)
density correction -- but exposes it through two interchangeable "arms" so the
two ways of capturing the noise can be compared head-to-head:

* **Arm "sde"** (stochastic integrator): the OU density-correction state is
  evolved *inline* by a stochastic integrator (Euler-Maruyama or one of the
  weak second-order Tang & Xiao SRK methods).  One simulation = one realization
  of the noisy trajectory.  The macro time step ``dt`` can be large because the
  stochastic integrator is designed to capture the weak statistics of the noise.

* **Arm "profile"** (pre-generated noise + deterministic integrator): the OU
  density-correction path ``x(t)`` is generated *in advance* (sampled exactly on
  a fine grid, so it is statistically unbiased for any step), stored, and then
  *replayed* by a small Python module (:class:`ProfileAtmDensity`) that
  interpolates it at the integrator's stage times.  The orbit is then propagated
  with an ordinary deterministic Runge-Kutta integrator.  To capture the
  roughness of the forcing this arm must use a *small* ``dt``, which is the cost
  this study is measuring.

The single scalar **figure of merit (FoM)** returned by :func:`runRealization`
is the final semi-major axis ``a(tf)`` of the decaying orbit (in metres); the
final altitude is also returned for convenience.  Drag makes ``a`` decay, and the
density noise injects run-to-run spread -- so the *standard deviation* of ``a``
across realizations is the statistic that genuinely exercises the noise model,
while the mean is dominated by the deterministic drag.

The heavy lifting (sampling many realizations, caching, Pareto analysis,
plotting) lives in the sibling ``runComparison.py`` / ``analyzeResults.py`` /
``plotResults.py`` scripts.  This file only knows how to build and run *one*
realization for a given configuration.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Literal, Optional

import numpy as np

from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel
from Basilisk.architecture.numbaModel import NumbaModel
from Basilisk.simulation import mujoco
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import pointMassGravityModel
from Basilisk.simulation import NBodyGravity
from Basilisk.simulation import exponentialAtmosphere
from Basilisk.simulation import cannonballDrag
from Basilisk.simulation import MJStochasticAtmDensity
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simSetPlanetEnvironment


# Arm identifiers
Arm = Literal["sde", "profile"]

# Stochastic-integrator method names (Arm "sde")
SDE_METHODS = ("EulerMayurama", "W2Ito1", "W2Ito2")

# Deterministic-integrator method names (Arm "profile")
DET_METHODS = ("RK4", "RK2", "RKF45", "RKF78")


CANNONBALL_SCENE_XML = r"""
<mujoco>
	<worldbody>
		<body name="ball">
			<freejoint/>
			<inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
		</body>
	</worldbody>
</mujoco>
"""


@dataclass(frozen=True)
class ScenarioParams:
    """Physical and OU parameters shared by both arms.

    The physical/atmosphere/OU constants match ``scenarioStochasticDrag.py``.
    The two *study* knobs -- ``orbits`` and ``stationaryStd`` -- are set to the
    values the whole study was validated at, chosen from a signal/validity/cost
    sweep (see below), so the defaults give an interesting, well-posed problem
    out of the box with no overrides needed:

      * ``orbits = 2.0``: the final semi-major axis a(tf) spreads by ~130 m
        across noise realizations -- ~5000x the profile arm's discretization-bias
        floor (~0.03 m) and well above the SDE arm's resolvable bias (~a few m),
        so there is ample signal to separate integrators.  (Going much longer,
        e.g. the tutorial's 7.1 orbits, drives the craft toward reentry where the
        SMA decays ~80 km and its spread explodes to >>km -- chaotic divergence,
        not a clean weak-order signal, so it is a poor accuracy benchmark.)
      * ``stationaryStd = 0.30``: strong, clearly-resolvable noise while keeping
        the multiplicative density factor (1 + x) essentially always positive
        (only ~0.03% of steps hit x < -1).  Raising it to ~0.5 makes ~2.5% of
        steps unphysical (negative density -> drag becomes thrust), so 0.30 is
        the practical ceiling for this multiplicative model.

    Both knobs can still be overridden (constructor args or the runner's
    ``--orbits`` / ``--stationaryStd`` flags) to explore the trade-off.
    """
    initialAlt: float = 250.0           # [km] initial circular altitude
    inclination: float = 33.3           # [deg]
    dragCoeff: float = 2.2              # [-]
    projectedArea: float = 10.0         # [m^2]
    mass: float = 1.0                  # [kg]
    stationaryStd: float = 0.30         # [-] OU stationary std of density correction
    timeConstant: float = 1.8 * 60.0    # [s] OU time constant tau
    orbits: float = 2.0                # [-] number of orbital periods to propagate

    def deriveOrbit(self):
        """Return (planet, oe, rN, vN, orbitPeriod, mu) for these parameters."""
        planet = simIncludeGravBody.BODY_DATA["earth"]
        oe = orbitalMotion.ClassicElements()
        oe.a = planet.radEquator + self.initialAlt * 1000.0  # [m]
        oe.e = 0.0
        oe.i = self.inclination * macros.D2R
        oe.Omega = 48.2 * macros.D2R
        oe.omega = 347.8 * macros.D2R
        oe.f = 85.3 * macros.D2R
        rN, vN = orbitalMotion.elem2rv(planet.mu, oe)
        oe = orbitalMotion.rv2elem(planet.mu, rN, vN)
        orbitPeriod = 2.0 * np.pi / np.sqrt(planet.mu / oe.a**3)
        return planet, oe, rN, vN, orbitPeriod, planet.mu


@dataclass
class RealizationResult:
    """Outcome of a single realization."""
    fomSemiMajorAxis: float            # [m] final semi-major axis a(tf)
    fomAltitude: float                 # [km] final altitude
    wallSeconds: float                 # [s] ExecuteSimulation wall time only
    nSteps: int                        # number of macro steps taken


# ---------------------------------------------------------------------------
# Exact OU path generation (Arm "profile")
# ---------------------------------------------------------------------------
def generateOUPath(
    times: np.ndarray,
    stationaryStd: float,
    timeConstant: float,
    rng: np.random.Generator,
    x0: float = 0.0,
) -> np.ndarray:
    r"""Sample an Ornstein-Uhlenbeck path *exactly* on the given time grid.

    The OU process is

    .. math::
        dx = -\theta x\,dt + \sigma\,dW,\qquad
        \theta = 1/\tau,\quad \sigma = \sigma_{st}\sqrt{2/\tau}

    which matches :ref:`MeanRevertingNoise<meanRevertingNoise>`.  The exact
    transition over a step :math:`\Delta` is Gaussian:

    .. math::
        x_{n+1} = x_n e^{-\theta\Delta}
                  + \sigma_{st}\sqrt{1 - e^{-2\theta\Delta}}\; Z_n,
        \qquad Z_n \sim \mathcal{N}(0,1)

    This is unbiased for *any* grid spacing, so the profile is the statistical
    ground truth for the noise; a deterministic integrator replaying it only
    incurs error from how finely it resolves the (interpolated) forcing.

    Args:
        times: monotonically increasing sample times [s] (need not be uniform).
        stationaryStd: OU stationary standard deviation (sigma_st).
        timeConstant: OU time constant tau [s].
        rng: numpy Generator (seeded by the caller for reproducibility).
        x0: initial state (default 0, matching meanRevertingNoise.cpp).

    Returns:
        Array of x(t) values with the same length as ``times``.
    """
    theta = 1.0 / timeConstant
    x = np.empty_like(times, dtype=float)
    x[0] = x0
    dts = np.diff(times)
    # Vectorized per-step transition constants
    decay = np.exp(-theta * dts)
    noiseStd = stationaryStd * np.sqrt(np.maximum(1.0 - np.exp(-2.0 * theta * dts), 0.0))
    z = rng.standard_normal(dts.size)
    for n in range(dts.size):
        x[n + 1] = x[n] * decay[n] + noiseStd[n] * z[n]
    return x


class ProfileAtmDensityPy(sysModel.SysModel):
    """Pure-Python reference implementation of the profile replay module.

    Kept for clarity and cross-checking only.  **Do not use it for timing**: a
    Python ``SysModel`` is called through the interpreter every integrator
    stage, so its wall time is dominated by Python overhead rather than the
    algorithm, which would unfairly penalise the profile arm against the
    C-speed stochastic arm.  Use :class:`ProfileAtmDensity` (a Numba module that
    compiles to a C-callable) for the actual comparison.

    It reads the unperturbed (exponential) atmospheric density, multiplies it by
    ``(1 + x(t))`` where ``x(t)`` is linearly interpolated from the supplied
    profile at the current integrator stage time, and writes the corrected
    density.  It carries no integrated state, so it works with any deterministic
    integrator.
    """

    def __init__(self, profileTimes: np.ndarray, profileValues: np.ndarray):
        super().__init__()
        self.atmoDensInMsg = messaging.AtmoPropsMsgReader()
        self.atmoDensOutMsg = messaging.AtmoPropsMsg()
        self._times = np.asarray(profileTimes, dtype=float)
        self._values = np.asarray(profileValues, dtype=float)

    def UpdateState(self, CurrentSimNanos: int):
        t = CurrentSimNanos * macros.NANO2SEC
        # np.interp clamps to the endpoints outside the profile range, which is
        # the desired behavior at t=0 and t=tf.
        x = float(np.interp(t, self._times, self._values))
        out = self.atmoDensInMsg()
        out.neutralDensity *= 1.0 + x
        self.atmoDensOutMsg.write(out, CurrentSimNanos, self.moduleID)


class ProfileAtmDensity(NumbaModel):
    """Replays a pre-generated OU density-correction path ``x(t)`` at C speed.

    This is the deterministic-arm counterpart of
    :ref:`StochasticAtmDensity<stochasticAtmDensity>`.  It is a
    :ref:`NumbaModel<numbaModules>`, so ``UpdateStateImpl`` JIT-compiles once to
    a C-callable that the Basilisk scheduler invokes directly every integrator
    stage with zero per-tick Python overhead.  This is essential for a fair
    speed comparison: the C++ stochastic arm and this profile arm then differ in
    wall time only because of their *algorithms* (step size, number of stage
    evaluations), not the implementation language.

    It reads the unperturbed (exponential) atmospheric density, multiplies it by
    ``(1 + x(t))`` where ``x(t)`` is linearly interpolated from the pre-generated
    profile at the current integrator stage time, and writes the corrected
    density.  The profile is generated on a *uniform* grid (see
    :func:`generateOUPath` callers), so interpolation is O(1) index arithmetic.

    The profile arrays live in ``self.memory`` (a C-level structured buffer
    after ``Reset``); ``t0`` and ``dtGrid`` describe the uniform grid so the
    compiled kernel can locate the bracketing nodes without a search.  Linear
    interpolation keeps the replayed forcing piecewise-linear (continuous but
    rough), which is what forces a deterministic integrator to take small
    steps -- exactly the cost this study quantifies.
    """

    def __init__(self, profileTimes: np.ndarray, profileValues: np.ndarray):
        super().__init__()
        self.atmoDensInMsg = messaging.AtmoPropsMsgReader()
        self.atmoDensOutMsg = messaging.AtmoPropsMsg()

        times = np.asarray(profileTimes, dtype=float)
        values = np.asarray(profileValues, dtype=float)
        # Uniform grid -> store start, spacing, count and the values only.
        self.memory.t0 = float(times[0])
        self.memory.dtGrid = float(times[1] - times[0])
        self.memory.n = int(times.size)
        self.memory.values = values.copy()

    @staticmethod
    def UpdateStateImpl(atmoDensInMsgPayload, atmoDensInMsgIsLinked,
                        atmoDensOutMsgPayload, CurrentSimNanos, memory):
        # Linear interpolation on a uniform grid, clamped at the endpoints.
        t = CurrentSimNanos * 1.0e-9
        pos = (t - memory.t0) / memory.dtGrid
        n = memory.n
        if pos <= 0.0:
            x = memory.values[0]
        elif pos >= n - 1:
            x = memory.values[n - 1]
        else:
            i = int(pos)
            frac = pos - i
            x = memory.values[i] * (1.0 - frac) + memory.values[i + 1] * frac

        atmoDensOutMsgPayload.neutralDensity = \
            atmoDensInMsgPayload.neutralDensity * (1.0 + x)
        atmoDensOutMsgPayload.localTemp = atmoDensInMsgPayload.localTemp


# ---------------------------------------------------------------------------
# Integrator factories
# ---------------------------------------------------------------------------
def makeStochasticIntegrator(scene, method: str, seed: int):
    """Build a stochastic integrator for Arm 'sde' and seed its RNG."""
    if method == "EulerMayurama":
        integ = svIntegrators.svStochasticIntegratorMayurama(scene)
    elif method == "W2Ito1":
        integ = svIntegrators.svStochasticIntegratorW2Ito1(scene)
    elif method == "W2Ito2":
        integ = svIntegrators.svStochasticIntegratorW2Ito2(scene)
    else:
        raise ValueError(f"Unknown stochastic method: {method}")
    integ.setRNGSeed(seed)
    return integ


def makeDeterministicIntegrator(scene, method: str, relTol: Optional[float]):
    """Build a deterministic integrator for Arm 'profile'.

    Fixed-step methods (RK2/RK4) ignore ``relTol``; adaptive ones (RKF45/RKF78)
    use it to set the relative tolerance when provided.
    """
    if method == "RK2":
        return svIntegrators.svIntegratorRK2(scene)
    elif method == "RK4":
        return svIntegrators.svIntegratorRK4(scene)
    elif method == "RKF45":
        integ = svIntegrators.svIntegratorRKF45(scene)
    elif method == "RKF78":
        integ = svIntegrators.svIntegratorRKF78(scene)
    else:
        raise ValueError(f"Unknown deterministic method: {method}")
    if relTol is not None:
        integ.relTol = relTol
    return integ


# ---------------------------------------------------------------------------
# Scenario assembly
# ---------------------------------------------------------------------------
def _buildCommonScene(params: ScenarioParams, dt: float):
    """Build the sim/scene/gravity/atmosphere/drag common to both arms.

    Returns a dict of handles; the caller wires in the density model and
    integrator appropriate to its arm.  Everything that must outlive the
    function (to avoid premature garbage collection / segfaults) is returned.
    """
    planet, oe, rN, vN, orbitPeriod, mu = params.deriveOrbit()
    tf = params.orbits * orbitPeriod

    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("test")
    process.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene(CANNONBALL_SCENE_XML)
    scSim.AddModelToTask("test", scene)

    body = scene.getBody("ball")
    dragPoint = body.getCenterOfMass()

    gravity = NBodyGravity.NBodyGravity()
    scene.AddModelToDynamicsTask(gravity)
    gravityModel = pointMassGravityModel.PointMassGravityModel()
    gravityModel.muBody = planet.mu
    gravity.addGravitySource("earth", gravityModel, True)
    gravity.addGravityTarget("ball", body)

    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmo.ModelTag = "ExpAtmo"
    simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")
    atmo.addSpacecraftToModel(dragPoint.stateOutMsg)
    # IMPORTANT: the base atmosphere must run in the dynamics task (evaluated at
    # every integrator stage), NOT the outer task (macro rate).  Otherwise the
    # altitude-dependent base density is stale within a step while the density
    # correction is sampled fresh at stage times, which distorts the drag/noise
    # coupling and inflates the variance of the figure of merit (verified: ~3x
    # too large with the base atmo in the outer task).  This keeps both arms and
    # an independent scipy reference in agreement on the moments.
    scene.AddModelToDynamicsTask(atmo)

    drag = cannonballDrag.CannonballDrag()
    drag.ModelTag = "DragEff"
    dragGeometry = messaging.DragGeometryMsgPayload()
    dragGeometry.dragCoeff = params.dragCoeff
    dragGeometry.projectedArea = params.projectedArea
    dragGeometry.r_CP_S = [0.0, 0.0, 0.0]
    dragGeometryMsg = messaging.DragGeometryMsg().write(dragGeometry)
    drag.dragGeometryInMsg.subscribeTo(dragGeometryMsg)
    drag.referenceFrameStateInMsg.subscribeTo(dragPoint.stateOutMsg)

    dragActuator = scene.addForceActuator("dragForce", dragPoint)
    dragActuator.forceInMsg.subscribeTo(drag.forceOutMsg)
    scene.AddModelToDynamicsTask(drag)

    return {
        "scSim": scSim, "scene": scene, "body": body, "dragPoint": dragPoint,
        "gravity": gravity, "gravityModel": gravityModel, "atmo": atmo,
        "drag": drag, "dragActuator": dragActuator, "dragGeometryMsg": dragGeometryMsg,
        "planet": planet, "oe": oe, "rN": rN, "vN": vN, "mu": mu, "tf": tf,
        "orbitPeriod": orbitPeriod,
    }


def runRealization(
    params: ScenarioParams,
    arm: Arm,
    method: str,
    dt: float,
    seed: int,
    relTol: Optional[float] = None,
    profileGridDt: float = 0.25,
) -> RealizationResult:
    """Run a single realization and return its figure of merit + wall time.

    Args:
        params: shared scenario parameters.
        arm: "sde" (stochastic integrator) or "profile" (pre-generated noise +
            deterministic integrator).
        method: integrator method name (see SDE_METHODS / DET_METHODS).
        dt: macro task time step [s].
        seed: realization seed -- seeds the C++ RNG (sde) or the numpy profile
            RNG (profile), so the two arms can be driven by independent but
            reproducible noise streams.
        relTol: relative tolerance for adaptive deterministic integrators.
        profileGridDt: spacing [s] of the *master* OU profile grid for the
            "profile" arm.  This is held FIXED (independent of ``dt``) and fine
            enough to resolve the OU correlation time, so it defines a single
            well-posed noisy trajectory.  Refining the integrator ``dt`` then
            measures genuine *integrator* convergence toward that fixed
            trajectory's answer (not a moving noise target), which is what makes
            the deterministic arm a proper convergence study.  ``profileGridDt``
            must be <= the smallest ``dt`` used by the profile arm.

    Returns:
        RealizationResult with the final semi-major axis (FoM), altitude, and
        the wall time spent *only* inside ExecuteSimulation.
    """
    h = _buildCommonScene(params, dt)
    scSim, scene, body, dragPoint, drag = (
        h["scSim"], h["scene"], h["body"], h["dragPoint"], h["drag"])
    tf = h["tf"]

    # Keep references alive for the whole function (integrator GC segfault guard).
    integ = None
    densityModel = None
    profile = None

    if arm == "sde":
        integ = makeStochasticIntegrator(scene, method, seed)
        scene.setIntegrator(integ)

        densityModel = MJStochasticAtmDensity.StochasticAtmDensity()
        densityModel.setStationaryStd(params.stationaryStd)
        densityModel.setTimeConstant(params.timeConstant)
        densityModel.ModelTag = "StochasticExpAtmo"
        densityModel.atmoDensInMsg.subscribeTo(h["atmo"].envOutMsgs[0])
        # OU state has both drift and diffusion -> both dynamics tasks
        scene.AddModelToDynamicsTask(densityModel)
        scene.AddModelToDiffusionDynamicsTask(densityModel)

    elif arm == "profile":
        integ = makeDeterministicIntegrator(scene, method, relTol)
        scene.setIntegrator(integ)

        # Pre-generate the exact OU profile on a FIXED fine master grid (spacing
        # profileGridDt, independent of dt) so the noisy trajectory is well
        # defined and refining dt is a genuine integrator-convergence study.
        nNodes = int(np.ceil(tf / profileGridDt)) + 1
        profileTimes = np.linspace(0.0, tf, nNodes)
        rng = np.random.default_rng(seed)
        profileValues = generateOUPath(
            profileTimes, params.stationaryStd, params.timeConstant, rng)
        profile = (profileTimes, profileValues)

        densityModel = ProfileAtmDensity(profileTimes, profileValues)
        densityModel.ModelTag = "ProfileExpAtmo"
        densityModel.atmoDensInMsg.subscribeTo(h["atmo"].envOutMsgs[0])
        scene.AddModelToDynamicsTask(densityModel)

    else:
        raise ValueError(f"Unknown arm: {arm}")

    # Drag reads the (corrected) density from whichever model this arm uses.
    drag.atmoDensInMsg.subscribeTo(densityModel.atmoDensOutMsg)

    scSim.InitializeSimulation()
    body.setPosition(h["rN"])
    body.setVelocity(h["vN"])

    scSim.ConfigureStopTime(macros.sec2nano(tf))

    # Time ONLY the integration, not the build / InitializeSimulation, so the
    # comparison reflects per-step integrator cost.
    t0 = time.perf_counter()
    scSim.ExecuteSimulation()
    wall = time.perf_counter() - t0

    # Read final state directly from the body's origin site.
    state = body.getOrigin().stateOutMsg.read()
    r = np.array(state.r_BN_N, dtype=float)
    v = np.array(state.v_BN_N, dtype=float)
    finalOe = orbitalMotion.rv2elem(h["mu"], r, v)
    altitude = (np.linalg.norm(r) - h["planet"].radEquator) / 1000.0
    nSteps = int(np.round(tf / dt))

    # Explicitly drop references so the next realization starts clean.
    del integ, densityModel, profile, scene, scSim

    return RealizationResult(
        fomSemiMajorAxis=float(finalOe.a),
        fomAltitude=float(altitude),
        wallSeconds=float(wall),
        nSteps=nSteps,
    )
