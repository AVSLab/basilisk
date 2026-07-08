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
  evolved *inline* by a stochastic integrator -- one of the Roessler additive
  strong-order-1.5 methods (SRA1, SOSRA) or the Tang & Xiao weak-order-2 method
  (W2Ito2).  One simulation = one realization of the noisy trajectory.  The
  macro time step ``dt`` can be large because these methods are designed to
  capture the trajectory (SRA1/SOSRA) or the weak statistics (W2Ito2) of the
  noise with high order.  The OU density correction is *additive* noise (its
  diffusion ``sigma_st*sqrt(2/tau)`` is state-independent), which is exactly the
  regime the SRA family is built for.

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

# Numba is used to compile the OU-path recurrence (Arm "profile") to C speed, so
# the cost of generating the noise -- which is now *timed* and charged to the
# profile arm for a fair comparison against the SDE arm's inline C++ noise draws
# -- reflects a compiled generator, not Python-interpreter overhead. Imported
# independently of Basilisk (it is also what backs the ProfileAtmDensity
# NumbaModel).
#
# We deliberately do NOT provide a pure-Python fallback for the recurrence: if
# numba is missing, running the profile arm would time an interpreted loop that
# is ~100-150x slower than the compiled one, silently and unfairly inflating the
# profile arm's wall time and corrupting the whole comparison. So the module
# stays importable without numba (login-node metadata ops still work), but the
# recurrence itself RAISES if it is ever *called* without numba -- see
# ``_ouRecurrence``. ``_njit`` below is only a decorator placeholder for that
# no-numba import case; it is never actually run as the loop.
try:
    from numba import njit as _njit
    _NUMBA_AVAILABLE = True
except Exception:  # numba missing (e.g. login node): import OK, but no timed run
    _NUMBA_AVAILABLE = False

    def _njit(*args, **kwargs):
        """Placeholder so the ``@_njit`` decorator parses when numba is absent.

        It intentionally leaves the decorated function as-is; that function
        (``_ouRecurrence``) guards its own body and raises if invoked without
        numba, so no interpreted recurrence is ever timed.
        """
        if len(args) == 1 and callable(args[0]) and not kwargs:
            return args[0]
        def _decorator(func):
            return func
        return _decorator

# Basilisk is imported lazily/guarded so that this module can be imported WITHOUT
# a working Basilisk install -- e.g. on a Slurm login node, where compiled .so
# files may fail to load (missing GLIBCXX, blocked compiler module). That lets
# metadata-only operations (config grid, ScenarioParams, --printTaskCount used by
# run_study.sh to size the array, the OU path generator) work on the login node.
# Anything that actually runs a realization needs Basilisk and is only ever
# called inside a Slurm job (compute node), where the import succeeds.
try:
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
    _BASILISK_AVAILABLE = True
    _BASILISK_IMPORT_ERROR = None
    _NumbaModelBase = NumbaModel
    _SysModelBase = sysModel.SysModel
except Exception as _exc:  # ImportError, or .so load failure
    _BASILISK_AVAILABLE = False
    _BASILISK_IMPORT_ERROR = _exc
    # Dummy bases so the class statements below are still valid Python; these
    # classes are only ever *instantiated* inside a realization (compute node).
    _NumbaModelBase = object
    _SysModelBase = object


def _requireBasilisk():
    """Raise a clear error if a realization is attempted without Basilisk."""
    if not _BASILISK_AVAILABLE:
        raise RuntimeError(
            "Basilisk could not be imported in this process, so a simulation "
            "cannot run here (this is expected on a login node). Original error: "
            f"{_BASILISK_IMPORT_ERROR!r}")


# Arm identifiers
Arm = Literal["sde", "profile"]

# Stochastic-integrator method names (Arm "sde").
#   * SRA1   -- Roessler SRA, strong order 1.5 / weak order 2, ADDITIVE noise
#   * SOSRA  -- stability-optimized SRA1 (same orders), ADDITIVE noise
#   * W2Ito2 -- Tang & Xiao, weak order 2 (general Ito noise)
# SRA1/SOSRA are valid here because the OU density correction is additive (its
# diffusion sigma_st*sqrt(2/tau) does not depend on the state).
SDE_METHODS = ("SRA1", "SOSRA", "W2Ito2")

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
    wallSeconds: float                 # [s] integration wall; profile arm also
                                       #     includes its (numba) OU-path gen
    nSteps: int                        # number of macro steps taken


# ---------------------------------------------------------------------------
# Exact OU path generation (Arm "profile")
# ---------------------------------------------------------------------------
def _ouRecurrenceGuard():
    """Raise if the OU recurrence is invoked without numba (never fall back).

    Kept as a separate helper so it stays pure Python: it must run even in the
    no-numba build, where ``_ouRecurrence`` is the raw (uncompiled) function.
    """
    raise RuntimeError(
        "numba is required to generate the OU noise path but could not be "
        "imported. The profile arm times its noise generation, so running it "
        "with an interpreted (uncompiled) recurrence -- ~100x slower -- would "
        "silently and unfairly inflate the profile arm's wall time and corrupt "
        "the comparison. Install numba in this environment (it is also needed by "
        "the ProfileAtmDensity NumbaModel) before running any timed profile-arm "
        "realization.")


@_njit(cache=True, fastmath=False)
def _ouRecurrence(decay, noiseStd, z, x0):
    """OU exact-transition recurrence, compiled to C speed by numba.

    This is the sequential (non-vectorizable) hot loop of :func:`generateOUPath`.
    It is isolated so it can be JIT-compiled: the interpreted cost of this loop
    (~100x slower) would otherwise unfairly penalise the profile arm now that OU
    generation is *timed* and charged to that arm.  The Gaussian draws ``z`` are
    supplied by the caller (from numpy's ``default_rng``) so the produced path is
    bit-identical to the pure-Python version and stays shareable with the scipy
    reference for CRN.

    When numba is present this whole guard folds away at compile time (the global
    ``_NUMBA_AVAILABLE`` is a constant ``True``); when numba is absent the
    function is plain Python and the guard raises rather than run interpreted.
    """
    if not _NUMBA_AVAILABLE:
        _ouRecurrenceGuard()
    n = z.shape[0]
    x = np.empty(n + 1, dtype=np.float64)
    x[0] = x0
    for k in range(n):
        x[k + 1] = x[k] * decay[k] + noiseStd[k] * z[k]
    return x


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
    times = np.asarray(times, dtype=float)
    dts = np.diff(times)
    # Vectorized per-step transition constants.
    decay = np.exp(-theta * dts)
    noiseStd = stationaryStd * np.sqrt(np.maximum(1.0 - np.exp(-2.0 * theta * dts), 0.0))
    # Draw the standard normals with numpy (NOT numba) so the path is
    # bit-identical to the reference's and shareable for CRN; run the sequential
    # recurrence in the compiled kernel.
    z = rng.standard_normal(dts.size)
    if times.size == 1:
        return np.array([x0], dtype=float)
    return _ouRecurrence(decay, noiseStd, z, float(x0))


# Process-wide guard so the numba JIT of _ouRecurrence is triggered at most once.
_OU_WARMED_UP = False


def _warmUpOUPath(params: "ScenarioParams", profileTimes: np.ndarray) -> None:
    """Trigger numba's one-time JIT compile of the OU recurrence, untimed.

    Called once per process before the first *timed* profile realization so the
    compile latency is never charged to any realization's wall.  Uses tiny dummy
    inputs of the SAME dtype/shape-signature as the real call, which is all numba
    needs to specialise the kernel; the real path is generated normally afterward.
    A no-op after the first call and harmless if numba is absent (``_ouRecurrence``
    is then a plain Python function).
    """
    global _OU_WARMED_UP
    if _OU_WARMED_UP or not _NUMBA_AVAILABLE:
        _OU_WARMED_UP = True
        return
    decay = np.ones(2, dtype=float)
    noiseStd = np.zeros(2, dtype=float)
    z = np.zeros(2, dtype=float)
    _ouRecurrence(decay, noiseStd, z, 0.0)
    _OU_WARMED_UP = True


class ProfileAtmDensityPy(_SysModelBase):
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


class ProfileAtmDensity(_NumbaModelBase):
    """Replays a pre-generated OU density-correction path ``x(t)`` at C speed.

    This is the deterministic-arm counterpart of
    :ref:`StochasticAtmDensity<stochasticAtmDensity>`.  It is a
    :ref:`NumbaModel<numbaModules>`, so ``UpdateStateImpl`` JIT-compiles once to
    a C-callable that the Basilisk scheduler invokes directly every integrator
    stage with zero per-tick Python overhead.  This is essential for a fair
    speed comparison: the C++ stochastic arm and this profile arm then differ in
    wall time only because of their *algorithms* (step size, number of stage
    evaluations), not the implementation language.  The one-time OU-path
    *generation* charged to the profile arm is likewise compiled (see
    :func:`_ouRecurrence`), so the fairness holds for the whole timed cost, not
    just the per-stage replay.

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
    """Build a stochastic integrator for Arm 'sde' and seed its RNG.

    The concrete integrator classes all live in ``svIntegrators`` and follow the
    ``svStochasticIntegrator<Method>`` naming, take the scene (``DynamicObject``)
    as their sole constructor argument, and expose ``setRNGSeed`` (inherited from
    ``StochasticRKIntegratorBase``, which forwards the seed to the integrator's
    default ``RandomGaussianNoiseGenerator``).  Reseeding per realization gives
    each run an independent, reproducible Wiener stream.
    """
    className = f"svStochasticIntegrator{method}"
    if not hasattr(svIntegrators, className):
        raise ValueError(f"Unknown stochastic method: {method} "
                         f"(no {className} in svIntegrators)")
    integ = getattr(svIntegrators, className)(scene)
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
        RealizationResult with the final semi-major axis (FoM), altitude, and the
        per-realization wall time.  For the SDE arm this is the time inside
        ExecuteSimulation (which already includes drawing the noise inline).  For
        the profile arm it is ExecuteSimulation PLUS the time to generate the OU
        noise path (numba-compiled), so both arms are charged for producing their
        noise -- otherwise the profile arm would look artificially cheaper.  Build
        and InitializeSimulation are excluded from both.
    """
    _requireBasilisk()
    h = _buildCommonScene(params, dt)
    scSim, scene, body, dragPoint, drag = (
        h["scSim"], h["scene"], h["body"], h["dragPoint"], h["drag"])
    tf = h["tf"]

    # Keep references alive for the whole function (integrator GC segfault guard).
    integ = None
    densityModel = None
    profile = None
    # Extra wall charged to the profile arm for generating its OU noise path; the
    # SDE arm draws its noise inline (already inside the timed region), so this
    # stays zero for it.
    ouGenWall = 0.0

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
        #
        # The GENERATION of this path is a real cost the profile arm pays that
        # the SDE arm does not (the SDE arm draws its noise inline, inside the
        # timed ExecuteSimulation). To compare the two arms fairly we TIME the
        # generation and add it to this realization's wall (below). To keep that
        # charge fair, the recurrence is numba-compiled (a production noise
        # generator would be compiled, just as the SDE arm's noise is C++), and
        # the JIT is warmed up ONCE outside the timed region so no realization
        # pays the one-time compile cost.
        nNodes = int(np.ceil(tf / profileGridDt)) + 1
        profileTimes = np.linspace(0.0, tf, nNodes)
        _warmUpOUPath(params, profileTimes)
        rng = np.random.default_rng(seed)
        tOU = time.perf_counter()
        profileValues = generateOUPath(
            profileTimes, params.stationaryStd, params.timeConstant, rng)
        ouGenWall = time.perf_counter() - tOU
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

    # Time the integration.  For the SDE arm this already includes the cost of
    # drawing the noise inline; for the profile arm we ADD the (separately timed,
    # numba-compiled) OU-path generation so both arms are charged for producing
    # their noise -- see the comment where ouGenWall is measured.
    t0 = time.perf_counter()
    scSim.ExecuteSimulation()
    wall = time.perf_counter() - t0
    wall += ouGenWall

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
