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
"""
Tests the Euler-Maruyama stochastic integrator (svStochasticIntegratorMayurama).

This module covers the deterministic (zero-noise) limit, single-realization agreement
with a Python Euler-Maruyama reference, and Monte-Carlo weak-convergence of the mean and
variance for the Ornstein-Uhlenbeck process. The higher-order weak Runge-Kutta methods
(W2Ito1, W2Ito2, and the Roessler families) are verified separately by exact numerical
equivalence to committed reference trajectories (test_stochasticIntegratorsJulia.py and
test_stochasticIntegratorsPaper.py).
"""
from __future__ import annotations

from typing import Callable, List, Literal, get_args, Any
import tqdm
import itertools

import pytest
import numpy as np
import numpy.typing as npt
import numpy.testing
import matplotlib.pyplot as plt

from Basilisk.utilities import macros
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import pythonVariableLogger
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import dynParamManager

try:
    from Basilisk.simulation import mujoco
    couldImportMujoco = True
except:
    couldImportMujoco = False

# This module provides the Euler-Maruyama coverage (single-path Ornstein-Uhlenbeck
# agreement and Monte-Carlo weak-convergence checks) plus the DETERMINISTIC-LIMIT check
# run across every stochastic integrator. The weak/strong-order Runge-Kutta methods are
# additionally verified by exact numerical equivalence to committed references in
# test_stochasticIntegratorsJulia.py and test_stochasticIntegratorsPaper.py.
Method = Literal["EulerMayurama"]
METHODS = get_args(Method)

# Every concrete stochastic integrator, exercised on a zero-noise (deterministic) system
# by test_deterministic. This both checks each integrator's drift tableau against the
# analytic ODE solution (an oracle-INDEPENDENT check) and guards the m==0 code path: a
# weak/strong method that indexes a per-source noise vector outside its k<m loop would
# read an empty Eigen vector here (see the W2Ito m==0 regression). Keep in sync with the
# classes exposed in svIntegrators.i.
ALL_INTEGRATORS = [
    "svStochasticIntegratorMayurama",
    "svStochasticIntegratorSRIW1", "svStochasticIntegratorSOSRI",
    "svStochasticIntegratorSRA1", "svStochasticIntegratorSOSRA",
    "svStochasticIntegratorEulerHeun", "svStochasticIntegratorRKMil",
    "svStochasticIntegratorW2Ito1", "svStochasticIntegratorW2Ito2",
    "svStochasticIntegratorDRI1", "svStochasticIntegratorDRI1NM",
    "svStochasticIntegratorRI1", "svStochasticIntegratorRI3",
    "svStochasticIntegratorRI5", "svStochasticIntegratorRI6",
    "svStochasticIntegratorRS1", "svStochasticIntegratorRS2",
    "svStochasticIntegratorSIEA", "svStochasticIntegratorSMEA",
    "svStochasticIntegratorSIEB", "svStochasticIntegratorSMEB",
    "svStochasticIntegratorRDI1WM",
]

# Function of the form y = f(t,x) where x and y are vectors of the same size
DynFunc = Callable[[float, npt.NDArray[np.float64]], npt.NDArray[np.float64]]

def eulerMayuramaIntegrate(
        f: DynFunc,
        g_list: List[DynFunc],
        x0: npt.NDArray[np.float64],
        dt: float,
        tf: float,
        rng_seed: int
    ):
    """
    Euler-Mayurama integrator for the vector SDE:
        dX = f(t,X) dt + sum_k g_list[k](t,X) dW_k

    Args:
        f: Drift function.
        g_list: List of diffusion functions.
        x0: Initial state.
        dt: Time step.
        tf: Final time.
        rng_seed: Random seed.
    Returns:
        Array of state trajectories, including time as the first column.
    """
    n = x0.size
    m = len(g_list)

    nsteps = int(np.floor(tf / dt))

    t = dt * np.arange(nsteps+1)
    x = np.zeros([nsteps+1, n], dtype=float)
    x[0,:] = x0

    # Mirror the Basilisk integrator's noise source: the shared RandomGaussianNoiseGenerator.
    # Euler-Mayurama uses only the Wiener increment dW (the generator also draws dZ, which
    # this method ignores, exactly as the C++ integrator does). Basilisk's timeStep == 0
    # init call is skipped by the integrator's guard, so no throw-away sample here.
    rng = svIntegrators.RandomGaussianNoiseGenerator()
    rng.setSeed(rng_seed)

    for step in range(nsteps):
        dW = rng.generate(m, dt).dW

        x[step+1,:] = x[step,:] + f(t[step], x[step,:])*dt
        for k, g in enumerate(g_list):
            x[step+1,:] += g(t[step], x[step,:])*dW[k]

    return np.column_stack([t, x])

class ExponentialSystem:
    """A simple deterministic system with one state: dx/dt = x*t."""

    def __init__(self, x0: float = 1):
        """Initialize the system with initial state x0."""
        self.x0 = np.array([x0])

    def f(self, t: float, x: npt.NDArray[np.float64]):
        """Drift function for the exponential system."""
        return np.array([t*x[0]])

    g = []

class OrnsteinUhlenbeckSystem:
    """A process defined by

        dx = theta*(mu - x)*dt + sigma*dW
    """
    def __init__(self, x0: float = .1, mu: float = 0, theta: float = .1, sigma: float = .01):
        """Initialize the OU process parameters."""
        self.x0 = np.array([x0])
        self.mu = mu
        self.theta = theta
        self.sigma = sigma

    def f(self, t: float, x: npt.NDArray[np.float64]):
        """Drift function for the OU process."""
        return np.array([self.theta*(self.mu - x[0])])

    def g1(self, t: float, x: npt.NDArray[np.float64]):
        """Diffusion function for the OU process."""
        return np.array([self.sigma])

    @property
    def g(self):
        """Return list of diffusion functions."""
        return [self.g1]

    def mean(self, t: float):
        """E[x(t)]"""
        return np.array([
            self.x0[0]*np.exp(-self.theta*t) + self.mu*(1 - np.exp(-self.theta*t))
        ])

    def var(self, t: float):
        """Var(x(t))"""
        return np.array([
            self.sigma**2/(2*self.theta) * (1- np.exp(-2*self.theta*t))
        ])

class ComplexOrnsteinUhlenbeckSystem:
    """A process defined by

        dx = -theta*x*dt + sigma_x1*dW_1 + sigma_x2*dW_2
        dy = -theta*y*dt + sigma_y1*dW_1 + sigma_y2*dW_2
    """
    def __init__(
            self,
            x0: float = .1, y0: float = -.1,
            theta_x: float = .1, theta_y: float = .073,
            sigma_x1: float = .015, sigma_x2: float = .011,
            sigma_y1: float = 0, sigma_y2: float = .029
        ):
        """Initialize the complex OU process parameters."""
        self.x0 = np.array([x0, y0])
        self.theta_x = theta_x
        self.theta_y = theta_y
        self.sigma_x1 = sigma_x1
        self.sigma_x2 = sigma_x2
        self.sigma_y1 = sigma_y1
        self.sigma_y2 = sigma_y2

    def f(self, t: float, x: npt.NDArray[np.float64]):
        """Drift function for the complex OU process."""
        return np.array([-self.theta_x*x[0], -self.theta_y*x[1]])

    def g1(self, t: float, x: npt.NDArray[np.float64]):
        """First diffusion function for the complex OU process."""
        return np.array([self.sigma_x1, self.sigma_y1])

    def g2(self, t: float, x: npt.NDArray[np.float64]):
        """Second diffusion function for the complex OU process."""
        return np.array([self.sigma_x2, self.sigma_y2])

    @property
    def g(self):
        """Return list of diffusion functions."""
        return [self.g1, self.g2]

class Example1System:
    """Example 1 dynamical system in:

        Tang, X., Xiao, A. Efficient weak second-order stochastic Runge-Kutta methods
        for Itô stochastic differential equations. Bit Numer Math 57, 241-260 (2017).
        https://doi.org/10.1007/s10543-016-0618-9
    """
    def __init__(self, y1_0: float = 1, y2_0: float = 1):
        """Initialize the Example 1 system."""
        self.x0 = np.array([y1_0, y2_0])

    def f(self, t: float, x: npt.NDArray[np.float64]):
        """Drift function for Example 1 system."""
        y1, y2 = x
        return np.array([
            -273/512*y1,
            -1/160*y1 - (785/512 - np.sqrt(2)/8)*y2
        ])

    def g1(self, t: float, x: npt.NDArray[np.float64]):
        """First diffusion function for Example 1 system."""
        y1, y2 = x
        return np.array([
            1/4*y1,
            (1 - 2*np.sqrt(2)/8)/4*y2
        ])

    def g2(self, t: float, x: npt.NDArray[np.float64]):
        """Second diffusion function for Example 1 system."""
        y1, y2 = x
        return np.array([
            1/16*y1,
            1/10*y1 + 1/16*y2
        ])

    @property
    def g(self):
        """Return list of diffusion functions."""
        return [self.g1, self.g2]

def getBasiliskSim(method: Method, dt: float, x0: npt.NDArray[np.float64], f: DynFunc, g: List[DynFunc], seed: int | None):
    """
    Set up and return a Basilisk simulation for a given SDE and integrator method.

    Args:
        method: Integration method (only "EulerMayurama").
        dt: Time step.
        x0: Initial state.
        f: Drift function.
        g: List of diffusion functions.
        seed: RNG seed (or None for random).

    Returns:
        Tuple of (scSim, stateModel, integratorObject, stateLogger).
    """
    from Basilisk.simulation import StatefulSysModel

    # Declared inside, since StatefulSysModel may be undefined if not running with mujoco
    class GenericStochasticStateModel(StatefulSysModel.StatefulSysModel):

        def __init__(self, x0: npt.NDArray[np.float64], f: DynFunc, g: List[DynFunc]):
            super().__init__()
            self.x0 = x0
            self.f = f
            self.g = g

        def registerStates(self, registerer: StatefulSysModel.DynParamRegisterer):
            """Called once during InitializeSimulation"""
            # We get one noise source per diffusion function g:
            m = len(self.g)
            n = self.x0.size

            self.states: List[dynParamManager.StateData] = []
            for i in range(n):
                self.states.append( registerer.registerState(1, 1, f"y{i+1}") )
                self.states[-1].setNumNoiseSources(m)
                self.states[-1].setState([[self.x0[i]]])

            # We want every noise source to be shared between states
            for k in range(m):
                registerer.registerSharedNoiseSource([
                    (state, k)
                    for state in self.states
                ])

        def UpdateState(self, CurrentSimNanos: int):
            """Called at every integrator step"""
            m = len(self.g)
            n = self.x0.size

            t = macros.NANO2SEC * CurrentSimNanos

            # Collect current state into a numpy array
            x = np.array([state.getState()[0][0] for state in self.states])

            # Compute f and store in the derivative of the states
            deriv = self.f(t, x)
            for i in range(n):
                self.states[i].setDerivative( [[deriv[i]]] )

            # Compute g_k and store in the diffusion of the states
            for k in range(m):
                diff = self.g[k](t, x)
                for i in range(n):
                    self.states[i].setDiffusion( [[diff[i]]], index=k )

    # Create sim, process, and task
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene("<mujoco/>") # empty scene, no multi-body dynamics
    scSim.AddModelToTask("test", scene)

    # ``method`` is either one of the Method literals (e.g. "EulerMayurama") or the exact
    # class name of any stochastic integrator (used by test_deterministic to sweep all of
    # ALL_INTEGRATORS). Map the friendly literal, otherwise resolve the class by name.
    if method == "EulerMayurama":
        integratorClass = svIntegrators.svStochasticIntegratorMayurama
    elif hasattr(svIntegrators, method):
        integratorClass = getattr(svIntegrators, method)
    else:
        raise NotImplementedError(method)

    integratorObject = integratorClass(scene)
    if seed is not None:
        integratorObject.setRNGSeed(seed)
    scene.setIntegrator(integratorObject)

    stateModel = GenericStochasticStateModel(x0, f, g)
    stateModel.ModelTag = "testModel"

    # The same model computes both the drift and diffusion of the state
    # so they must be added to both tasks
    scene.AddModelToDynamicsTask(stateModel)
    scene.AddModelToDiffusionDynamicsTask(stateModel)

    stateLogger = pythonVariableLogger.PythonVariableLogger(
        {"x": lambda _: np.array([state.getState()[0][0] for state in stateModel.states])}
    )
    scSim.AddModelToTask("test", stateLogger)

    scSim.InitializeSimulation()

    return scSim, stateModel, integratorObject, stateLogger

def estimateErrorAndEmpiricalVariance(
        computeTrajectory: Callable[[], npt.NDArray[np.float64]],
        G: Callable[[npt.NDArray[np.float64]], float],
        estimateGOnTrajectory: float,
        M1: int,
        M2: int,
    ):
    r"""Computes the error and empirical variance according to the
    equations described in Section 4 of Tang & Xiao.

    Args:
        computeTrajectory (Callable[[], npt.NDArray[np.float64]]): A function that,
        when called, realizes one simulation by forward propagating the system from
        t0. The result is the state at the end point. In the paper: ``y_N``.
        G (Callable[[npt.NDArray[np.float64]], float]): A differentiable function
        that takes in a state vector and outputs a single number.
        estimateGOnTrajectory (float): The estimate of the application of the function
        G on the random variable that is the last state of the propagated system.
        In the paper: ``E[G(y(t_N))]``
        M1 (int): Number of batches.
        M2 (int): Number of trajectories per batch.

    Returns:
        tuple[float, float]: The integrator error (``\hat{\mu}`` in the paper), and
        the empirical variance (``\hat{\sigma}_\mu^2`` in the paper).
    """

    muHat = [0. for _ in range(M1)]
    for j, _ in tqdm.tqdm(itertools.product(range(M1), range(M2)), total=M1*M2):
        muHat[j] += (G(computeTrajectory()) - estimateGOnTrajectory)/M2

    muHatAvg = sum(muHat) / M1

    sigmaMuSquared = 0
    for j in range(M1):
        sigmaMuSquared += (muHat[j] - muHatAvg)**2/(M1-1)

    return muHatAvg, sigmaMuSquared


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("method", METHODS)
def test_deterministic(method: Method, plot: bool = False):
    """
    Test deterministic integration (no diffusion) for Euler-Maruyama.
    Compares Basilisk and Python implementations against the analytical solution
    for the exponential system dx/dt = x*t.

    Args:
        method: Integration method (only "EulerMayurama").
        plot: If True, plot the relative error.
    """

    dt = .01
    tf = 1
    seed = 10

    system = ExponentialSystem()

    scSim, stateModel, integratorObject, stateLogger = getBasiliskSim(method, dt, system.x0, system.f, system.g, seed)
    scSim.ConfigureStopTime( macros.sec2nano(tf) )
    scSim.ExecuteSimulation()

    tBasilisk = macros.NANO2SEC* stateLogger.times()
    xBasilisk = stateLogger.x

    txPython = eulerMayuramaIntegrate(system.f, system.g, system.x0, dt, tf, seed)
    tPython = txPython[:,0]
    xPython = txPython[:,1]

    xExpected = np.exp( tPython **2 / 2 )

    if plot:
        fig, ax = plt.subplots()
        ax.plot(tBasilisk, 100*(xBasilisk - xExpected)/xExpected, label="Basilisk")
        ax.plot(tPython, 100*(xPython - xExpected)/xExpected, ls="--", label="Python")
        ax.legend()
        ax.set_ylabel("Relative Error [%]")
        plt.show()

    # The Python and Basilisk implementation should behave identially
    numpy.testing.assert_allclose(
        xBasilisk[-1],
        xPython[-1],
        atol=1e-12,
        rtol=0
    )

    expectedIntegrationError = .05

    # The Basilisk should have some integration error w.r.t analytical solution
    numpy.testing.assert_allclose(
        xBasilisk[-1],
        xExpected[-1],
        atol=expectedIntegrationError,
        rtol=0
    )


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("integratorClassName", ALL_INTEGRATORS)
def test_deterministicLimitAllIntegrators(integratorClassName: str):
    """Every stochastic integrator must reduce to a correct ODE solver when there is no
    noise. This runs each integrator on a ZERO-NOISE (m==0) system and checks it against
    the analytic solution.

    This check is deliberately oracle-INDEPENDENT: unlike the Julia/paper equivalence
    tests (which replay prescribed increments through the same recurrence the reference
    used), the exponential system has a known closed-form solution, so a wrong drift
    tableau (alpha / A0 / c0) or a broken deterministic step is caught here even if the
    committed reference happened to share the same mistake.

    It also guards the m==0 code path: an integrator that reads a per-source noise entry
    (dW/dZ) outside its ``k < m`` loop reads an empty Eigen vector when there are no noise
    sources, which this test exercises directly.

    The system is dx/dt = -x (with g = []), exact solution x(t) = x0 * exp(-t). A decaying
    linear drift is used (rather than the growing dx/dt = x*t) so that a mis-scaled drift
    weight produces a clearly bounded, easily-diagnosed error.
    """
    dt = 1.0 / 32.0
    tf = 1.0
    x0 = np.array([2.0])

    f = lambda t, x: np.array([-x[0]])   # dx/dt = -x
    g: List[DynFunc] = []                # zero noise sources => m == 0 (deterministic)

    scSim, stateModel, integratorObject, stateLogger = getBasiliskSim(
        integratorClassName, dt, x0, f, g, seed=None)
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    xBasilisk = np.array(stateLogger.x, dtype=float).reshape(-1)
    xExpected = x0[0] * np.exp(-tf)

    # Tolerance is set by the method's DETERMINISTIC (zero-noise) order, which is the
    # order of its drift step, not its stochastic order:
    #   - Euler-Maruyama and RKMil take a plain forward-Euler drift step (order 1), so at
    #     dt = 1/32 the analytic error is ~1.2e-2 (matches hand-computed Euler exactly).
    #   - every other method here has a higher-order (>=2) drift tableau, error <~1e-3.
    # A mis-scaled or mis-placed drift weight pushes the error well past these bounds.
    orderOneMethods = {"svStochasticIntegratorMayurama", "svStochasticIntegratorRKMil"}
    tol = 2e-2 if integratorClassName in orderOneMethods else 2e-3
    numpy.testing.assert_allclose(
        xBasilisk[-1], xExpected, atol=tol, rtol=0,
        err_msg=f"{integratorClassName}: deterministic-limit solution is wrong",
    )


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("method", METHODS)
def test_ou(method: Method, plot: bool = False):
    """
    Test Ornstein-Uhlenbeck process integration for all integrator methods.
    Compares Basilisk and Python implementations for a single realization.

    Args:
        method: Integration method.
        plot: If True, plot the state trajectories.
    """

    dt = 1
    tf = 10
    seed = 10

    system = OrnsteinUhlenbeckSystem()

    scSim, stateModel, integratorObject, stateLogger = getBasiliskSim(method, dt, system.x0, system.f, system.g, seed)
    scSim.ConfigureStopTime( macros.sec2nano(tf) )
    scSim.ExecuteSimulation()

    tBasilisk = macros.NANO2SEC* stateLogger.times()
    xBasilisk = stateLogger.x

    txPython = eulerMayuramaIntegrate(system.f, system.g, system.x0, dt, tf, seed)
    tPython = txPython[:,0]
    xPython = txPython[:,1]

    if plot:
        fig, ax = plt.subplots()
        ax.plot(tBasilisk, xBasilisk, label="Basilisk")
        ax.plot(tPython, xPython, ls="--", label="Python")
        ax.legend()
        ax.set_ylabel("x")
        ax.set_xlabel("t")
        plt.show()

    # The Python and Basilisk implementation should behave identially
    numpy.testing.assert_allclose(
        xBasilisk[-1],
        xPython[-1],
        atol=1e-12,
        rtol=0
    )

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("method", METHODS)
def test_ouComplex(method: Method, plot: bool = False):
    """
    Test integration of a two-dimensional coupled Ornstein-Uhlenbeck process.
    Compares Basilisk and Python implementations for a single realization.

    Args:
        method: Integration method.
        plot: If True, plot the state trajectories.
    """

    dt = 1
    tf = 10
    seed = 10

    system = ComplexOrnsteinUhlenbeckSystem()

    scSim, stateModel, integratorObject, stateLogger = getBasiliskSim(method, dt, system.x0, system.f, system.g, seed)
    scSim.ConfigureStopTime( macros.sec2nano(tf) )
    scSim.ExecuteSimulation()

    tBasilisk = macros.NANO2SEC* stateLogger.times()
    xBasilisk = stateLogger.x

    txPython = eulerMayuramaIntegrate(system.f, system.g, system.x0, dt, tf, seed)
    tPython = txPython[:,0]
    xPython = txPython[:,1:]

    if plot:
        fig, axs = plt.subplots(2, sharex=True)
        for i, ax in enumerate(axs):
            ax.plot(tBasilisk, xBasilisk[:,i], label="Basilisk")
            ax.plot(tPython, xPython[:,i], ls="--", label="Python")
            ax.legend()
            ax.set_ylabel("xy"[i])
            ax.set_xlabel("t")
        plt.show()

    # The Python and Basilisk implementation should behave identially
    numpy.testing.assert_allclose(
        xBasilisk[-1,:],
        xPython[-1,:],
        atol=1e-12,
        rtol=0
    )

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("method", METHODS)
def test_example1(method: Method, plot: bool = False):
    """
    Test Example 1 system from Tang & Xiao (2017) for all integrator methods.
    Compares Basilisk and Python implementations for a single realization.

    Args:
        method: Integration method.
        plot: If True, plot the state trajectories.
    """

    dt = 1
    tf = 10
    seed = 10

    system = Example1System()

    scSim, stateModel, integratorObject, stateLogger = getBasiliskSim(method, dt, system.x0, system.f, system.g, seed)
    scSim.ConfigureStopTime( macros.sec2nano(tf) )
    scSim.ExecuteSimulation()

    tBasilisk = macros.NANO2SEC* stateLogger.times()
    xBasilisk = stateLogger.x

    txPython = eulerMayuramaIntegrate(system.f, system.g, system.x0, dt, tf, seed)
    tPython = txPython[:,0]
    xPython = txPython[:,1:]

    if plot:
        fig, axs = plt.subplots(2, sharex=True)
        for i, ax in enumerate(axs):
            ax.plot(tBasilisk, xBasilisk[:,i], label="Basilisk")
            ax.plot(tPython, xPython[:,i], ls="--", label="Python")
            ax.legend()
            ax.set_ylabel(f"y{i+1}")
            ax.set_xlabel("t")
        plt.show()

    # The Python and Basilisk implementation should behave identially
    numpy.testing.assert_allclose(
        xBasilisk[-1,:],
        xPython[-1,:],
        atol=1e-12,
        rtol=0
    )

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.flaky(reruns=6)
@pytest.mark.parametrize("method", METHODS)
@pytest.mark.parametrize("figureOfMerit", ["mean", "variance"])
def test_validateOu(
        method: Method,
        figureOfMerit: Literal["mean", "variance"],
        tf: float = 0.1
    ):
    """
    Validate the weak accuracy of the integrators for the Ornstein-Uhlenbeck process.
    Compares the empirical mean or variance of the final state to the analytical value
    using multiple Monte Carlo batches.

    Args:
        method: Integration method.
        figureOfMerit: "mean" or "variance" to validate.
    """

    dt = .05

    system = OrnsteinUhlenbeckSystem()

    def basiliskTrajectory():
        scSim, stateModel, integratorObject, stateLogger = getBasiliskSim(method, dt, system.x0, system.f, system.g, None)
        scSim.ConfigureStopTime( macros.sec2nano(tf) )
        scSim.ExecuteSimulation()

        xBasilisk = stateLogger.x

        return np.array([xBasilisk[-1]])

    # G is some differentiable function on the final state after
    # propagation
    # estimateGOnTrajectory should be the correct E[G(x(t))]

    if figureOfMerit == "mean":
        def G(arr: npt.NDArray[np.float64]) -> float:
            return arr[0]

        # E[G(x(t))] = E[x(t)]
        estimateGOnTrajectory = system.mean(tf)[0]

    elif figureOfMerit == "variance":
        def G(arr: npt.NDArray[np.float64]) -> float:
            return arr[0]**2

        # E[G(x(t))] = E[x(t)**2] = Var(x(t)) + E[x(t)]**2
        estimateGOnTrajectory = system.var(tf)[0] + system.mean(tf)[0]**2

    err, varErr = estimateErrorAndEmpiricalVariance(
        basiliskTrajectory,
        G,
        estimateGOnTrajectory,
        M1 = 10,
        M2 = 100
    )
    twoSigma = 2*np.sqrt(varErr)

    print(method, figureOfMerit, "error", err, "+-", twoSigma)

    # We expect the error to be zero, but we allow some tolerance
    # given that the error is estimated with a certain variance
    np.testing.assert_allclose(
        err,
        0,
        atol = twoSigma,
        rtol = 0
    )

# when running in pytest, we use skipAssert=True, because the test
# keeps failing for low tf and we can't afford a high tf at CI testing time
@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.flaky(reruns=6)
@pytest.mark.parametrize("method", METHODS)
def test_validateExample1(method: Method, tf: float = 0.1, skipAssert: bool = True):
    """
    Validate the weak accuracy of the integrators for Example 1 from Tang & Xiao (2017).
    Compares the empirical variance of the final state to the analytical value using
    multiple Monte Carlo batches.

    Args:
        method: Integration method.
    """

    dt = 2.**-3

    system = Example1System()

    def basiliskTrajectory():
        scSim, stateModel, integratorObject, stateLogger = getBasiliskSim(method, dt, system.x0, system.f, system.g, None)
        scSim.ConfigureStopTime( macros.sec2nano(tf) )
        scSim.ExecuteSimulation()

        xBasilisk = stateLogger.x

        return xBasilisk[-1,:]

    # Use the same G and E[G(x(t))] as in Example 1 in the paper
    def G(arr: npt.NDArray[np.float64]) -> float:
        return arr[1]**2

    estimateGOnTrajectory = 149/150*np.exp(-5/2*tf) +1/150*np.exp(-tf)

    err, varErr = estimateErrorAndEmpiricalVariance(
        basiliskTrajectory,
        G,
        estimateGOnTrajectory,
        M1 = 10,
        M2 = 100
    )
    twoSigma = 2*np.sqrt(varErr)

    print(method, "variance error", err, "+-", twoSigma)

    if not skipAssert:
        # We expect the error to be zero, but we allow some tolerance
        # given that the error is estimated with a certain variance
        np.testing.assert_allclose(
            err,
            0,
            atol = twoSigma,
            rtol = 0
        )

if __name__ == "__main__":
    pytest.main([__file__])

    # run this test with a higher tf, enough to pass
    for method in METHODS:
        test_validateExample1(method, tf=5, skipAssert=False)
