#
#  ISC License
#
#  Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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
Tests the Euler-Mayurama and SRK integrator described in:

Tang, X., Xiao, A. Efficient weak second-order stochastic Runge-Kutta methods
for Itô stochastic differential equations. Bit Numer Math 57, 241-260 (2017).
https://doi.org/10.1007/s10543-016-0618-9
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
    from Basilisk.simulation import StatefulSysModel
    couldImportMujoco = True
except:
    couldImportMujoco = False

SRKMethod = Literal["W2Ito1", "W2Ito2"]
Method = Literal["W2Ito1", "W2Ito2", "EulerMayurama"]
METHODS = get_args(Method)

# Function of the form y = f(t,x) where x and y are vectors of the same size
DynFunc = Callable[[float, npt.NDArray[np.float64]], npt.NDArray[np.float64]]

# Coefficient sets (from Tang & Xiao Table 2, W2-Ito1 & W2-Ito2)
W2Ito1Coefficients = {
    'alpha':  np.array([1/6, 2/3, 1/6]),
    'beta0':  np.array([-1, 1, 1]),
    'beta1':  np.array([2, 0, -2]),

    'A0':     np.array([[0.0, 0.0, 0.0],
                        [1/2, 0.0, 0.0],
                        [-1, 2, 0]]),

    'B0':     np.array([[0.0, 0.0, 0.0],
                        [(6-np.sqrt(6))/10, 0.0, 0.0],
                        [(3+2*np.sqrt(6))/5, 0.0, 0.0]]),

    'A1':     np.array([[0.0, 0.0, 0.0],
                        [1/4, 0.0, 0.0],
                        [1/4, 0.0, 0.0]]),

    'B1':     np.array([[0.0, 0.0, 0.0],
                        [1/2, 0.0, 0.0],
                        [-1/2, 0.0, 0.0]]),

    'B2':     np.array([[0.0, 0.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0]]),
}

W2Ito2Coefficients = {
    'alpha':  np.array([1/6, 1/3, 1/3, 1/6]),
    'beta0':  np.array([0, -1, 1, 1]),
    'beta1':  np.array([0, 2, 0, -2]),

    'A0':     np.array([[0.0, 0.0, 0.0, 0.0],
                        [1/2, 0.0, 0.0, 0.0],
                        [0.0, 1/2, 0.0, 0.0],
                        [0.0, 0.0, 1.0, 0.0]]),

    'B0':     np.array([[0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0]]),

    'A1':     np.array([[0.0, 0.0, 0.0, 0.0],
                        [1/2, 0.0, 0.0, 0.0],
                        [1/2, 0.0, 0.0, 0.0],
                        [1/2, 0.0, 0.0, 0.0]]),

    'B1':     np.array([[0.0, 0.0,  0.0, 0.0],
                        [0.0, 0.0,  0.0, 0.0],
                        [0.0, 1/2,  0.0, 0.0],
                        [0.0, -1/2, 0.0, 0.0]]),

    'B2':     np.array([[0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0]]),
}

SRK_COEFFICIENTS: dict[SRKMethod, dict[str, npt.NDArray[Any]]] = {
    "W2Ito1": W2Ito1Coefficients,
    "W2Ito2": W2Ito2Coefficients
}

def srk2Integrate(
        f: DynFunc,
        g_list: List[DynFunc],
        x0: npt.NDArray[np.float64],
        dt: float,
        tf: float,
        rng_seed: int,
        alpha: npt.NDArray[np.float64],
        beta0: npt.NDArray[np.float64],
        beta1: npt.NDArray[np.float64],
        A0: npt.NDArray[np.float64],
        B0: npt.NDArray[np.float64],
        A1: npt.NDArray[np.float64],
        B1: npt.NDArray[np.float64],
        B2: npt.NDArray[np.float64],
    ):
    """
    Generic s-stage SRK integrator for vector SDE:
        dX = f(t,X) dt + sum_k g_list[k](t,X) dW_k

    Method described in Tang & Xiao.
    Args:
        f: Drift function.
        g_list: List of diffusion functions.
        x0: Initial state.
        dt: Time step.
        tf: Final time.
        rng_seed: Random seed.
        alpha, beta0, beta1, A0, B0, A1, B1, B2: SRK coefficients.
    Returns:
        Array of state trajectories, including time as the first column.
    """

    def wrapped_f(full_state: npt.NDArray[Any]):
        t = full_state[0]
        x = full_state[1:]
        return np.concatenate([[1], f(t, x)])

    wrapped_g_list = []
    for g in g_list:
        def wrapped_g(full_state: npt.NDArray[Any], g: DynFunc = g):
            t = full_state[0]
            x = full_state[1:]
            return np.concatenate([[0], g(t, x)])
        wrapped_g_list.append(wrapped_g)

    s = alpha.size
    n = x0.size
    m = len(g_list)

    nsteps = int(np.floor(tf / dt))

    x = np.zeros([nsteps+1, n+1], dtype=float)
    x[0,0] = 0
    x[0,1:] = x0

    rng = svIntegrators.SRKRandomVariableGenerator()
    rng.setSeed(rng_seed)

    # throw the first away, similar to how Basilisk
    # calls the equationsOfMotion once at the beginning
    rng.generate(m, 0)

    for step in range(nsteps):
        X = x[step,:].copy()

        # generate random variables
        rvs: svIntegrators.SRKRandomVariables = rng.generate(m, dt)
        Ik: List[List[float]] = rvs.Ik
        Ikl: List[List[float]] = rvs.Ikl
        xi: float = rvs.xi

        # allocate stage arrays
        H0 = [X.copy() for _ in range(s)]
        Hk = [[X.copy() for _ in range(s)] for _ in range(m)]

        for i in range(s):

            # compute H0 stages
            sumA = np.zeros(n+1)
            sumB = np.zeros(n+1)
            for j in range(s):
                sumA += A0[i, j] * wrapped_f(H0[j]) * dt
                for k in range(m):
                    sumB += B0[i, j] * wrapped_g_list[k](Hk[k][j]) * Ik[k]
            H0[i] = X + sumA + sumB

            # compute Hk stages
            for k in range(m):
                sumA = np.zeros(n+1)
                sumB1 = np.zeros(n+1)
                sumB2 = np.zeros(n+1)
                for j in range(s):
                    sumA += A1[i, j] * wrapped_f(H0[j]) * dt
                    sumB1 += B1[i, j] * wrapped_g_list[k](Hk[k][j]) * xi
                    for l in range(m):
                        if l != k:
                            sumB2 += B2[i, j] * wrapped_g_list[l](Hk[k][j]) * Ikl[k][l]
                Hk[k][i] = X + sumA + sumB1 + sumB2

        # combine increments
        drift = np.zeros(n+1)
        for i in range(s):
            drift += alpha[i] * wrapped_f(H0[i]) * dt

        diffusion = np.zeros(n+1)
        for k in range(m):
            for i in range(s):
                diffusion += beta0[i] * wrapped_g_list[k](Hk[k][i]) * Ik[k]
                diffusion += beta1[i] * wrapped_g_list[k](Hk[k][i]) * Ikl[k][k]

        x[step+1,:] = X + drift + diffusion

    return x

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

    rng = svIntegrators.EulerMayuramaRandomVariableGenerator()
    rng.setSeed(rng_seed)

    # throw the first away, similar to how Basilisk
    # calls the equationsOfMotion once at the beginning
    rng.generate(m, 0)

    for step in range(nsteps):
        # generate random variables
        pseudoTimeSteps = rng.generate(m, dt)

        x[step+1,:] = x[step,:] + f(t[step], x[step,:])*dt
        for k, g in enumerate(g_list):
            x[step+1,:] += g(t[step], x[step,:])*pseudoTimeSteps[k]

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
            self.sigma**2/(2*self.theta) * (1- np.exp(-self.theta*t))
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
        method: Integration method ("W2Ito1", "W2Ito2", or "EulerMayurama").
        dt: Time step.
        x0: Initial state.
        f: Drift function.
        g: List of diffusion functions.
        seed: RNG seed (or None for random).

    Returns:
        Tuple of (scSim, stateModel, integratorObject, stateLogger).
    """

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

    if method == "W2Ito1":
        integratorClass = svIntegrators.svStochIntegratorW2Ito1
    elif method == "W2Ito2":
        integratorClass = svIntegrators.svStochIntegratorW2Ito2
    elif method == "EulerMayurama":
        integratorClass = svIntegrators.svStochIntegratorMayurama
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
    Test deterministic integration (no diffusion) for all integrator methods.
    Compares Basilisk and Python implementations against the analytical solution
    for the exponential system dx/dt = x*t.

    Args:
        method: Integration method ("W2Ito1", "W2Ito2", or "EulerMayurama").
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

    if method == "EulerMayurama":
        txPython = eulerMayuramaIntegrate(system.f, system.g, system.x0, dt, tf, seed)
    else:
        txPython = srk2Integrate(system.f, system.g, system.x0, dt, tf, seed, **SRK_COEFFICIENTS[method])
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

    if method == "EulerMayurama":
        expectedIntegrationError = .05
    elif method == "W2Ito1":
        expectedIntegrationError = 1e-6
    elif method == "W2Ito2":
        expectedIntegrationError = 1e-8

    # The Basilisk should have some integration error w.r.t analytical solution
    numpy.testing.assert_allclose(
        xBasilisk[-1],
        xExpected[-1],
        atol=expectedIntegrationError,
        rtol=0
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

    if method == "EulerMayurama":
        txPython = eulerMayuramaIntegrate(system.f, system.g, system.x0, dt, tf, seed)
    else:
        txPython = srk2Integrate(system.f, system.g, system.x0, dt, tf, seed, **SRK_COEFFICIENTS[method])
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

    if method == "EulerMayurama":
        txPython = eulerMayuramaIntegrate(system.f, system.g, system.x0, dt, tf, seed)
    else:
        txPython = srk2Integrate(system.f, system.g, system.x0, dt, tf, seed, **SRK_COEFFICIENTS[method])
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

    if method == "EulerMayurama":
        txPython = eulerMayuramaIntegrate(system.f, system.g, system.x0, dt, tf, seed)
    else:
        txPython = srk2Integrate(system.f, system.g, system.x0, dt, tf, seed, **SRK_COEFFICIENTS[method])
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
@pytest.mark.parametrize("method", METHODS)
@pytest.mark.parametrize("figureOfMerit", ["mean", "variance"])
def test_validateOu(method: Method, figureOfMerit: Literal["mean", "variance"]):
    """
    Validate the weak accuracy of the integrators for the Ornstein-Uhlenbeck process.
    Compares the empirical mean or variance of the final state to the analytical value
    using multiple Monte Carlo batches.

    Args:
        method: Integration method.
        figureOfMerit: "mean" or "variance" to validate.
    """

    dt = .05
    tf = 5

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
    fourSigma = 4*np.sqrt(varErr)

    print(method, figureOfMerit, "error", err, "+-", fourSigma)

    # We expect the error to be zero, but we allow some tolerance
    # given that the error is estimated with a certain variance
    np.testing.assert_allclose(
        err,
        0,
        atol = fourSigma,
        rtol = 0
    )

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("method", METHODS)
def test_validateExample1(method: Method):
    """
    Validate the weak accuracy of the integrators for Example 1 from Tang & Xiao (2017).
    Compares the empirical variance of the final state to the analytical value using
    multiple Monte Carlo batches.

    Args:
        method: Integration method.
    """

    dt = 2.**-3
    tf = 4

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
    fourSigma = 4*np.sqrt(varErr)

    print(method, "variance error", err, "+-", fourSigma)

    # We expect the error to be zero, but we allow some tolerance
    # given that the error is estimated with a certain variance
    np.testing.assert_allclose(
        err,
        0,
        atol = fourSigma,
        rtol = 0
    )

if __name__ == "__main__":
    pytest.main([__file__])
