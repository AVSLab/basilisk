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
The SRK integrator tested here is described in:

Tang, X., Xiao, A. Efficient weak second-order stochastic Runge-Kutta methods
for Itô stochastic differential equations. Bit Numer Math 57, 241-260 (2017).
https://doi.org/10.1007/s10543-016-0618-9
"""
from __future__ import annotations

from typing import Callable, List

from Basilisk.utilities import macros
from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import dynParamManager

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import StatefulSysModel
    couldImportMujoco = True
except:
    couldImportMujoco = False

import pytest
import numpy as np
import numpy.typing as npt

# Function of the form y = f(t,x) where x and y are vectors of the same size
DynFunc = Callable[[float, npt.NDArray[np.float64]], npt.NDArray[np.float64]]

import numpy as np

# Coefficient sets (from Tang & Xiao Table 2, W2-Ito1 & W2-Ito2)
W2_Ito1 = {
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

W2_Ito2 = {
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

def srk2_integrate(
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
    """

    def wrapped_f(full_state):
        t = full_state[0]
        x = full_state[1:]
        return np.concatenate([[1], f(t, x)])

    wrapped_g_list = []
    for g in g_list:
        def wrapped_g(full_state, g=g):
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

    for step in range(nsteps):
        t = x[step,0]
        X = x[step,:].copy()

        # generate random variables
        rvs: svIntegrators.SRKRandomVariables = rng.generate(m, dt)
        Ik: List[List[float]] = rvs.Ik
        Ikl: List[List[float]] = rvs.Ikl
        xi: float = rvs.xi

        # allocate stage arrays
        H0 = [X.copy() for _ in range(s)]
        Hk = [[X.copy() for _ in range(s)] for _ in range(m)]

        # compute H0 stages
        for i in range(s):
            sumA = np.zeros(n+1)
            sumB = np.zeros(n+1)
            for j in range(s):
                sumA += A0[i, j] * wrapped_f(H0[j]) * dt
                for k in range(m):
                    sumB += B0[i, j] * wrapped_g_list[k](Hk[k][j]) * Ik[k]
            H0[i] = X + sumA + sumB

        # compute Hk stages
        for k in range(m):
            for i in range(s):
                sumA = np.zeros(n+1)
                sumB1 = np.zeros(n+1)
                sumB2 = np.zeros(n+1)
                for j in range(s):
                    sumA += A1[i, j] * wrapped_f(H0[j]) * dt
                    sumB1 += B1[i, j] * wrapped_g_list[k](Hk[k][j]) * xi
                    for l in range(m):
                        if l != k:
                            sumB2 += B2[i, j] * wrapped_g_list[l](Hk[k][i]) * Ikl[k][l]
                Hk[k][i] = X + sumA + sumB1 + sumB2

        # combine increments
        drift = np.zeros(n+1)
        for i in range(s):
            drift += alpha[i] * f(t, H0[i]) * dt

        diffusion = np.zeros(n+1)
        for k in range(m):
            for i in range(s):
                diffusion += beta0[i] * wrapped_g_list[k](Hk[k][i]) * Ik[k]
                diffusion += beta1[i] * wrapped_g_list[k](Hk[k][i]) * Ikl[k][k]

        x[step+1,:] = X + drift + diffusion

    return x

class ExponentialSystem:
    """A simple system with one state: dx/dt = x*t."""

    x0 = np.array([1])

    @staticmethod
    def f(t: float, x: npt.NDArray[np.float64]):
        return np.array([t*x[0]])

    g = []

class Example1System:
    """Example 1 dynamical system in:

        Tang, X., Xiao, A. Efficient weak second-order stochastic Runge-Kutta methods
        for Itô stochastic differential equations. Bit Numer Math 57, 241-260 (2017).
        https://doi.org/10.1007/s10543-016-0618-9
    """
    x0 = np.array([1, 1])

    @staticmethod
    def f(t: float, x: npt.NDArray[np.float64]):
        y1, y2 = x
        return np.array([
            -273/512*y1,
            -1/160*y1 - (785/512 - np.sqrt(2)/8)*y2
        ])

    @staticmethod
    def g1(t: float, x: npt.NDArray[np.float64]):
        y1, y2 = x
        return np.array([
            1/4*y1,
            (1 - 2*np.sqrt(2)/8)/4*y2
        ])

    @staticmethod
    def g2(t: float, x: npt.NDArray[np.float64]):
        y1, y2 = x
        return np.array([
            1/16*y1,
            1/10*y1 + 1/16*y2
        ])

    g = [g1, g2]

def get_basilisk_sim(dt: float, x0: npt.NDArray[np.float64], f: DynFunc, g: List[DynFunc], seed: int):

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

    integratorObject = svIntegrators.svStochIntegratorW2Ito1(scene)
    scene.setIntegrator(integratorObject)

    stateModel = GenericStochasticStateModel(x0, f, g)
    stateModel.ModelTag = "testModel"

    scene.AddModelToDynamicsTask(stateModel)

    scSim.InitializeSimulation()

    return scSim, stateModel, integratorObject

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_deterministic():

    dt = 1
    tf = 1
    seed = 10

    system = ExponentialSystem

    scSim, stateModel, integratorObject = get_basilisk_sim(dt, system.x0, system.f, system.g, seed)
    scSim.ConfigureStopTime( macros.sec2nano(tf) )
    scSim.ExecuteSimulation()

    x1Basilisk = stateModel.states[0].getState()[0][0]

    xPython = srk2_integrate(system.f, system.g, system.x0, dt, tf, seed, **W2_Ito1)
    x1Python = xPython[1,:]

    x1expected = np.exp( tf**2 / 2 )

    raise NotImplementedError("WIP")

if __name__ == "__main__":
    if True:
        test_deterministic()
    else:
        pytest.main([__file__])
