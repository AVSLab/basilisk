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
Shared test helpers for the stochastic-integrator equivalence tests.

Both equivalence suites (``test_stochasticIntegratorsJulia.py`` and
``test_stochasticIntegratorsPaper.py``) drive a native integrator on a small toy SDE with
a prescribed sequence of Gaussian increments and compare the resulting trajectory against
a committed reference. This module factors out the identical MuJoCo/StatefulSysModel
scaffolding they share, so the per-suite files only supply the SDE (drift ``f`` and
per-source diffusion columns ``gCols``) and the reference case dict.
"""
from __future__ import annotations

from typing import Callable, List

import numpy as np

from Basilisk.utilities import macros
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import pythonVariableLogger
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import dynParamManager


def buildPrescribedNoiseSim(case: dict, integratorClassName: str,
                            f: Callable, gCols: List[Callable]):
    """Build a MuJoCo simulation that replays a reference case's prescribed noise.

    Args:
        case: reference-case dict with keys dt, u0, n_states, n_steps, dW, dZ and
            (optionally) m. ``m`` defaults to ``n_states`` (the diagonal-noise
            convention used by the Julia suite).
        integratorClassName: exact ``svIntegrators`` class name to instantiate.
        f: drift f(t, x) -> length-n array.
        gCols: list of m diffusion columns, each g_k(t, x) -> length-n array.

    Returns:
        (scSim, stateModel, integrator, stateLogger). Run to case["tf"] and read
        ``stateLogger.x``.
    """
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import StatefulSysModel

    dt = case["dt"]
    x0 = np.array(case["u0"], dtype=float)
    n = case["n_states"]
    m = case.get("m", n)

    class GenericStochasticStateModel(StatefulSysModel.StatefulSysModel):
        def registerStates(self, registerer: StatefulSysModel.DynParamRegisterer):
            self.states: List[dynParamManager.StateData] = []
            for i in range(n):
                self.states.append(registerer.registerState(1, 1, f"y{i + 1}"))
                self.states[-1].setNumNoiseSources(m)
                self.states[-1].setState([[x0[i]]])
            # Every noise source is shared across all states, so source k drives every
            # state through the k-th diffusion column (supports non-diagonal noise).
            for k in range(m):
                registerer.registerSharedNoiseSource(
                    [(state, k) for state in self.states]
                )

        def UpdateState(self, CurrentSimNanos: int):
            t = macros.NANO2SEC * CurrentSimNanos
            x = np.array([state.getState()[0][0] for state in self.states])
            deriv = f(t, x)
            for i in range(n):
                self.states[i].setDerivative([[deriv[i]]])
            for k in range(m):
                col = gCols[k](t, x)
                for i in range(n):
                    self.states[i].setDiffusion([[col[i]]], index=k)

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene("<mujoco/>")
    scSim.AddModelToTask("test", scene)

    integrator = getattr(svIntegrators, integratorClassName)(scene)

    # Replay the reference increments through a prescribed-noise generator.
    prescribed = svIntegrators.PrescribedGaussianNoiseGenerator()
    dWsteps = case["dW"]
    dZsteps = case["dZ"]
    for step in range(case["n_steps"]):
        prescribed.pushStep(list(dWsteps[step]), list(dZsteps[step]))
    integrator.setNoiseGenerator(prescribed)
    scene.setIntegrator(integrator)

    stateModel = GenericStochasticStateModel()
    stateModel.ModelTag = "testModel"
    scene.AddModelToDynamicsTask(stateModel)
    scene.AddModelToDiffusionDynamicsTask(stateModel)

    stateLogger = pythonVariableLogger.PythonVariableLogger(
        {"x": lambda _: np.array([s.getState()[0][0] for s in stateModel.states])}
    )
    scSim.AddModelToTask("test", stateLogger)
    scSim.InitializeSimulation()
    return scSim, stateModel, integrator, stateLogger
