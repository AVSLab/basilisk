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
import numpy as np
import numpy.testing as npt

from Basilisk.simulation import spacecraft
from Basilisk.simulation import meanRevertingNoiseStateEffector
from Basilisk.simulation import svIntegrators
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


def test_meanRevertingNoiseStateEffector():
    """
    Statistical test equivalent to test_meanRevertingNoise.py using dens = 1 + x.
    """
    dt = 0.002  # s
    tf = 100.0  # s
    sigmaStationary = 0.8
    timeConstant = 0.1

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("process")
    task_name = "task"
    proc.addTask(sim.CreateNewTask(task_name, macros.sec2nano(dt)))

    sc_object = spacecraft.Spacecraft()
    sim.AddModelToTask(task_name, sc_object)

    integrator = svIntegrators.svStochasticIntegratorMayurama(sc_object)
    integrator.setRNGSeed(0)
    sc_object.setIntegrator(integrator)

    effector = meanRevertingNoiseStateEffector.MeanRevertingNoiseStateEffector()
    effector.setStationaryStd(sigmaStationary)
    effector.setTimeConstant(timeConstant)
    effector.setStateName("meanRevertingNoiseState")
    sc_object.addStateEffector(effector)

    sim.InitializeSimulation()

    assert effector.getTimeConstant() == timeConstant
    assert effector.getStationaryStd() == sigmaStationary

    steps = int(tf / dt)
    dens = np.zeros(steps)
    t = np.zeros(steps)
    state_obj = sc_object.dynManager.getStateObject(effector.getStateName())

    for i in range(steps):
        sim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        sim.ExecuteSimulation()
        t[i] = (i + 1) * dt
        dens[i] = 1.0 + state_obj.getState()[0][0]

    y = np.asarray(dens)

    meanTarget = 1.0
    varTarget = sigmaStationary**2
    phiTarget = np.exp(-dt / timeConstant)

    meanEst = float(np.mean(y))
    varEst = float(np.var(y, ddof=1))
    yc0 = y[:-1] - meanEst
    yc1 = y[1:] - meanEst
    phiEst = float(np.dot(yc0, yc1) / np.dot(yc0, yc0))

    phiEstClamped = min(max(phiEst, 1e-6), 0.999999)
    timeConstantEst = -dt / np.log(phiEstClamped)

    print(f"mean={meanEst:.3f}  var={varEst:.3f}  tau_hat={timeConstantEst:.3f}")

    npt.assert_allclose(meanEst, meanTarget, atol=0.2)
    npt.assert_allclose(varEst, varTarget, rtol=0.3)
    npt.assert_allclose(phiEst, phiTarget, rtol=0.03, atol=0.003)
    npt.assert_allclose(timeConstantEst, timeConstant, rtol=0.05, atol=0.1)


if __name__ == "__main__":
    test_meanRevertingNoiseStateEffector()
