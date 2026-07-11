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
Unit test for the InhomogeneousGeometricBrownianMotion (IGBM) MuJoCo dynamics base class.

IGBM is the Ito SDE

    dX_t = (mu - X_t)/tau dt + sigma X_t dW_t

with mean-reverting drift and *multiplicative* diffusion. For mu > 0 and X_0 > 0 the
process stays strictly positive. Like MeanRevertingNoise, the class is configured in
stationary form (mean mu, time constant tau, stationary std sigma_st); the SDE volatility
sigma is derived internally so that the stationary distribution has exactly

    E[X_inf]   = mu
    Std[X_inf] = sigma_st

The test integrates a single long path with the strong Euler-Maruyama integrator (started
at the stationary mean) and checks the empirical time-average mean and variance against
those configured values. A single long ergodic path is used rather than a Monte Carlo
ensemble so the test stays cheap while still exercising the drift, the multiplicative
diffusion, and the positivity of the process.
"""
import pytest
import numpy as np
import numpy.testing as npt
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import svIntegrators
from Basilisk.utilities import macros

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import MJInhomogeneousGeometricBrownianMotion as MJIGBM

    couldImportMujoco = True
except Exception:
    couldImportMujoco = False


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_inhomogeneousGeometricBrownianMotion(showPlots: bool = False):
    """Verify IGBM reproduces the analytic stationary mean and variance, and stays positive."""
    dt = 0.001  # [s] small step for the multiplicative-noise Euler-Maruyama path
    tf = 400.0  # [s] long ergodic path for stable time-average statistics

    mean = 1.0             # mu
    timeConstant = 0.2     # tau [s]
    sigmaStationary = 1.0 / 3.0  # sigma_st: stationary std of the process

    varTarget = sigmaStationary**2

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene("<mujoco/>")  # empty scene, no multi-body dynamics
    scSim.AddModelToTask("test", scene)

    # A strong integrator is required: multiplicative noise means weak integrators do not
    # produce the correct sample-wise dynamic shape.
    integratorObject = svIntegrators.svStochasticIntegratorMayurama(scene)
    integratorObject.setRNGSeed(0)
    scene.setIntegrator(integratorObject)

    # Minimal Python subclass that just surfaces the state each step (the base class is
    # abstract; writeOutput is where a real user would publish a corrected quantity).
    class PyIGBM(MJIGBM.InhomogeneousGeometricBrownianMotion):
        def __init__(self):
            super().__init__()
            self.samples = []

        def writeOutput(self, CurrentSimNanos, x):
            self.samples.append(x)

    igbm = PyIGBM()
    igbm.setMean(mean)
    igbm.setTimeConstant(timeConstant)
    igbm.setStationaryStd(sigmaStationary)
    scene.AddModelToDynamicsTask(igbm)

    scSim.InitializeSimulation()

    # Accessors round-trip and initial condition set at the stationary mean.
    assert igbm.getMean() == mean
    assert igbm.getTimeConstant() == timeConstant
    assert igbm.getStationaryStd() == sigmaStationary
    assert igbm.getTheta() == 1.0 / timeConstant
    # Derived SDE volatility: sigma^2 = (2/tau) sigma_st^2 / (mu^2 + sigma_st^2)
    sigmaExpected = np.sqrt(
        (2.0 / timeConstant) * sigmaStationary**2 / (mean**2 + sigmaStationary**2)
    )
    npt.assert_allclose(igbm.getSigma(), sigmaExpected, rtol=1e-14)
    igbm.setStateValue(mean)
    assert igbm.getStateValue() == mean

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    y = np.asarray(igbm.samples, dtype=float)

    if showPlots:
        fig, ax = plt.subplots()
        ax.plot(np.arange(len(y)) * dt, y)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("IGBM state x")
        plt.show()

    # Discard an initial transient (a few reversion time constants) before averaging.
    burn = int(round(5.0 * timeConstant / dt))
    ys = y[burn:]

    meanEst = float(np.mean(ys))
    varEst = float(np.var(ys, ddof=1))

    print(f"mean={meanEst:.4f} (target {mean})  var={varEst:.4f} (target {varTarget:.4f})")

    # The process must stay strictly positive (mu>0, X0>0, multiplicative noise).
    assert np.all(y > 0.0), "IGBM path went non-positive"

    npt.assert_allclose(meanEst, mean, rtol=0.05, atol=0.02)
    npt.assert_allclose(varEst, varTarget, rtol=0.20)


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_inhomogeneousGeometricBrownianMotion_rejectsNonPositiveState():
    """setStateValue must reject a non-positive initial state: the multiplicative process
    is only well-posed from a positive state."""
    from Basilisk.architecture import bskLogging

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(0.1)))
    scene = mujoco.MJScene("<mujoco/>")
    scSim.AddModelToTask("test", scene)
    integ = svIntegrators.svStochasticIntegratorMayurama(scene)
    scene.setIntegrator(integ)

    class PyIGBM(MJIGBM.InhomogeneousGeometricBrownianMotion):
        def writeOutput(self, CurrentSimNanos, x):
            pass

    igbm = PyIGBM()
    scene.AddModelToDynamicsTask(igbm)
    scSim.InitializeSimulation()

    for bad in (0.0, -0.5):
        with pytest.raises(bskLogging.BasiliskError):
            igbm.setStateValue(bad)
    # A positive value is still accepted.
    igbm.setStateValue(0.5)
    assert igbm.getStateValue() == 0.5


def test_inhomogeneousGeometricBrownianMotion_rejectsBadParameters():
    """The parameter setters reject values outside their valid range."""
    from Basilisk.architecture import bskLogging

    igbm = MJIGBM.InhomogeneousGeometricBrownianMotion()
    for bad in (0.0, -1.0):
        with pytest.raises(bskLogging.BasiliskError):
            igbm.setMean(bad)  # mu must be > 0
    for bad in (0.0, -0.1):
        with pytest.raises(bskLogging.BasiliskError):
            igbm.setTimeConstant(bad)  # tau must be > 0
    with pytest.raises(bskLogging.BasiliskError):
        igbm.setStationaryStd(-0.01)  # sigma_st must be >= 0
    # Valid values are accepted.
    igbm.setMean(2.0); igbm.setTimeConstant(0.5); igbm.setStationaryStd(0.0)
    assert igbm.getMean() == 2.0
    assert igbm.getTimeConstant() == 0.5
    assert igbm.getStationaryStd() == 0.0


if __name__ == "__main__":
    assert couldImportMujoco
    test_inhomogeneousGeometricBrownianMotion(True)
