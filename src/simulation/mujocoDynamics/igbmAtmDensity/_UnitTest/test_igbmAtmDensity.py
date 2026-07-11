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
import pytest
import numpy as np
import numpy.testing as npt
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import svIntegrators
from Basilisk.architecture import messaging
from Basilisk.utilities import macros

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import MJInhomogeneousGeometricBrownianMotion as MJIGBM
    from Basilisk.simulation import MJIgbmAtmDensity

    couldImportMujoco = True
except Exception:
    couldImportMujoco = False


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("usePython", [False, True])
def test_igbmAtmDensity(usePython: bool, showPlots: bool = False):
    """
    Unit test for IgbmAtmDensity.

    Verifies that the output density follows the stationary statistics of the inhomogeneous
    geometric Brownian motion (IGBM)

        dX = (mu - X)/tau dt + sigma X dW,

    configured in stationary form (mean mu, time constant tau, stationary std sigma_st,
    with sigma derived internally), that it never goes non-positive, and that a Python
    subclass of the IGBM base also works.
    """
    dt = 0.001  # [s] small step for the multiplicative-noise Euler-Maruyama path
    tf = 400.0  # [s]

    mean = 1.0             # mu (mean-preserving correction of a unit nominal density)
    timeConstant = 0.2     # tau [s]
    sigmaStationary = 1.0 / 3.0  # sigma_st: stationary std of the density factor

    varTarget = sigmaStationary**2

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene("<mujoco/>")  # empty scene, no multi-body dynamics
    scSim.AddModelToTask("test", scene)

    # A strong integrator is required for multiplicative noise.
    integratorObject = svIntegrators.svStochasticIntegratorMayurama(scene)
    integratorObject.setRNGSeed(0)
    scene.setIntegrator(integratorObject)

    nominal = messaging.AtmoPropsMsg()
    nominal.write(messaging.AtmoPropsMsgPayload(neutralDensity=1))

    if usePython:
        class PyIgbmAtmDensity(MJIGBM.InhomogeneousGeometricBrownianMotion):

            def __init__(self):
                super().__init__()
                self.atmoDensInMsg = messaging.AtmoPropsMsgReader()
                self.atmoDensOutMsg = messaging.AtmoPropsMsg()

            def writeOutput(self, CurrentSimNanos, x):
                payload = messaging.AtmoPropsMsgPayload(
                    neutralDensity=self.atmoDensInMsg().neutralDensity * x
                )
                self.atmoDensOutMsg.write(payload, CurrentSimNanos, self.moduleID)

        stochasticAtm = PyIgbmAtmDensity()

    else:
        stochasticAtm = MJIgbmAtmDensity.IgbmAtmDensity()

    stochasticAtm.setMean(mean)
    stochasticAtm.setTimeConstant(timeConstant)
    stochasticAtm.setStationaryStd(sigmaStationary)
    stochasticAtm.atmoDensInMsg.subscribeTo(nominal)
    scene.AddModelToDynamicsTask(stochasticAtm)

    atmoDensRecorder = stochasticAtm.atmoDensOutMsg.recorder()
    scSim.AddModelToTask("test", atmoDensRecorder)

    scSim.InitializeSimulation()

    assert stochasticAtm.getMean() == mean
    assert stochasticAtm.getTimeConstant() == timeConstant
    assert stochasticAtm.getStationaryStd() == sigmaStationary
    # Start at the stationary mean so no long transient biases the statistics.
    stochasticAtm.setStateValue(mean)

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    t = atmoDensRecorder.times()
    dens = atmoDensRecorder.neutralDensity

    if showPlots:
        fig, ax = plt.subplots()
        ax.plot(t, dens)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Stochastic Density [kg/m^3]")
        plt.show()

    y = np.asarray(dens, dtype=float)

    # Discard an initial transient (a few reversion time constants) before averaging.
    burn = int(round(5.0 * timeConstant / dt))
    ys = y[burn:]

    meanEst = float(np.mean(ys))
    varEst = float(np.var(ys, ddof=1))

    print(f"mean={meanEst:.4f} (target {mean})  var={varEst:.4f} (target {varTarget:.4f})")

    # The corrected density is clamped to be non-negative (see the dedicated clamp test).
    assert np.all(ys >= 0.0), "IGBM-corrected density went negative"

    npt.assert_allclose(meanEst, mean, rtol=0.05, atol=0.02)
    npt.assert_allclose(varEst, varTarget, rtol=0.20)


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_igbmAtmDensity_passesThroughOtherFields():
    """Only neutralDensity is corrected; the other AtmoPropsMsgPayload fields (localTemp)
    pass through unchanged."""
    dt = 0.01
    localTemp = 933.7  # [K] arbitrary nonzero value that must survive unchanged

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))
    scene = mujoco.MJScene("<mujoco/>")
    scSim.AddModelToTask("test", scene)
    integ = svIntegrators.svStochasticIntegratorMayurama(scene)
    integ.setRNGSeed(0)
    scene.setIntegrator(integ)

    nominal = messaging.AtmoPropsMsg()
    nominal.write(messaging.AtmoPropsMsgPayload(neutralDensity=2.5, localTemp=localTemp))

    stochasticAtm = MJIgbmAtmDensity.IgbmAtmDensity()
    stochasticAtm.setMean(1.0)
    stochasticAtm.setTimeConstant(1.0)
    stochasticAtm.setStationaryStd(0.2)
    stochasticAtm.atmoDensInMsg.subscribeTo(nominal)
    scene.AddModelToDynamicsTask(stochasticAtm)

    rec = stochasticAtm.atmoDensOutMsg.recorder()
    scSim.AddModelToTask("test", rec)
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(10 * dt))
    scSim.ExecuteSimulation()

    temps = np.asarray(rec.localTemp, dtype=float)
    # localTemp is copied verbatim (never scaled), so every logged sample equals the input.
    npt.assert_allclose(temps[1:], localTemp, rtol=0, atol=0)
    # neutralDensity is scaled by the (here mild) correction factor and stays positive.
    assert np.all(np.asarray(rec.neutralDensity, dtype=float)[1:] > 0.0)


@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_igbmAtmDensity_clampsNegativeDensity():
    """The IGBM process is integrated as written (pure math), so an explicit integrator
    can drive the factor negative under a large step/increment. IgbmAtmDensity must clamp
    the corrected density to be non-negative rather than emit an unphysical negative value.
    Prescribing large negative Wiener increments forces the factor below zero."""
    dt = 1.0  # large step
    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))
    scene = mujoco.MJScene("<mujoco/>")
    scSim.AddModelToTask("test", scene)

    integ = svIntegrators.svStochasticIntegratorMayurama(scene)
    prescribed = svIntegrators.PrescribedGaussianNoiseGenerator()
    nSteps = 5
    for _ in range(nSteps):
        prescribed.pushStep([-4.0], [])  # ~4-sigma negative excursions at dt=1
    integ.setNoiseGenerator(prescribed)
    scene.setIntegrator(integ)

    nominal = messaging.AtmoPropsMsg()
    nominal.write(messaging.AtmoPropsMsgPayload(neutralDensity=1.0))

    stochasticAtm = MJIgbmAtmDensity.IgbmAtmDensity()
    stochasticAtm.setMean(1.0)
    stochasticAtm.setTimeConstant(1.0)
    stochasticAtm.setStationaryStd(1.0)  # sigma ~ 1, so the factor is easily driven < 0
    stochasticAtm.atmoDensInMsg.subscribeTo(nominal)
    scene.AddModelToDynamicsTask(stochasticAtm)

    rec = stochasticAtm.atmoDensOutMsg.recorder()
    scSim.AddModelToTask("test", rec)
    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(nSteps * dt))
    scSim.ExecuteSimulation()

    dens = np.asarray(rec.neutralDensity, dtype=float)
    # Never negative, and the clamp actually engaged (some steps hit exactly zero) for
    # this deliberately extreme increment sequence.
    assert np.all(dens >= 0.0), f"corrected density went negative: min={dens.min()}"
    assert np.any(dens == 0.0), "expected the non-negativity clamp to engage for these increments"


def test_igbmAtmDensity_rejectsBadParameters():
    """The inherited parameter setters reject values outside their valid range."""
    from Basilisk.architecture import bskLogging

    atm = MJIgbmAtmDensity.IgbmAtmDensity()
    for bad in (0.0, -1.0):
        with pytest.raises(bskLogging.BasiliskError):
            atm.setMean(bad)
    for bad in (0.0, -0.1):
        with pytest.raises(bskLogging.BasiliskError):
            atm.setTimeConstant(bad)
    with pytest.raises(bskLogging.BasiliskError):
        atm.setStationaryStd(-0.01)


if __name__ == "__main__":
    assert couldImportMujoco
    test_igbmAtmDensity(True, True)
