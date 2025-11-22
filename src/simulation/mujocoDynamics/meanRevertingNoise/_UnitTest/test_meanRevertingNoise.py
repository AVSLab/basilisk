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
    from Basilisk.simulation import MJMeanRevertingNoise

    couldImportMujoco = True
except:
    raise
    couldImportMujoco = False

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
@pytest.mark.parametrize("usePython", [False, True])
def test_meanRevertingNoise(usePython: bool = False, showPlots: bool = False):
    """
    Unit test for classes that inherit from MeanRevertingNoise.
    """
    dt = .002 # s
    tf = 100 # s

    sigmaStationary = 0.8 # kg/m^3
    timeConstant = 0.1 # s

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    # Create the MJScene from a simple cannonball body
    scene = mujoco.MJScene("<mujoco/>") # empty scene, no multi-body dynamics
    scSim.AddModelToTask("test", scene)

    # must use strong integrator, since weak integrators do not produce
    # the right sample-wise dynamic shape
    integratorObject = svIntegrators.svStochIntegratorMayurama(scene)
    integratorObject.setRNGSeed(0)
    scene.setIntegrator(integratorObject)

    nominal = messaging.AtmoPropsMsg()
    nominal.write(messaging.AtmoPropsMsgPayload(neutralDensity=1))

    if usePython:
        class PyStochasticAtmDensity(MJMeanRevertingNoise.MeanRevertingNoise):

            def __init__(self):
                super().__init__()
                self.atmoDensInMsg = messaging.AtmoPropsMsgReader()
                self.atmoDensOutMsg = messaging.AtmoPropsMsg()

            def writeOutput(self, CurrentSimNanos, x):
                payload = messaging.AtmoPropsMsgPayload(
                    neutralDensity = self.atmoDensInMsg().neutralDensity * (1 + x)
                )
                self.atmoDensOutMsg.write(payload, self.moduleID, CurrentSimNanos)

        stochasticAtm = PyStochasticAtmDensity()

    else:
        stochasticAtm = MJMeanRevertingNoise.StochasticAtmDensity()
    stochasticAtm.setStationaryStd(sigmaStationary)
    stochasticAtm.setTimeConstant(timeConstant)
    stochasticAtm.atmoDensInMsg.subscribeTo( nominal )
    scene.AddModelToDynamicsTask(stochasticAtm)

    atmoDensRecorder = stochasticAtm.atmoDensOutMsg.recorder()

    scSim.AddModelToTask("test", atmoDensRecorder)

    scSim.InitializeSimulation()

    assert stochasticAtm.getTimeConstant() == timeConstant
    assert stochasticAtm.getStationaryStd() == sigmaStationary

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    t = atmoDensRecorder.times()
    dens = atmoDensRecorder.neutralDensity

    # Plot results
    if showPlots:
        fig, ax = plt.subplots()
        ax.plot(t, dens)
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Stochastic Density [kg/m^3]")

        plt.show()

    y = np.asarray(dens)

    # targets
    meanTarget = 1.0
    varTarget  = sigmaStationary**2
    phiTarget  = np.exp(-dt / timeConstant)

    # estimates
    meanEst = float(np.mean(y))
    varEst  = float(np.var(y, ddof=1))
    yc0 = y[:-1] - meanEst
    yc1 = y[1:]  - meanEst
    phiEst = float(np.dot(yc0, yc1) / np.dot(yc0, yc0))

    phiphiEstClamped = min(max(phiEst, 1e-6), 0.999999)
    timeConstantEst = -dt / np.log(phiphiEstClamped)

    print(f"mean={meanEst:.3f}  var={varEst:.3f}  tau_hat={timeConstantEst:.3f}")

    # assertions
    npt.assert_allclose(meanEst, meanTarget, atol=0.2)
    npt.assert_allclose(varEst,  varTarget,  rtol=0.3)
    npt.assert_allclose(phiEst,  phiTarget,  rtol=0.03, atol=0.003)
    npt.assert_allclose(timeConstantEst, timeConstant, rtol=0.05, atol=0.1)


if __name__ == "__main__":
    assert couldImportMujoco
    test_meanRevertingNoise(True, True)
