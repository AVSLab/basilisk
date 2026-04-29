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

from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import svIntegrators
from Basilisk.architecture import messaging
from Basilisk.utilities import macros

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import MJStochasticDragCoeff

    couldImportMujoco = True
except:
    couldImportMujoco = False

@pytest.mark.skipif(not couldImportMujoco, reason="Compiled Basilisk without --mujoco")
def test_stochasticDragCoeff(showPlots: bool = False):
    """
    Unit test for StochasticDragCoeff.

    Verifies that the output drag coefficient follows OU statistics: mean, variance,
    and correlation time match theoretical predictions for a constant nominal input.
    """
    dt = .002  # [s]
    tf = 100   # [s]

    nominalDragCoeff = 2.2
    sigmaStationary = 0.4
    timeConstant = 0.1  # [s]

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("test")
    dynProcess.addTask(scSim.CreateNewTask("test", macros.sec2nano(dt)))

    scene = mujoco.MJScene("<mujoco/>")
    scSim.AddModelToTask("test", scene)

    integratorObject = svIntegrators.svStochasticIntegratorMayurama(scene)
    integratorObject.setRNGSeed(0)
    scene.setIntegrator(integratorObject)

    nominalPayload = messaging.DragGeometryMsgPayload()
    nominalPayload.dragCoeff = nominalDragCoeff
    nominalMsg = messaging.DragGeometryMsg().write(nominalPayload)

    dragModule = MJStochasticDragCoeff.StochasticDragCoeff()
    dragModule.setStationaryStd(sigmaStationary)
    dragModule.setTimeConstant(timeConstant)
    dragModule.dragGeomInMsg.subscribeTo(nominalMsg)
    scene.AddModelToDynamicsTask(dragModule)

    recorder = dragModule.dragGeomOutMsg.recorder()
    scSim.AddModelToTask("test", recorder)

    scSim.InitializeSimulation()

    assert dragModule.getTimeConstant() == timeConstant
    assert dragModule.getStationaryStd() == sigmaStationary

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()

    cd = np.asarray(recorder.dragCoeff)

    meanTarget = nominalDragCoeff
    varTarget  = (nominalDragCoeff * sigmaStationary) ** 2
    phiTarget  = np.exp(-dt / timeConstant)

    meanEst = float(np.mean(cd))
    varEst  = float(np.var(cd, ddof=1))
    yc0 = cd[:-1] - meanEst
    yc1 = cd[1:]  - meanEst
    phiEst = float(np.dot(yc0, yc1) / np.dot(yc0, yc0))

    phiClamped = min(max(phiEst, 1e-6), 0.999999)
    timeConstantEst = -dt / np.log(phiClamped)

    print(f"mean={meanEst:.3f}  var={varEst:.3f}  tau_hat={timeConstantEst:.3f}")

    npt.assert_allclose(meanEst, meanTarget, atol=0.2 * nominalDragCoeff)
    npt.assert_allclose(varEst,  varTarget,  rtol=0.3)
    npt.assert_allclose(phiEst,  phiTarget,  rtol=0.03, atol=0.003)
    npt.assert_allclose(timeConstantEst, timeConstant, rtol=0.05, atol=0.1)


if __name__ == "__main__":
    assert couldImportMujoco
    test_stochasticDragCoeff(True)
