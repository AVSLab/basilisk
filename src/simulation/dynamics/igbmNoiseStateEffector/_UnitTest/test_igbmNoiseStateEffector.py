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
from Basilisk.simulation import igbmNoiseStateEffector
from Basilisk.simulation import svIntegrators
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


def test_igbmNoiseStateEffector():
    """
    Statistical test equivalent to test_meanRevertingNoiseStateEffector.py using the
    factor dens = 1 + delta, which follows an IGBM with stationary mean mu and
    stationary std sigma_st. Also checks the factor stays strictly positive.
    """
    dt = 0.001  # s
    tf = 100.0  # s
    mean = 1.0
    sigmaStationary = 1.0 / 3.0
    timeConstant = 0.2

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("process")
    task_name = "task"
    proc.addTask(sim.CreateNewTask(task_name, macros.sec2nano(dt)))

    sc_object = spacecraft.Spacecraft()
    sim.AddModelToTask(task_name, sc_object)

    integrator = svIntegrators.svStochasticIntegratorMayurama(sc_object)
    integrator.setRNGSeed(0)
    sc_object.setIntegrator(integrator)

    effector = igbmNoiseStateEffector.IgbmNoiseStateEffector()
    effector.setMean(mean)
    effector.setStationaryStd(sigmaStationary)
    effector.setTimeConstant(timeConstant)
    effector.setStateName("igbmNoiseState")
    sc_object.addStateEffector(effector)

    sim.InitializeSimulation()

    assert effector.getMean() == mean
    assert effector.getTimeConstant() == timeConstant
    assert effector.getStationaryStd() == sigmaStationary
    # Default initial correction is mu - 1 (factor at its mean level).
    assert effector.getStateValue() == mean - 1.0

    steps = int(tf / dt)
    dens = np.zeros(steps)
    state_obj = sc_object.dynManager.getStateObject(effector.getStateName())

    for i in range(steps):
        sim.ConfigureStopTime(macros.sec2nano((i + 1) * dt))
        sim.ExecuteSimulation()
        dens[i] = 1.0 + state_obj.getState()[0][0]

    y = np.asarray(dens)

    # Discard an initial transient (a few reversion time constants) before averaging.
    burn = int(round(5.0 * timeConstant / dt))
    ys = y[burn:]

    meanEst = float(np.mean(ys))
    varEst = float(np.var(ys, ddof=1))

    print(f"mean={meanEst:.4f} (target {mean})  var={varEst:.4f} (target {sigmaStationary**2:.4f})")

    # At this small step the sampled factor stays positive; the exact process is positive
    # and downstream consumers clamp against the rare negative an explicit step can give.
    assert np.all(y > 0.0), "IGBM factor went non-positive"

    npt.assert_allclose(meanEst, mean, rtol=0.05, atol=0.02)
    npt.assert_allclose(varEst, sigmaStationary**2, rtol=0.30)


def test_igbmNoiseStateEffector_rejectsNonPositiveInitialFactor():
    """setStateValue must reject an initial correction <= -1, since the process is only
    well-posed from a positive initial factor 1 + delta."""
    import pytest
    from Basilisk.architecture import bskLogging

    effector = igbmNoiseStateEffector.IgbmNoiseStateEffector()
    for bad in (-1.0, -1.5):
        with pytest.raises(bskLogging.BasiliskError):
            effector.setStateValue(bad)
    # A correction above -1 (initial factor positive) is accepted.
    effector.setStateValue(-0.5)
    assert effector.getStateValue() == -0.5


def test_igbmNoiseStateEffector_rejectsBadParameters():
    """The parameter setters reject values outside their valid range."""
    import pytest
    from Basilisk.architecture import bskLogging

    effector = igbmNoiseStateEffector.IgbmNoiseStateEffector()
    for bad in (0.0, -1.0):
        with pytest.raises(bskLogging.BasiliskError):
            effector.setMean(bad)
    for bad in (0.0, -0.1):
        with pytest.raises(bskLogging.BasiliskError):
            effector.setTimeConstant(bad)
    with pytest.raises(bskLogging.BasiliskError):
        effector.setStationaryStd(-0.01)
    effector.setMean(2.0); effector.setTimeConstant(0.5); effector.setStationaryStd(0.0)
    assert effector.getMean() == 2.0
    assert effector.getTimeConstant() == 0.5
    assert effector.getStationaryStd() == 0.0


if __name__ == "__main__":
    test_igbmNoiseStateEffector()
    test_igbmNoiseStateEffector_rejectsNonPositiveInitialFactor()
    test_igbmNoiseStateEffector_rejectsBadParameters()
