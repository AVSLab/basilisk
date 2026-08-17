# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import numpy as np

from Basilisk.simulation import fuelTank
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros


def test_centerOfMassPrimeMassRateScaling():
    r"""Test the complete reported first-moment quotient rule.

    Let the spacecraft first mass moment and center of mass be

    .. math::

        \mathbf{q}_B = \sum_i m_i\mathbf{r}_{iB},
        \qquad
        \mathbf{c}_B = \frac{\mathbf{q}_B}{M}.

    Their body-frame derivatives must satisfy

    .. math::

        \mathbf{c}'_B =
        \frac{\mathbf{q}'_B}{M}
        - \frac{\dot{M}}{M}\mathbf{c}_B.

    For the fixed hub and stationary offset tank used here,
    :math:`\mathbf{q}'_B=\dot m_T\mathbf r_{T/B}`. The first term moves the
    center of mass toward the hub as the tank drains. The second is the
    denominator contribution and contains one division by total mass.
    """
    dynamics = _initializeUpdateOnlyTank()

    tankMass = 20.0  # [kg]
    tankMassRate = -0.25  # [kg/s]
    tankOffset_B = np.array([2.0, -1.0, 0.5])  # [m]
    totalMass = 120.0  # [kg]
    centerOfMass_B = tankMass*tankOffset_B/totalMass  # [m]
    expectedCenterOfMassPrime_B = (
        tankMassRate*tankOffset_B/totalMass
        - tankMassRate*centerOfMass_B/totalMass
    )  # [m/s]
    centerOfMassPrime_B = np.asarray(
        dynamics.dynManager.getPropertyReference("centerOfMassPrimeSC")
    ).reshape(3)

    np.testing.assert_allclose(
        centerOfMassPrime_B,
        expectedCenterOfMassPrime_B,
        rtol=1.0e-14,
        atol=1.0e-16,
    )


def _initializeUpdateOnlyTank():
    """Initialize a fixed hub with one stationary, leaking offset tank."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    taskName = "testTask"
    process.addTask(
        simulation.CreateNewTask(taskName, macros.sec2nano(0.1))
    )  # [s]

    dynamics = spacecraft.Spacecraft()
    dynamics.hub.mHub = 100.0  # [kg]
    dynamics.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # [m]
    dynamics.hub.IHubPntBc_B = np.diag(
        [80.0, 90.0, 100.0]
    )  # [kg*m^2]

    tank = fuelTank.FuelTank()
    tankModel = fuelTank.FuelTankModelConstantVolume()
    tankModel.propMassInit = 20.0  # [kg]
    tankModel.radiusTankInit = 0.5  # [m]
    tankModel.r_TcT_TInit = [[0.0], [0.0], [0.0]]  # [m]
    tank.setTankModel(tankModel)
    tank.setR_TB_B([2.0, -1.0, 0.5])  # [m]
    tank.setFuelLeakRate(0.25)  # [kg/s]
    tank.setUpdateOnly(True)
    dynamics.addStateEffector(tank)

    simulation.AddModelToTask(taskName, tank)
    simulation.AddModelToTask(taskName, dynamics)
    simulation.InitializeSimulation()
    return dynamics
