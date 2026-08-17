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


def test_updateOnlyExcludesDepletionRateTerms():
    r"""Verify update-only depletion reports derivatives without altering motion.

    ``FuelTank.setUpdateOnly(True)`` updates and reports the retained
    mass properties while omitting depletion-dependent rate terms from the
    equations of motion. A stationary leaking tank must therefore report its
    mass and center-of-mass derivatives while leaving the force-free hub
    acceleration equal to zero.
    """
    dynamics = _initializeUpdateOnlyTank()

    massRate = np.asarray(
        dynamics.dynManager.getPropertyReference("mDot_SC")
    ).reshape(-1)
    velocityDerivative_N = np.asarray(
        dynamics.dynManager.getStateObject(
            dynamics.hub.nameOfHubVelocity
        ).getStateDeriv()
    ).reshape(3)
    rateDerivative_B = np.asarray(
        dynamics.dynManager.getStateObject(
            dynamics.hub.nameOfHubOmega
        ).getStateDeriv()
    ).reshape(3)

    np.testing.assert_allclose(massRate, [-0.25], rtol=0.0, atol=1.0e-15)
    np.testing.assert_allclose(velocityDerivative_N, np.zeros(3), atol=1.0e-15)
    np.testing.assert_allclose(rateDerivative_B, np.zeros(3), atol=1.0e-15)


def test_mixedUpdateOnlyPreservesLegacyRateDynamics():
    r"""Verify update-only depletion does not alter another tank's dynamics.

    The first tank uses the legacy coupled-depletion model. The second tank is
    either a nondepleting control or an update-only depleting tank. Because
    update-only depletion changes reported retained mass-property derivatives
    but not the rate terms used by the equations of motion, both cases must
    produce the same hub accelerations. Absolute acceleration values also pin
    the established coupled-depletion equations independently of that
    comparison.
    """
    control = _initializeMixedTanks(secondTankLeakRate=0.0)
    updateOnly = _initializeMixedTanks(secondTankLeakRate=0.1)

    def _stateDerivatives(dynamics):
        velocityDerivative_N = np.asarray(
            dynamics.dynManager.getStateObject(
                dynamics.hub.nameOfHubVelocity
            ).getStateDeriv()
        ).reshape(3)
        rateDerivative_B = np.asarray(
            dynamics.dynManager.getStateObject(
                dynamics.hub.nameOfHubOmega
            ).getStateDeriv()
        ).reshape(3)
        return velocityDerivative_N, rateDerivative_B

    controlVelocityDerivative_N, controlRateDerivative_B = (
        _stateDerivatives(control)
    )
    updateVelocityDerivative_N, updateRateDerivative_B = (
        _stateDerivatives(updateOnly)
    )
    # Regression baselines recorded from the current coupled-depletion
    # implementation, not analytic variable-mass truth values.
    expectedVelocityDerivative_N = np.array(
        [0.01177374597473359, 0.00592476336088501, 0.00543057702170967]
    )  # [m/s^2]
    expectedRateDerivative_B = np.array(
        [0.00944849527546575, 0.01310294567357445, 0.01307406994734512]
    )  # [rad/s^2]
    np.testing.assert_allclose(
        controlVelocityDerivative_N,
        expectedVelocityDerivative_N,
        rtol=1.0e-13,
        atol=1.0e-15,
    )
    np.testing.assert_allclose(
        controlRateDerivative_B,
        expectedRateDerivative_B,
        rtol=1.0e-13,
        atol=1.0e-15,
    )
    np.testing.assert_allclose(
        updateVelocityDerivative_N,
        controlVelocityDerivative_N,
        rtol=0.0,
        atol=1.0e-14,
    )
    np.testing.assert_allclose(
        updateRateDerivative_B,
        controlRateDerivative_B,
        rtol=0.0,
        atol=1.0e-14,
    )

    reportedMassRate = np.asarray(
        updateOnly.dynManager.getPropertyReference("mDot_SC")
    ).reshape(-1)
    np.testing.assert_allclose(
        reportedMassRate,
        [-0.35],
        rtol=0.0,
        atol=1.0e-15,
    )

    tankAMass = 20.0  # [kg]
    tankBMass = 10.0  # [kg]
    tankAMassRate = -0.25  # [kg/s]
    tankBMassRate = -0.1  # [kg/s]
    tankAOffset_B = np.array([2.0, -1.0, 0.5])  # [m]
    tankBOffset_B = np.array([-0.5, 1.5, 0.75])  # [m]
    totalMass = 130.0  # [kg]
    centerOfMass_B = (
        tankAMass*tankAOffset_B + tankBMass*tankBOffset_B
    )/totalMass  # [m]
    expectedCenterOfMassPrime_B = (
        (
            tankAMassRate*tankAOffset_B
            + tankBMassRate*tankBOffset_B
        )/totalMass
        - (tankAMassRate + tankBMassRate)
            * centerOfMass_B/totalMass
    )  # [m/s]
    centerOfMassPrime_B = np.asarray(
        updateOnly.dynManager.getPropertyReference("centerOfMassPrimeSC")
    ).reshape(3)
    np.testing.assert_allclose(
        centerOfMassPrime_B,
        expectedCenterOfMassPrime_B,
        rtol=1.0e-14,
        atol=1.0e-16,
    )


def test_updateOnlyTankDerivativesMatchFiniteDifference():
    r"""Compare retained tank derivatives with integrated mass properties.

    An emptying tank has a moving retained center of mass and a changing
    inertia. In update-only mode these derivatives must still be reported,
    even though their rate-dependent loads are omitted from the equations of
    motion.
    """
    differenceTimeStep = 1.0e-3  # [s]
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    taskName = "testTask"
    process.addTask(
        simulation.CreateNewTask(
            taskName,
            macros.sec2nano(differenceTimeStep),
        )
    )

    dynamics = spacecraft.Spacecraft()
    dynamics.hub.mHub = 100.0  # [kg]
    dynamics.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # [m]
    dynamics.hub.IHubPntBc_B = np.diag(
        [80.0, 90.0, 100.0]
    )  # [kg*m^2]

    tank = fuelTank.FuelTank()
    tankModel = fuelTank.FuelTankModelEmptying()
    tankModel.propMassInit = 40.0  # [kg]
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

    tankMassState = dynamics.dynManager.getStateObject(
        tank.getNameOfMassState()
    )
    tankMassState.setState([[20.0]])  # [kg]
    dynamics.equationsOfMotion(0.0, differenceTimeStep)

    centerOfMass_B = np.asarray(
        dynamics.dynManager.getPropertyReference("centerOfMassSC")
    ).reshape(3).copy()
    inertia_B = np.asarray(
        dynamics.dynManager.getPropertyReference("inertiaSC")
    ).copy()
    centerOfMassPrime_B = np.asarray(
        dynamics.dynManager.getPropertyReference("centerOfMassPrimeSC")
    ).reshape(3).copy()
    inertiaPrime_B = np.asarray(
        dynamics.dynManager.getPropertyReference("inertiaPrimeSC")
    ).copy()

    simulation.ConfigureStopTime(macros.sec2nano(differenceTimeStep))
    simulation.ExecuteSimulation()
    centerOfMassNext_B = np.asarray(
        dynamics.dynManager.getPropertyReference("centerOfMassSC")
    ).reshape(3).copy()
    inertiaNext_B = np.asarray(
        dynamics.dynManager.getPropertyReference("inertiaSC")
    ).copy()

    np.testing.assert_allclose(
        centerOfMassPrime_B,
        (centerOfMassNext_B - centerOfMass_B)/differenceTimeStep,
        rtol=2.0e-5,
        atol=1.0e-10,
    )
    np.testing.assert_allclose(
        inertiaPrime_B,
        (inertiaNext_B - inertia_B)/differenceTimeStep,
        rtol=2.0e-5,
        atol=1.0e-9,
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


def _initializeMixedTanks(secondTankLeakRate):
    """Initialize one coupled-depletion tank and one update-only tank."""
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
    dynamics.hub.omega_BN_BInit = [[0.1], [-0.2], [0.15]]  # [rad/s]

    tankA = fuelTank.FuelTank()
    tankAModel = fuelTank.FuelTankModelConstantVolume()
    tankAModel.propMassInit = 20.0  # [kg]
    tankAModel.radiusTankInit = 0.5  # [m]
    tankAModel.r_TcT_TInit = [[0.0], [0.0], [0.0]]  # [m]
    tankA.setTankModel(tankAModel)
    tankA.setR_TB_B([2.0, -1.0, 0.5])  # [m]
    tankA.setFuelLeakRate(0.25)  # [kg/s]
    tankA.setUpdateOnly(False)
    dynamics.addStateEffector(tankA)

    tankB = fuelTank.FuelTank()
    tankBModel = fuelTank.FuelTankModelConstantVolume()
    tankBModel.propMassInit = 10.0  # [kg]
    tankBModel.radiusTankInit = 0.4  # [m]
    tankBModel.r_TcT_TInit = [[0.0], [0.0], [0.0]]  # [m]
    tankB.setTankModel(tankBModel)
    tankB.setR_TB_B([-0.5, 1.5, 0.75])  # [m]
    tankB.setFuelLeakRate(secondTankLeakRate)
    tankB.setUpdateOnly(True)
    dynamics.addStateEffector(tankB)

    simulation.AddModelToTask(taskName, tankA)
    simulation.AddModelToTask(taskName, tankB)
    simulation.AddModelToTask(taskName, dynamics)
    simulation.InitializeSimulation()
    return dynamics
