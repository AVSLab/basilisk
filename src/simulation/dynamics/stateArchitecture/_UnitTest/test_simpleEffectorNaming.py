# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# This file is distributed under the ISC License in LICENSE.

"""Exercise mixed slosh, fuel, thruster, and noise naming through complete simulations."""

import gc
import weakref
from dataclasses import dataclass

import numpy as np
import pytest

from Basilisk.architecture import bskLogging, messaging
from Basilisk.simulation import (
    fuelTank,
    igbmNoiseStateEffector,
    linearSpringMassDamper,
    meanRevertingNoiseStateEffector,
    spacecraft,
    sphericalPendulum,
    svIntegrators,
    thrusterStateEffector,
)
from Basilisk.utilities import SimulationBaseClass, deprecated, macros


FIELDS = {
    "tank": ("nameOfMassState",),
    "damper": ("nameOfRhoState", "nameOfRhoDotState", "nameOfMassState"),
    "pendulum": (
        "nameOfPhiState", "nameOfThetaState", "nameOfPhiDotState", "nameOfThetaDotState", "nameOfMassState",
    ),
    "thruster": ("nameOfKappaState",),
    "ou": ("stateName",),
    "igbm": ("stateName",),
}
PREFIXES = {
    "tank": ("fuelTankMass",),
    "damper": ("linearSpringMassDamperRho", "linearSpringMassDamperRhoDot", "linearSpringMassDamperMass"),
    "pendulum": (
        "sphericalPendulumPhi", "sphericalPendulumTheta", "sphericalPendulumPhiDot",
        "sphericalPendulumThetaDot", "sphericalPendulumMass",
    ),
    "thruster": ("kappaState",),
    "ou": ("meanRevertingNoiseState",),
    "igbm": ("igbmNoiseState",),
}


def get_name(kind, effector, field):
    """Read each module's established Python naming interface."""
    if kind == "tank":
        return effector.getNameOfMassState()
    if kind in ("ou", "igbm"):
        return effector.getStateName()
    return getattr(effector, field)


def set_name(kind, effector, field, value):
    """Set a name without exercising unrelated deprecated interfaces."""
    if kind == "tank":
        effector.setNameOfMassState(value)
    elif kind in ("ou", "igbm"):
        effector.setStateName(value)
    else:
        setattr(effector, field, value)


@dataclass
class MixedSimulation:
    """Retain Python owners of every C++ attachment and integrator."""

    simulation: object
    vehicle: object
    banks: list
    integrator: object
    setup_names: list


def build_simulation(manager_local, custom_names=False, stochastic=False):
    """Construct two mixed banks and attach them in reverse construction order."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("process")
    step = 0.01  # [s]
    process.addTask(simulation.CreateNewTask("task", macros.sec2nano(step)))
    vehicle = spacecraft.Spacecraft()
    vehicle.hub.mHub = 100.0  # [kg]
    vehicle.hub.IHubPntBc_B = np.eye(3) * 100.0  # [kg m^2]
    simulation.AddModelToTask("task", vehicle)
    if manager_local:
        import effectorNamingTestSupport

        effectorNamingTestSupport.enableManagerLocalNaming(vehicle.dynManager)
    # Noise states require a stochastic integrator even when their diffusion is zero.
    integrator = svIntegrators.svStochasticIntegratorMayurama(vehicle)
    integrator.setRNGSeed(42)
    vehicle.setIntegrator(integrator)
    banks = [
        {
            "tank": fuelTank.FuelTank(),
            "damper": linearSpringMassDamper.LinearSpringMassDamper(),
            "pendulum": sphericalPendulum.SphericalPendulum(),
            "thruster": thrusterStateEffector.ThrusterStateEffector(),
            "ou": meanRevertingNoiseStateEffector.MeanRevertingNoiseStateEffector(),
            "igbm": igbmNoiseStateEffector.IgbmNoiseStateEffector(),
        }
        for _ in range(2)
    ]
    banks.reverse()
    setup_names = []
    for index, bank in enumerate(banks):
        model = fuelTank.FuelTankModelConstantVolume()
        model.propMassInit = 2.0  # [kg]
        model.radiusTankInit = 0.5  # [m]
        bank["tank"].setTankModel(model)
        bank["tank"].setFuelLeakRate(0.01)  # [kg/s]
        bank["damper"].massInit = 0.5  # [kg]
        bank["damper"].rhoInit = 0.1 * (index + 1)  # [m]
        bank["damper"].rhoDotInit = 0.02  # [m/s]
        bank["damper"].pHat_B = [1.0, 0.0, 0.0]
        bank["pendulum"].massInit = 0.5  # [kg]
        bank["pendulum"].pendulumRadius = 0.5  # [m]
        bank["pendulum"].phiDotInit = 0.1  # [rad/s]
        for particle in (bank["damper"], bank["pendulum"]):
            bank["tank"].pushFuelSloshParticle(particle)
        for _ in range(index + 2):
            config = thrusterStateEffector.THRSimConfig()
            config.thrLoc_B = [0.0, 1.0, 0.0]  # [m]
            config.thrDir_B = [1.0, 0.0, 0.0]
            config.MaxThrust = 0.2  # [N]
            config.steadyIsp = 200.0  # [s]
            config.cutoffFrequency = 2.0  # [rad/s]
            bank["thruster"].addThruster(config)
        bank["thruster"].kappaInit = messaging.DoubleVector([0.2] * (index + 2))  # [-]
        bank["tank"].addThrusterSet(bank["thruster"])
        for noise in (bank["ou"], bank["igbm"]):
            noise.setStateValue(0.1 * (index + 1))  # [-]
            noise.setTimeConstant(0.5)  # [s]
            noise.setStationaryStd(0.2 if stochastic else 0.0)  # [-]
        bank_names = {}
        # Tank registration precedes its slosh particles under both naming policies.
        for kind, effector in bank.items():
            if custom_names:
                for field in FIELDS[kind]:
                    set_name(kind, effector, field, f"bank{index}_{kind}_{field}")
            bank_names[kind] = tuple(get_name(kind, effector, field) for field in FIELDS[kind])
            vehicle.addStateEffector(effector)
            simulation.AddModelToTask("task", effector)
        setup_names.append(bank_names)
    return MixedSimulation(simulation, vehicle, banks, integrator, setup_names)


def run_simulation(bundle, manager_local, custom_names=False):
    """Check resolved names, state shapes, depletion, and a short coupled integration."""
    bundle.simulation.InitializeSimulation()
    manager = bundle.vehicle.dynManager
    for index, bank in enumerate(bundle.banks):
        for kind, effector in bank.items():
            names = tuple(get_name(kind, effector, field) for field in FIELDS[kind])
            expected = (
                tuple(prefix + str(index + 1) for prefix in PREFIXES[kind])
                if manager_local and not custom_names else bundle.setup_names[index][kind]
            )
            assert names == expected
            for name in names:
                state = manager.getStateObject(name)
                assert state.getName() == name
                rows = index + 2 if kind == "thruster" else 1
                assert np.asarray(state.getState()).shape == (rows, 1)
    stop_time = 0.02  # [s]
    bundle.simulation.ConfigureStopTime(macros.sec2nano(stop_time))
    bundle.simulation.ExecuteSimulation()
    values = []
    for bank in bundle.banks:
        for kind, effector in bank.items():
            for field in FIELDS[kind]:
                values.extend(np.asarray(manager.getStateObject(get_name(kind, effector, field)).getState()).ravel())
        assert manager.getStateObject(bank["tank"].getNameOfMassState()).getState()[0][0] < 2.0  # [kg]
        assert bank["damper"].massState.getState()[0][0] < 0.5  # [kg]
        assert bank["pendulum"].massState.getState()[0][0] < 0.5  # [kg]
        assert bank["thruster"].mDotTotal > 0.0  # [kg/s]
    assert np.all(np.isfinite(values))
    return np.asarray(values)


@pytest.mark.parametrize("custom_names", [False, True])
def test_mixed_lifetimes(manager_local, custom_names):
    """All six families remain isolated across live, cyclic, and freshly rebuilt simulations."""
    live = build_simulation(manager_local, custom_names)
    reference = run_simulation(live, manager_local, custom_names)
    delayed = build_simulation(manager_local, custom_names)
    np.testing.assert_allclose(run_simulation(delayed, manager_local, custom_names), reference, rtol=0.0, atol=1e-14)
    delayed.cycle = delayed
    delayed_ref = weakref.ref(delayed)
    del delayed
    gc.collect()
    assert delayed_ref() is None
    current = build_simulation(manager_local, custom_names)
    np.testing.assert_allclose(run_simulation(current, manager_local, custom_names), reference, rtol=0.0, atol=1e-14)
    # Changing the current manager must leave equal names in the live manager untouched.
    old_noise = live.banks[0]["ou"].getStateValue()
    current.banks[0]["ou"].setStateValue(0.75)  # [-]
    assert live.banks[0]["ou"].getStateValue() == old_noise


@pytest.mark.parametrize("custom_names", [False, True])
def test_mixed_physics_matches_legacy(naming_support, custom_names):
    """Renaming preserves coupled mass depletion, thrust, slosh motion, and noise drift."""
    legacy = build_simulation(False, custom_names)
    local = build_simulation(True, custom_names)
    np.testing.assert_allclose(
        run_simulation(local, True, custom_names), run_simulation(legacy, False, custom_names),
        rtol=0.0, atol=1e-14,
    )


@pytest.mark.parametrize("kind,field", [(kind, field) for kind, fields in FIELDS.items() for field in fields])
def test_python_custom_assignments_are_tracked(naming_support, kind, field):
    """Assigning an automatic-looking name through each Python interface makes it explicit."""
    bundle = build_simulation(True)
    effector = bundle.banks[0][kind]
    original = get_name(kind, effector, field)
    set_name(kind, effector, field, original)
    bundle.simulation.InitializeSimulation()
    assert get_name(kind, effector, field) == original
    with pytest.raises(bskLogging.BasiliskError, match="resolved names cannot be changed"):
        set_name(kind, effector, field, "tooLate")
    assert get_name(kind, effector, field) == original


def test_fuel_tank_deprecated_attribute_tracks_custom_names(naming_support):
    """The legacy Python attribute still warns and routes assignments through the name setter."""
    bundle = build_simulation(True)
    tank = bundle.banks[0]["tank"]
    original = tank.getNameOfMassState()
    with pytest.warns((deprecated.BSKDeprecationWarning, deprecated.BSKUrgentDeprecationWarning)):
        tank.nameOfMassState = original
    bundle.simulation.InitializeSimulation()
    assert tank.getNameOfMassState() == original


def test_stochastic_builds_repeat_with_local_names(naming_support):
    """Fixed seeds produce repeatable stochastic trajectories despite intervening constructions."""
    first = build_simulation(True, stochastic=True)
    reference = run_simulation(first, True)
    unused = build_simulation(False, stochastic=True)
    second = build_simulation(True, stochastic=True)
    np.testing.assert_array_equal(run_simulation(second, True), reference)
    assert unused.banks[0]["ou"].getStateName() != first.banks[0]["ou"].getStateName()


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
