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

"""Verify Python lifetimes and native access through fuel-tank connections."""

import gc
import weakref

import pytest

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import (
    fuelTank,
    linearSpringMassDamper,
    sphericalPendulum,
    stateArchitecture,
    thrusterDynamicEffector,
    thrusterStateEffector,
)


@pytest.fixture(
    params=[
        (thrusterDynamicEffector.ThrusterDynamicEffector, "addThrusterSet"),
        (thrusterStateEffector.ThrusterStateEffector, "addThrusterSet"),
        (linearSpringMassDamper.LinearSpringMassDamper, "pushFuelSloshParticle"),
        (sphericalPendulum.SphericalPendulum, "pushFuelSloshParticle"),
    ],
    ids=["dynamicThruster", "stateThruster", "springMassDamper", "pendulum"],
)
def connection(request):
    """Supply each concrete effector and its tank connection method."""
    return request.param


def _make_tank():
    """Create a tank with a model that can compute finite mass properties."""
    tank = fuelTank.FuelTank()
    model = fuelTank.FuelTankModelConstantVolume()
    model.propMassInit = 12.0  # [kg]
    model.radiusTankInit = 0.5  # [m]
    tank.setTankModel(model)
    return tank


def test_tank_retains_connections(connection):
    """Each connection survives local scope and is released with its tank."""
    constructor, method = connection
    tank = _make_tank()

    def connect():
        effector = constructor()
        getattr(tank, method)(effector)
        assert effector.thisown
        return weakref.ref(effector)

    first, second = connect(), connect()
    gc.collect()
    # Fail on the old wrappers without dereferencing their dangling C++ pointers.
    assert first() is not None
    assert second() is not None
    tank_ref = weakref.ref(tank)
    del tank
    gc.collect()
    assert tank_ref() is None
    assert first() is None
    assert second() is None


def test_connection_survives_tank(connection):
    """A tank neither owns nor destroys an effector still held by the caller."""
    constructor, method = connection
    tank = _make_tank()
    effector = constructor()
    getattr(tank, method)(effector)
    tank_ref = weakref.ref(tank)
    del tank
    gc.collect()
    assert tank_ref() is None
    assert effector.thisown
    effector.ModelTag = "survivingEffector"
    assert effector.ModelTag == "survivingEffector"


def test_connection_cycle_is_collectible(connection):
    """User references back to the tank remain visible to garbage collection."""
    constructor, method = connection
    tank = _make_tank()
    effector = constructor()
    getattr(tank, method)(effector)
    object.__setattr__(effector, "tank_reference", tank)
    tank_ref, effector_ref = weakref.ref(tank), weakref.ref(effector)
    del tank, effector
    gc.collect()
    assert tank_ref() is None
    assert effector_ref() is None


@pytest.mark.parametrize(
    "thruster_module", [thrusterDynamicEffector, thrusterStateEffector],
    ids=["dynamicThruster", "stateThruster"],
)
def test_thruster_mass_flow_after_scope_exit(thruster_module):
    """The tank can read thruster flow after the creating helper has returned."""
    tank = _make_tank()
    manager = stateArchitecture.DynParamManager()
    tank.registerStates(manager)
    thrust = 9.80665  # [N]
    specific_impulse = 10.0  # [s]
    standard_gravity = 9.80665  # [m/s^2]

    def connect():
        constructor = (
            thrusterDynamicEffector.ThrusterDynamicEffector
            if thruster_module is thrusterDynamicEffector
            else thrusterStateEffector.ThrusterStateEffector
        )
        effector = constructor()
        config = thruster_module.THRSimConfig()
        config.MaxThrust = thrust
        config.steadyIsp = specific_impulse
        effector.addThruster(config)
        if thruster_module is thrusterDynamicEffector:
            effector.ComputeThrusterFire(config, 0.0)
        else:
            effector.registerStates(manager)
            manager.getStateObject(effector.nameOfKappaState).setState([[1.0]])  # [-] full thrust
            effector.computeDerivatives(
                0.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
            )
        tank.addThrusterSet(effector)
        return weakref.ref(effector)

    effector_ref = connect()
    gc.collect()
    assert effector_ref() is not None
    tank.updateEffectorMassProps(0.0)
    expected_rate = -thrust/(standard_gravity*specific_impulse)  # [kg/s]
    assert tank.effProps.mEffDot == pytest.approx(expected_rate)


@pytest.mark.parametrize(
    "constructor", [linearSpringMassDamper.LinearSpringMassDamper,
                    sphericalPendulum.SphericalPendulum],
    ids=["springMassDamper", "pendulum"],
)
def test_slosh_allocation_after_scope_exit(constructor):
    """The tank can retrieve particle mass and assign its share of fuel flow."""
    tank = _make_tank()
    leak_rate = 0.2  # [kg/s]
    particle_mass = 4.0  # [kg]
    tank.setFuelLeakRate(leak_rate)
    manager = stateArchitecture.DynParamManager()
    tank.registerStates(manager)

    def connect():
        particle = constructor()
        particle.massInit = particle_mass
        particle.registerStates(manager)
        tank.pushFuelSloshParticle(particle=particle)
        return weakref.ref(particle)

    particle_ref = connect()
    gc.collect()
    assert particle_ref() is not None
    tank.updateEffectorMassProps(0.0)
    tank_mass = tank.effProps.mEff
    expected_rate = -leak_rate*particle_mass/(tank_mass + particle_mass)  # [kg/s]
    assert particle_ref().fuelMass == pytest.approx(particle_mass)
    assert particle_ref().fuelMassDot == pytest.approx(expected_rate)
    assert tank.effProps.mEffDot + particle_ref().fuelMassDot == pytest.approx(-leak_rate)


@pytest.mark.parametrize("method", ["addThrusterSet", "pushFuelSloshParticle"])
def test_null_connection_is_rejected(method):
    """Null connections raise before entering the lists used by mass assembly."""
    tank = _make_tank()
    with pytest.raises(BasiliskError, match="non-null"):
        getattr(tank, method)(None)
    manager = stateArchitecture.DynParamManager()
    tank.registerStates(manager)
    tank.updateEffectorMassProps(0.0)
    assert tank.effProps.mEffDot == 0.0


def test_dynamic_thruster_requires_tank_model():
    """A rejected connection is not retained and can be retried after configuration."""
    tank = fuelTank.FuelTank()
    effector = thrusterDynamicEffector.ThrusterDynamicEffector()
    effector_ref = weakref.ref(effector)
    with pytest.raises(BasiliskError, match="setTankModel"):
        tank.addThrusterSet(effector)
    del effector
    gc.collect()
    assert effector_ref() is None
    model = fuelTank.FuelTankModelConstantVolume()
    model.propMassInit = 12.0  # [kg]
    model.radiusTankInit = 0.5  # [m]
    tank.setTankModel(model)
    tank.addThrusterSet(thrusterDynamicEffector.ThrusterDynamicEffector())
    manager = stateArchitecture.DynParamManager()
    tank.registerStates(manager)
    tank.updateEffectorMassProps(0.0)
    assert tank.effProps.mEffDot == 0.0


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
