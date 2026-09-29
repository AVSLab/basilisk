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
#

"""Verify Python integrator ownership transfer and propagation after collection."""

import gc
import weakref

import numpy as np
import pytest

from Basilisk import hasBuildFeature
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import (
    spacecraft,
    spacecraftChargingDynamics,
    svIntegrators,
)
from Basilisk.utilities import SimulationBaseClass, macros

MUJOCO_ENABLED = hasBuildFeature("mujoco")
if MUJOCO_ENABLED:
    from Basilisk.simulation import mujoco


@pytest.fixture(params=["spacecraft", "spacecraftChargingDynamics", "mujoco"])
def dynamics_object(request):
    """Construct a dynamics owner that exposes the shared interface."""
    if request.param == "mujoco":
        if not MUJOCO_ENABLED:
            pytest.skip("Requires Basilisk built with --mujoco True")
        return mujoco.MJScene("<mujoco/>")
    constructors = {
        "spacecraft": spacecraft.Spacecraft,
        "spacecraftChargingDynamics": spacecraftChargingDynamics.SpacecraftChargingDynamics,
    }
    return constructors[request.param]()


@pytest.mark.parametrize("use_attribute", [False, True])
@pytest.mark.parametrize("disown_method", [None, "disown", "thisown"])
def test_transfer_and_reinstall(dynamics_object, use_attribute, disown_method):
    """Both entry points accept explicit disowning and reinstalling the active object."""
    integrator = svIntegrators.svIntegratorRK4(dynamics_object)
    address = int(integrator.this)
    assert integrator.thisown
    if disown_method == "disown":
        integrator.this.disown()
    elif disown_method == "thisown":
        integrator.thisown = False
    if use_attribute:
        dynamics_object.integrator = integrator
    else:
        dynamics_object.setIntegrator(integrator)
    assert not integrator.thisown
    assert int(dynamics_object.integrator.this) == address
    assert int(dynamics_object.getIntegrator().this) == address
    assert not dynamics_object.integrator.thisown

    dynamics_object.setIntegrator(integrator)
    dynamics_object.integrator = dynamics_object.integrator
    assert int(dynamics_object.integrator.this) == address
    other = spacecraft.Spacecraft()
    with pytest.raises(BasiliskError, match="already owned"):
        other.setIntegrator(integrator)
    proxy_ref = weakref.ref(integrator)
    del integrator
    gc.collect()
    assert proxy_ref() is None
    assert int(dynamics_object.integrator.this) == address


def test_rejection_preserves_active_integrator(dynamics_object):
    """Rejected null or mismatched integrators leave the existing owner usable."""
    original_address = int(dynamics_object.integrator.this)
    with pytest.raises(BasiliskError, match="null pointer"):
        dynamics_object.setIntegrator(None)

    other = spacecraft.Spacecraft()
    rejected = svIntegrators.svIntegratorRK4(other)
    with pytest.raises(BasiliskError, match="created using this DynamicObject"):
        dynamics_object.setIntegrator(rejected)
    assert not rejected.thisown
    with pytest.raises(BasiliskError, match="already owned"):
        other.setIntegrator(rejected)
    del rejected
    gc.collect()

    with pytest.raises(BasiliskError, match="already owned"):
        dynamics_object.setIntegrator(other.integrator)
    assert int(dynamics_object.integrator.this) == original_address
    assert other.integrator is not None


@pytest.mark.parametrize("proxy_source", ["original", "getter", "attribute"])
def test_owner_can_be_destroyed_before_disowned_proxy(proxy_source):
    """Dropping an integrator proxy after its C++ owner cannot delete it a second time."""
    body = spacecraft.Spacecraft()
    other = spacecraft.Spacecraft()
    integrator = svIntegrators.svIntegratorRK4(body)
    body.setIntegrator(integrator)
    if proxy_source == "getter":
        integrator = body.getIntegrator()
    elif proxy_source == "attribute":
        integrator = body.integrator
    body_ref = weakref.ref(body)
    del body
    gc.collect()
    assert body_ref() is None
    assert not integrator.thisown
    with pytest.raises(BasiliskError, match="already owned"):
        other.setIntegrator(integrator)
    del integrator
    gc.collect()


def test_replaced_integrator_cannot_be_reinstalled():
    """Original and borrowed proxies cannot reinstall an integrator after replacement."""
    body = spacecraft.Spacecraft()
    other = spacecraft.Spacecraft()
    original = svIntegrators.svIntegratorRK4(body)
    body.setIntegrator(original)
    borrowed = body.getIntegrator()
    body.setIntegrator(svIntegrators.svIntegratorRK2(body))
    for proxy in (original, borrowed):
        for target in (body, other):
            with pytest.raises(BasiliskError, match="already owned"):
                target.setIntegrator(proxy)


def _configure_spacecraft(speed):
    """Create a spacecraft in force-free translation at the supplied speed in m/s."""
    body = spacecraft.Spacecraft()
    body.hub.mHub = 1.0  # [kg]
    body.hub.IHubPntBc_B = np.eye(3).tolist()  # [kg*m^2]
    body.hub.v_CN_NInit = [speed, 0.0, 0.0]  # [m/s]
    return body


def _propagate(bodies):
    """Propagate all supplied spacecraft for one second and return recorded positions."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("ownershipProcess")
    step = macros.sec2nano(0.1)  # [ns]
    process.addTask(simulation.CreateNewTask("ownershipTask", step))
    recorders = []
    for index, body in enumerate(bodies):
        body.ModelTag = f"ownershipBody{index}"
        simulation.AddModelToTask("ownershipTask", body)
        recorder = body.scStateOutMsg.recorder()
        simulation.AddModelToTask("ownershipTask", recorder)
        recorders.append(recorder)
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(1.0))  # [ns]
    simulation.ExecuteSimulation()
    return [recorder.r_BN_N[-1].copy() for recorder in recorders]


@pytest.mark.parametrize(
    "integrator_name", ["svIntegratorRK4", "svIntegratorRK2", "svIntegratorRKF45"]
)
@pytest.mark.parametrize("use_attribute", [False, True])
def test_inline_integrator_survives_collection(integrator_name, use_attribute):
    """Temporary integrator wrappers may disappear before initialization and propagation."""
    speed = 1.0  # [m/s]
    body = _configure_spacecraft(speed)
    constructor = getattr(svIntegrators, integrator_name)
    if use_attribute:
        body.integrator = constructor(body)
    else:
        body.setIntegrator(constructor(body))
    gc.collect()
    expected_position = [1.0, 0.0, 0.0]  # [m]
    position_tolerance = 1e-12  # [m]
    np.testing.assert_allclose(_propagate([body])[0], expected_position, atol=position_tolerance)


@pytest.mark.parametrize("use_attribute", [False, True])
def test_explicitly_disowned_integrator_survives_collection(use_attribute):
    """Legacy explicit disowning before installation still allows normal propagation."""
    speed = 1.0  # [m/s]
    body = _configure_spacecraft(speed)
    integrator = svIntegrators.svIntegratorRK4(body)
    integrator.this.disown()
    if use_attribute:
        body.integrator = integrator
    else:
        body.setIntegrator(integrator)
    del integrator
    gc.collect()
    expected_position = [1.0, 0.0, 0.0]  # [m]
    position_tolerance = 1e-12  # [m]
    np.testing.assert_allclose(_propagate([body])[0], expected_position, atol=position_tolerance)


def test_replacement_preserves_synchronized_propagation():
    """Replacing a primary integrator preserves both synchronized spacecraft states."""
    primary_speed = 1.0  # [m/s]
    secondary_speed = 2.0  # [m/s]
    primary = _configure_spacecraft(primary_speed)
    secondary = _configure_spacecraft(secondary_speed)
    primary.setIntegrator(svIntegrators.svIntegratorRK2(primary))
    primary.syncDynamicsIntegration(secondary)
    primary.setIntegrator(svIntegrators.svIntegratorRK4(primary))

    original_secondary = int(secondary.integrator.this)
    rejected = svIntegrators.svIntegratorRK4(secondary)
    secondary.setIntegrator(rejected)
    assert not rejected.thisown
    with pytest.raises(BasiliskError, match="already owned"):
        secondary.setIntegrator(rejected)
    del rejected
    gc.collect()
    assert int(secondary.integrator.this) == original_secondary

    expected_positions = [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]  # [m]
    position_tolerance = 1e-12  # [m]
    np.testing.assert_allclose(
        _propagate([primary, secondary]), expected_positions, atol=position_tolerance
    )
