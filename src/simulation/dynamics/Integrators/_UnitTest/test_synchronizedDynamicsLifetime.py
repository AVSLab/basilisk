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

"""Verify retention, release, and propagation of synchronized dynamics objects."""

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
from Basilisk.utilities import macros

MUJOCO_ENABLED = hasBuildFeature("mujoco")
if MUJOCO_ENABLED:
    from Basilisk.simulation import mujoco


@pytest.fixture(params=["spacecraft", "charging", "mujoco"])
def constructor(request):
    """Exercise the shared synchronization interface across dynamics classes."""
    if request.param == "mujoco":
        if not MUJOCO_ENABLED:
            pytest.skip("Requires Basilisk built with --mujoco True")
        return lambda: mujoco.MJScene("<mujoco/>")
    return {
        "spacecraft": spacecraft.Spacecraft,
        "charging": spacecraftChargingDynamics.SpacecraftChargingDynamics,
    }[request.param]


@pytest.mark.parametrize("replace_integrator", [False, True])
def test_primary_retains_secondary(constructor, replace_integrator):
    """A connection keeps its secondary alive and releases it with the primary."""
    primary = constructor()
    secondary = constructor()
    primary_ref, secondary_ref = weakref.ref(primary), weakref.ref(secondary)
    primary.syncDynamicsIntegration(secondary)
    primary.syncDynamicsIntegration(secondary)
    if replace_integrator:
        primary.setIntegrator(svIntegrators.svIntegratorRK4(primary))
    del secondary
    gc.collect()
    assert secondary_ref() is not None
    assert secondary_ref().isDynamicsSynced
    del primary
    gc.collect()
    assert primary_ref() is None
    assert secondary_ref() is None


def test_disowned_primary_rejects_connection(constructor):
    """Reject a primary without Python ownership before retaining or linking its secondary."""
    primary = constructor()
    secondary = constructor()
    secondary_ref = weakref.ref(secondary)
    primary.thisown = False
    try:
        with pytest.raises(BasiliskError, match="owning Python primary object"):
            primary.syncDynamicsIntegration(secondary)
    finally:
        primary.thisown = True

    assert not secondary.isDynamicsSynced
    assert not hasattr(primary, "_bsk_synced_dynamics")
    del secondary
    gc.collect()
    assert secondary_ref() is None


@pytest.mark.skipif(not MUJOCO_ENABLED, reason="Requires Basilisk built with --mujoco True")
@pytest.mark.parametrize("already_connected", [False, True])
def test_borrowed_scene_alias_preserves_connection_state(already_connected):
    """Reject a temporary scene alias without changing an existing owner's connections."""
    primary = mujoco.MJScene('<mujoco><worldbody><body name="probe"/></worldbody></mujoco>')
    secondary = spacecraft.Spacecraft()
    primary_ref, secondary_ref = weakref.ref(primary), weakref.ref(secondary)
    if already_connected:
        primary.syncDynamicsIntegration(secondary)

    alias = primary.getBody("probe").getScene()
    assert int(alias.this) == int(primary.this)
    assert not alias.thisown
    with pytest.raises(BasiliskError, match="owning Python primary object"):
        alias.syncDynamicsIntegration(secondary)
    assert secondary.isDynamicsSynced == already_connected

    del alias, secondary
    gc.collect()
    assert (secondary_ref() is not None) == already_connected
    assert primary_ref() is primary
    del primary
    gc.collect()
    assert primary_ref() is None
    assert secondary_ref() is None


def test_surviving_secondary_can_be_reused(constructor):
    """Destroying a primary releases the synchronized flag on a surviving object."""
    primary = constructor()
    secondary = constructor()
    primary_ref = weakref.ref(primary)
    primary.syncDynamicsIntegration(secondary)
    del primary
    gc.collect()
    assert primary_ref() is None
    assert not secondary.isDynamicsSynced
    replacement_primary = constructor()
    replacement_primary.syncDynamicsIntegration(secondary)
    assert secondary.isDynamicsSynced
    del replacement_primary
    gc.collect()
    assert not secondary.isDynamicsSynced


def test_reference_cycle_is_collectible(constructor):
    """A user-held reference back to the primary does not leak the connected group."""
    primary = constructor()
    secondary = constructor()
    primary.syncDynamicsIntegration(secondary)
    object.__setattr__(secondary, "primary_reference", primary)
    primary_ref, secondary_ref = weakref.ref(primary), weakref.ref(secondary)
    del primary, secondary
    gc.collect()
    assert primary_ref() is None
    assert secondary_ref() is None


def _spacecraft_with_speed(speed):
    """Create an initialized free spacecraft with velocity ``speed`` in m/s."""
    body = spacecraft.Spacecraft()
    body.hub.mHub = 1.0  # [kg]
    body.hub.IHubPntBc_B = np.eye(3).tolist()  # [kg*m^2]
    body.hub.v_CN_NInit = [speed, 0.0, 0.0]  # [m/s]
    body.Reset(0)
    return body


@pytest.mark.parametrize("integrator_name", ["svIntegratorRK4", "svIntegratorRKF45"])
def test_propagation_after_secondary_leaves_scope(integrator_name):
    """Only the synchronization connection retains the secondary during a step."""
    primary_speed = 1.0  # [m/s]
    secondary_speed = 2.0  # [m/s]
    primary = _spacecraft_with_speed(primary_speed)
    primary.setIntegrator(getattr(svIntegrators, integrator_name)(primary))

    def connect_secondary():
        secondary = _spacecraft_with_speed(secondary_speed)
        primary.syncDynamicsIntegration(secondary)
        return weakref.ref(secondary)

    secondary_ref = connect_secondary()
    gc.collect()
    # Fail safely on the old implementation, before accessing freed storage.
    assert secondary_ref() is not None
    stop_time = macros.sec2nano(1.0)  # [ns]
    primary.UpdateState(stop_time)
    secondary_ref().UpdateState(stop_time)
    expected_position = [2.0, 0.0, 0.0]  # [m]
    tolerance = 1e-12  # [m]
    np.testing.assert_allclose(
        secondary_ref().scStateOutMsg.read().r_BN_N,
        expected_position, rtol=0.0, atol=tolerance,
    )

    secondary = secondary_ref()
    del primary
    gc.collect()
    assert not secondary.isDynamicsSynced
    # Its original integrator remains usable after the primary is destroyed.
    secondary.UpdateState(2 * stop_time)
    expected_position = [4.0, 0.0, 0.0]  # [m]
    np.testing.assert_allclose(
        secondary.scStateOutMsg.read().r_BN_N,
        expected_position, rtol=0.0, atol=tolerance,
    )


def test_invalid_connections_preserve_lifetimes():
    """Rejected graph changes neither retain new objects nor create cycles."""
    primary = spacecraft.Spacecraft()
    secondary = spacecraft.Spacecraft()
    other = spacecraft.Spacecraft()
    primary.syncDynamicsIntegration(secondary)
    with pytest.raises(BasiliskError):
        primary.syncDynamicsIntegration(None)
    with pytest.raises(BasiliskError):
        primary.syncDynamicsIntegration(primary)
    with pytest.raises(BasiliskError):
        other.syncDynamicsIntegration(secondary)
    with pytest.raises(BasiliskError):
        secondary.syncDynamicsIntegration(other)
    with pytest.raises(BasiliskError):
        other.syncDynamicsIntegration(primary)
    other_ref = weakref.ref(other)
    del other
    gc.collect()
    assert other_ref() is None
    assert secondary.isDynamicsSynced
    primary_ref = weakref.ref(primary)
    del primary
    gc.collect()
    assert primary_ref() is None
    assert not secondary.isDynamicsSynced


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
