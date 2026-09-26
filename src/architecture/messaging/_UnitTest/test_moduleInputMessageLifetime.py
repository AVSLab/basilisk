# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

"""Check source lifetimes for subscriptions created inside module methods."""

import gc
import importlib
import weakref

import numpy as np
import pytest

from Basilisk.architecture import bskLogging, messaging
from Basilisk.fswAlgorithms import formationBarycenter
from Basilisk.utilities import simHelpers


# Module, class, configuration method, retained-reader vector, message type.
CASES = [
    ("exponentialAtmosphere", "ExponentialAtmosphere", "addSpacecraftToModel", "scStateInMsgs", "SCStatesMsg"),
    ("msisAtmosphere", "MsisAtmosphere", "addSpacecraftToModel", "scStateInMsgs", "SCStatesMsg"),
    ("tabularAtmosphere", "TabularAtmosphere", "addSpacecraftToModel", "scStateInMsgs", "SCStatesMsg"),
    ("magneticFieldCenteredDipole", "MagneticFieldCenteredDipole", "addSpacecraftToModel", "scStateInMsgs", "SCStatesMsg"),
    ("magneticFieldWMM", "MagneticFieldWMM", "addSpacecraftToModel", "scStateInMsgs", "SCStatesMsg"),
    ("zeroWindModel", "ZeroWindModel", "addSpacecraftToModel", "scStateInMsgs", "SCStatesMsg"),
    ("eclipse", "Eclipse", "addSpacecraftToModel", "positionInMsgs", "SCStatesMsg"),
    ("groundLocation", "GroundLocation", "addSpacecraftToModel", "scStateInMsgs", "SCStatesMsg"),
    ("spacecraftLocation", "SpacecraftLocation", "addSpacecraftToModel", "scStateInMsgs", "SCStatesMsg"),
    ("stripLocation", "StripLocation", "addSpacecraftToModel", "scStateInMsgs", "SCStatesMsg"),
    ("spacecraftChargingEquilibrium", "SpacecraftChargingEquilibrium", "addSpacecraft", "scStateInMsgs", "SCStatesMsg"),
    ("msmForceTorque", "MsmForceTorque", "addSpacecraftToModel", "scStateInMsgs", "SCStatesMsg"),
    ("eclipse", "Eclipse", "addPlanetToModel", "planetInMsgs", "SpicePlanetStateMsg"),
    ("ephemerisConverter", "EphemerisConverter", "addSpiceInputMsg", "spiceInMsgs", "SpicePlanetStateMsg"),
]


def _add_source(model, method, source):
    """Supply the additional geometry required by the MSM configuration API."""
    if type(model).__name__ == "MsmForceTorque":
        radii = messaging.DoubleVector([1.0])  # [m]
        positions = simHelpers.npList2EigenXdVector([[0.0, 0.0, 0.0]])  # [m]
        getattr(model, method)(source, radii, positions)
    else:
        getattr(model, method)(source)


@pytest.mark.parametrize("release", ["destroy", "unsubscribe", "replace", "clear"])
@pytest.mark.parametrize("case", CASES, ids=[case[0] + "-" + case[2] for case in CASES])
def test_configuration_retains_and_releases_sources(case, release):
    """Native readers retain local inputs through growth and release them afterward.

    Check weak references before reading so the unfixed code fails without
    dereferencing freed storage. Distinct payloads and timestamps also verify
    that every reader retains its own source after vector reallocation.
    """
    module_name, class_name, method, field, message_name = case
    module = importlib.import_module("Basilisk.simulation." + module_name)
    model = getattr(module, class_name)()
    message_type = getattr(messaging, message_name)
    payload_type = getattr(messaging, message_name + "Payload")
    position_field = "r_BN_N" if message_name == "SCStatesMsg" else "PositionVector"
    count = 2 if method == "addSpacecraft" else 8
    source_refs = []
    expected_positions = []
    for index in range(count):
        position = [float(index + 1), 20.0, 30.0]  # [m]
        timestamp = index + 10  # [ns]
        payload = payload_type()
        setattr(payload, position_field, position)
        source = message_type().write(payload, timestamp)
        source_refs.append(weakref.ref(source))
        expected_positions.append(position)
        _add_source(model, method, source)
        del source
    gc.collect()
    assert all(ref() is not None for ref in source_refs), "C++ retained a pointer to a collected input"

    for index, position in enumerate(expected_positions):
        np.testing.assert_array_equal(getattr(getattr(model, field)[index](), position_field), position)
        assert getattr(model, field)[index].timeWritten() == index + 10

    if release == "unsubscribe":
        for index in range(count):
            getattr(model, field)[index].unsubscribe()
    elif release == "replace":
        replacement = message_type()
        replacement_ref = weakref.ref(replacement)
        for index in range(count):
            getattr(model, field)[index].subscribeTo(replacement)
        del replacement
        gc.collect()
        assert replacement_ref() is not None
    elif release == "clear":
        getattr(model, field).clear()
    else:
        del model
    gc.collect()
    assert all(ref() is None for ref in source_refs), "Readers leaked their former sources"
    if release == "replace":
        del model
        gc.collect()
        assert replacement_ref() is None


def test_copied_reader_outlives_module():
    """A copied native reader retains the source after its module is destroyed."""
    from Basilisk.simulation import eclipse

    model = eclipse.Eclipse()
    source = messaging.SCStatesMsg()
    source_ref = weakref.ref(source)
    model.addSpacecraftToModel(source)
    reader = messaging.SCStatesMsgReader(model.positionInMsgs[0])
    del source, model
    gc.collect()
    assert source_ref() is not None
    reader.unsubscribe()
    gc.collect()
    assert source_ref() is None


def test_formation_retains_both_inputs():
    """The barycenter method retains both inputs and produces the expected state."""
    model = formationBarycenter.FormationBarycenter()
    nav_payload = messaging.NavTransMsgPayload()
    nav_payload.r_BN_N = [1.0, 2.0, 3.0]  # [m]
    nav_payload.v_BN_N = [4.0, 5.0, 6.0]  # [m/s]
    config_payload = messaging.VehicleConfigMsgPayload()
    config_payload.massSC = 10.0  # [kg]
    nav = messaging.NavTransMsg().write(nav_payload)
    config = messaging.VehicleConfigMsg().write(config_payload)
    nav_ref, config_ref = weakref.ref(nav), weakref.ref(config)
    model.addSpacecraftToModel(nav, config)
    del nav, config
    gc.collect()
    assert nav_ref() is not None
    assert config_ref() is not None
    model.SelfInit()
    model.Reset(0)
    model.UpdateState(0)
    np.testing.assert_array_equal(model.transOutMsg.read().r_BN_N, nav_payload.r_BN_N)
    np.testing.assert_array_equal(model.transOutMsg.read().v_BN_N, nav_payload.v_BN_N)
    del model
    gc.collect()
    assert nav_ref() is None
    assert config_ref() is None


def test_rejected_input_is_not_retained():
    """Rejecting a third charging-model spacecraft preserves both prior inputs."""
    from Basilisk.simulation import spacecraftChargingEquilibrium

    model = spacecraftChargingEquilibrium.SpacecraftChargingEquilibrium()
    first, second = messaging.SCStatesMsg(), messaging.SCStatesMsg()
    model.addSpacecraft(first)
    model.addSpacecraft(second)
    rejected = messaging.SCStatesMsg()
    rejected_ref = weakref.ref(rejected)
    with pytest.raises(bskLogging.BasiliskError):
        model.addSpacecraft(rejected)
    del rejected
    gc.collect()
    assert rejected_ref() is None
    assert model.scStateInMsgs[0].isSubscribedTo(first)
    assert model.scStateInMsgs[1].isSubscribedTo(second)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
