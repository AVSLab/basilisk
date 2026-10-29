# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

"""Check gateway lifetimes for subscriptions created inside module methods."""

import gc
import importlib
import weakref

import numpy as np
import pytest

from Basilisk.architecture import messaging
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
]


def _add_source(model, method, source):
    """Supply the additional configuration required by each connection API."""
    if type(model).__name__ == "MsmForceTorque":
        radii = messaging.DoubleVector([1.0])  # [m]
        positions = simHelpers.npList2EigenXdVector([[0.0, 0.0, 0.0]])  # [m]
        getattr(model, method)(source, radii, positions)
    else:
        getattr(model, method)(source)


@pytest.mark.parametrize("release", ["reassign", "destroy"])
@pytest.mark.parametrize("case", [case for case in CASES if case[4] == "SCStatesMsg"],
                         ids=lambda case: case[0])
def test_configuration_retains_redirected_gateway(case, release):
    """Registered readers retain gateway storage after producer reassignment or deletion."""
    from Basilisk.simulation import spacecraft

    module_name, class_name, method, field, _ = case
    module = importlib.import_module("Basilisk.simulation." + module_name)
    consumer = getattr(module, class_name)()
    producer = spacecraft.Spacecraft()
    gateway = messaging.SCStatesMsg()
    gateway_ref = weakref.ref(gateway)
    payload = messaging.SCStatesMsgPayload()
    payload.r_BN_N = [1.0, 2.0, 3.0]  # [m]
    timestamp = 10  # [ns]
    module_id = producer.moduleID
    producer.scStateOutMsg = gateway
    producer.scStateOutMsg.write(payload, timestamp, module_id)
    _add_source(consumer, method, producer.scStateOutMsg)
    del gateway

    if release == "reassign":
        producer.scStateOutMsg = messaging.SCStatesMsg()
        producer.scStateOutMsg.write(messaging.SCStatesMsgPayload(), timestamp, module_id)
    else:
        del producer
    gc.collect()
    # Check ownership before reading, so a regression cannot access freed storage.
    assert gateway_ref() is not None
    reader = getattr(consumer, field)[0]
    np.testing.assert_array_equal(reader().r_BN_N, payload.r_BN_N)
    assert reader.timeWritten() == timestamp
    assert reader.moduleID() == module_id

    # The reader must remain attached to live gateway storage, not a payload snapshot.
    payload.r_BN_N = [4.0, 5.0, 6.0]  # [m]
    next_timestamp = 20  # [ns]
    gateway_ref().write(payload, next_timestamp, module_id)
    np.testing.assert_array_equal(reader().r_BN_N, payload.r_BN_N)
    assert reader.timeWritten() == next_timestamp
    reader.unsubscribe()
    gc.collect()
    assert gateway_ref() is None
