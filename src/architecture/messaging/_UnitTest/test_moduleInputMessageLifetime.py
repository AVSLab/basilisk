# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

"""Check source lifetimes for subscriptions created inside module methods."""

import gc
import importlib
import subprocess
import sys
import weakref

import numpy as np
import pytest

from Basilisk import hasBuildFeature
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

# Additional configuration paths; the optional final entry selects an overload.
CASES += [
    ("simpleBattery", "SimpleBattery", "addPowerNodeToModel", "nodePowerUseInMsgs", "PowerNodeUsageMsg"),
    ("simplePowerMonitor", "SimplePowerMonitor", "addPowerNodeToModel", "nodePowerUseInMsgs", "PowerNodeUsageMsg"),
    ("simpleStorageUnit", "SimpleStorageUnit", "addDataNodeToModel", "nodeDataUseInMsgs", "DataNodeUsageMsg"),
    ("partitionedStorageUnit", "PartitionedStorageUnit", "addDataNodeToModel", "nodeDataUseInMsgs", "DataNodeUsageMsg"),
    ("simpleTransmitter", "SimpleTransmitter", "addStorageUnitToTransmitter", "storageUnitInMsgs", "DataStorageStatusMsg"),
    ("spaceToGroundTransmitter", "SpaceToGroundTransmitter", "addStorageUnitToTransmitter", "storageUnitInMsgs", "DataStorageStatusMsg"),
    ("spaceToGroundTransmitter", "SpaceToGroundTransmitter", "addAccessMsgToTransmitter", "groundLocationAccessInMsgs", "AccessMsg"),
    ("mappingInstrument", "MappingInstrument", "addMappingPoint", "accessInMsgs", "AccessMsg"),
    ("simpleAntenna", "SimpleAntenna", "addPlanetToModel", "planetInMsgs", "SpicePlanetStateMsg"),
    ("vizInterface", "VizInterface", "addCamMsgToModule", "cameraConfInMsgs", "CameraConfigMsg"),
    ("albedo", "Albedo", "addPlanetandAlbedoAverageModel", "planets", "SpicePlanetStateMsg"),
    ("albedo", "Albedo", "addPlanetandAlbedoAverageModel", "planets", "SpicePlanetStateMsg", "grid"),
    ("albedo", "Albedo", "addPlanetandAlbedoDataModel", "planets", "SpicePlanetStateMsg"),
]

PRIVATE_CASES = [
    ("smallBodyNavEKF", "SmallBodyNavEKF", "addThrusterToFilter", "THROutputMsg"),
    ("thrusterDynamicEffector", "ThrusterDynamicEffector", "addThruster", "SCStatesMsg"),
    ("thrusterStateEffector", "ThrusterStateEffector", "addThruster", "SCStatesMsg"),
]


def _add_source(model, method, source, variant=None):
    """Supply the additional configuration required by each connection API."""
    if type(model).__name__ == "MsmForceTorque":
        radii = messaging.DoubleVector([1.0])  # [m]
        positions = simHelpers.npList2EigenXdVector([[0.0, 0.0, 0.0]])  # [m]
        getattr(model, method)(source, radii, positions)
    elif method == "addMappingPoint":
        model.addMappingPoint(source, "testData")
    elif method == "addThruster":
        from Basilisk.simulation.THRSimConfig import THRSimConfig
        model.addThruster(THRSimConfig(), source)
    elif method == "addPlanetandAlbedoDataModel":
        # The file is only opened during Reset; this test checks registration.
        model.addPlanetandAlbedoDataModel(source, "", "unused.csv")
    elif variant == "grid":
        average = 0.3  # [-] albedo
        model.addPlanetandAlbedoAverageModel(source, average, 2, 4)
    else:
        getattr(model, method)(source)


def _reader(model, field, index):
    """Access a stored reader, including readers nested inside planet entries."""
    entry = getattr(model, field)[index]
    return entry.planetMsg if field == "planets" else entry


def _payload_marker(message_name, index):
    """Return a distinguishable payload field and value, with units below."""
    values = {
        "SCStatesMsg": ("r_BN_N", [float(index + 1), 20.0, 30.0]),  # [m]
        "SpicePlanetStateMsg": ("PositionVector", [float(index + 1), 20.0, 30.0]),  # [m]
        "PowerNodeUsageMsg": ("netPower", float(index + 1)),  # [W]
        "DataNodeUsageMsg": ("baudRate", float(index + 1)),  # [bit/s]
        "DataStorageStatusMsg": ("storageLevel", float(index + 1)),  # [bit]
        "AccessMsg": ("slantRange", float(index + 1)),  # [m]
        "CameraConfigMsg": ("cameraID", index + 1),
    }
    return values[message_name]


@pytest.mark.parametrize("release", ["destroy", "unsubscribe", "replace", "clear"])
@pytest.mark.parametrize("case", CASES, ids=["-".join([case[0], case[2], *case[5:]]) for case in CASES])
def test_configuration_retains_and_releases_sources(case, release):
    """Native readers retain local inputs through growth and release them afterward.

    Check weak references before reading so the unfixed code fails without
    dereferencing freed storage. Distinct payloads and timestamps also verify
    that every reader retains its own source after vector reallocation.
    """
    module_name, class_name, method, field, message_name = case[:5]
    if module_name == "vizInterface" and not hasBuildFeature("vizInterface"):
        pytest.skip("Requires Basilisk built with --vizInterface True")
    package = "fswAlgorithms" if module_name == "smallBodyNavEKF" else "simulation"
    module = importlib.import_module("Basilisk." + package + "." + module_name)
    model = getattr(module, class_name)()
    message_type = getattr(messaging, message_name)
    payload_type = getattr(messaging, message_name + "Payload")
    count = 2 if method == "addSpacecraft" else 8
    source_refs = []
    expected_values = []
    for index in range(count):
        payload_field, value = _payload_marker(message_name, index)
        timestamp = index + 10  # [ns]
        payload = payload_type()
        setattr(payload, payload_field, value)
        source = message_type().write(payload, timestamp)
        source_refs.append(weakref.ref(source))
        expected_values.append(value)
        _add_source(model, method, source, *case[5:])
        del source
    gc.collect()
    assert all(ref() is not None for ref in source_refs), "C++ retained a pointer to a collected input"

    for index, value in enumerate(expected_values):
        np.testing.assert_array_equal(getattr(_reader(model, field, index)(), payload_field), value)
        assert _reader(model, field, index).timeWritten() == index + 10

    if release == "unsubscribe":
        for index in range(count):
            _reader(model, field, index).unsubscribe()
    elif release == "replace":
        replacement = message_type()
        replacement_ref = weakref.ref(replacement)
        for index in range(count):
            _reader(model, field, index).subscribeTo(replacement)
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


def test_private_facet_readers_retain_sources():
    """Private facet readers keep temporary inputs alive through native reads."""
    from Basilisk.simulation import facetSRPDynamicEffector

    model = facetSRPDynamicEffector.FacetSRPDynamicEffector()
    count = 8
    model.setNumArticulatedFacets(count)
    source_refs = []
    for index in range(count):
        payload = messaging.HingedRigidBodyMsgPayload()
        payload.theta = 0.1 * index  # [rad]
        source = messaging.HingedRigidBodyMsg().write(payload)
        source_refs.append(weakref.ref(source))
        model.addArticulatedFacet(source)
        del source
    gc.collect()
    assert all(ref() is not None for ref in source_refs)
    model.ReadMessages()
    del model
    gc.collect()
    assert all(ref() is None for ref in source_refs)


@pytest.mark.parametrize("case", PRIVATE_CASES, ids=[case[0] for case in PRIVATE_CASES])
def test_private_thruster_readers_retain_sources(case):
    """Private readers retain distinct inputs through growth and module destruction."""
    module_name, class_name, method, message_name = case
    package = "fswAlgorithms" if module_name == "smallBodyNavEKF" else "simulation"
    module = importlib.import_module("Basilisk." + package + "." + module_name)
    model = getattr(module, class_name)()
    source_refs = []
    for _ in range(8):
        source = getattr(messaging, message_name)()
        source_refs.append(weakref.ref(source))
        _add_source(model, method, source)
        del source
    gc.collect()
    assert all(ref() is not None for ref in source_refs)
    del model
    gc.collect()
    assert all(ref() is None for ref in source_refs)


@pytest.mark.parametrize("module_name,class_name", [case[:2] for case in PRIVATE_CASES[1:]])
def test_thruster_without_attached_body_still_supported(module_name, class_name):
    """The original one-argument overload still registers a hub-mounted thruster."""
    from Basilisk.simulation.THRSimConfig import THRSimConfig

    module = importlib.import_module("Basilisk.simulation." + module_name)
    model = getattr(module, class_name)()
    model.addThruster(THRSimConfig())
    assert len(model.thrusterData) == 1
    assert len(model.thrusterOutMsgs) == 1


@pytest.mark.parametrize("configure_first", [False, True])
@pytest.mark.parametrize("copy_reader", [False, True])
def test_pending_facet_readers_retain_sources(configure_first, copy_reader):
    """Pending and active facet readers preserve retention in either setup order."""
    from Basilisk.simulation import facetedSpacecraftModel

    model = facetedSpacecraftModel.FacetedSpacecraftModel()
    count = 8
    if configure_first:
        model.setNumTotalFacets(count)
    source_refs = []
    for index in range(count):
        payload = messaging.HingedRigidBodyMsgPayload()
        payload.theta = 0.1 * index  # [rad]
        source = messaging.HingedRigidBodyMsg().write(payload)
        source_refs.append(weakref.ref(source))
        model.addArticulatedFacet(source)
        del source
    gc.collect()
    assert all(ref() is not None for ref in source_refs)

    # Configure or reconfigure the active list from the privately stored requests.
    model.setNumTotalFacets(count)
    for index in range(count):
        assert model.articulatedFacetDataInMsgs[index]().theta == 0.1 * index
        model.articulatedFacetDataInMsgs[index].unsubscribe()
    gc.collect()
    assert all(ref() is not None for ref in source_refs), "Pending readers still need their sources"
    model.setNumTotalFacets(count)
    if copy_reader:
        reader = messaging.HingedRigidBodyMsgReader(model.articulatedFacetDataInMsgs[0])
    del model
    gc.collect()
    assert all(ref() is None for ref in source_refs[1:])
    if copy_reader:
        assert source_refs[0]() is not None
        assert reader().theta == 0.0
        reader.unsubscribe()
        gc.collect()
    assert source_refs[0]() is None


def test_downlink_retains_sources_after_duplicate_registration():
    """A rejected duplicate must not replace the retention on another reader."""
    from Basilisk.simulation import downlinkHandling

    model = downlinkHandling.DownlinkHandling()
    source_refs = []
    count = 8
    for index in range(count):
        payload = messaging.DataStorageStatusMsgPayload()
        payload.storageLevel = 100.0 * (index + 1)  # [bit]
        source = messaging.DataStorageStatusMsg().write(payload)
        source_refs.append(weakref.ref(source))
        assert model.addStorageUnitToDownlink(source)
        del source
    gc.collect()
    assert all(ref() is not None for ref in source_refs)
    assert model.addStorageUnitToDownlink(source_refs[0]()) is False
    gc.collect()
    assert all(ref() is not None for ref in source_refs)
    model.Reset(0)
    model.UpdateState(0)
    assert model.downlinkOutMsg.read().availableDataBits == 100.0 * count
    del model
    gc.collect()
    assert all(ref() is None for ref in source_refs)


def test_downlink_rejects_invalid_reader_without_retaining_sources():
    """The internal reader entry point rejects mismatched and null sources."""
    from Basilisk.simulation import downlinkHandling

    model = downlinkHandling.DownlinkHandling()
    source = messaging.DataStorageStatusMsg()
    other = messaging.DataStorageStatusMsg()
    source_ref, other_ref = weakref.ref(source), weakref.ref(other)
    with pytest.raises(bskLogging.BasiliskError):
        model._addStorageUnitReader(source, other.addSubscriber())
    with pytest.raises(bskLogging.BasiliskError):
        model.addStorageUnitToDownlink(None)
    del source, other
    gc.collect()
    assert source_ref() is None
    assert other_ref() is None


@pytest.mark.parametrize("import_order", ["albedo, earthRadiationModel", "earthRadiationModel, albedo"])
def test_albedo_retention_with_shared_base_bindings(import_order):
    """Both radiation wrappers must agree on the shared planet-vector type."""
    script = f"""
import gc
import weakref
from Basilisk.simulation import {import_order}
from Basilisk.architecture import messaging

model = albedo.Albedo()
payload = messaging.SpicePlanetStateMsgPayload()
payload.PositionVector = [1.0, 2.0, 3.0]  # [m]
source = messaging.SpicePlanetStateMsg().write(payload)
source_ref = weakref.ref(source)
model.addPlanetandAlbedoAverageModel(source)
del source
gc.collect()
assert source_ref() is not None
assert list(model.planets[0].planetMsg().PositionVector) == [1.0, 2.0, 3.0]
model.planets.clear()
gc.collect()
assert source_ref() is None
"""
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def test_mapping_rejects_input_without_replacing_retention():
    """A rejected mapping name leaves the previously registered source intact."""
    from Basilisk.simulation import mappingInstrument

    model = mappingInstrument.MappingInstrument()
    accepted = messaging.AccessMsg()
    rejected = messaging.AccessMsg()
    accepted_ref, rejected_ref = weakref.ref(accepted), weakref.ref(rejected)
    model.addMappingPoint(accepted, "valid")
    with pytest.raises(bskLogging.BasiliskError):
        model.addMappingPoint(rejected, "x" * 1000)
    del accepted, rejected
    gc.collect()
    assert accepted_ref() is not None
    assert rejected_ref() is None
    model.accessInMsgs[0].unsubscribe()
    gc.collect()
    assert accepted_ref() is None


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
