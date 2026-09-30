# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

"""Check the Python interfaces of module-owned output-message collections.

The module remains alive while its borrowed messages, readers, and recorders are
used, as in a normal scheduled simulation. Native tests separately check deletion
and allocation-failure cleanup; these tests check SWIG access and stable storage.
"""

import gc
import importlib

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.utilities import simHelpers


# Each message type must exercise data that differs from freshly initialized storage.
NONZERO_PAYLOAD_FIELDS = {
    "AccessMsg": ("slantRange", 125.5),  # [m]
    "AlbedoMsg": ("albedoAtInstrument", 0.375),  # [-]
    "AtmoPropsMsg": ("localTemp", 273.5),  # [K]
    "AttRefMsg": ("omega_RN_N", [0.125, -0.25, 0.375]),  # [rad/s]
    "CameraImageMsg": ("cameraID", 17),
    "ChargeMsmMsg": ("q", [[1.25e-9], [-2.5e-9]]),  # [C], Eigen column vector
    "CmdForceInertialMsg": ("forceRequestInertial", [1.25, -2.5, 3.75]),  # [N]
    "CmdTorqueBodyMsg": ("torqueRequestBody", [0.125, -0.25, 0.375]),  # [N*m]
    "DataNodeUsageMsg": ("baudRate", 125.5),  # [baud]
    "EclipseMsg": ("illuminationFactor", 0.375),  # [-]
    "EphemerisMsg": ("r_BdyZero_N", [1.25, -2.5, 3.75]),  # [m]
    "FacetElementBodyMsg": ("area", 1.25),  # [m^2]
    "GroundStateMsg": ("r_LN_N", [1.25, -2.5, 3.75]),  # [m]
    "HingedRigidBodyMsg": ("theta", 0.125),  # [rad]
    "LandmarkMsg": ("pL", [17, 29]),  # [pixel]
    "LinearTranslationRigidBodyMsg": ("rho", 1.25),  # [m]
    "MagneticFieldMsg": ("magField_N", [1.25e-5, -2.5e-5, 3.75e-5]),  # [T]
    "ProjectedAreaMsg": ("area", 1.25),  # [m^2]
    "RWConfigLogMsg": ("Omega", 12.5),  # [rad/s]
    "SCStatesMsg": ("r_BN_N", [1.25, -2.5, 3.75]),  # [m]
    "ScChargingCurrentsMsg": ("electronCurrent", -1.25e-6),  # [A]
    "SingleActuatorMsg": ("input", 1.25),  # [N or N*m], depending on the actuator
    "SpicePlanetStateMsg": ("PositionVector", [1.25, -2.5, 3.75]),  # [m]
    "SwDataMsg": ("dataValue", 17.5),  # Units depend on the space-weather output.
    "THROutputMsg": ("thrustForce", 1.25),  # [N]
    "TransRefMsg": ("r_RN_N", [1.25, -2.5, 3.75]),  # [m]
    "VSCMGConfigMsg": ("Omega", 12.5),  # [rad/s]
    "VoltMsg": ("voltage", 12.5),  # [V]
    "WindMsg": ("v_air_N", [1.25, -2.5, 3.75]),  # [m/s]
}


def _check_payload(reader, payload):
    """Compare values across the distinct SWIG payload proxy classes."""
    actual = reader()
    for field in type(payload).__fields__():
        actual_field = field
        if field == "shadowFactor":
            field = "illuminationFactor"
            # Module-specific SWIG payloads may expose only the original field name.
            if hasattr(actual, field):
                actual_field = field
        np.testing.assert_array_equal(getattr(actual, actual_field), getattr(payload, field))


def _write_and_check_output(output, reader, recorder, timestamp, module_id):
    """Verify a nonzero payload through both a subscriber and a recorder."""
    message_name = type(output).__name__
    payload = getattr(messaging, message_name + "Payload")()
    field, value = NONZERO_PAYLOAD_FIELDS[message_name]
    assert not np.array_equal(getattr(payload, field), value)
    setattr(payload, field, value)
    output.write(payload, timestamp, module_id)
    assert reader.isWritten()
    assert reader.timeWritten() == timestamp
    assert reader.moduleID() == module_id
    _check_payload(reader, payload)
    recorder.UpdateState(timestamp)
    assert list(recorder.times()) == [timestamp]
    np.testing.assert_array_equal(getattr(recorder, field)[0], value)


def _check_outputs(model, fields, grow=None):
    """Read and record every output, including proxies obtained before growth."""
    snapshots = []
    for field in fields:
        outputs = getattr(model, field)
        assert len(outputs) > 0
        for index, output in enumerate(outputs):
            assert not output.thisown
            snapshots.append((field, index, output, output.addSubscriber(), output.recorder()))
    if grow is not None:
        grow()
    gc.collect()
    timestamp = 123  # [ns]
    module_id = 17
    for field, index, output, reader, recorder in snapshots:
        assert int(getattr(model, field)[index].this) == int(output.this)
        _write_and_check_output(output, reader, recorder, timestamp, module_id)


ENVIRONMENT_CASES = [
    ("exponentialAtmosphere", "ExponentialAtmosphere", "addSpacecraftToModel", ("envOutMsgs",)),
    ("msisAtmosphere", "MsisAtmosphere", "addSpacecraftToModel", ("envOutMsgs",)),
    ("tabularAtmosphere", "TabularAtmosphere", "addSpacecraftToModel", ("envOutMsgs",)),
    ("magneticFieldCenteredDipole", "MagneticFieldCenteredDipole", "addSpacecraftToModel", ("envOutMsgs",)),
    ("magneticFieldWMM", "MagneticFieldWMM", "addSpacecraftToModel", ("envOutMsgs",)),
    ("zeroWindModel", "ZeroWindModel", "addSpacecraftToModel", ("envOutMsgs",)),
    ("eclipse", "Eclipse", "addSpacecraftToModel", ("eclipseOutMsgs",)),
    ("groundLocation", "GroundLocation", "addSpacecraftToModel", ("accessOutMsgs",)),
    ("spacecraftLocation", "SpacecraftLocation", "addSpacecraftToModel", ("accessOutMsgs",)),
    ("stripLocation", "StripLocation", "addSpacecraftToModel", ("accessOutMsgs",)),
    ("spacecraftChargingEquilibrium", "SpacecraftChargingEquilibrium", "addSpacecraft", ("voltOutMsgs", "currentsOutMsgs")),
]


@pytest.mark.parametrize("module_name,class_name,method,fields", ENVIRONMENT_CASES,
                         ids=[case[0] for case in ENVIRONMENT_CASES])
def test_environment_output_growth(module_name, class_name, method, fields):
    """Adding spacecraft preserves existing output addresses and Python access."""
    module = importlib.import_module("Basilisk.simulation." + module_name)
    model = getattr(module, class_name)()
    source = messaging.SCStatesMsg()

    def grow():
        for _ in range(1 if method == "addSpacecraft" else 16):
            getattr(model, method)(source)

    grow()
    _check_outputs(model, fields, grow)


EFFECTOR_CASES = [
    ("nHingedRigidBodyStateEffector", "NHingedRigidBodyStateEffector", "addHingedPanel", "HingedPanel",
     ("nHingedRigidBodyOutMsgs", "nHingedRigidBodyConfigLogOutMsgs")),
    ("spinningBodyNDOFStateEffector", "SpinningBodyNDOFStateEffector", "addSpinningBody", "SpinningBody",
     ("spinningBodyOutMsgs", "spinningBodyConfigLogOutMsgs")),
    ("linearTranslationNDOFStateEffector", "LinearTranslationNDOFStateEffector", "addTranslatingBody", "TranslatingBody",
     ("translatingBodyOutMsgs", "translatingBodyConfigLogOutMsgs")),
    ("reactionWheelStateEffector", "ReactionWheelStateEffector", "addReactionWheel", "RWConfigPayload", ("rwOutMsgs",)),
    ("vscmgStateEffector", "VSCMGStateEffector", "AddVSCMG", "VSCMGConfigMsgPayload", ("vscmgOutMsgs",)),
]


@pytest.mark.parametrize("module_name,class_name,method,config_name,fields", EFFECTOR_CASES,
                         ids=[case[0] for case in EFFECTOR_CASES])
def test_effector_output_growth(module_name, class_name, method, config_name, fields):
    """Adding effectors preserves state and configuration output-message proxies."""
    module = importlib.import_module("Basilisk.simulation." + module_name)
    model = getattr(module, class_name)()

    def grow():
        for _ in range(8):
            getattr(model, method)(getattr(module, config_name)())

    grow()
    _check_outputs(model, fields, grow)


@pytest.mark.parametrize("module_name", ["thrusterDynamicEffector", "thrusterStateEffector"])
@pytest.mark.parametrize("attached", [False, True])
def test_thruster_output_growth(module_name, attached):
    """Both thruster-addition overloads preserve existing output messages."""
    module = importlib.import_module("Basilisk.simulation." + module_name)
    class_name = module_name[0].upper() + module_name[1:]
    model = getattr(module, class_name)()
    body = messaging.SCStatesMsg()

    def grow():
        for _ in range(8):
            config = module.THRSimConfig()
            if attached:
                model.addThruster(config, body)
            else:
                model.addThruster(config)

    grow()
    _check_outputs(model, ("thrusterOutMsgs",), grow)


FIXED_CASES = [
    ("spaceWeatherData", "SpaceWeatherData", ("swDataOutMsgs",)),
    ("spinningBodyTwoDOFStateEffector", "SpinningBodyTwoDOFStateEffector",
     ("spinningBodyOutMsgs", "spinningBodyConfigLogOutMsgs")),
    ("dualHingedRigidBodyStateEffector", "DualHingedRigidBodyStateEffector",
     ("dualHingedRigidBodyOutMsgs", "dualHingedRigidBodyConfigLogOutMsgs")),
]


@pytest.mark.parametrize("module_name,class_name,fields", FIXED_CASES,
                         ids=[case[0] for case in FIXED_CASES])
def test_fixed_output_collections(module_name, class_name, fields):
    """Constructor-created output collections retain the existing Python API."""
    module = importlib.import_module("Basilisk.simulation." + module_name)
    model = getattr(module, class_name)()
    _check_outputs(model, fields)


CONTROLLER_CASES = [
    ("fswAlgorithms.hingedJointArrayMotor", "HingedJointArrayMotor", "addHingedJoint", "motorTorquesOutMsgs"),
    ("fswAlgorithms.thrJointCompensation", "ThrJointCompensation", "addHingedJoint", "motorTorquesOutMsgs"),
    ("fswAlgorithms.jointMotionCompensator", "JointMotionCompensator", "addSpacecraft", "hubTorqueOutMsgs"),
    ("simulation.thrOnTimeToForce", "ThrOnTimeToForce", "addThruster", "thrusterForceOutMsgs"),
]


@pytest.mark.parametrize("module_name,class_name,method,field", CONTROLLER_CASES,
                         ids=[case[0] for case in CONTROLLER_CASES])
def test_controller_output_growth(module_name, class_name, method, field):
    """Controller output collections remain usable after adding joints or thrusters."""
    module = pytest.importorskip("Basilisk." + module_name)
    model = getattr(module, class_name)()

    def grow():
        for _ in range(8):
            getattr(model, method)()

    grow()
    _check_outputs(model, (field,), grow)


@pytest.mark.parametrize("case", ["albedo", "groundMapping", "ephemerisConverter", "msmForceTorque",
                                 "pinholeCamera", "mappingInstrument", "vizInterface"])
def test_other_output_growth(case):
    """Instrument, mapping, ephemeris, and electrostatic outputs retain stable storage."""
    module = pytest.importorskip("Basilisk.simulation." + case)
    class_name = case[0].upper() + case[1:]
    model = getattr(module, class_name)()
    sources = {
        "state": messaging.SCStatesMsg(), "spice": messaging.SpicePlanetStateMsg(),
        "access": messaging.AccessMsg(), "camera": messaging.CameraConfigMsg(),
    }
    zero_position = [0.0, 0.0, 0.0]  # [m]
    unit_direction = [1.0, 0.0, 0.0]
    field_of_view = 0.5  # [rad]
    sphere_radius = 1.0  # [m]
    cases = {
        "albedo": (("albOutMsgs",), lambda: model.addInstrumentConfig(field_of_view, unit_direction, zero_position)),
        "groundMapping": (("accessOutMsgs", "currentGroundStateOutMsgs"), lambda: model.addPointToModel(zero_position)),
        "ephemerisConverter": (("ephemOutMsgs",), lambda: model.addSpiceInputMsg(sources["spice"])),
        "msmForceTorque": (("eTorqueOutMsgs", "eForceOutMsgs", "chargeMsmOutMsgs"),
                           lambda: model.addSpacecraftToModel(
                               sources["state"], messaging.DoubleVector([sphere_radius]),
                               simHelpers.npList2EigenXdVector([zero_position]))),
        "pinholeCamera": (("landmarkOutMsgs",), lambda: model.addLandmark(zero_position, unit_direction)),
        "mappingInstrument": (("dataNodeOutMsgs",), lambda: model.addMappingPoint(sources["access"], "test")),
        "vizInterface": (("opnavImageOutMsgs",), lambda: model.addCamMsgToModule(sources["camera"])),
    }
    fields, add = cases[case]

    def grow():
        for _ in range(8):
            add()

    grow()
    _check_outputs(model, fields, grow)


@pytest.mark.parametrize("case", ["spiceInterface", "facetedSpacecraftModel", "facetedSpacecraftProjectedArea"])
def test_reconfigured_outputs(case):
    """Replacing an output collection exposes the newly configured messages."""
    module = importlib.import_module("Basilisk.simulation." + case)
    model = getattr(module, case[0].upper() + case[1:])()
    if case == "spiceInterface":
        for names in (["earth", "moon"], ["sun"]):
            model.addPlanetNames(module.StringVector(names))
            model.addSpacecraftNames(module.StringVector(names))
            fields = ("planetStateOutMsgs", "scStateOutMsgs", "attRefStateOutMsgs", "transRefStateOutMsgs")
            assert all(len(getattr(model, field)) == len(names) for field in fields)
            _check_outputs(model, fields)
    else:
        method, field = {
            "facetedSpacecraftModel": ("setNumTotalFacets", "facetElementBodyOutMsgs"),
            "facetedSpacecraftProjectedArea": ("setNumFacets", "facetProjectedAreaOutMsgs"),
        }[case]
        for count in (3, 1, 0, 4):
            getattr(model, method)(count)
            assert len(getattr(model, field)) == count
            if count:
                _check_outputs(model, (field,))


def test_planet_ephemeris_outputs():
    """Analytical ephemeris messages retain indexing, subscriptions, and recording."""
    from Basilisk.simulation import planetEphemeris

    model = planetEphemeris.PlanetEphemeris()
    model.setPlanetNames(planetEphemeris.StringVector(["earth", "moon"]))
    _check_outputs(model, ("planetOutMsgs",))


def test_nested_visualization_outputs():
    """Nested wheel and thruster collections keep the existing two-index Python API."""
    module = pytest.importorskip("Basilisk.simulation.dataFileToViz")
    model = module.DataFileToViz()
    model.setNumOfSatellites(2)
    model.appendThrClusterMap(module.VizThrConfig([module.ThrClusterMap()]), module.IntVector([3]))
    model.appendThrClusterMap(module.VizThrConfig([module.ThrClusterMap()]), module.IntVector([2]))
    model.appendNumOfRWs(4)
    model.appendNumOfRWs(2)
    _check_outputs(model, ("scStateOutMsgs",))
    for field, counts in (("thrScOutMsgs", (3, 2)), ("rwScOutMsgs", (4, 2))):
        for spacecraft, count in enumerate(counts):
            outputs = getattr(model, field)[spacecraft]
            assert len(outputs) == count
            for output in outputs:
                assert not output.thisown
                reader = output.addSubscriber()
                recorder = output.recorder()
                timestamp = 456  # [ns]
                _write_and_check_output(output, reader, recorder, timestamp, 0)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
