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

"""Check effector dimensions before attachment, reset, and runtime native accesses."""

import numpy as np
import pytest

from Basilisk.architecture import astroConstants, messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import (
    MtbEffector,
    facetedSRPEffector,
    radiationPressure,
    reactionWheelStateEffector,
    spacecraft,
    spinningBodyOneDOFStateEffector,
    thrusterDynamicEffector,
    thrusterStateEffector,
)
from Basilisk.utilities import SimulationBaseClass, macros


CAPACITY = len(messaging.THRArrayOnTimeCmdMsgPayload().OnTimeRequest)
THRUSTERS = [thrusterStateEffector, thrusterDynamicEffector]


def attach(effector, state=False, branch=False):
    """Create an attached-only effector with a valid parent and optional appendage."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    process.addTask(sim.CreateNewTask("task", macros.sec2nano(0.01)))  # [ns]
    parent = spacecraft.Spacecraft()
    parent.hub.mHub = 100.0  # [kg]
    parent.hub.IHubPntBc_B = np.eye(3) * 10.0  # [kg*m^2]
    if state:
        parent.addStateEffector(effector)
    elif branch:
        appendage = spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector()
        appendage.mass = 1.0  # [kg]
        appendage.IPntSc_S = np.eye(3)  # [kg*m^2]
        appendage.sHat_S = [1.0, 0.0, 0.0]  # [-]
        appendage.dcm_S0B = np.eye(3)  # [-]
        appendage.addDynamicEffector(effector)
        parent.addStateEffector(appendage)
        # Spacecraft attachment stores a native pointer; retain its Python owner.
        sim._dimension_appendage = appendage
    else:
        parent.addDynamicEffector(effector)
    sim.AddModelToTask("task", parent)
    return sim, parent


def sun_message():
    """Place the Sun one astronomical unit from the stationary spacecraft."""
    payload = messaging.SpicePlanetStateMsgPayload()
    payload.PositionVector = [astroConstants.AU * 1000.0, 0.0, 0.0]  # [m]
    return messaging.SpicePlanetStateMsg().write(payload)


def mtb(count):
    """Create connected torque-bar inputs, with the last bar commanded when present."""
    effector = MtbEffector.MtbEffector()
    config = messaging.MTBArrayConfigMsgPayload()
    config.numMTB = count
    command = messaging.MTBCmdMsgPayload()
    if 0 < count <= CAPACITY:
        config.GtMatrix_B = [1.0] * count + [0.0] * (2 * count)  # [-]
        config.maxMtbDipoles = [2.0] * count  # [A*m^2]
        command.mtbDipoleCmds = [0.0] * (count - 1) + [1.0]  # [A*m^2]
    field = messaging.MagneticFieldMsgPayload()
    field.magField_N = [0.0, 0.0, 1e-5]  # [T]
    config_message = messaging.MTBArrayConfigMsg().write(config)
    effector.mtbParamsInMsg.subscribeTo(config_message)
    effector.mtbCmdInMsg.subscribeTo(messaging.MTBCmdMsg().write(command))
    effector.magInMsg.subscribeTo(messaging.MagneticFieldMsg().write(field))
    return effector, config_message


@pytest.mark.parametrize("path", ["reset", "attachment", "runtime"])
@pytest.mark.parametrize("count", [-1, CAPACITY + 1])
def test_mtb_invalid_count(path, count):
    """Reject negative and overflowing counts before transposing or mapping the payload."""
    effector, config_message = mtb(0 if path == "runtime" else count)
    sim, parent = attach(effector)
    if path == "runtime":
        sim.InitializeSimulation()
        payload = messaging.MTBArrayConfigMsgPayload()
        payload.numMTB = count
        config_message.write(payload)
    with pytest.raises(BasiliskError, match="numMTB"):
        if path == "reset":
            effector.Reset(0)
        elif path == "attachment":
            sim.InitializeSimulation()
        else:
            effector.computeForceTorque(0.0, 0.0)  # [s]


@pytest.mark.parametrize("count", [0, CAPACITY])
def test_mtb_count_boundaries(count):
    """Accept an empty array and include the final torque bar at payload capacity."""
    effector, config_message = mtb(count)
    sim, parent = attach(effector)
    sim.InitializeSimulation()
    expected = [0.0, -1e-5 if count else 0.0, 0.0]  # [N*m]
    np.testing.assert_allclose(np.asarray(effector.torqueExternalPntB_B).ravel(), expected, atol=1e-15)


def thrusters(module, count):
    """Create independent thrusters with finite parameters and zero initial firing states."""
    cls = module.ThrusterStateEffector if module is thrusterStateEffector else module.ThrusterDynamicEffector
    effector = cls()
    for _ in range(count):
        config = module.THRSimConfig()
        config.thrDir_B = [1.0, 0.0, 0.0]  # [-]
        config.MaxThrust = 1.0  # [N]
        config.steadyIsp = 200.0  # [s]
        config.cutoffFrequency = 1.0  # [rad/s]
        effector.addThruster(config)
    return effector


@pytest.mark.parametrize("path", ["reset", "attachment"])
@pytest.mark.parametrize("initial", [[], [0.0, 0.0], [np.nan], [np.inf], [-np.inf], [-0.1], [1.1]])
def test_thruster_initial_state_validation(path, initial):
    """Require exactly one finite thrust factor in [0, 1] per thruster before state registration."""
    effector = thrusters(thrusterStateEffector, 1)
    effector.kappaInit = messaging.DoubleVector(initial)  # [-]
    sim, parent = attach(effector, state=True)
    with pytest.raises(BasiliskError, match="kappaInit"):
        if path == "reset":
            effector.Reset(0)
        else:
            sim.InitializeSimulation()


def test_thruster_initial_state_boundaries_and_reset():
    """Accept both thrust-factor endpoints and preserve integrated states across repeated Reset calls."""
    effector = thrusters(thrusterStateEffector, 2)
    effector.kappaInit = messaging.DoubleVector([0.0, 1.0])  # [-]
    sim, parent = attach(effector, state=True)
    sim.InitializeSimulation()
    state = parent.dynManager.getStateObject(effector.nameOfKappaState)
    np.testing.assert_array_equal(np.asarray(state.getState()).ravel(), [0.0, 1.0])
    state.setState([[0.25], [0.75]])  # [-]
    effector.Reset(0)
    effector.Reset(0)
    np.testing.assert_array_equal(np.asarray(state.getState()).ravel(), [0.25, 0.75])
    assert list(effector.NewThrustCmds) == [0.0, 0.0]


@pytest.mark.parametrize("module", THRUSTERS)
@pytest.mark.parametrize("path", ["reset", "attachment", "runtime"])
def test_thruster_command_capacity(module, path):
    """Reject excessive thruster counts whenever a fixed-size command input is connected."""
    effector = thrusters(module, CAPACITY + 1)
    command = messaging.THRArrayOnTimeCmdMsg().write(messaging.THRArrayOnTimeCmdMsgPayload())
    sim, parent = attach(effector, state=module is thrusterStateEffector)
    if path == "runtime":
        sim.InitializeSimulation()
    effector.cmdsInMsg.subscribeTo(command)
    with pytest.raises(BasiliskError, match="MAX_EFF_CNT"):
        if path == "reset":
            effector.Reset(0)
        elif path == "attachment":
            sim.InitializeSimulation()
        else:
            effector.ReadInputs()


def test_branch_thruster_command_capacity():
    """The dynamic thruster property-linking path must enforce the command capacity too."""
    effector = thrusters(thrusterDynamicEffector, CAPACITY + 1)
    effector.cmdsInMsg.subscribeTo(messaging.THRArrayOnTimeCmdMsg())
    sim, parent = attach(effector, branch=True)
    with pytest.raises(BasiliskError, match="MAX_EFF_CNT"):
        sim.InitializeSimulation()


@pytest.mark.parametrize("module", THRUSTERS)
@pytest.mark.parametrize("linked", [False, True])
def test_thruster_command_boundaries(module, linked):
    """Consume the last command safely and allow larger unlinked sets with zero commands."""
    count = CAPACITY if linked else CAPACITY + 1
    effector = thrusters(module, count)
    if linked:
        payload = messaging.THRArrayOnTimeCmdMsgPayload()
        payload.OnTimeRequest = [0.0] * (count - 1) + [0.2]  # [s]
        effector.cmdsInMsg.subscribeTo(messaging.THRArrayOnTimeCmdMsg().write(payload))
    sim, parent = attach(effector, state=module is thrusterStateEffector)
    sim.InitializeSimulation()
    # Reading without a scheduled Reset must size the command buffer safely.
    effector.NewThrustCmds.clear()
    assert effector.ReadInputs()
    expected = [0.0] * count  # [s]
    if linked:
        expected[-1] = 0.2  # [s]
    np.testing.assert_array_equal(list(effector.NewThrustCmds), expected)
    effector.Reset(0)
    assert list(effector.NewThrustCmds) == [0.0] * count


@pytest.mark.parametrize("module", THRUSTERS)
@pytest.mark.parametrize("count", [0, 1, 2])
def test_thruster_direct_command_dimensions(module, count):
    """Allow partial manual commands but reject vectors that exceed the thruster count."""
    effector = thrusters(module, 1)
    effector.NewThrustCmds = messaging.DoubleVector([0.0] * count)  # [s]
    def configure():
        """Apply the manually supplied commands through the appropriate module interface."""
        if module is thrusterStateEffector:
            effector.ConfigureThrustRequests()
        else:
            effector.ConfigureThrustRequests(0.0)  # [s]
    if count > 1:
        with pytest.raises(BasiliskError, match="NewThrustCmds"):
            configure()
    else:
        configure()


def wheels(count):
    """Create balanced wheels with a nonzero spin inertia and distinct speeds."""
    effector = reactionWheelStateEffector.ReactionWheelStateEffector()
    for index in range(count):
        config = reactionWheelStateEffector.RWConfigPayload()
        config.RWModel = reactionWheelStateEffector.BalancedWheels
        config.gsHat_B = [1.0, 0.0, 0.0]  # [-]
        config.Js = 0.1  # [kg*m^2]
        config.Omega = (index + 1) * 0.01  # [rad/s]
        config.betaStatic = -1.0  # [-]
        effector.addReactionWheel(config)
    return effector


@pytest.mark.parametrize("path", ["reset", "attachment", "input", "output"])
def test_wheel_capacity(path):
    """Guard both fixed-size wheel payloads even when the command input is unlinked."""
    capacity = len(messaging.RWSpeedMsgPayload().wheelSpeeds)
    effector = wheels(capacity + 1)
    sim, parent = attach(effector, state=True)
    with pytest.raises(BasiliskError, match="MAX_EFF_CNT"):
        if path == "reset":
            effector.Reset(0)
        elif path == "attachment":
            sim.InitializeSimulation()
        elif path == "input":
            effector.ReadInputs()
        else:
            effector.writeOutputStateMessages(0)


def test_wheel_capacity_boundary():
    """Publish the final wheel speed and read its torque without a scheduled Reset."""
    count = len(messaging.RWSpeedMsgPayload().wheelSpeeds)
    effector = wheels(count)
    payload = messaging.ArrayMotorTorqueMsgPayload()
    payload.motorTorque = [0.0] * (count - 1) + [0.2]  # [N*m]
    effector.rwMotorCmdInMsg.subscribeTo(messaging.ArrayMotorTorqueMsg().write(payload))
    sim, parent = attach(effector, state=True)
    sim.InitializeSimulation()
    effector.writeOutputStateMessages(0)
    assert effector.rwSpeedOutMsg.read().wheelSpeeds[-1] == pytest.approx(count * 0.01)  # [rad/s]
    effector.NewRWCmds.clear()
    effector.ReadInputs()
    assert len(effector.NewRWCmds) == count
    assert effector.NewRWCmds[count - 1].u_cmd == pytest.approx(0.2)  # [N*m]


@pytest.mark.parametrize("count", [0, 1, 2])
def test_wheel_direct_command_dimensions(count):
    """Allow partial manual commands but reject vectors that exceed the wheel count."""
    effector = wheels(1)
    effector.NewRWCmds = messaging.RWCmdMsgPayloadVector(count)
    if count > 1:
        with pytest.raises(BasiliskError, match="NewRWCmds"):
            effector.ConfigureRWRequests(0.0)  # [s]
    else:
        effector.ConfigureRWRequests(0.0)  # [s]


def state_devices(kind, count):
    """Return a state effector and the names of its device vector and integrated state."""
    if kind == "thruster":
        effector = thrusters(thrusterStateEffector, count)
        return effector, "thrusterData", effector.nameOfKappaState
    effector = wheels(count)
    return effector, "ReactionWheelData", effector.nameOfReactionWheelOmegasState


@pytest.mark.parametrize("kind", ["thruster", "body_thruster", "wheel"])
@pytest.mark.parametrize("initial_count", [0, 1])
def test_registered_device_addition(kind, initial_count):
    """Reject both thruster overloads and wheel additions without altering a runnable simulation."""
    device_kind = "wheel" if kind == "wheel" else "thruster"
    effector, vector_name, state_name = state_devices(device_kind, initial_count)
    donor, _, _ = state_devices(device_kind, 1)
    config = getattr(donor, vector_name)[0]
    sim, parent = attach(effector, state=True)
    sim.InitializeSimulation()
    state = parent.dynManager.getStateObject(state_name)
    before = np.asarray(state.getState()).copy()
    with pytest.raises(BasiliskError, match="cannot add .* after state registration"):
        if kind == "wheel":
            effector.addReactionWheel(config)
        elif kind == "body_thruster":
            effector.addThruster(config, parent.scStateOutMsg)
        else:
            effector.addThruster(config)
    assert len(getattr(effector, vector_name)) == initial_count
    if device_kind == "thruster":
        assert len(effector.kappaInit) == len(effector.thrusterOutMsgs) == initial_count
    else:
        assert len(effector.rwOutMsgs) == initial_count
    np.testing.assert_array_equal(state.getState(), before)
    sim.ConfigureStopTime(macros.sec2nano(0.02))  # [ns]
    sim.ExecuteSimulation()


@pytest.mark.parametrize("kind", ["thruster", "wheel"])
@pytest.mark.parametrize("change", ["append", "remove"])
@pytest.mark.parametrize("path", ["reset", "input", "output", "dynamics"])
def test_registered_device_count_mutation(kind, change, path):
    """Catch public-vector changes before Reset, messages, or an attached-only dynamics step."""
    effector, vector_name, state_name = state_devices(kind, 1 if change == "append" else 2)
    sim, parent = attach(effector, state=True)
    sim.InitializeSimulation()
    state = parent.dynManager.getStateObject(state_name)
    before = np.asarray(state.getState()).copy()
    devices = getattr(effector, vector_name)
    if change == "append":
        devices.push_back(devices[0])
        if kind == "thruster":
            effector.kappaInit.push_back(0.0)
    else:
        devices.pop_back()
        if kind == "thruster":
            effector.kappaInit.pop_back()
    with pytest.raises(BasiliskError, match="count cannot change after state registration"):
        if path == "reset":
            effector.Reset(0)
        elif path == "input":
            effector.ReadInputs()
        elif path == "output":
            effector.writeOutputStateMessages(0)
        else:
            sim.ConfigureStopTime(macros.sec2nano(0.02))  # [ns]
            sim.ExecuteSimulation()
    np.testing.assert_array_equal(state.getState(), before)


@pytest.mark.parametrize("change", ["add_angle", "remove_angle", "swap_angles"])
@pytest.mark.parametrize("path", ["reset", "dynamics", "log_output"])
def test_registered_wheel_angle_layout(change, path):
    """Preserve each wheel's angle-state allocation, including swaps with unchanged total size."""
    effector = wheels(2)
    if change != "add_angle":
        effector.ReactionWheelData[0].RWModel = reactionWheelStateEffector.JitterSimple
    sim, parent = attach(effector, state=True)
    sim.InitializeSimulation()
    effector.ReactionWheelData[0].RWModel = (
        reactionWheelStateEffector.JitterSimple if change == "add_angle"
        else reactionWheelStateEffector.BalancedWheels
    )
    if change == "swap_angles":
        effector.ReactionWheelData[1].RWModel = reactionWheelStateEffector.JitterSimple
    with pytest.raises(BasiliskError, match="jitter-state allocation cannot change"):
        if path == "reset":
            effector.Reset(0)
        elif path == "log_output":
            effector.WriteOutputMessages(0)
        else:
            sim.ConfigureStopTime(macros.sec2nano(0.02))  # [ns]
            sim.ExecuteSimulation()


@pytest.mark.parametrize("counter", ["numRW", "numRWJitter"])
def test_registered_wheel_counter_mutation(counter):
    """Reject edits to public derived counts before allocating or reading derivative states."""
    effector = wheels(1)
    sim, parent = attach(effector, state=True)
    sim.InitializeSimulation()
    setattr(effector, counter, getattr(effector, counter) + 1)
    sim.ConfigureStopTime(macros.sec2nano(0.02))  # [ns]
    with pytest.raises(BasiliskError, match="must match the registered state layout"):
        sim.ExecuteSimulation()


@pytest.mark.parametrize("kind", ["thruster", "wheel"])
@pytest.mark.parametrize("priority", [None, -100, 100])
def test_registered_layout_reset_lifecycle(kind, priority):
    """Allow setup after an early Reset and preserve states regardless of scheduled Reset order."""
    effector, vector_name, state_name = state_devices(kind, 1)
    donor, _, _ = state_devices(kind, 1)
    effector.Reset(0)
    config = getattr(donor, vector_name)[0]
    if kind == "thruster":
        effector.addThruster(config)
    else:
        effector.addReactionWheel(config)
    sim, parent = attach(effector, state=True)
    if priority is not None:
        sim.AddModelToTask("task", effector, priority)
    sim.InitializeSimulation()
    state = parent.dynManager.getStateObject(state_name)
    # Thrust factors are dimensionless; wheel speeds are in rad/s.
    values = [[0.25], [0.75]]  # [-] or [rad/s], according to kind
    state.setState(values)
    if kind == "thruster":
        effector.thrusterData[0].MaxThrust = 2.0  # [N]
    else:
        effector.ReactionWheelData[0].u_max = 0.5  # [N*m]
    effector.Reset(0)
    effector.Reset(0)
    np.testing.assert_array_equal(state.getState(), values)
    sim.ConfigureStopTime(macros.sec2nano(0.02))  # [ns]
    sim.ExecuteSimulation()
    assert np.isfinite(state.getState()).all()


def facets(count=1):
    """Create connected SRP facets with unit area facing the Sun."""
    effector = facetedSRPEffector.FacetedSRPEffector()
    effector.setNumFacets(count)
    effector.sunStateInMsg.subscribeTo(sun_message())
    geometry = messaging.FacetElementBodyMsgPayload()
    geometry.area = 1.0  # [m^2]
    geometry.nHat_B = [1.0, 0.0, 0.0]  # [-]
    area = messaging.ProjectedAreaMsgPayload()
    area.area = 1.0  # [m^2]
    for index in range(count):
        effector.facetElementBodyInMsgs[index].subscribeTo(messaging.FacetElementBodyMsg().write(geometry))
        effector.facetProjectedAreaInMsgs[index].subscribeTo(messaging.ProjectedAreaMsg().write(area))
    return effector


@pytest.mark.parametrize("path", ["reset", "attachment", "runtime"])
@pytest.mark.parametrize("sizes", [(0, 0), (0, 1), (1, 0), (2, 1), (1, 2), (2, 2)])
def test_facet_message_dimensions(path, sizes):
    """Validate both public vectors before either validation or force evaluation indexes them."""
    effector = facets()
    sim, parent = attach(effector)
    if path == "runtime":
        sim.InitializeSimulation()
    effector.facetElementBodyInMsgs.resize(sizes[0])
    effector.facetProjectedAreaInMsgs.resize(sizes[1])
    with pytest.raises(BasiliskError, match="numFacets"):
        if path == "reset":
            effector.Reset(0)
        elif path == "attachment":
            sim.InitializeSimulation()
        else:
            effector.computeForceTorque(0.0, 0.0)  # [s]


@pytest.mark.parametrize("count", [0, 1])
def test_facet_count_boundaries(count):
    """Allow an empty facet set and retain the expected force for one absorbing facet."""
    effector = facets(count)
    sim, parent = attach(effector)
    sim.InitializeSimulation()
    expected = [-count * astroConstants.SOLAR_FLUX_EARTH / astroConstants.SPEED_LIGHT, 0.0, 0.0]  # [N]
    np.testing.assert_allclose(np.asarray(effector.forceExternal_B).ravel(), expected, atol=1e-15)


def lookup(sizes, faceted=True):
    """Create lookup tables with independently specified direction, force, and torque lengths."""
    effector = radiationPressure.RadiationPressure()
    effector.sunEphmInMsg.subscribeTo(sun_message())
    if faceted:
        effector.setUseFacetedCPUModel()
    for _ in range(sizes[0]):
        effector.addSHatLookupBEntry([1.0, 0.0, 0.0])  # [-]
    for _ in range(sizes[1]):
        effector.addForceLookupBEntry([-1.0, 0.0, 0.0])  # [N]
    for _ in range(sizes[2]):
        effector.addTorqueLookupBEntry([0.0, 0.0, 0.2])  # [N*m]
    return effector


@pytest.mark.parametrize("path", ["reset", "attachment"])
@pytest.mark.parametrize("sizes", [(0, 0, 0), (0, 1, 1), (1, 0, 1), (1, 1, 0), (2, 1, 1), (1, 2, 1), (1, 1, 2)])
def test_lookup_dimensions(path, sizes):
    """Reject empty or mismatched selected lookup tables at initialization and Reset."""
    effector = lookup(sizes)
    sim, parent = attach(effector)
    with pytest.raises(BasiliskError, match="nonzero number of entries"):
        if path == "reset":
            effector.Reset(0)
        else:
            sim.InitializeSimulation()


@pytest.mark.parametrize("append", ["addSHatLookupBEntry", "addForceLookupBEntry", "addTorqueLookupBEntry"])
def test_lookup_runtime_dimensions(append):
    """Appending to any single table after initialization must fail before lookup access."""
    effector = lookup((1, 1, 1))
    sim, parent = attach(effector)
    sim.InitializeSimulation()
    getattr(effector, append)([0.0, 0.0, 0.0])
    with pytest.raises(BasiliskError, match="nonzero number of entries"):
        effector.computeForceTorque(0.0, 0.0)  # [s]


def test_lookup_valid_and_cannonball_unused_tables():
    """Use a single valid lookup row, and check cannonball mode ignores unused table dimensions."""
    effector = lookup((1, 1, 1))
    effector.readInputMessages()
    sim, parent = attach(effector)
    sim.InitializeSimulation()
    np.testing.assert_allclose(np.asarray(effector.forceExternal_B).ravel(), [-1.0, 0.0, 0.0])
    np.testing.assert_allclose(np.asarray(effector.torqueExternalPntB_B).ravel(), [0.0, 0.0, 0.2])
    effector = lookup((0, 1, 0), faceted=False)
    sim, parent = attach(effector)
    sim.InitializeSimulation()
    effector.Reset(0)
    # Switching models after initialization must validate before lookup access.
    effector.setUseFacetedCPUModel()
    with pytest.raises(BasiliskError, match="nonzero number of entries"):
        effector.computeForceTorque(0.0, 0.0)  # [s]
