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

"""Verify safe external-load defaults and command sampling through both attachment paths."""

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.simulation import extForceTorque, spacecraft, spinningBodyOneDOFStateEffector
from Basilisk.utilities import SimulationBaseClass, macros


STATIC_FORCE_N = np.array([0.3, -0.2, 0.1])  # [N]
STATIC_FORCE_B = np.array([0.1, 0.2, -0.1])  # [N]
STATIC_TORQUE_B = np.array([0.01, -0.02, 0.03])  # [N*m]
COMMAND_FORCE_N = np.array([1.0, -2.0, 3.0])  # [N]
COMMAND_FORCE_B = np.array([-4.0, 5.0, -6.0])  # [N]
COMMAND_TORQUE_B = np.array([0.2, -0.3, 0.4])  # [N*m]


def make_effector(written=True):
    """Configure static loads and connect all command inputs, optionally writing nonzero commands."""
    effector = extForceTorque.ExtForceTorque()
    effector.extForce_N = STATIC_FORCE_N
    effector.extForce_B = STATIC_FORCE_B
    effector.extTorquePntB_B = STATIC_TORQUE_B
    messages = (
        messaging.CmdForceInertialMsg(), messaging.CmdForceBodyMsg(), messaging.CmdTorqueBodyMsg()
    )
    effector.cmdForceInertialInMsg.subscribeTo(messages[0])
    effector.cmdForceBodyInMsg.subscribeTo(messages[1])
    effector.cmdTorqueInMsg.subscribeTo(messages[2])
    if written:
        write_commands(messages, 1.0)
    return effector, messages


def write_commands(messages, scale):
    """Write scaled force and torque commands without sampling them in the effector."""
    force_n = messaging.CmdForceInertialMsgPayload()
    force_n.forceRequestInertial = COMMAND_FORCE_N * scale  # [N]
    force_b = messaging.CmdForceBodyMsgPayload()
    force_b.forceRequestBody = COMMAND_FORCE_B * scale  # [N]
    torque_b = messaging.CmdTorqueBodyMsgPayload()
    torque_b.torqueRequestBody = COMMAND_TORQUE_B * scale  # [N*m]
    for message, payload in zip(messages, (force_n, force_b, torque_b)):
        message.write(payload)


def check_loads(effector, command_scale):
    """Compare reported loads with the independent sum of static and sampled commands."""
    for actual, configured, commanded in (
        (effector.forceExternal_N, STATIC_FORCE_N, COMMAND_FORCE_N),
        (effector.forceExternal_B, STATIC_FORCE_B, COMMAND_FORCE_B),
        (effector.torqueExternalPntB_B, STATIC_TORQUE_B, COMMAND_TORQUE_B),
    ):
        np.testing.assert_allclose(np.asarray(actual).ravel(), configured + command_scale * commanded, atol=1e-15)


def attach(effector, branch, priority=None):
    """Attach to the hub or an appendage, retaining ownership of the native parent objects."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    process.addTask(sim.CreateNewTask("task", macros.sec2nano(0.01)))  # [ns]
    parent = spacecraft.Spacecraft()
    parent.hub.mHub = 100.0  # [kg]
    parent.hub.IHubPntBc_B = np.eye(3) * 10.0  # [kg*m^2]
    if branch:
        appendage = spinningBodyOneDOFStateEffector.SpinningBodyOneDOFStateEffector()
        appendage.mass = 1.0  # [kg]
        appendage.IPntSc_S = np.eye(3)  # [kg*m^2]
        appendage.sHat_S = [0.0, 0.0, 1.0]  # [-]
        appendage.dcm_S0B = [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]  # [-]
        appendage.addDynamicEffector(effector)
        parent.addStateEffector(appendage)
        sim._ext_force_appendage = appendage
    else:
        parent.addDynamicEffector(effector)
    sim.AddModelToTask("task", parent)
    if priority is not None:
        sim.AddModelToTask("task", effector, priority)
    return sim, parent


@pytest.mark.parametrize("branch", [False, True], ids=["hub", "appendage"])
@pytest.mark.parametrize("priority", [None, -100, 100], ids=["unscheduled", "after_parent", "before_parent"])
@pytest.mark.parametrize("written", [False, True], ids=["unwritten", "written"])
def test_command_defaults_and_scheduling(branch, priority, written):
    """Apply only static loads until commands are sampled, regardless of attachment or Reset order."""
    effector, messages = make_effector(written)
    # Check construction before either attachment or a scheduled Reset can clear the buffers.
    effector.computeForceTorque(0.0, 0.0)  # [s]
    check_loads(effector, 0.0)
    sim, parent = attach(effector, branch, priority)
    sim.InitializeSimulation()
    check_loads(effector, 0.0)
    sim.ConfigureStopTime(macros.sec2nano(0.02))  # [ns]
    sim.ExecuteSimulation()
    check_loads(effector, 1.0 if written and priority is not None else 0.0)
    assert np.isfinite(parent.scStateOutMsg.read().r_BN_N).all()


@pytest.mark.parametrize("branch", [False, True], ids=["hub", "appendage"])
def test_cached_commands_and_repeated_reset(branch):
    """Preserve sampled commands through attachment, hold them between reads, and clear them on Reset."""
    effector, messages = make_effector()
    effector.UpdateState(0)
    sim, parent = attach(effector, branch)
    sim.InitializeSimulation()
    check_loads(effector, 1.0)

    write_commands(messages, 2.0)
    for _ in range(2):
        effector.computeForceTorque(0.0, 0.0)  # [s]
        check_loads(effector, 1.0)
    effector.UpdateState(0)
    effector.computeForceTorque(0.0, 0.0)  # [s]
    check_loads(effector, 2.0)

    position = parent.dynManager.getStateObject(parent.hub.nameOfHubPosition)
    position.setState([[1.0], [2.0], [3.0]])  # [m]
    before = np.asarray(position.getState()).copy()
    for _ in range(2):
        effector.Reset(0)
        effector.computeForceTorque(0.0, 0.0)  # [s]
        check_loads(effector, 0.0)
        np.testing.assert_array_equal(position.getState(), before)
    effector.UpdateState(0)
    effector.computeForceTorque(0.0, 0.0)  # [s]
    check_loads(effector, 2.0)
