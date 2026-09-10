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
#   Unit Test Script
#   Module Name:        spinningBodyNDOFStateEffector
#   Author:             robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date:      June 24, 2026
#

"""
Regression tests for issues #1534 and #571: state-effector configuration validation.

When its spacecraft registers states, ``SpinningBodyNDOFStateEffector``
validates that each attached body's ``dcm_S0P`` is a proper rotation and each
``ISPntSc_S`` is a symmetric, positive-definite inertia tensor. These tests
break exactly one precondition at a time and assert initialization raises a
``BasiliskError``. (``sHat_S`` is validated by its own setter.)

Command-array tests exercise the payload capacity during reset, spacecraft
initialization, and input processing, including the last valid array entry.
"""

import pytest

from Basilisk.architecture import messaging
from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.simulation import spacecraft, spinningBodyNDOFStateEffector
from Basilisk.utilities import SimulationBaseClass, macros


def _validBody():
    """Build a single SpinningBody with a fully valid configuration."""
    body = spinningBodyNDOFStateEffector.SpinningBody()
    body.setMass(50.0)  # [kg]
    body.setISPntSc_S([[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]])  # [kg-m^2]
    body.setDCM_S0P([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])  # [-] proper rotation
    body.setSHat_S([[0.0], [0.0], [1.0]])  # [-] unit vector
    return body


# Each case breaks exactly one precondition on the attached body.
RESET_ERROR_CASES = [
    ("dcm_S0P not orthogonal", "dcm_S0P",
     lambda b: b.setDCM_S0P([[1.0, 0.5, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])),
    ("dcm_S0P left-handed", "dcm_S0P",
     lambda b: b.setDCM_S0P([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])),
    ("ISPntSc_S not symmetric", "ISPntSc_S",
     lambda b: b.setISPntSc_S([[100.0, 5.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]])),
    ("ISPntSc_S not positive definite", "ISPntSc_S",
     lambda b: b.setISPntSc_S([[-100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]])),
]


@pytest.mark.parametrize("brokenPrecondition, expectedMessage, breakIt", RESET_ERROR_CASES,
                         ids=[c[0] for c in RESET_ERROR_CASES])
@pytest.mark.parametrize("validationPath", ["attachment", "reset"])
def test_spinningBodyNDOF_rejectsInvalidConfiguration(brokenPrecondition, expectedMessage, breakIt,
                                                      validationPath):
    """Attachment initialization and direct Reset() must reject the same invalid configuration."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    testProc.addTask(unitTestSim.CreateNewTask("testTask", macros.sec2nano(0.001)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg-m^2]

    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    body = _validBody()
    breakIt(body)
    spinningBodyEffector.addSpinningBody(body)

    if validationPath == "reset":
        with pytest.raises(BasiliskError, match=expectedMessage):
            spinningBodyEffector.Reset(0)
        return

    scObject.addStateEffector(spinningBodyEffector)
    unitTestSim.AddModelToTask("testTask", scObject)

    with pytest.raises(BasiliskError, match=expectedMessage):
        unitTestSim.InitializeSimulation()


def test_spinningBodyNDOF_resetAcceptsValidConfig():
    """A fully valid configuration must initialize without raising."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    testProc.addTask(unitTestSim.CreateNewTask("testTask", macros.sec2nano(0.001)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg-m^2]

    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    spinningBodyEffector.addSpinningBody(_validBody())

    scObject.addStateEffector(spinningBodyEffector)
    unitTestSim.AddModelToTask("testTask", scObject)

    unitTestSim.InitializeSimulation()


def test_spinningBodyNDOF_resetAcceptsMasslessConnector():
    """A massless connector body (mass == 0, zero inertia) followed by a massive body
    must initialize without error. This mirrors the branching modeling pattern."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    testProc.addTask(unitTestSim.CreateNewTask("testTask", macros.sec2nano(0.001)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg-m^2]

    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()

    connector = spinningBodyNDOFStateEffector.SpinningBody()
    connector.setMass(0.0)  # [kg] massless connector
    connector.setISPntSc_S([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])  # [kg-m^2]
    connector.setDCM_S0P([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])  # [-]
    connector.setSHat_S([[1.0], [0.0], [0.0]])  # [-]
    spinningBodyEffector.addSpinningBody(connector)

    spinningBodyEffector.addSpinningBody(_validBody())

    scObject.addStateEffector(spinningBodyEffector)
    unitTestSim.AddModelToTask("testTask", scObject)

    unitTestSim.InitializeSimulation()


@pytest.mark.parametrize("validationPath", ["attachment", "reset"])
def test_spinningBodyNDOF_rejectsEmptyChain(validationPath):
    """Attachment initialization and direct Reset() must reject an empty body chain."""
    unitTestSim = SimulationBaseClass.SimBaseClass()
    testProc = unitTestSim.CreateNewProcess("testProcess")
    timeStep = macros.sec2nano(0.001)  # [ns]
    testProc.addTask(unitTestSim.CreateNewTask("testTask", timeStep))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = 750.0  # [kg]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg-m^2]

    spinningBodyEffector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    if validationPath == "reset":
        with pytest.raises(BasiliskError, match="at least one spinning body"):
            spinningBodyEffector.Reset(0)
        return

    scObject.addStateEffector(spinningBodyEffector)
    unitTestSim.AddModelToTask("testTask", scObject)

    with pytest.raises(BasiliskError, match="at least one spinning body"):
        unitTestSim.InitializeSimulation()


def _command_bounds_simulation(body_count):
    """Attach a valid chain without scheduling the effector's reset or update."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("testProcess")
    time_step = macros.sec2nano(0.01)  # [ns]
    process.addTask(sim.CreateNewTask("testTask", time_step))
    sc = spacecraft.Spacecraft()
    sc.hub.mHub = 750.0  # [kg]
    sc.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]  # [kg*m^2]
    effector = spinningBodyNDOFStateEffector.SpinningBodyNDOFStateEffector()
    for _ in range(body_count):
        effector.addSpinningBody(_validBody())
    sc.addStateEffector(effector)
    sim.AddModelToTask("testTask", sc)
    return sim, effector


def _link_command_inputs(effector, inputs, written):
    """Connect the requested command inputs and retain their message objects."""
    messages = []
    if inputs in ("lock", "both"):
        message = messaging.ArrayEffectorLockMsg()
        if written:
            message.write(messaging.ArrayEffectorLockMsgPayload())
        effector.motorLockInMsg.subscribeTo(message)
        messages.append(message)
    if inputs in ("torque", "both"):
        message = messaging.ArrayMotorTorqueMsg()
        if written:
            message.write(messaging.ArrayMotorTorqueMsgPayload())
        effector.motorTorqueInMsg.subscribeTo(message)
        messages.append(message)
    return messages


@pytest.mark.parametrize("validation_path", ["reset", "attachment"])
@pytest.mark.parametrize("extra_bodies", [0, 1])
@pytest.mark.parametrize("inputs, written", [
    ("none", False), ("lock", False), ("lock", True),
    ("torque", False), ("torque", True), ("both", True),
])
def test_command_array_capacity_validation(validation_path, extra_bodies, inputs, written):
    """Reject oversized chains only when an array input is linked, even if unwritten."""
    sim, effector = _command_bounds_simulation(messaging.MAX_EFF_CNT + extra_bodies)
    messages = _link_command_inputs(effector, inputs, written)
    validate = (lambda: effector.Reset(0)) if validation_path == "reset" else sim.InitializeSimulation
    if extra_bodies and messages:
        with pytest.raises(BasiliskError, match="MAX_EFF_CNT"):
            validate()
    else:
        validate()


@pytest.mark.parametrize("inputs", ["lock", "torque"])
@pytest.mark.parametrize("written", [False, True])
def test_command_array_linked_after_initialization(inputs, written):
    """Reject an oversized array subscription added after configuration validation."""
    sim, effector = _command_bounds_simulation(messaging.MAX_EFF_CNT + 1)
    sim.InitializeSimulation()
    messages = _link_command_inputs(effector, inputs, written)
    with pytest.raises(BasiliskError, match="MAX_EFF_CNT"):
        effector.UpdateState(0)
    assert len(messages) == 1


@pytest.mark.parametrize("lock_last", [False, True])
def test_command_array_last_entry(lock_last):
    """The final valid torque and lock entries control the final body's motion."""
    sim, effector = _command_bounds_simulation(messaging.MAX_EFF_CNT)
    lock_payload = messaging.ArrayEffectorLockMsgPayload()
    lock_payload.effectorLockFlag = [1] * (messaging.MAX_EFF_CNT - 1) + [int(lock_last)]
    lock_message = messaging.ArrayEffectorLockMsg().write(lock_payload)
    effector.motorLockInMsg.subscribeTo(lock_message)
    torque_payload = messaging.ArrayMotorTorqueMsgPayload()
    torque_payload.motorTorque = [0.0] * (messaging.MAX_EFF_CNT - 1) + [0.1]  # [Nm]
    torque_message = messaging.ArrayMotorTorqueMsg().write(torque_payload)
    effector.motorTorqueInMsg.subscribeTo(torque_message)
    sim.AddModelToTask("testTask", effector)
    recorder = effector.spinningBodyOutMsgs[-1].recorder()
    sim.AddModelToTask("testTask", recorder)
    sim.InitializeSimulation()
    stop_time = macros.sec2nano(0.02)  # [ns]
    sim.ConfigureStopTime(stop_time)
    sim.ExecuteSimulation()
    if lock_last:
        assert recorder.theta[-1] == 0.0
        assert recorder.thetaDot[-1] == 0.0
    else:
        assert recorder.theta[-1] > 0.0
        assert recorder.thetaDot[-1] > 0.0


if __name__ == "__main__":
    test_spinningBodyNDOF_resetAcceptsValidConfig()
    test_spinningBodyNDOF_resetAcceptsMasslessConnector()
    test_spinningBodyNDOF_rejectsEmptyChain("attachment")
    for case in RESET_ERROR_CASES:
        test_spinningBodyNDOF_rejectsInvalidConfiguration(*case, "attachment")
