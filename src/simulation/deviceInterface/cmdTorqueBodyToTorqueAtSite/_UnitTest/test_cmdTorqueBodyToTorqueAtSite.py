#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import numpy as np
import pytest

from Basilisk.architecture import bskLogging, messaging
from Basilisk.simulation import cmdTorqueBodyToTorqueAtSite
from Basilisk.utilities import SimulationBaseClass, macros, RigidBodyKinematics as rbk


def runModule(torque_B, dcm_SB = None):
    """Run the module for one second in a simulation and return the module and an output reader."""
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    process.addTask(sim.CreateNewTask("task", macros.sec2nano(0.5)))

    module = cmdTorqueBodyToTorqueAtSite.CmdTorqueBodyToTorqueAtSite()
    module.ModelTag = "torqueBridge"
    if dcm_SB is not None:
        module.dcm_SB = dcm_SB

    cmdPayload = messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=torque_B)
    cmdMsg = messaging.CmdTorqueBodyMsg().write(cmdPayload)
    module.cmdTorqueInMsg.subscribeTo(cmdMsg)
    sim.AddModelToTask("task", module)

    outReader = module.torqueOutMsg.addSubscriber()

    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(1.0))
    sim.ExecuteSimulation()

    return module, outReader


@pytest.mark.parametrize("useRotation", [False, True])
def test_output_torque_and_header(useRotation):
    """
    **Validation Test Description**

    This unit test verifies that :class:`cmdTorqueBodyToTorqueAtSite.CmdTorqueBodyToTorqueAtSite`
    maps a body-frame torque command into the site frame, both for the default
    aligned frames and for a general body-to-site rotation, and that the output
    message header carries the write time and the module ID.

    **Description of Variables Being Tested**

    This unit test checks ``torque_S`` against ``dcm_SB @ torqueRequestBody``,
    the output message write time, and the output message module ID.
    """
    torque_B = [0.4, -1.2, 2.5]  # [N*m]
    dcm_SB = rbk.MRP2C([0.1, -0.3, 0.2]) if useRotation else None

    module, outReader = runModule(torque_B, dcm_SB)

    expected_S = (np.array(dcm_SB) if useRotation else np.eye(3)) @ np.array(torque_B)
    np.testing.assert_allclose(outReader().torque_S, expected_S, rtol = 0, atol = 1e-12)
    assert outReader.timeWritten() == macros.sec2nano(1.0)
    assert outReader.moduleID() == module.moduleID


def test_dcm_setter_copies_input_matrix():
    """Changing the caller's matrix must not alter the rotation or output torque.

    Assign an identity rotation, then invalidate the original NumPy array.
    The module must retain the validated rotation and preserve the commanded
    torque on its next update.
    """
    torque_B = [0.4, -1.2, 2.5]  # [N*m]
    dcm_SB = np.eye(3)
    module, outReader = runModule(torque_B, dcm_SB)

    dcm_SB[0, 0] = 2.0  # [-] Deliberately invalidate the caller's matrix.
    updateTime = macros.sec2nano(2.0)  # [ns]
    module.UpdateState(updateTime)

    np.testing.assert_array_equal(module.dcm_SB, np.eye(3))
    np.testing.assert_allclose(outReader().torque_S, torque_B, rtol = 0, atol = 1e-12)


def test_reset_writes_zero_torque_and_header():
    """Reset must publish zero torque with the reset time and module ID.

    Run the module with a nonzero command, then reset at a later time and
    inspect the output before another update can overwrite its header.
    """
    torque_B = [0.4, -1.2, 2.5]  # [N*m]
    module, outReader = runModule(torque_B)
    resetTime = macros.sec2nano(2.0)  # [ns]

    module.Reset(resetTime)

    np.testing.assert_array_equal(outReader().torque_S, np.zeros(3))
    assert outReader.timeWritten() == resetTime
    assert outReader.moduleID() == module.moduleID


def test_reset_rejects_missing_input_message():
    """
    **Validation Test Description**

    This unit test verifies that :class:`cmdTorqueBodyToTorqueAtSite.CmdTorqueBodyToTorqueAtSite`
    rejects reset calls when the required command torque message is not connected.

    **Description of Variables Being Tested**

    This unit test checks the ``cmdTorqueInMsg`` link validation.
    """
    module = cmdTorqueBodyToTorqueAtSite.CmdTorqueBodyToTorqueAtSite()
    module.bskLogger = bskLogging.BSKLogger()

    with pytest.raises(bskLogging.BasiliskError, match="CmdTorqueBodyToTorqueAtSite.cmdTorqueInMsg"):
        module.Reset(0)


@pytest.mark.parametrize("badDcm", [
    np.eye(2), # wrong shape/size
    2.0 * np.eye(3), # not orthonormal
    np.diag([1.0, 1.0, -1.0]), # reflection, det = -1
])
def test_dcm_setter_rejects_invalid_rotation(badDcm):
    """
    **Validation Test Description**

    This unit test verifies that ``dcm_SB`` only accepts 3x3 proper rotation matrices.

    **Description of Variables Being Tested**

    This unit test checks that a wrong shape, a non-orthonormal matrix, and a
    reflection each raise ``ValueError``.
    """
    module = cmdTorqueBodyToTorqueAtSite.CmdTorqueBodyToTorqueAtSite()
    with pytest.raises(ValueError):
        module.dcm_SB = badDcm
