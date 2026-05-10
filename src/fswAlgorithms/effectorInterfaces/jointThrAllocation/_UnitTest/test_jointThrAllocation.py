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
from Basilisk.fswAlgorithms import jointThrAllocation


def _linked_input_messages():
    armConfigMsg = messaging.THRArmConfigMsg().write(messaging.THRArmConfigMsgPayload())
    comStatesMsg = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload())
    hubStatesMsg = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload())
    transForceMsg = messaging.CmdForceInertialMsg().write(messaging.CmdForceInertialMsgPayload())
    rotTorqueMsg = messaging.CmdTorqueBodyMsg().write(messaging.CmdTorqueBodyMsgPayload())
    return {
        "armConfigInMsg": armConfigMsg,
        "CoMStatesInMsg": comStatesMsg,
        "hubStatesInMsg": hubStatesMsg,
        "transForceInMsg": transForceMsg,
        "rotTorqueInMsg": rotTorqueMsg,
    }


def test_map_matrix():
    """
    **Validation Test Description**

    This unit test verifies that :func:`jointThrAllocation.mapMatrix` maps
    thruster force magnitudes into the expected stacked force and torque
    wrench.

    **Description of Variables Being Tested**

    This unit test checks the force block and torque block of the returned
    wrench mapping matrix.
    """
    rVec_B = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # [m]
    fHatVec_B = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    r_ComB_B = np.array([0.0, 0.0, 0.0])  # [m]

    mapping = jointThrAllocation.mapMatrix(rVec_B, fHatVec_B, r_ComB_B)
    expected = np.array(
        [
            [0.0, 0.0],
            [1.0, 0.0],
            [0.0, 1.0],
            [0.0, 1.0],
            [0.0, 0.0],
            [1.0, 0.0],
        ]
    )

    np.testing.assert_allclose(mapping, expected)


def test_allocation_configuration_helpers():
    """
    **Validation Test Description**

    This unit test verifies that :class:`jointThrAllocation.JointThrAllocation`
    can be constructed from the standard ``Basilisk.fswAlgorithms`` import path
    and that the helper methods resolve scalar and vector configuration values.

    **Description of Variables Being Tested**

    This unit test checks the resolved thrust force bounds, thrust weights, and
    initial decision vector shape.
    """
    allocation = jointThrAllocation.JointThrAllocation()
    allocation.nJoint = 2
    allocation.nThr = 2

    allocation.setThrForceMax(4.0)  # [N]
    np.testing.assert_allclose(allocation.resolveThrForceMax(), np.array([4.0, 4.0]))  # [N]

    allocation.setWf(1.0e-5)
    allocation.resolveWf()
    np.testing.assert_allclose(allocation.Wf, np.array([1.0e-5, 1.0e-5]))

    allocation.initialGuesses()
    assert allocation.x0.shape == (5, 4)

    bounds = allocation.bounds()
    assert bounds[0] == (-np.pi, np.pi)
    assert bounds[1] == (-np.pi, np.pi)
    assert bounds[2] == (0.0, 4.0)  # [N]
    assert bounds[3] == (0.0, 4.0)  # [N]


@pytest.mark.parametrize(
    "missing_msg_name",
    [
        "armConfigInMsg",
        "CoMStatesInMsg",
        "hubStatesInMsg",
        "transForceInMsg",
        "rotTorqueInMsg",
    ],
)
def test_reset_rejects_missing_input_message(missing_msg_name):
    """
    **Validation Test Description**

    This unit test verifies that :class:`jointThrAllocation.JointThrAllocation`
    rejects reset calls when any required input message is not connected.

    **Description of Variables Being Tested**

    This unit test checks each required input message reader.
    """
    allocation = jointThrAllocation.JointThrAllocation()
    allocation.bskLogger = bskLogging.BSKLogger()

    for msgName, msg in _linked_input_messages().items():
        if msgName != missing_msg_name:
            getattr(allocation, msgName).subscribeTo(msg)

    with pytest.raises(bskLogging.BasiliskError, match=f"JointThrAllocation.{missing_msg_name}"):
        allocation.Reset(0)
