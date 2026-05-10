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
from Basilisk.simulation import stateMerge


def test_state_merge_output_payload():
    """
    **Validation Test Description**

    This unit test verifies that :class:`stateMerge.StateMerge` can be
    constructed from the standard ``Basilisk.simulation`` import path and that
    it merges attitude and translation spacecraft state inputs into one output
    payload.

    **Description of Variables Being Tested**

    This unit test checks the merged ``sigma_BN``, ``omega_BN_B``, ``r_BN_N``,
    and ``v_BN_N`` fields.
    """
    module = stateMerge.StateMerge()

    attPayload = messaging.SCStatesMsgPayload()
    attPayload.sigma_BN = [0.1, -0.2, 0.3]  # [rad]
    attPayload.omega_BN_B = [0.01, -0.02, 0.03]  # [rad/s]

    transPayload = messaging.SCStatesMsgPayload()
    transPayload.r_CN_N = [1.0, 2.0, 3.0]  # [m]
    transPayload.v_CN_N = [-0.1, 0.2, -0.3]  # [m/s]

    attMsg = messaging.SCStatesMsg().write(attPayload)
    transMsg = messaging.SCStatesMsg().write(transPayload)
    module.attStateInMsg.subscribeTo(attMsg)
    module.transStateInMsg.subscribeTo(transMsg)

    module.Reset(0)
    module.UpdateState(1)

    stateOut = module.stateOutMsg.read()
    np.testing.assert_allclose(stateOut.sigma_BN, attPayload.sigma_BN)
    np.testing.assert_allclose(stateOut.omega_BN_B, attPayload.omega_BN_B)
    np.testing.assert_allclose(stateOut.r_BN_N, transPayload.r_CN_N)
    np.testing.assert_allclose(stateOut.v_BN_N, transPayload.v_CN_N)


@pytest.mark.parametrize("missing_msg_name", ["attStateInMsg", "transStateInMsg"])
def test_reset_rejects_missing_input_message(missing_msg_name):
    """
    **Validation Test Description**

    This unit test verifies that :class:`stateMerge.StateMerge` rejects reset
    calls when any required input message is not connected.

    **Description of Variables Being Tested**

    This unit test checks each required input message reader.
    """
    module = stateMerge.StateMerge()
    module.bskLogger = bskLogging.BSKLogger()

    inputMessages = {
        "attStateInMsg": messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload()),
        "transStateInMsg": messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload()),
    }
    for msgName, msg in inputMessages.items():
        if msgName != missing_msg_name:
            getattr(module, msgName).subscribeTo(msg)

    with pytest.raises(bskLogging.BasiliskError, match=f"StateMerge.{missing_msg_name}"):
        module.Reset(0)
