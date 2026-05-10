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
from Basilisk.fswAlgorithms import thrFiringRound


def test_thr_firing_round_output_payload():
    """
    **Validation Test Description**

    This unit test verifies that :class:`thrFiringRound.ThrFiringRound` can be
    constructed from the standard ``Basilisk.fswAlgorithms`` import path and
    that it converts thruster force commands into rounded on-time commands.

    **Description of Variables Being Tested**

    This unit test checks the output ``OnTimeRequest`` field.
    """
    module = thrFiringRound.ThrFiringRound()
    module.setControlPeriodSec(1.0)  # [s]
    module.setOnTimeResolutionSec(0.1)  # [s]
    module.setThrForceMax([1.0, 2.0])  # [N]

    forcePayload = messaging.THRArrayCmdForceMsgPayload()
    forcePayload.thrForce = [0.24, 2.0, -1.0]  # [N]
    forceMsg = messaging.THRArrayCmdForceMsg().write(forcePayload)
    module.thrForceInMsg.subscribeTo(forceMsg)

    module.Reset(0)
    module.UpdateState(1)

    onTimeOut = module.thrOnTimeOutMsg.read()
    np.testing.assert_allclose(onTimeOut.OnTimeRequest[:2], [0.2, 1.0])  # [s]
    np.testing.assert_allclose(onTimeOut.OnTimeRequest[2:], np.zeros(34))  # [s]


def test_thr_firing_round_configuration_helpers():
    """
    **Validation Test Description**

    This unit test verifies that :class:`thrFiringRound.ThrFiringRound`
    resolves scalar and vector thrust limit configurations.

    **Description of Variables Being Tested**

    This unit test checks the resolved number of thrusters and maximum thrust
    vector.
    """
    module = thrFiringRound.ThrFiringRound()
    module.setThrForceMax(3.0)  # [N]

    forceCmd = np.array([1.0, 2.0, 3.0])  # [N]
    assert module.resolveNumThrusters(forceCmd) == 3
    np.testing.assert_allclose(module.resolveThrForceMax(3), [3.0, 3.0, 3.0])  # [N]

    module.setThrForceMax([1.0, 2.0])  # [N]
    assert module.resolveNumThrusters(forceCmd) == 2
    np.testing.assert_allclose(module.resolveThrForceMax(2), [1.0, 2.0])  # [N]


def test_thr_firing_round_minimum_fire_time():
    """
    **Validation Test Description**

    This unit test verifies that :class:`thrFiringRound.ThrFiringRound` can
    apply an optional minimum fire-time threshold to rounded on-time commands.

    **Description of Variables Being Tested**

    This unit test checks that ``OnTimeRequest`` is set to zero when the
    rounded on-time is less than ``thrMinFireTimeSec``.
    """
    module = thrFiringRound.ThrFiringRound()
    module.setControlPeriodSec(1.0)  # [s]
    module.setOnTimeResolutionSec(0.1)  # [s]
    module.setThrMinFireTime(0.15)  # [s]
    module.setThrForceMax(1.0)  # [N]

    forcePayload = messaging.THRArrayCmdForceMsgPayload()
    forcePayload.thrForce = [0.14, 0.16]  # [N]
    forceMsg = messaging.THRArrayCmdForceMsg().write(forcePayload)
    module.thrForceInMsg.subscribeTo(forceMsg)

    module.Reset(0)
    module.UpdateState(1)

    onTimeOut = module.thrOnTimeOutMsg.read()
    np.testing.assert_allclose(onTimeOut.OnTimeRequest[:2], [0.0, 0.2])  # [s]


def test_reset_rejects_missing_input_message():
    """
    **Validation Test Description**

    This unit test verifies that :class:`thrFiringRound.ThrFiringRound`
    rejects reset calls when its required input message is not connected.

    **Description of Variables Being Tested**

    This unit test checks the ``thrForceInMsg`` input message reader.
    """
    module = thrFiringRound.ThrFiringRound()
    module.bskLogger = bskLogging.BSKLogger()

    with pytest.raises(bskLogging.BasiliskError, match="ThrFiringRound.thrForceInMsg"):
        module.Reset(0)
