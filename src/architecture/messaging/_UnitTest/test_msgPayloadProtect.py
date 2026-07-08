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
#
#   Unit Test Script
#   Module Name:        message payload field protection (issue #305)
#   Author:             robotrocketscience (https://github.com/robotrocketscience)
#   Creation Date:      2026-07-07
#
"""Message payloads must reject assignment of field names that do not already
exist, the same way SWIG-wrapped modules do (issue #305). This guards against
silent typos such as ``payload.sigm_BN = ...`` instead of ``payload.sigma_BN``.

Only the ``*Payload`` data structs are frozen; the Msg/Reader/Recorder/Writer
and vector wrapper classes must stay unfrozen (their custom ``__getattr__`` has
side effects), so this file also checks that a recorder can still be built.
"""
import pytest
from Basilisk.architecture import messaging


def test_payload_rejects_unknown_field():
    payload = messaging.SCStatesMsgPayload()
    payload.r_BN_N = [1.0, 2.0, 3.0]  # [m] existing field: allowed
    with pytest.raises(ValueError):
        payload.r_BN_Ntypo = [1.0, 2.0, 3.0]  # [m] typo'd field: rejected


def test_payload_existing_fields_still_settable():
    payload = messaging.SCStatesMsgPayload()
    payload.r_BN_N = [1.0, 2.0, 3.0]  # [m]
    payload.v_BN_N = [4.0, 5.0, 6.0]  # [m/s]
    assert list(payload.r_BN_N) == [1.0, 2.0, 3.0]
    assert list(payload.v_BN_N) == [4.0, 5.0, 6.0]


def test_c_payload_rejects_unknown_field():
    payload = messaging.CModuleTemplateMsgPayload()
    payload.dataVector = [1.0, 2.0, 3.0]
    with pytest.raises(ValueError):
        payload.notAField = 1


def test_recorder_class_not_frozen():
    # The recorder is created through the normal API; freezing must not have
    # been applied to the recorder class (its __getattr__ side-effects would
    # otherwise break construction).
    recorder = messaging.SCStatesMsg().recorder()
    assert recorder is not None


if __name__ == "__main__":
    test_payload_rejects_unknown_field()
    test_payload_existing_fields_still_settable()
    test_c_payload_rejects_unknown_field()
    test_recorder_class_not_frozen()
    print("all passed")
