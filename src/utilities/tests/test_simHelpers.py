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

"""Regression tests for epoch messages created by simulation helpers."""

import gc
import weakref

import pytest

from Basilisk.simulation import spiceInterface
from Basilisk.utilities import (
    deprecated,
    simHelpers,
    simIncludeGravBody,
    unitTestSupport,
)


EPOCH_STRING = "2023 DEC 15 12:34:56.125 UTC"


@pytest.mark.parametrize(
    "helper_module", [simHelpers, unitTestSupport],
    ids=["simHelpers", "unitTestSupport"],
)
def test_epoch_payload_outlives_temporary_message(helper_module):
    """Reading a temporary epoch retains its storage until the payload is gone."""
    references = []

    def create_epoch():
        """Observe the temporary message without retaining it in the test."""
        epochMsg = helper_module.timeStringToGregorianUTCMsg(EPOCH_STRING)
        references.append(weakref.ref(epochMsg))
        return epochMsg

    with deprecated.ignore(
        r"Basilisk\.utilities\.unitTestSupport\.timeStringToGregorianUTCMsg"
    ):
        payload = create_epoch().read()

    gc.collect()
    assert references[0]() is not None
    assert (payload.year, payload.month, payload.day) == (2023, 12, 15)  # [UTC date]
    assert payload.seconds == 56.125  # [s]

    del payload
    gc.collect()
    assert references[0]() is None


@pytest.mark.parametrize(
    "helper_module", [simHelpers, unitTestSupport],
    ids=["simHelpers", "unitTestSupport"],
)
def test_unused_epoch_messages_are_released(helper_module):
    """Repeated conversions must not retain messages after callers release them."""
    references = []
    with deprecated.ignore(
        r"Basilisk\.utilities\.unitTestSupport\.timeStringToGregorianUTCMsg"
    ):
        # Weak references detect each retained message without a long stress loop.
        for _ in range(3):
            epochMsg = helper_module.timeStringToGregorianUTCMsg(EPOCH_STRING)
            references.append(weakref.ref(epochMsg))
            del epochMsg

    gc.collect()
    assert all(reference() is None for reference in references)


@pytest.mark.parametrize(
    "helper_module", [simHelpers, unitTestSupport],
    ids=["simHelpers", "unitTestSupport"],
)
def test_epoch_recorder_controls_message_lifetime(helper_module):
    """A recorder retains a helper-created epoch until the recorder is released."""
    with deprecated.ignore(
        r"Basilisk\.utilities\.unitTestSupport\.timeStringToGregorianUTCMsg"
    ):
        epochMsg = helper_module.timeStringToGregorianUTCMsg(EPOCH_STRING)
    reference = weakref.ref(epochMsg)
    recorder = epochMsg.recorder()
    del epochMsg

    gc.collect()
    assert reference() is not None
    recorder.UpdateState(0)  # [ns]
    assert recorder.size() == 1
    assert (recorder.year[0], recorder.month[0], recorder.day[0]) == (
        2023, 12, 15
    )  # [UTC date]
    assert recorder.seconds[0] == 56.125  # [s]

    del recorder
    gc.collect()
    assert reference() is None


@pytest.mark.parametrize(
    "release_action", ["unsubscribe", "resubscribe", "destroy"]
)
def test_spice_subscription_controls_epoch_lifetime(release_action):
    """SPICE keeps a helper-created epoch alive only while subscribed to it."""
    spice_object = spiceInterface.SpiceInterface()
    epochMsg = simHelpers.timeStringToGregorianUTCMsg(EPOCH_STRING)
    reference = weakref.ref(epochMsg)
    spice_object.epochInMsg.subscribeTo(epochMsg)
    del epochMsg

    gc.collect()
    assert reference() is not None
    payload = spice_object.epochInMsg()
    assert (payload.year, payload.month, payload.day) == (2023, 12, 15)  # [UTC date]
    assert (payload.hours, payload.minutes, payload.seconds) == (
        12, 34, 56.125
    )  # [h, min, s]
    del payload

    if release_action == "unsubscribe":
        spice_object.epochInMsg.unsubscribe()
    elif release_action == "resubscribe":
        spice_object.epochInMsg.subscribeTo(
            simHelpers.timeStringToGregorianUTCMsg(EPOCH_STRING)
        )
    else:
        del spice_object

    gc.collect()
    assert reference() is None


def test_spice_factory_releases_replaced_epoch_messages():
    """Repeated factory setup retains only the epoch for the current interface."""
    factory = simIncludeGravBody.gravBodyFactory()
    references = []
    for _ in range(3):
        # No planetary kernels are needed to exercise epoch subscription ownership.
        factory.createSpiceInterface(
            time=EPOCH_STRING, spiceKernelFileNames=(), epochInMsg=True
        )
        references.append(weakref.ref(factory.epochMsg))

    gc.collect()
    assert all(reference() is None for reference in references[:-1])
    assert references[-1]() is not None

    del factory
    gc.collect()
    assert references[-1]() is None
