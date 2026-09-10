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
def test_unused_epoch_messages_are_released(helper_module):
    """Repeated conversions must not retain messages after callers release them."""
    references = []
    with deprecated.ignore(
        r"Basilisk\.utilities\.unitTestSupport\.timeStringToGregorianUTCMsg"
    ):
        for _ in range(100):
            epochMsg = helper_module.timeStringToGregorianUTCMsg(EPOCH_STRING)
            references.append(weakref.ref(epochMsg))
            del epochMsg

    gc.collect()
    assert all(reference() is None for reference in references)


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
    for _ in range(20):
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
