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

"""Tests for common message-wrapper helpers and typemaps."""

import importlib

import pytest

from Basilisk.architecture import messaging
from Basilisk.architecture.messaging import messagingSupport


def test_common_helpers_are_reexported():
    """Common helpers retain their package and payload-module Python names."""

    payload_module = importlib.import_module(
        "Basilisk.architecture.messaging.AttRefMsgPayload"
    )
    helper_names = (
        "new_ThrustConfigArray",
        "ThrustConfigArray_setitem",
        "new_RWConfigArray",
        "RWConfigArray_setitem",
        "new_ReconfigBurnArray",
        "ReconfigBurnArray_setitem",
    )

    for helper_name in helper_names:
        support_helper = getattr(messagingSupport, helper_name)
        assert getattr(messaging, helper_name) is support_helper
        assert getattr(payload_module, helper_name) is support_helper

    assert messaging.TimeVector is messagingSupport.TimeVector
    assert messaging.DoubleVector is messagingSupport.DoubleVector
    assert messaging.StringVector is messagingSupport.StringVector


def test_common_config_array_helpers_cross_extension_boundaries():
    """Shared array helpers accept payloads from their dedicated extensions."""

    thruster = messaging.THRConfigMsgPayload()
    thruster.maxThrust = 1.25  # [N]
    thruster_array = messaging.new_ThrustConfigArray(messaging.MAX_EFF_CNT)

    wheel = messaging.RWConfigElementMsgPayload()
    wheel.uMax = 2.5  # [N*m]
    wheel_array = messaging.new_RWConfigArray(messaging.MAX_EFF_CNT)

    burn = messaging.ReconfigBurnInfoMsgPayload()
    burn.thrustOnTime = 3.75  # [s]
    burn_array = messaging.new_ReconfigBurnArray(3)

    try:
        messaging.ThrustConfigArray_setitem(thruster_array, 0, thruster)
        messaging.RWConfigArray_setitem(wheel_array, 0, wheel)
        messaging.ReconfigBurnArray_setitem(burn_array, 0, burn)

        thruster_config = messaging.THRArrayConfigMsgPayload()
        thruster_config.thrusters = thruster_array
        assert messaging.ThrustConfigArray_getitem(
            thruster_config.thrusters, 0
        ).maxThrust == pytest.approx(1.25)  # [N]

        assert messaging.RWConfigArray_getitem(
            wheel_array, 0
        ).uMax == pytest.approx(2.5)  # [N*m]

        burn_config = messaging.ReconfigBurnArrayInfoMsgPayload()
        burn_config.burnArray = burn_array
        assert messaging.ReconfigBurnArray_getitem(
            burn_config.burnArray, 0
        ).thrustOnTime == pytest.approx(3.75)  # [s]
    finally:
        messaging.delete_ThrustConfigArray(thruster_array)
        messaging.delete_RWConfigArray(wheel_array)
        messaging.delete_ReconfigBurnArray(burn_array)


def test_common_fixed_array_typemaps():
    """Support typemaps preserve list conversion for nested payload arrays."""

    wheel = messaging.RWConfigElementMsgPayload()
    wheel.uMax = 0.25  # [N*m]
    constellation = messaging.RWConstellationMsgPayload()
    constellation.reactionWheels = [wheel]
    assert constellation.reactionWheels[0].uMax == pytest.approx(0.25)  # [N*m]

    accelerometer_packet = messaging.AccPktDataMsgPayload()
    accelerometer_packet.measTime = 50  # [ns]
    accelerometer_data = messaging.AccDataMsgPayload()
    accelerometer_data.accPkts = [accelerometer_packet]
    assert accelerometer_data.accPkts[0].measTime == 50  # [ns]

    css_unit = messaging.CSSUnitConfigMsgPayload()
    css_unit.CBias = 0.75  # [-]
    css_config = messaging.CSSConfigMsgPayload()
    css_config.cssVals = [css_unit]
    assert css_config.cssVals[0].CBias == pytest.approx(0.75)  # [-]

    availability = messaging.RWAvailabilityMsgPayload()
    availability.wheelAvailability = [messaging.UNAVAILABLE]
    assert availability.wheelAvailability[0] == messaging.UNAVAILABLE
