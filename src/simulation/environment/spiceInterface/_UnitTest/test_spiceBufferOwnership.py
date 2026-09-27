# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
# Distributed under the ISC license; see LICENSE.

"""Check SPICE time conversion and the public interface after scratch-buffer cleanup."""

import pytest

from Basilisk.architecture import bskLogging
from Basilisk.simulation import spiceInterface
from Basilisk.utilities import macros


@pytest.mark.parametrize("picture, expected", [
    (None, "JAN 01,2000  12:00:01.0000 (UTC)"),
    ("YYYY-MM-DD HR:MN:SC.### ::UTC", "2000-01-01 12:00:01.000"),
])
def test_time_conversion_after_repeated_reset(picture, expected):
    """Preserve default and custom time strings and Julian dates across resets."""
    model = spiceInterface.SpiceInterface()
    model.UTCCalInit = "2000 JAN 01 12:00:00 UTC"
    if picture is not None:
        model.timeOutPicture = picture

    elapsed_seconds = 1.0  # [s]
    initial_julian_date = 2451545.0  # [day]
    seconds_per_day = 86400.0  # [s/day]
    date_tolerance = 5.0e-8  # [day] Half a unit in the final SPICE Julian-date decimal place.
    for _ in range(3):
        model.Reset(0)
        model.UpdateState(macros.sec2nano(elapsed_seconds))
        assert model.getCurrentTimeString() == expected
        assert model.julianDateCurrent == pytest.approx(
            initial_julian_date + elapsed_seconds / seconds_per_day,
            rel=0.0, abs=date_tolerance,
        )


@pytest.mark.parametrize("picture_length", range(7))
def test_time_output_rejects_insufficient_capacity(picture_length):
    """Reject invalid output capacities before passing storage to SPICE."""
    model = spiceInterface.SpiceInterface(auto_configure_kernels=False)
    model.timeOutPicture = "X" * picture_length
    with pytest.raises(bskLogging.BasiliskError, match="output format string is not long enough"):
        model.getCurrentTimeString()


@pytest.mark.parametrize("field", ["spiceBuffer", "charBufferSize"])
def test_scratch_storage_is_private(field):
    """Python callers cannot replace the buffer or change its capacity."""
    model = spiceInterface.SpiceInterface(auto_configure_kernels=False)
    with pytest.raises(AttributeError):
        getattr(model, field)
    with pytest.raises(ValueError, match=field):
        setattr(model, field, 1)
