# ISC License
#
# Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder
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

"""Check the GPS time and UTC Julian date of the SPICE time output message against their definitions."""

import datetime

import pytest

from Basilisk import __path__
from Basilisk.simulation import spiceInterface
from Basilisk.utilities import SimulationBaseClass, macros

bsk_path = __path__[0]

GPS_EPOCH = datetime.datetime(1980, 1, 6)  # GPS epoch, 1980 January 6, 00:00:00 UTC
UNIX_EPOCH = datetime.datetime(1970, 1, 1)  # Unix epoch, 1970 January 1, 00:00:00 UTC
UNIX_EPOCH_JULIAN_DATE = 2440587.5  # [d] Julian date of the Unix epoch
SECONDS_PER_DAY = 86400.0  # [s]
SECONDS_PER_WEEK = 604800.0  # [s]
WEEKS_PER_ROLLOVER = 1024  # GPS weeks between rollovers


def run_spice_time_output(epoch):
    """Run ``spiceInterface`` for a single update at ``epoch`` and return the ``spiceTimeOutMsg`` recorder.

    :param epoch: UTC time at which the simulation starts
    :return: recorder of ``spiceTimeOutMsg``
    """
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    process.addTask(sim.CreateNewTask("task", macros.sec2nano(1.0)))

    spice = spiceInterface.SpiceInterface()
    spice.SPICEDataPath = bsk_path + "/supportData/EphemerisData/"
    spice.UTCCalInit = epoch.strftime("%Y %B %d, %H:%M:%S.%f") + " (UTC)"
    sim.AddModelToTask("task", spice)

    time_log = spice.spiceTimeOutMsg.recorder()
    sim.AddModelToTask("task", time_log)

    sim.InitializeSimulation()
    sim.ConfigureStopTime(0)
    sim.ExecuteSimulation()
    return time_log


# gps_utc_offset is GPS - UTC, the leap seconds introduced since the GPS epoch, i.e. TAI - UTC - 19 s [s]
@pytest.mark.parametrize("epoch, gps_utc_offset", [
    (datetime.datetime(1999, 8, 21, 23, 59, 47), 13),  # GPS week 1024 starts at this UTC time
    (datetime.datetime(2015, 7, 1, 0, 0, 0), 17),  # just after the leap second of 2015 June 30
    (datetime.datetime(2024, 4, 4, 0, 0, 0), 18),  # TDB - TT near its maximum of about +1.7 ms
    (datetime.datetime(2026, 3, 20, 12, 0, 0, 123000), 18),  # fractional UTC seconds
    (datetime.datetime(2048, 6, 1, 0, 0, 0), 18),  # past GPS week 3551, where week*604800 s overflows a 32-bit int
])
def test_gps_time_and_julian_date(epoch, gps_utc_offset):
    """The GPS week, GPS seconds of week and UTC Julian date of ``spiceTimeOutMsg`` equal their definitions.

    GPS time is the UTC time elapsed since the GPS epoch plus the leap seconds introduced since then, and the
    Julian date is the UTC Julian date.  The epochs include a GPS week rollover, a date just after a leap second,
    a date near the maximum of the periodic difference between TDB and atomic time, and a date after GPS week 3551,
    where the GPS week expressed in seconds no longer fits in a signed 32-bit integer.
    """
    time_log = run_spice_time_output(epoch)

    gps_time = (epoch - GPS_EPOCH).total_seconds() + gps_utc_offset  # [s]
    julian_date = UNIX_EPOCH_JULIAN_DATE + (epoch - UNIX_EPOCH).total_seconds()/SECONDS_PER_DAY  # [d]

    gps_week = int(time_log.GPSWeek[-1]) + WEEKS_PER_ROLLOVER*int(time_log.GPSRollovers[-1])
    assert gps_week == int(gps_time//SECONDS_PER_WEEK)
    assert time_log.GPSSeconds[-1] == pytest.approx(gps_time % SECONDS_PER_WEEK, rel=0.0, abs=1.0e-6)  # [s]
    assert time_log.JulianDateCurrent[-1] == pytest.approx(julian_date, rel=0.0, abs=1.0e-9)  # [d]


def test_gps_time_before_gps_epoch():
    """Before the GPS epoch, the GPS week, seconds and rollovers are held at zero."""
    time_log = run_spice_time_output(datetime.datetime(1979, 6, 1, 0, 0, 0))

    assert int(time_log.GPSWeek[-1]) == 0
    assert int(time_log.GPSRollovers[-1]) == 0
    assert time_log.GPSSeconds[-1] == 0.0  # [s]


@pytest.mark.parametrize("epoch, gps_utc_offset, has_leap_second", [
    (datetime.datetime(1998, 12, 31, 23, 59, 59), 12, True),  # before J2000
    (datetime.datetime(2015, 6, 30, 23, 59, 59), 16, True),  # June leap second
    (datetime.datetime(2016, 12, 31, 23, 59, 59), 17, True),  # December leap second
    (datetime.datetime(2026, 3, 20, 23, 59, 59), 18, False),  # ordinary midnight
])
def test_time_output_across_midnight(epoch, gps_utc_offset, has_leap_second):
    """Keep SPICE JDUTC fixed during an inserted second while GPS time advances.

    Sample before, within and after the leap second, and also cross an ordinary
    midnight to ensure fractional UTC seconds are preserved on normal days.
    """
    spice = spiceInterface.SpiceInterface()
    spice.UTCCalInit = epoch.strftime("%Y-%m-%d %H:%M:%S") + " UTC"
    spice.Reset(0)

    midnight = epoch + datetime.timedelta(seconds=1)
    midnight_julian_date = UNIX_EPOCH_JULIAN_DATE + (midnight - UNIX_EPOCH).total_seconds()/SECONDS_PER_DAY  # [d]
    initial_gps_time = (epoch - GPS_EPOCH).total_seconds() + gps_utc_offset  # [s]
    sample_times = (0.0, 0.5, 0.999, 1.0, 1.001, 1.5, 1.999, 2.0, 2.001, 2.5, 3.0)  # [s]
    previous_julian_date = None
    for elapsed in sample_times:
        spice.UpdateState(macros.sec2nano(elapsed))
        time_output = spice.spiceTimeOutMsg.read()
        if has_leap_second:
            seconds_from_midnight = min(elapsed - 1.0, 0.0) + max(elapsed - 2.0, 0.0)  # [s]
        else:
            seconds_from_midnight = elapsed - 1.0  # [s]
        expected_julian_date = midnight_julian_date + seconds_from_midnight/SECONDS_PER_DAY  # [d]
        expected_gps_time = initial_gps_time + elapsed  # [s]
        gps_week = int(time_output.GPSWeek) + WEEKS_PER_ROLLOVER*int(time_output.GPSRollovers)

        assert time_output.JulianDateCurrent == pytest.approx(expected_julian_date, rel=0.0, abs=1.0e-9)  # [d]
        if has_leap_second and 1.0 <= elapsed <= 2.0:  # [s] Inserted second through the following midnight.
            assert time_output.JulianDateCurrent == midnight_julian_date
        if previous_julian_date is not None:
            assert time_output.JulianDateCurrent >= previous_julian_date
        previous_julian_date = time_output.JulianDateCurrent
        assert gps_week == int(expected_gps_time//SECONDS_PER_WEEK)
        assert time_output.GPSSeconds == pytest.approx(expected_gps_time % SECONDS_PER_WEEK,
                                                     rel=0.0, abs=1.0e-6)  # [s]


def test_gps_time_crosses_gps_epoch():
    """Release the zero GPS outputs when a simulation crosses the GPS epoch."""
    spice = spiceInterface.SpiceInterface()
    spice.UTCCalInit = "1980 JAN 05 23:59:59 UTC"
    spice.Reset(0)

    sample_times = (0.0, 0.5, 1.0, 1.5, 2.0)  # [s]
    for elapsed in sample_times:
        spice.UpdateState(macros.sec2nano(elapsed))
        time_output = spice.spiceTimeOutMsg.read()
        expected_seconds = max(elapsed - 1.0, 0.0)  # [s]
        assert time_output.GPSWeek == 0
        assert time_output.GPSRollovers == 0
        assert time_output.GPSSeconds == pytest.approx(expected_seconds, rel=0.0, abs=1.0e-6)  # [s]


if __name__ == "__main__":
    test_gps_time_and_julian_date(datetime.datetime(1999, 8, 21, 23, 59, 47), 13)
