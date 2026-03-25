#
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
#
import tempfile
from pathlib import Path
import pytest

from Basilisk.architecture import bskLogging
from Basilisk.simulation import spaceWeatherData
from Basilisk.utilities import SimulationBaseClass, macros, unitTestSupport
from Basilisk.architecture.bskLogging import BasiliskError


_CELESTRAK_CSV = "\n".join([
    "DATE,BSRN,ND,KP1,KP2,KP3,KP4,KP5,KP6,KP7,KP8,KP_SUM,AP1,AP2,AP3,AP4,AP5,AP6,AP7,AP8,AP_AVG,CP,C9,ISN,F10.7_OBS,F10.7_ADJ,F10.7_DATA_TYPE,F10.7_OBS_CENTER81,F10.7_OBS_LAST81,F10.7_ADJ_CENTER81,F10.7_ADJ_LAST81",
    "2026-02-28,2626,4,20,27,13,13,20,17,20,23,153,7,12,5,5,7,6,7,9,7,0.4,2,58,140.7,138.1,OBS,141.7,144.2,139.2,139.9",
    "2026-03-01,2626,5,27,27,23,20,13,7,17,13,147,12,12,9,7,5,3,6,5,7,0.4,2,80,147.0,144.4,OBS,141.1,143.9,138.6,139.7",
    "2026-03-02,2626,6,13,7,10,3,17,10,7,13,80,5,3,4,2,6,4,3,5,4,0.1,0,81,147.6,145.0,OBS,140.3,143.9,138.0,139.7",
    "2026-03-03,2626,7,7,27,20,23,20,33,33,50,213,3,12,7,9,7,18,18,48,15,0.9,4,83,143.5,141.1,OBS,139.5,144.0,137.2,139.8",
    "2026-03-04,2626,8,23,30,17,20,7,7,13,10,127,9,15,6,7,3,3,5,4,6,0.3,1,68,141.2,138.8,OBS,138.5,144.3,136.3,140.1",
])


def test_space_weather_data_celestrak_example_columns():
    """Validate parsing against the provided CelesTrak-style rows.

    **Test Description**

    This test uses the provided CelesTrak CSV rows and validates output values
    against the supplied ground-truth weather channel values.

    **Description of Variables Being Tested**

    The test checks all 23 outputs in ``swDataOutMsgs`` against known values for
    the epoch ``2026-03-04 21:48:00``.
    """

    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    unit_task_name = "unitTask"
    unit_process_name = "unitProcess"

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess(unit_process_name)
    proc.addTask(sim.CreateNewTask(unit_task_name, macros.sec2nano(1.0)))  # [s]

    module = spaceWeatherData.SpaceWeatherData()
    module.ModelTag = "spaceWeatherData"

    with tempfile.TemporaryDirectory() as temp_dir:
        file_path = Path(temp_dir) / "space_weather.csv"
        file_path.write_text(_CELESTRAK_CSV)

        module.loadSpaceWeatherFile(str(file_path))

        timeInitString = "2026 March 04 21:48:00.000000"
        epochMsg = unitTestSupport.timeStringToGregorianUTCMsg(timeInitString)
        module.epochInMsg.subscribeTo(epochMsg)

        sim.AddModelToTask(unit_task_name, module)

        logs = []
        for msg_index in range(23):
            recorder = module.swDataOutMsgs[msg_index].recorder()
            logs.append(recorder)
            sim.AddModelToTask(unit_task_name, recorder)

        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(1.0))  # [s]
        sim.ExecuteSimulation()

        expected = [6.0, 4.0, 5.0, 3.0, 3.0, 7.0, 6.0, 15.0, 9.0,
                    48.0, 18.0, 18.0, 7.0, 9.0, 7.0, 12.0,
                    3.0, 5.0, 3.0, 4.0, 6.0, 138.5, 143.5]

        for msg_index in range(23):
            assert abs(logs[msg_index].dataValue[0] - expected[msg_index]) < 1e-12


def test_space_weather_data_stops_at_first_invalid_row():
    """Validate reading stops at the first invalid row (e.g., PRM monthly line).

    **Test Description**

    This test appends invalid PRM-style rows with missing AP fields after valid
    daily rows and verifies that the valid section is still used correctly.

    **Description of Variables Being Tested**

    The test checks the 23 weather outputs against the same ground-truth values
    as the valid CelesTrak example set.
    """

    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    unit_task_name = "unitTask"
    unit_process_name = "unitProcess"

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess(unit_process_name)
    proc.addTask(sim.CreateNewTask(unit_task_name, macros.sec2nano(1.0)))  # [s]

    module = spaceWeatherData.SpaceWeatherData()
    module.ModelTag = "spaceWeatherData"

    csv_text = "\n".join([
        "DATE,BSRN,ND,KP1,KP2,KP3,KP4,KP5,KP6,KP7,KP8,KP_SUM,AP1,AP2,AP3,AP4,AP5,AP6,AP7,AP8,AP_AVG,CP,C9,ISN,F10.7_OBS,F10.7_ADJ,F10.7_DATA_TYPE,F10.7_OBS_CENTER81,F10.7_OBS_LAST81,F10.7_ADJ_CENTER81,F10.7_ADJ_LAST81",
        "2026-02-28,2626,4,20,27,13,13,20,17,20,23,153,7,12,5,5,7,6,7,9,7,0.4,2,58,140.7,138.1,OBS,141.7,144.2,139.2,139.9",
        "2026-03-01,2626,5,27,27,23,20,13,7,17,13,147,12,12,9,7,5,3,6,5,7,0.4,2,80,147.0,144.4,OBS,141.1,143.9,138.6,139.7",
        "2026-03-02,2626,6,13,7,10,3,17,10,7,13,80,5,3,4,2,6,4,3,5,4,0.1,0,81,147.6,145.0,OBS,140.3,143.9,138.0,139.7",
        "2026-03-03,2626,7,7,27,20,23,20,33,33,50,213,3,12,7,9,7,18,18,48,15,0.9,4,83,143.5,141.1,OBS,139.5,144.0,137.2,139.8",
        "2026-03-04,2626,8,23,30,17,20,7,7,13,10,127,9,15,6,7,3,3,5,4,6,0.3,1,68,141.2,138.8,OBS,138.5,144.3,136.3,140.1",
        "2038-11-01,2797,16,,,,,,,,,,,,,,,,,,,,,64,95.7,94.3,PRM,96.0,95.4,94.7,96.1",
        "2038-12-01,2798,19,,,,,,,,,,,,,,,,,,,,,62,95.9,93.3,PRM,96.2,95.9,93.8,95.1",
    ])

    with tempfile.TemporaryDirectory() as temp_dir:
        file_path = Path(temp_dir) / "space_weather.csv"
        file_path.write_text(csv_text)

        module.loadSpaceWeatherFile(str(file_path))

        timeInitString = "2026 March 04 21:48:00.000000"
        epochMsg = unitTestSupport.timeStringToGregorianUTCMsg(timeInitString)
        module.epochInMsg.subscribeTo(epochMsg)

        sim.AddModelToTask(unit_task_name, module)

        logs = []
        for msg_index in range(23):
            recorder = module.swDataOutMsgs[msg_index].recorder()
            logs.append(recorder)
            sim.AddModelToTask(unit_task_name, recorder)

        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(1.0))  # [s]
        sim.ExecuteSimulation()

        expected = [6.0, 4.0, 5.0, 3.0, 3.0, 7.0, 6.0, 15.0, 9.0,
                    48.0, 18.0, 18.0, 7.0, 9.0, 7.0, 12.0,
                    3.0, 5.0, 3.0, 4.0, 6.0, 138.5, 143.5]

        for msg_index in range(23):
            assert abs(logs[msg_index].dataValue[0] - expected[msg_index]) < 1e-12


def test_space_weather_data_missing_required_column():
    """Validate missing required CSV columns lead to zero-state output publication.

    **Test Description**

    This test removes ``F10.7_OBS_CENTER81`` from the CSV header and checks that
    the module cannot build a valid weather table and thus publishes zeros.

    **Description of Variables Being Tested**

    The test checks all 23 weather outputs and verifies every value is zero.
    """

    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    module = spaceWeatherData.SpaceWeatherData()
    module.ModelTag = "spaceWeatherData"

    csv_text = "\n".join([
        "DATE,AP1,AP2,AP3,AP4,AP5,AP6,AP7,AP8,AP_AVG,F10.7_OBS",
        "2025-01-04,401,402,403,404,405,406,407,408,450,73",
        "2025-01-05,301,302,303,304,305,306,307,308,350,72",
        "2025-01-06,201,202,203,204,205,206,207,208,250,71",
        "2025-01-07,101,102,103,104,105,106,107,108,150,70",
    ])

    with tempfile.TemporaryDirectory() as temp_dir:
        file_path = Path(temp_dir) / "space_weather.csv"
        file_path.write_text(csv_text)

        with pytest.raises(BasiliskError, match="Space-weather file missing required column"):
            module.loadSpaceWeatherFile(str(file_path))


def test_space_weather_data_duplicate_date_rows():
    """Validate duplicate DATE rows are rejected and zero-state output is published.

    **Test Description**

    This test provides duplicate ``DATE`` rows in the weather table. The module
    should reject the file and publish zero outputs.

    **Description of Variables Being Tested**

    The test checks all 23 weather outputs and verifies every value is zero.
    """

    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    module = spaceWeatherData.SpaceWeatherData()
    module.ModelTag = "spaceWeatherData"

    csv_text = "\n".join([
        "DATE,AP1,AP2,AP3,AP4,AP5,AP6,AP7,AP8,AP_AVG,F10.7_OBS,F10.7_OBS_CENTER81",
        "2025-01-04,401,402,403,404,405,406,407,408,450,73,83",
        "2025-01-04,301,302,303,304,305,306,307,308,350,72,82",
        "2025-01-06,201,202,203,204,205,206,207,208,250,71,81",
        "2025-01-07,101,102,103,104,105,106,107,108,150,70,80",
    ])

    with tempfile.TemporaryDirectory() as temp_dir:
        file_path = Path(temp_dir) / "space_weather.csv"
        file_path.write_text(csv_text)

        with pytest.raises(BasiliskError, match="Duplicate DATE row found in space-weather table"):
            module.loadSpaceWeatherFile(str(file_path))


def test_space_weather_data_unsorted_rows():
    """Validate unsorted DATE rows are rejected and zero-state output is published.

    **Test Description**

    This test provides weather rows out of chronological order. The loader should
    reject the table and the module should publish zero outputs.

    **Description of Variables Being Tested**

    The test checks all 23 weather outputs and verifies every value is zero.
    """

    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    module = spaceWeatherData.SpaceWeatherData()
    module.ModelTag = "spaceWeatherData"

    csv_text = "\n".join([
        "DATE,AP1,AP2,AP3,AP4,AP5,AP6,AP7,AP8,AP_AVG,F10.7_OBS,F10.7_OBS_CENTER81",
        "2025-01-05,301,302,303,304,305,306,307,308,350,72,82",
        "2025-01-04,401,402,403,404,405,406,407,408,450,73,83",
        "2025-01-06,201,202,203,204,205,206,207,208,250,71,81",
        "2025-01-07,101,102,103,104,105,106,107,108,150,70,80",
    ])

    with tempfile.TemporaryDirectory() as temp_dir:
        file_path = Path(temp_dir) / "space_weather.csv"
        file_path.write_text(csv_text)

        with pytest.raises(BasiliskError, match="Space-weather DATE rows must be sorted in ascending order"):
            module.loadSpaceWeatherFile(str(file_path))


def test_reset_error_when_epoch_before_table():
    """Validate that Reset emits BSK_ERROR when the epoch date is not in the table.

    **Test Description**

    The simulation epoch is set to 2026-02-27, one day before the first row of
    the loaded table (2026-02-28).  The ``Reset`` probe calls ``computeSwState``
    at initialisation time; the four-day look-back window cannot be assembled and
    a ``BSK_ERROR`` is emitted, which Basilisk converts to a ``BasiliskError``.

    **Description of Variables Being Tested**

    ``BasiliskError`` raised during ``InitializeSimulation`` with the message
    ``"simulation start date is not covered"``.
    """

    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    unit_task_name = "unitTask"
    unit_process_name = "unitProcess"

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess(unit_process_name)
    proc.addTask(sim.CreateNewTask(unit_task_name, macros.sec2nano(1.0)))  # [s]

    module = spaceWeatherData.SpaceWeatherData()
    module.ModelTag = "spaceWeatherData"

    with tempfile.TemporaryDirectory() as temp_dir:
        file_path = Path(temp_dir) / "space_weather.csv"
        file_path.write_text(_CELESTRAK_CSV)
        module.loadSpaceWeatherFile(str(file_path))

        # epoch one day before the table start — D0 = 2026-02-27, not in table
        epochMsg = unitTestSupport.timeStringToGregorianUTCMsg("2026 February 27 00:00:00.000000")
        module.epochInMsg.subscribeTo(epochMsg)
        sim.AddModelToTask(unit_task_name, module)

        with pytest.raises(BasiliskError, match="simulation start date is not covered"):
            sim.InitializeSimulation()


def test_stale_output_retained_on_missing_day():
    """Validate that outputs are not overwritten when a required day is missing.

    **Test Description**

    The simulation runs two stages with the same module instance.  In stage 1
    (t = 1 s, epoch 2026-03-04) the look-up succeeds and known values are
    written.  In stage 2 (t = 25 h + 1 s) D0 advances to 2026-03-05 which is
    absent from the table; the module must log ``BSK_WARNING`` and return without
    writing, leaving the outputs at the stage-1 values.

    **Description of Variables Being Tested**

    - ``swDataOutMsgs[0]``  (``ap_24_0``, AP_AVG):        6.0 after both stages.
    - ``swDataOutMsgs[21]`` (``f107_1944_0``, F10.7c81): 138.5 after both stages.
    """
    # Suppress the expected BSK_WARNING, not testing them at the moment.
    bskLogging.setDefaultLogLevel(bskLogging.BSK_ERROR)
    unit_task_name = "unitTask"
    unit_process_name = "unitProcess"

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess(unit_process_name)
    proc.addTask(sim.CreateNewTask(unit_task_name, macros.sec2nano(1.0)))  # [s]

    module = spaceWeatherData.SpaceWeatherData()
    module.ModelTag = "spaceWeatherData"

    with tempfile.TemporaryDirectory() as temp_dir:
        file_path = Path(temp_dir) / "space_weather.csv"
        file_path.write_text(_CELESTRAK_CSV)
        module.loadSpaceWeatherFile(str(file_path))

        # epoch on the last day in the table; after 24 h D0 = 2026-03-05 (absent)
        epochMsg = unitTestSupport.timeStringToGregorianUTCMsg("2026 March 04 00:00:00.000000")
        module.epochInMsg.subscribeTo(epochMsg)
        sim.AddModelToTask(unit_task_name, module)

        recorder_avg   = module.swDataOutMsgs[0].recorder()
        recorder_f107c = module.swDataOutMsgs[21].recorder()
        sim.AddModelToTask(unit_task_name, recorder_avg)
        sim.AddModelToTask(unit_task_name, recorder_f107c)

        sim.InitializeSimulation()

        # Stage 1: successful look-up — D0 = 2026-03-04
        sim.ConfigureStopTime(macros.sec2nano(1.0))  # [s]
        sim.ExecuteSimulation()
        assert abs(recorder_avg.dataValue[0]   - 6.0)   < 1e-12
        assert abs(recorder_f107c.dataValue[0] - 138.5) < 1e-12

        # Stage 2: failed look-up — D0 = 2026-03-05 not in table, keep previous data
        sim.ConfigureStopTime(macros.sec2nano(25 * 3600 + 1.0))  # [s]
        sim.ExecuteSimulation()
        assert abs(recorder_avg.dataValue[-1]   - 6.0)   < 1e-12
        assert abs(recorder_f107c.dataValue[-1] - 138.5) < 1e-12


def test_space_weather_data_epoch_update():
    """Validate internal epoch update against the provided CelesTrak-style rows.

    **Test Description**

    This test uses the provided CelesTrak CSV rows and validates output values
    at a future epoch against the supplied ground-truth weather channel values.

    **Description of Variables Being Tested**

    The test checks all 23 outputs in ``swDataOutMsgs`` against known values for
    the epoch ``2026-03-04 21:48:00`` starting the simulation at ``2026-03-03 18:48:00``
    and propagating the simulation for 1 day and 3 hours.
    """

    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    unit_task_name = "unitTask"
    unit_process_name = "unitProcess"

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess(unit_process_name)
    proc.addTask(sim.CreateNewTask(unit_task_name, macros.sec2nano(3600.0)))  # [s]

    module = spaceWeatherData.SpaceWeatherData()
    module.ModelTag = "spaceWeatherData"

    with tempfile.TemporaryDirectory() as temp_dir:
        file_path = Path(temp_dir) / "space_weather.csv"
        file_path.write_text(_CELESTRAK_CSV)

        module.loadSpaceWeatherFile(str(file_path))

        timeInitString = "2026 March 03 18:48:00.000000"
        epochMsg = unitTestSupport.timeStringToGregorianUTCMsg(timeInitString)
        module.epochInMsg.subscribeTo(epochMsg)

        sim.AddModelToTask(unit_task_name, module)

        logs = []
        for msg_index in range(23):
            recorder = module.swDataOutMsgs[msg_index].recorder()
            logs.append(recorder)
            sim.AddModelToTask(unit_task_name, recorder)

        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(3600*24 + 3600*3))  # [s] 1 day and 3 hours
        sim.ExecuteSimulation()

        expected = [6.0, 4.0, 5.0, 3.0, 3.0, 7.0, 6.0, 15.0, 9.0,
                    48.0, 18.0, 18.0, 7.0, 9.0, 7.0, 12.0,
                    3.0, 5.0, 3.0, 4.0, 6.0, 138.5, 143.5]

        for msg_index in range(23):
            assert abs(logs[msg_index].dataValue[-1] - expected[msg_index]) < 1e-12


def test_space_weather_data_rejects_date_with_trailing_chars():
    """Validate that a DATE token with trailing characters is rejected.

    **Test Description**

    This test writes a CSV whose only data row has ``2026-03-04junk`` in the
    DATE column.  The parser must reject the row; the table is left empty and
    ``loadSpaceWeatherFile`` emits ``BSK_ERROR``.

    **Description of Variables Being Tested**

    ``BasiliskError`` raised during ``loadSpaceWeatherFile`` with the message
    ``"No valid weather rows were loaded"``.
    """

    module = spaceWeatherData.SpaceWeatherData()
    module.ModelTag = "spaceWeatherData"
    module.bskLogger.setLogLevel(bskLogging.BSK_ERROR)

    csv_text = "\n".join([
        "DATE,AP1,AP2,AP3,AP4,AP5,AP6,AP7,AP8,AP_AVG,F10.7_OBS,F10.7_OBS_CENTER81",
        "2026-03-04junk,9,15,6,7,3,3,5,4,6,141.2,138.5",
    ])

    with tempfile.TemporaryDirectory() as temp_dir:
        file_path = Path(temp_dir) / "space_weather.csv"
        file_path.write_text(csv_text)

        with pytest.raises(BasiliskError, match="No valid weather rows were loaded"):
            module.loadSpaceWeatherFile(str(file_path))


def test_space_weather_data_rejects_numeric_with_trailing_chars():
    """Validate that a numeric field with trailing characters is rejected.

    **Test Description**

    This test writes a CSV whose only data row has ``9abc`` in the ``AP1``
    column.  The strict numeric parser must reject the row; the table is left
    empty and ``loadSpaceWeatherFile`` emits ``BSK_ERROR``.

    **Description of Variables Being Tested**

    ``BasiliskError`` raised during ``loadSpaceWeatherFile`` with the message
    ``"No valid weather rows were loaded"``.
    """

    module = spaceWeatherData.SpaceWeatherData()
    module.ModelTag = "spaceWeatherData"
    module.bskLogger.setLogLevel(bskLogging.BSK_ERROR)

    csv_text = "\n".join([
        "DATE,AP1,AP2,AP3,AP4,AP5,AP6,AP7,AP8,AP_AVG,F10.7_OBS,F10.7_OBS_CENTER81",
        "2026-03-04,9abc,15,6,7,3,3,5,4,6,141.2,138.5",
    ])

    with tempfile.TemporaryDirectory() as temp_dir:
        file_path = Path(temp_dir) / "space_weather.csv"
        file_path.write_text(csv_text)

        with pytest.raises(BasiliskError, match="No valid weather rows were loaded"):
            module.loadSpaceWeatherFile(str(file_path))


_MINIMAL_HEADER = "DATE,AP1,AP2,AP3,AP4,AP5,AP6,AP7,AP8,AP_AVG,F10.7_OBS,F10.7_OBS_CENTER81"
_VALID_ROW = "2026-03-04,9,15,6,7,3,3,5,4,6,141.2,138.5"


def _load_single_row_csv(row: str) -> spaceWeatherData.SpaceWeatherData:
    """Helper: write a single-data-row CSV and call loadSpaceWeatherFile."""
    module = spaceWeatherData.SpaceWeatherData()
    module.ModelTag = "spaceWeatherData"
    module.bskLogger.setLogLevel(bskLogging.BSK_ERROR)
    csv_text = _MINIMAL_HEADER + "\n" + row
    with tempfile.TemporaryDirectory() as temp_dir:
        file_path = Path(temp_dir) / "sw.csv"
        file_path.write_text(csv_text)
        module.loadSpaceWeatherFile(str(file_path))
    return module


# ---------------------------------------------------------------------------
# Calendar-date validation
# ---------------------------------------------------------------------------

def test_invalid_date_april_31_rejected():
    """April 31 does not exist — the row must be rejected.

    **Test Description**

    A CSV with a single row dated ``2026-04-31`` is loaded.  April has only
    30 days, so the parser must reject the row and emit ``BSK_ERROR`` with
    ``"No valid weather rows were loaded"``.

    **Description of Variables Being Tested**

    ``BasiliskError`` raised during ``loadSpaceWeatherFile``.
    """
    with pytest.raises(BasiliskError, match="No valid weather rows were loaded"):
        _load_single_row_csv("2026-04-31,9,15,6,7,3,3,5,4,6,141.2,138.5")


def test_invalid_date_november_31_rejected():
    """November 31 does not exist — the row must be rejected.

    **Test Description**

    A CSV with a single row dated ``2026-11-31`` is loaded.  November has only
    30 days, so the parser must reject the row and emit ``BSK_ERROR``.

    **Description of Variables Being Tested**

    ``BasiliskError`` raised during ``loadSpaceWeatherFile``.
    """
    with pytest.raises(BasiliskError, match="No valid weather rows were loaded"):
        _load_single_row_csv("2026-11-31,9,15,6,7,3,3,5,4,6,141.2,138.5")


def test_invalid_date_feb_30_rejected():
    """February 30 never exists — the row must be rejected.

    **Test Description**

    A CSV with a single row dated ``2024-02-30`` is loaded.  Even in the
    leap year 2024, February has only 29 days, so the parser must reject it.

    **Description of Variables Being Tested**

    ``BasiliskError`` raised during ``loadSpaceWeatherFile``.
    """
    with pytest.raises(BasiliskError, match="No valid weather rows were loaded"):
        _load_single_row_csv("2024-02-30,9,15,6,7,3,3,5,4,6,141.2,138.5")


def test_invalid_date_feb_29_non_leap_rejected():
    """February 29 in a non-leap year must be rejected.

    **Test Description**

    A CSV with a single row dated ``2025-02-29`` is loaded.  2025 is not a
    leap year (not divisible by 4), so February has only 28 days.

    **Description of Variables Being Tested**

    ``BasiliskError`` raised during ``loadSpaceWeatherFile``.
    """
    with pytest.raises(BasiliskError, match="No valid weather rows were loaded"):
        _load_single_row_csv("2025-02-29,9,15,6,7,3,3,5,4,6,141.2,138.5")


def test_invalid_date_feb_29_century_non_leap_rejected():
    """February 29 in a century year that is not a 400-multiple must be rejected.

    **Test Description**

    2100 is divisible by 4 but not by 400, so it is not a leap year and
    February has only 28 days.

    **Description of Variables Being Tested**

    ``BasiliskError`` raised during ``loadSpaceWeatherFile``.
    """
    with pytest.raises(BasiliskError, match="No valid weather rows were loaded"):
        _load_single_row_csv("2100-02-29,9,15,6,7,3,3,5,4,6,141.2,138.5")


def test_valid_date_feb_29_leap_accepted():
    """February 29 in a leap year must be accepted.

    **Test Description**

    A four-row CSV that includes ``2024-02-29`` (2024 is a leap year) is
    loaded and the simulation is run with the epoch on ``2024-03-02 12:00:00``
    so that the look-back window covers 2024-02-29.  The table must load
    successfully and produce a finite ``ap_24_0`` output.

    **Description of Variables Being Tested**

    ``swDataOutMsgs[0].dataValue`` is non-zero after a successful look-up.
    """
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    unit_task_name = "unitTask"
    unit_process_name = "unitProcess"

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess(unit_process_name)
    proc.addTask(sim.CreateNewTask(unit_task_name, macros.sec2nano(1.0)))

    module = spaceWeatherData.SpaceWeatherData()
    module.ModelTag = "spaceWeatherData"

    # Build a 4-row window that straddles the leap day.
    csv_text = "\n".join([
        _MINIMAL_HEADER,
        "2024-02-28,1,2,3,4,5,6,7,8,4,130.0,128.0",
        "2024-02-29,9,10,11,12,13,14,15,16,12,131.0,129.0",
        "2024-03-01,17,18,19,20,21,22,23,24,20,132.0,130.0",
        "2024-03-02,25,26,27,28,29,30,31,32,28,133.0,131.0",
    ])

    with tempfile.TemporaryDirectory() as temp_dir:
        file_path = Path(temp_dir) / "sw_leap.csv"
        file_path.write_text(csv_text)
        module.loadSpaceWeatherFile(str(file_path))

        epochMsg = unitTestSupport.timeStringToGregorianUTCMsg("2024 March 02 12:00:00.000000")
        module.epochInMsg.subscribeTo(epochMsg)
        sim.AddModelToTask(unit_task_name, module)

        recorder = module.swDataOutMsgs[0].recorder()
        sim.AddModelToTask(unit_task_name, recorder)

        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(1.0))
        sim.ExecuteSimulation()

        # AP_AVG for 2024-03-02 is 28
        assert abs(recorder.dataValue[0] - 28.0) < 1e-12


def test_valid_date_feb_29_quad_century_leap_accepted():
    """February 29 in a 400-multiple year (true leap) must be accepted.

    **Test Description**

    2000 is divisible by 400 and therefore is a leap year.  A single row
    dated ``2000-02-29`` must not be rejected by the date validator.
    The test loads a four-row table that includes this date and verifies
    ``loadSpaceWeatherFile`` succeeds (no exception raised).

    **Description of Variables Being Tested**

    No exception from ``loadSpaceWeatherFile``.
    """
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)

    csv_text = "\n".join([
        _MINIMAL_HEADER,
        "2000-02-27,1,2,3,4,5,6,7,8,4,130.0,128.0",
        "2000-02-28,9,10,11,12,13,14,15,16,12,131.0,129.0",
        "2000-02-29,17,18,19,20,21,22,23,24,20,132.0,130.0",
        "2000-03-01,25,26,27,28,29,30,31,32,28,133.0,131.0",
    ])

    module = spaceWeatherData.SpaceWeatherData()
    module.ModelTag = "spaceWeatherData"
    with tempfile.TemporaryDirectory() as temp_dir:
        file_path = Path(temp_dir) / "sw_2000_leap.csv"
        file_path.write_text(csv_text)
        # Must not raise
        module.loadSpaceWeatherFile(str(file_path))


def test_invalid_date_day_zero_rejected():
    """Day 0 is never valid — the row must be rejected.

    **Test Description**

    A CSV with a single row dated ``2026-03-00`` is loaded.  Day-of-month
    zero is out of range for every month.

    **Description of Variables Being Tested**

    ``BasiliskError`` raised during ``loadSpaceWeatherFile``.
    """
    with pytest.raises(BasiliskError, match="No valid weather rows were loaded"):
        _load_single_row_csv("2026-03-00,9,15,6,7,3,3,5,4,6,141.2,138.5")


def test_invalid_date_month_zero_rejected():
    """Month 0 is never valid — the row must be rejected.

    **Test Description**

    A CSV with a single row dated ``2026-00-15`` is loaded.  Month zero is
    out of the valid [1, 12] range.

    **Description of Variables Being Tested**

    ``BasiliskError`` raised during ``loadSpaceWeatherFile``.
    """
    with pytest.raises(BasiliskError, match="No valid weather rows were loaded"):
        _load_single_row_csv("2026-00-15,9,15,6,7,3,3,5,4,6,141.2,138.5")


def test_invalid_date_month_13_rejected():
    """Month 13 is never valid — the row must be rejected.

    **Test Description**

    A CSV with a single row dated ``2026-13-01`` is loaded.  Month 13 is out
    of the valid [1, 12] range.

    **Description of Variables Being Tested**

    ``BasiliskError`` raised during ``loadSpaceWeatherFile``.
    """
    with pytest.raises(BasiliskError, match="No valid weather rows were loaded"):
        _load_single_row_csv("2026-13-01,9,15,6,7,3,3,5,4,6,141.2,138.5")


# ---------------------------------------------------------------------------
# Non-finite numeric values
# ---------------------------------------------------------------------------

def test_failed_reload_preserves_previous_table():
    """A failed second loadSpaceWeatherFile call must not destroy the first table.

    **Test Description**

    A valid CSV is loaded first, giving a working weather table.  A second
    call is then made with a file that has a missing required column.  The
    second load must fail (emitting ``BSK_ERROR``) while leaving the module
    still able to serve data from the first load.

    **Description of Variables Being Tested**

    ``swDataOutMsgs[0].dataValue`` equals the expected AP_AVG value from the
    first table after the failed reload.
    """
    bskLogging.setDefaultLogLevel(bskLogging.BSK_WARNING)
    unit_task_name = "unitTask"
    unit_process_name = "unitProcess"

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess(unit_process_name)
    proc.addTask(sim.CreateNewTask(unit_task_name, macros.sec2nano(1.0)))

    module = spaceWeatherData.SpaceWeatherData()
    module.ModelTag = "spaceWeatherData"

    bad_csv = "\n".join([
        "DATE,AP1,AP2,AP3,AP4,AP5,AP6,AP7,AP8,AP_AVG,F10.7_OBS",  # F10.7_OBS_CENTER81 missing
        "2026-03-04,9,15,6,7,3,3,5,4,6,141.2",
    ])

    with tempfile.TemporaryDirectory() as temp_dir:
        good_path = Path(temp_dir) / "good.csv"
        bad_path = Path(temp_dir) / "bad.csv"
        good_path.write_text(_CELESTRAK_CSV)
        bad_path.write_text(bad_csv)

        module.loadSpaceWeatherFile(str(good_path))

        with pytest.raises(BasiliskError):
            module.loadSpaceWeatherFile(str(bad_path))

        # Table from the good file must still be intact
        epochMsg = unitTestSupport.timeStringToGregorianUTCMsg("2026 March 04 21:48:00.000000")
        module.epochInMsg.subscribeTo(epochMsg)
        sim.AddModelToTask(unit_task_name, module)
        recorder = module.swDataOutMsgs[0].recorder()
        sim.AddModelToTask(unit_task_name, recorder)

        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(1.0))
        sim.ExecuteSimulation()

        assert abs(recorder.dataValue[0] - 6.0) < 1e-12


def test_nan_in_ap_field_rejected():
    """A NaN string in an AP column must be rejected.

    **Test Description**

    A CSV row with ``nan`` in the ``AP1`` column is loaded.  The strict
    numeric parser must reject the non-finite value, leaving the table empty.

    **Description of Variables Being Tested**

    ``BasiliskError`` raised during ``loadSpaceWeatherFile``.
    """
    with pytest.raises(BasiliskError, match="No valid weather rows were loaded"):
        _load_single_row_csv("2026-03-04,nan,15,6,7,3,3,5,4,6,141.2,138.5")


def test_inf_in_f107_field_rejected():
    """An Inf string in an F10.7 column must be rejected.

    **Test Description**

    A CSV row with ``inf`` in the ``F10.7_OBS`` column is loaded.  The
    strict numeric parser must reject the non-finite value.

    **Description of Variables Being Tested**

    ``BasiliskError`` raised during ``loadSpaceWeatherFile``.
    """
    with pytest.raises(BasiliskError, match="No valid weather rows were loaded"):
        _load_single_row_csv("2026-03-04,9,15,6,7,3,3,5,4,6,inf,138.5")


def test_neg_inf_in_ap_avg_field_rejected():
    """-Inf in the AP_AVG column must be rejected.

    **Test Description**

    A CSV row with ``-inf`` in ``AP_AVG`` is loaded.  Negative infinity is
    non-finite and must be rejected by the strict numeric parser.

    **Description of Variables Being Tested**

    ``BasiliskError`` raised during ``loadSpaceWeatherFile``.
    """
    with pytest.raises(BasiliskError, match="No valid weather rows were loaded"):
        _load_single_row_csv("2026-03-04,9,15,6,7,3,3,5,4,-inf,141.2,138.5")


if __name__ == "__main__":
    test_space_weather_data_celestrak_example_columns()
    test_space_weather_data_stops_at_first_invalid_row()
    test_space_weather_data_missing_required_column()
    test_space_weather_data_duplicate_date_rows()
    test_space_weather_data_unsorted_rows()
    test_reset_error_when_epoch_before_table()
    test_stale_output_retained_on_missing_day()
    test_space_weather_data_rejects_date_with_trailing_chars()
    test_space_weather_data_rejects_numeric_with_trailing_chars()
    test_invalid_date_april_31_rejected()
    test_invalid_date_november_31_rejected()
    test_invalid_date_feb_30_rejected()
    test_invalid_date_feb_29_non_leap_rejected()
    test_invalid_date_feb_29_century_non_leap_rejected()
    test_valid_date_feb_29_leap_accepted()
    test_valid_date_feb_29_quad_century_leap_accepted()
    test_invalid_date_day_zero_rejected()
    test_invalid_date_month_zero_rejected()
    test_invalid_date_month_13_rejected()
    test_nan_in_ap_field_rejected()
    test_inf_in_f107_field_rejected()
    test_neg_inf_in_ap_avg_field_rejected()
    test_failed_reload_preserves_previous_table()
