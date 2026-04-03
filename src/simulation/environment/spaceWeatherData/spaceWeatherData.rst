Executive Summary
-----------------

The ``spaceWeatherData`` module reads a CelesTrak-style space-weather CSV table and publishes the
23 :ref:`SwDataMsgPayload` output messages required by :ref:`msisAtmosphere`.  On every simulation
update step the module determines the current UTC calendar date and time from the linked
:ref:`EpochMsgPayload`, looks up the corresponding rows in the loaded table, and assembles the
NRLMSISE-00 / NRLMSIS 2.0 space-weather index vector before writing it to its output message array.

Message Connection Descriptions
---------------------------------

The following table lists all the module input and output messages.  The module msg connection is
set by the user from Python.  The msg type contains a link to the message structure definition,
while the description provides information on what this message is used for.

.. list-table:: Module I/O Messages
    :widths: 25 25 50
    :header-rows: 1

    * - Msg Variable Name
      - Msg Type
      - Description
    * - ``epochInMsg``
      - :ref:`EpochMsgPayload`
      - Simulation start epoch in UTC (year, month, day, hours, minutes, seconds).  Must be linked
        before ``Reset`` is called; an error is logged otherwise.
    * - ``swDataOutMsgs[0]``
      - :ref:`SwDataMsgPayload`
      - Daily Ap average (``AP_AVG``) of the current day — MSIS channel ``ap_24_0``.
    * - ``swDataOutMsgs[1..20]``
      - :ref:`SwDataMsgPayload`
      - Twenty consecutive 3-hour Ap values stepping back from the current bin — MSIS channels
        ``ap_3_0`` through ``ap_3_-57``.
    * - ``swDataOutMsgs[21]``
      - :ref:`SwDataMsgPayload`
      - Centered 81-day observed F10.7 of the current day (``F10.7_OBS_CENTER81``) — MSIS channel
        ``f107_1944_0``.
    * - ``swDataOutMsgs[22]``
      - :ref:`SwDataMsgPayload`
      - Observed daily F10.7 of the previous day (``F10.7_OBS``) — MSIS channel ``f107_24_-24``.

.. note::

   ``swDataOutMsgs`` is a ``std::vector`` of heap-allocated ``Message<SwDataMsgPayload>``
   pointers initialised to length 23 in the constructor.  In Python the vector elements are
   accessed by index: ``swModule.swDataOutMsgs[i]``.

Detailed Module Description
----------------------------

The ``spaceWeatherData`` module converts a Gregorian calendar date (year, month, day) to a signed
Unix day number.  The resulting integer counts whole days since the Unix epoch (1970-01-01 = 0)
and is used as the unique key when looking up rows in the loaded weather table.

On every call to ``UpdateState`` the module performs the following steps to determine the current
UTC calendar date:

1. Read :ref:`EpochMsgPayload` (``year``, ``month``, ``day``, ``hours``, ``minutes``, ``seconds``).
2. Compute :math:`D_{\text{epoch}}` from the calendar date.
3. Compute :math:`t_{\text{epoch}}` from the time-of-day fields.
4. Add the simulation time: :math:`T_{\text{total}} = t_{\text{epoch}} + t_{\text{sim}}`.
5. Extract the whole-day offset :math:`\Delta D` and residual :math:`s_{\text{day}}`.
6. Determine the current day number :math:`D_0 = D_{\text{epoch}} + \Delta D`.
7. Determine the current 3-hour Ap bin :math:`b_{\text{cur}} = \lfloor s_{\text{day}} / 10800 \rfloor`.

The table below defines the symbols used above.

.. list-table:: Symbol definitions
    :widths: 20 15 65
    :header-rows: 1

    * - Symbol
      - Unit
      - Description
    * - :math:`t_{\text{sim}}`
      - s
      - Simulation time elapsed since the BSK epoch (``CurrentSimNanos`` converted to seconds).
    * - :math:`t_{\text{epoch}}`
      - s
      - UTC seconds-of-day of the epoch (hour x 3600 + minute x 60 + second).
    * - :math:`D_{\text{epoch}}`
      - day
      - Unix day number of the epoch date (days since 1970-01-01).
    * - :math:`T_{\text{total}}`
      - s
      - Total UTC seconds elapsed from the start of the epoch calendar day:
        :math:`T_{\text{total}} = t_{\text{epoch}} + t_{\text{sim}}`.
    * - :math:`\Delta D`
      - day
      - Whole-day offset from the epoch date:
        :math:`\Delta D = \lfloor T_{\text{total}} / 86400 \rfloor`.
    * - :math:`s_{\text{day}}`
      - s
      - Seconds elapsed since midnight of the current UTC day:
        :math:`s_{\text{day}} = T_{\text{total}} - \Delta D \times 86400`.
    * - :math:`D_{0}`
      - day
      - Unix day number of the current UTC date:
        :math:`D_{0} = D_{\text{epoch}} + \Delta D`.
    * - :math:`b_{\text{cur}}`
      - —
      - Index of the current 3-hour Ap bin (0-based):
        :math:`b_{\text{cur}} = \lfloor s_{\text{day}} / 3600 / 3 \rfloor`.

The 23 output channels map directly to the ``sw`` array consumed by the ``msisAtmosphere`` module.
The mapping implemented by ``computeSwState`` is:

.. list-table:: Output channel mapping
    :widths: 12 30 58
    :header-rows: 1

    * - Channel
      - MSIS label
      - Source column and day
    * - ``swDataOutMsgs[0]``
      - ``ap_24_0``
      - ``AP_AVG`` of day :math:`D_0`
    * - ``swDataOutMsgs[1]``
      - ``ap_3_0``
      - 3-hour Ap at bin :math:`b_{\text{cur}}` of day :math:`D_0`
    * - ``swDataOutMsgs[2]``
      - ``ap_3_-3``
      - 3-hour Ap at bin :math:`b_{\text{cur}} - 1`, wrapping to :math:`D_{-1}` as needed
    * - ``swDataOutMsgs[3]``
      - ``ap_3_-6``
      - 3-hour Ap at bin :math:`b_{\text{cur}} - 2`, wrapping to :math:`D_{-1}` or :math:`D_{-2}`
    * - … (pattern continues)
      - …
      - Each subsequent channel steps back one 3-hour bin, wrapping across day boundaries up to :math:`D_{-3}`
    * - ``swDataOutMsgs[20]``
      - ``ap_3_-57``
      - 3-hour Ap at bin :math:`b_{\text{cur}} - 19`, up to day :math:`D_{-3}`
    * - ``swDataOutMsgs[21]``
      - ``f107_1944_0``
      - ``F10.7_OBS_CENTER81`` of day :math:`D_0`
    * - ``swDataOutMsgs[22]``
      - ``f107_24_-24``
      - ``F10.7_OBS`` of day :math:`D_{-1}`

The 3-hour Ap look-back (channels 1-20) is computed starting from the current 3-hour bin
:math:`b_{\text{cur}}` within day :math:`D_0`.  The module subtracts one bin index per channel.
Whenever the bin index drops below zero it is incremented by 8 (the number of 3-hour bins per
day) and the day offset is advanced by one.  If the resulting day offset exceeds 3 the look-up
fails and the entire output is set to zero for the current step.

``loadSpaceWeatherFile`` performs the following actions in order:

1. Open the file; log an error and return if the file cannot be read.
2. Parse the header row and verify that all required column names are present.  Log an error and
   return for any missing column.
3. Parse each subsequent non-empty line into a ``DailySpaceWeather`` record.  Stop at the first
   line that fails to parse (see assumption 4 in `Module Assumptions and Limitations`_).
4. Verify that the resulting list contains at least one valid row.
5. Verify that ``DATE`` values are strictly ascending and contain no duplicates.
6. If all checks pass, store the validated list and record the loaded file path.

If any validation step fails a BSK_ERROR is thrown.

During ``Reset`` the module probes whether the simulation start date is covered by the loaded
table; a ``BSK_ERROR`` is emitted immediately if it is not, rather than failing silently at run
time.  A zero-state is always published at ``Reset``.

If the current simulation date (or any of the three preceding days) is missing from the table
the module logs a warning and keeps publishing the last available data.
This can happen only during the simulation, because the first epoch is tested at ``Reset``.

Module Assumptions and Limitations
------------------------------------

#. **Required columns.** The input CSV must contain a header row with at least the following column
   names (additional columns are ignored):

   ``DATE``, ``AP1``, ``AP2``, ``AP3``, ``AP4``, ``AP5``, ``AP6``, ``AP7``, ``AP8``,
   ``AP_AVG``, ``F10.7_OBS``, ``F10.7_OBS_CENTER81``

#. **Date format.** Every ``DATE`` cell must follow the ``YYYY-MM-DD`` format.  Month values must
   be in [1, 12] and day values in [1, 31]; otherwise the row is treated as invalid.

#. **Table ordering and uniqueness.** ``DATE`` values must be strictly ascending and free of
   duplicates.  The loader returns an error and discards the entire file if either condition is
   violated.

#. **Early termination on invalid rows.** The file parser stops reading at the first row that
   cannot be fully parsed (missing, non-numeric, or numeric fields with trailing characters in AP
   or F10.7 columns, or a ``DATE`` token with trailing characters).  This is intentional: the
   monthly predicted section at the end of CelesTrak files omits AP values and should not be
   ingested.

#. **Run-time look-up window.** At each update step the module requires four consecutive calendar
   days to be present in the table: the current day, and the three preceding days.  If any of
   these four entries is absent the module outputs the preceding state and logs a warning.

#. **Epoch is UTC.** The simulation epoch provided through :ref:`EpochMsgPayload` is interpreted
   as UTC.  No leap-second or timezone correction is applied.

#. **Three-hour AP bins.** The table stores exactly 8 three-hour Ap indices per day
   (``AP1`` … ``AP8``), corresponding to the 00-03 UT, 03-06 UT, … 21-24 UT intervals.
   The module derives the current 3-hour bin from the fractional UTC time of day.

#. **No interpolation.** All space-weather indices are taken directly from the table; no
   interpolation between rows is performed.

User Guide
-----------

Obtaining a space-weather file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Download the historical + short-range predicted file directly from CelesTrak:

.. code-block:: none

    https://celestrak.org/SpaceData/SW-Last5Years.csv

The file covers the past five years of observed data followed by a short predicted section.
Because the predicted rows omit AP values, the module's early-termination parser automatically
stops before ingesting them.

For simulations that extend beyond the five-year window, use the complete historical archive:

.. code-block:: none

    https://celestrak.org/SpaceData/SW-All.csv

Basic Setup
~~~~~~~~~~~

The minimal Python setup connects the epoch message, loads the CSV file, and subscribes each
``msisAtmosphere`` input to the corresponding output:

.. code-block:: python

    from Basilisk.simulation import msisAtmosphere, spaceWeatherData
    from Basilisk.utilities import unitTestSupport, SimulationBaseClass

    scSim = SimulationBaseClass.SimBaseClass()
    dynProcess = scSim.CreateNewProcess("DynamicsProcess")
    dynTaskName = "DynamicsTask"
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, macros.sec2nano(10.0)))

    # --- Space-weather module ---
    swModule = spaceWeatherData.SpaceWeatherData()
    swModule.ModelTag = "spaceWeatherData"
    swModule.loadSpaceWeatherFile("SW-Last5Years.csv")

    # Connect the simulation epoch
    timeInitString = "2026 March 04 21:48:00.000000"
    epochMsg = unitTestSupport.timeStringToGregorianUTCMsg(timeInitString)
    swModule.epochInMsg.subscribeTo(epochMsg)

    # --- Atmosphere module ---
    atmoModule = msisAtmosphere.MsisAtmosphere()
    atmoModule.epochInMsg.subscribeTo(epochMsg)
    atmoModule.ModelTag = "msisAtmosphere"
    for msgIndex in range(23):
        atmoModule.swDataInMsgs[msgIndex].subscribeTo(swModule.swDataOutMsgs[msgIndex])

    scSim.AddModelToTask(dynTaskName, swModule)
    scSim.AddModelToTask(dynTaskName, atmoModule)

.. note::

    ``swModule`` must be added to the task **before** ``atmoModule``, or have a higher priority,
    so that the output messages are written before the atmosphere module reads them within the
    same task step.

Choosing the Correct Epoch
~~~~~~~~~~~~~~~~~~~~~~~~~~

The epoch message must represent the UTC date and time at which the simulation clock reads zero.
The module adds the BSK simulation time (in seconds) to this epoch to obtain the current UTC
instant.  Providing a wrong epoch will shift all space-weather indices in time and may cause
look-up failures if the resulting date falls outside the loaded table.

.. code-block:: python

    # Epoch at J2000.0 (2000-01-01 11:58:55.816 UTC)
    epochMsg = unitTestSupport.timeStringToGregorianUTCMsg("2000 January 01 11:58:55.816000")
    swModule.epochInMsg.subscribeTo(epochMsg)

Error Conditions and Mitigation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
    :widths: 45 55
    :header-rows: 1

    * - Symptom
      - Likely cause and remedy
    * - ``BSK_ERROR: SpaceWeatherData.epochInMsg was not linked``
      - ``epochInMsg`` was not subscribed before ``Reset``.  Subscribe to a valid
        :ref:`EpochMsgPayload` message before initialising the simulation.
    * - ``BSK_ERROR: space-weather table is empty``
      - ``loadSpaceWeatherFile`` was not called, or the call failed.  Check the file path and
        verify that the CSV contains all required columns.
    * - ``BSK_ERROR: simulation start date is not covered by the loaded space-weather table``
      - The epoch resolves to a calendar date for which the module cannot assemble a full
        look-back window (current day plus three preceding days).  Verify the epoch is
        correct and that the loaded CSV file covers the simulation start date.
    * - ``BSK_ERROR: missing required column``
      - The CSV header does not contain one of the mandatory column names.  Ensure the file was
        downloaded from CelesTrak and has not been manually edited.
    * - ``BSK_ERROR: Duplicate DATE row``
      - Two rows share the same ``DATE``.  Remove the duplicate and reload.
    * - ``BSK_ERROR: DATE rows must be sorted``
      - Rows are not in ascending date order.  Sort the file by date and reload.
    * - ``BSK_ERROR: No valid weather rows were loaded``
      - Every data row failed to parse, leaving the table empty.  Check that the CSV is a valid
        CelesTrak file and that at least the first data row is well-formed.
    * - ``BSK_WARNING: Failed to retrieve a state. Publishing the last available data. This can happen if the date is not in the file or if the file is empty.``
      - The current simulation date or one of the three preceding days is absent from the table.
        Use a longer or more recent CSV file so that the look-up window falls within the loaded
        range.
    * - ``BSK_WARNING: Stopped reading space-weather table at first invalid row``
      - A data row could not be parsed (malformed ``DATE`` token, missing field, or a numeric
        field such as an AP or F10.7 value that contains non-numeric trailing characters).  Rows
        before this point are still loaded.  Verify the file has not been manually edited.
