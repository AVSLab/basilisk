/*
 ISC License

 Copyright (c) 2026, PIC4SeR & AVS Lab, Politecnico di Torino & Argotec S.R.L., University of Colorado Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#include "simulation/environment/spaceWeatherData/spaceWeatherData.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {
/*! Remove leading and trailing ASCII spaces from a CSV token.
    @param token Raw token string extracted from a CSV field.
    @return Trimmed copy of the token, or an empty string if the token is all spaces.
*/
std::string trimToken(const std::string& token)
{
    const std::string whiteSpace = " \t\r";
    const size_t firstChar = token.find_first_not_of(whiteSpace);
    if (firstChar == std::string::npos) {
        return "";
    }
    const size_t lastChar = token.find_last_not_of(whiteSpace);
    return token.substr(firstChar, lastChar - firstChar + 1);
}

/*! Split a single CSV line on commas and trim each resulting token.
    @param line One text line from a CSV file.
    @return Ordered vector of trimmed field strings.
*/
std::vector<std::string> splitCsvLine(const std::string& line)
{
    std::vector<std::string> tokens{};
    std::stringstream lineStream(line);
    std::string token{};
    while (std::getline(lineStream, token, ',')) {
        tokens.push_back(trimToken(token));
    }
    return tokens;
}

/*! Parse a numeric token strictly, rejecting any trailing non-numeric characters and
    non-finite values (NaN, +/-Inf).
    @param str Trimmed CSV token to convert.
    @return Parsed finite double value.
    @throws std::invalid_argument if the token is empty, non-numeric, has trailing characters,
            or resolves to NaN or infinity.
    @throws std::out_of_range if the value is outside the representable double range.
*/
double parseDouble(const std::string& str)
{
    size_t pos = 0;
    const double value = std::stod(str, &pos);
    if (pos != str.size()) {
        throw std::invalid_argument("trailing characters in numeric token");
    }
    if (!std::isfinite(value)) {
        throw std::invalid_argument("non-finite numeric token");
    }
    return value;
}

/*! Return true when the given year is a proleptic Gregorian leap year.
    @param year Gregorian year.
    @return True if the year is divisible by 4, except for century years not divisible by 400.
*/
bool isLeapYear(int year)
{
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

/*! Return the number of days in a given month of a given year.
    @param year  Gregorian year (used only to resolve February in leap years).
    @param month Month of year in the range [1, 12].
    @return Day count for the month, or 0 if month is out of [1, 12].
*/
int daysInMonth(int year, int month)
{
    static const int days[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month < 1 || month > 12) {
        return 0;
    }
    if (month == 2 && isLeapYear(year)) {
        return 29;
    }
    return days[month];
}

/*! Return the zero-based column index for a named header field.
    @param headerColumns Ordered list of column name strings from the CSV header row.
    @param key Column name to search for (case-sensitive).
    @return Non-negative index when the column is found; -1 otherwise.
*/
int findHeaderIndex(const std::vector<std::string>& headerColumns, const std::string& key)
{
    for (size_t index = 0; index < headerColumns.size(); index++) {
        if (headerColumns[index] == key) {
            return static_cast<int>(index);
        }
    }
    return -1;
}
}

/*! Constructor. Allocates the 23 heap-owned output messages and pushes their
    pointers into the swDataOutMsgs vector.
*/
SpaceWeatherData::SpaceWeatherData()
{
    for (uint64_t msgIndex = 0U; msgIndex < numSwMessages; msgIndex++) {
        this->swDataOutMsgs.push_back(new Message<SwDataMsgPayload>());
    }
}

/*! Destructor. Frees every output message owned by this module.
*/
SpaceWeatherData::~SpaceWeatherData()
{
    for (auto* outputMsg : this->swDataOutMsgs) {
        delete outputMsg;
    }
}

/*! Reset the module to its initial state.

    Verifies that epochInMsg is linked and that the space-weather table has been
    loaded.  Probes whether the simulation start date is covered by the loaded
    table and emits a BSK_ERROR immediately if it is not.  Publishes a zero-state
    message.

    @param CurrentSimNanos Current simulation time in nanoseconds.
*/
void SpaceWeatherData::Reset(uint64_t CurrentSimNanos)
{
    if (!this->epochInMsg.isLinked()) {
        this->bskLogger.bskLog(BSK_ERROR, "SpaceWeatherData.epochInMsg was not linked.");
    }
    if (this->weatherData.empty()) {
        this->bskLogger.bskLog(BSK_ERROR, "SpaceWeatherData space-weather table is empty. Call loadSpaceWeatherFile().");
    }
    std::array<double, numSwMessages> probe = {};
    if (!this->computeSwState(CurrentSimNanos, probe)) {
        this->bskLogger.bskLog(BSK_ERROR,
            "SpaceWeatherData: simulation start date is not covered by the loaded table. "
            "Check the epoch and the date range of the loaded file.");
    }

    this->publishZeroState(CurrentSimNanos);
}

/*! Update the module output messages for the current simulation time.

    Resolves the current UTC calendar date from the epoch message and elapsed
    simulation time, assembles the 23-element NRLMSISE-00 / NRLMSIS 2.0
    space-weather index vector, and writes each element to the corresponding
    output message.  If the required look-up window is not available in the
    loaded table a BSK_WARNING is logged and the previous output is retained.

    @param CurrentSimNanos Current simulation time in nanoseconds.
*/
void SpaceWeatherData::UpdateState(uint64_t CurrentSimNanos)
{
    std::array<double, numSwMessages> swState = {};

    if (!this->computeSwState(CurrentSimNanos, swState)) {
        this->bskLogger.bskLog(BSK_WARNING, "Failed to retrieve a state. Publishing the last available data."
            "\nThis can happen if the date is not in the file or if the file is empty.");
        return;
    }

    for (uint64_t msgIndex = 0U; msgIndex < numSwMessages; msgIndex++) {
        SwDataMsgPayload outMsg = this->swDataOutMsgs[msgIndex]->zeroMsgPayload;
        outMsg.dataValue = swState[msgIndex];
        this->swDataOutMsgs[msgIndex]->write(&outMsg, this->moduleID, CurrentSimNanos);
    }
}

/*! Load and validate a CelesTrak-style space-weather CSV file.

    Opens the file, parses the header row, reads data rows into DailySpaceWeather
    records until the first unparsable line, and validates that DATE values are
    strictly ascending with no duplicates.  On success the internal weather table
    is replaced with the new data.  On any failure a BSK_ERROR is logged and the
    previously loaded table (if any) is left intact.

    @param fileName Path to the space-weather CSV file (absolute or relative).
*/
void SpaceWeatherData::loadSpaceWeatherFile(const std::string& fileName)
{
    // Do not clear weatherData yet — preserve the existing table until the new
    // file is fully validated so that a failed reload does not destroy a
    // previously loaded good state.

    std::ifstream weatherFile(fileName);
    if (!weatherFile.good()) {
        this->bskLogger.bskLog(BSK_ERROR, "Could not open space-weather file: %s", fileName.c_str());
        return;
    }

    std::string line;
    if (!std::getline(weatherFile, line)) {
        this->bskLogger.bskLog(BSK_ERROR, "Space-weather file is empty: %s", fileName.c_str());
        return;
    }
    const std::vector<std::string> headerColumns = splitCsvLine(line);

    const std::vector<std::string> requiredColumns = {
        "DATE", "AP1", "AP2", "AP3", "AP4", "AP5", "AP6", "AP7", "AP8", "AP_AVG", "F10.7_OBS", "F10.7_OBS_CENTER81"
    };
    for (const auto& columnName : requiredColumns) {
        if (findHeaderIndex(headerColumns, columnName) < 0) {
            this->bskLogger.bskLog(BSK_ERROR,
                                   "Space-weather file missing required column '%s': %s",
                                   columnName.c_str(),
                                   fileName.c_str());
            return;
        }
    }

    std::vector<DailySpaceWeather> loadedData;
    while (std::getline(weatherFile, line)) {
        if (line.empty()) {
            continue;
        }
        bool parseOk = false;
        DailySpaceWeather row = parseWeatherLine(line, headerColumns, parseOk);
        if (parseOk) {
            loadedData.push_back(row);
            continue;
        }

        this->bskLogger.bskLog(BSK_WARNING,
                               "Stopped reading space-weather table at first invalid row: %s",
                               line.c_str());
        break;
    }

    if (loadedData.empty()) {
        this->bskLogger.bskLog(BSK_ERROR, "No valid weather rows were loaded from: %s", fileName.c_str());
        return;
    }

    for (size_t rowIndex = 1; rowIndex < loadedData.size(); rowIndex++) {
        if (loadedData[rowIndex - 1].dayNumber == loadedData[rowIndex].dayNumber) {
            this->bskLogger.bskLog(BSK_ERROR,
                                   "Duplicate DATE row found in space-weather table '%s'.",
                                   fileName.c_str());
            return;
        }
        if (loadedData[rowIndex - 1].dayNumber > loadedData[rowIndex].dayNumber) {
            this->bskLogger.bskLog(BSK_ERROR,
                                   "Space-weather DATE rows must be sorted in ascending order: %s",
                                   fileName.c_str());
            return;
        }
    }

    this->weatherData = std::move(loadedData);
    this->loadedFileName = fileName;
}

/*! Write a zero-valued payload to every output message.

    Called at the end of Reset() so that downstream modules
    always receive at least one valid message before the first UpdateState().

    @param CurrentSimNanos Current simulation time in nanoseconds.
*/
void SpaceWeatherData::publishZeroState(uint64_t CurrentSimNanos)
{
    for (uint64_t msgIndex = 0U; msgIndex < numSwMessages; msgIndex++) {
        SwDataMsgPayload outMsg = this->swDataOutMsgs[msgIndex]->zeroMsgPayload;
        outMsg.dataValue = 0.0;
        this->swDataOutMsgs[msgIndex]->write(&outMsg, this->moduleID, CurrentSimNanos);
    }
}

/*! Determine the current Unix day number and seconds-of-day from the epoch message
    and elapsed simulation time.

    Reads the EpochMsgPayload, converts the calendar date to a Unix day number, adds the
    epoch time-of-day and the simulation elapsed time, and decomposes the result into a
    whole-day count and a residual seconds-of-day value.

    @param CurrentSimSeconds Simulation time elapsed since BSK epoch, in seconds.
    @param[out] currentDay   Unix day number of the current UTC date (days since 1970-01-01).
    @param[out] secondsOfDay Elapsed seconds since midnight of the current UTC day [0, 86400).
*/
void SpaceWeatherData::getEpochState(double CurrentSimSeconds, int64_t& currentDay, double& secondsOfDay)
{
    if (!this->epochInMsg.isWritten()) {
        this->bskLogger.bskLog(BSK_ERROR, "SpaceWeatherData space-weather epochInMsg was not written.");
    }
    const EpochMsgPayload epochMsg = this->epochInMsg();
    int year = epochMsg.year;
    int month = epochMsg.month;
    int day = epochMsg.day;
    int hours = epochMsg.hours;
    int minutes = epochMsg.minutes;
    double seconds = epochMsg.seconds;

    const int64_t epochDay = civilToUnixDayNumber(year, static_cast<unsigned>(month), static_cast<unsigned>(day));
    const double epochSecondsOfDay = static_cast<double>(hours) * 3600.0 + static_cast<double>(minutes) * 60.0 + seconds; // [s]
    const double totalSecondsFromEpoch = epochSecondsOfDay + CurrentSimSeconds; // [s]
    const int64_t dayOffset = static_cast<int64_t>(std::floor(totalSecondsFromEpoch / 86400.0)); // [day]
    secondsOfDay = totalSecondsFromEpoch - static_cast<double>(dayOffset) * 86400.0; // [s]
    secondsOfDay = std::max(0.0, secondsOfDay); // guard against negative FP residual at midnight
    currentDay = epochDay + dayOffset;
}

/*! Assemble the 23-element NRLMSISE-00 / NRLMSIS 2.0 space-weather state vector.

    Resolves the current UTC date and 3-hour Ap bin, retrieves the four required
    consecutive calendar-day records from the in-memory table, and fills the output
    array according to the channel mapping described in the module documentation.

    @param CurrentSimNanos Current simulation time in nanoseconds.
    @param[out] swState    23-element array to be populated with space-weather index values.
    @return True if all required table entries were found and the state was assembled
            successfully; false if any required day is missing or the table is empty.
*/
bool SpaceWeatherData::computeSwState(uint64_t CurrentSimNanos, std::array<double, numSwMessages>& swState)
{
    if (this->weatherData.empty()) {
        return false;
    }

    const double simTimeSeconds = static_cast<double>(CurrentSimNanos) * 1e-9; // [s]
    int64_t currentDay = 0; // [day]
    double secondsOfDay = 0.0; // [s]
    this->getEpochState(simTimeSeconds, currentDay, secondsOfDay);

    auto getDayEntry = [&](int64_t dayNumber, DailySpaceWeather& output) -> bool {
        const auto iterator = std::lower_bound(
            this->weatherData.begin(), this->weatherData.end(), dayNumber,
            [](const DailySpaceWeather& candidate, int64_t key) { return candidate.dayNumber < key; });
        if (iterator == this->weatherData.end() || iterator->dayNumber != dayNumber) {
            return false;
        }
        output = *iterator;
        return true;
    };

    const double currentDayHours = secondsOfDay / 3600.0; // [hr]
    const int currentApBin = static_cast<int>(std::floor(currentDayHours / 3.0)); // [count]

    DailySpaceWeather day0 = {};
    DailySpaceWeather dayMinus1 = {};
    DailySpaceWeather dayMinus2 = {};
    DailySpaceWeather dayMinus3 = {};
    if (!getDayEntry(currentDay, day0) || !getDayEntry(currentDay - 1, dayMinus1)
        || !getDayEntry(currentDay - 2, dayMinus2)) {
        return false;
    }
    /* D-3 is only needed when the 20-step look-back crosses a third day boundary,
       which occurs when the current 3-hour bin index is less than 3 (before 09:00 UTC). */
    const bool needsDayMinus3 = (currentApBin < 3);
    if (needsDayMinus3 && !getDayEntry(currentDay - 3, dayMinus3)) {
        return false;
    }

    const std::array<std::array<double, numApPerDay>, 4> apByDay = {
        day0.ap3Hr,
        dayMinus1.ap3Hr,
        dayMinus2.ap3Hr,
        dayMinus3.ap3Hr
    };

    /* MSIS channel mapping uses AP_AVG for ap_24_0 and then 20 AP samples spaced 3 hours apart
       starting at the current 3-hour bin and stepping backward across prior days. */
    swState[0] = day0.apAvg;
    for (int apIndex = 0; apIndex < 20; apIndex++) {
        int apBin = currentApBin - apIndex;
        int dayOffset = 0;
        while (apBin < 0) {
            apBin += static_cast<int>(numApPerDay);
            dayOffset += 1;
        }
        if (dayOffset > 3 || apBin >= static_cast<int>(numApPerDay)) {
            return false;
        }
        swState[1 + apIndex] = apByDay[dayOffset][static_cast<size_t>(apBin)];
    }
    swState[21] = day0.f107ObsCenter81;
    swState[22] = dayMinus1.f107Obs;

    return true;
}

/*! Convert a proleptic Gregorian calendar date to a signed Unix day number.

    Computes the number of whole days elapsed since 1970-01-01
    (which maps to day number 0).  The result is negative for dates
    before the Unix epoch.

    @param year  Gregorian year (may be negative).
    @param month Month of year in the range [1, 12].
    @param day   Day of month; valid range is [1, 28], [1, 29], [1, 30], or [1, 31]
                 depending on the month and whether year is a leap year.
    @return Signed count of days since 1970-01-01.
*/
int64_t SpaceWeatherData::civilToUnixDayNumber(int year, unsigned month, unsigned day)
{
    year -= (month <= 2U);
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(year - era * 400);                  // [year] year-of-era
    const unsigned doy = (153U * (month + (month > 2U ? static_cast<unsigned>(-3) : 9U)) + 2U) / 5U + day - 1U; // [day]
    const unsigned doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;                 // [day] day-of-era
    return static_cast<int64_t>(era) * 146097 + static_cast<int64_t>(doe) - 719468;
}

/*! Parse one data line of the space-weather CSV into a DailySpaceWeather record.

    Splits the line on commas, extracts the DATE field and all required numeric
    columns, and populates the output record.  Sets parseOk to false and returns a
    default-constructed record if any field is missing, empty, or non-numeric.

    @param line          One non-empty data line from the CSV file (no header).
    @param headerColumns Ordered list of column name strings from the CSV header row.
    @param[out] parseOk  Set to true when all required fields were parsed successfully;
                         false otherwise.
    @return Populated DailySpaceWeather record on success, or a default-constructed
            record on failure.
*/
SpaceWeatherData::DailySpaceWeather SpaceWeatherData::parseWeatherLine(const std::string& line,
                                                                       const std::vector<std::string>& headerColumns,
                                                                       bool& parseOk)
{
    parseOk = false;
    DailySpaceWeather row = {};

    const std::vector<std::string> values = splitCsvLine(line);
    const int dateIndex = findHeaderIndex(headerColumns, "DATE");
    const int apAvgIndex = findHeaderIndex(headerColumns, "AP_AVG");
    const int f107ObsIndex = findHeaderIndex(headerColumns, "F10.7_OBS");
    const int f107CenterIndex = findHeaderIndex(headerColumns, "F10.7_OBS_CENTER81");
    if (dateIndex < 0 || apAvgIndex < 0 || f107ObsIndex < 0 || f107CenterIndex < 0) {
        return row;
    }

    const int maxIndex = std::max(std::max(dateIndex, apAvgIndex), std::max(f107ObsIndex, f107CenterIndex));
    if (static_cast<int>(values.size()) <= maxIndex) {
        return row;
    }

    int year = 0;
    int month = 0;
    int day = 0;
    char extra;
    if (std::sscanf(values[dateIndex].c_str(), "%d-%d-%d%c", &year, &month, &day, &extra) != 3) {
        return row;
    }
    if (month < 1 || month > 12 || day < 1 || day > daysInMonth(year, month)) {
        return row;
    }

    if (values[apAvgIndex].empty() || values[f107ObsIndex].empty() || values[f107CenterIndex].empty()) {
        return row;
    }

    std::array<double, numApPerDay> apValues = {};
    for (uint64_t apIndex = 0U; apIndex < numApPerDay; apIndex++) {
        const std::string key = "AP" + std::to_string(apIndex + 1U);
        const int keyIndex = findHeaderIndex(headerColumns, key);
        if (keyIndex < 0 || static_cast<int>(values.size()) <= keyIndex) {
            return row;
        }
        if (values[keyIndex].empty()) {
            return row;
        }
    }

    try {
        for (uint64_t apIndex = 0U; apIndex < numApPerDay; apIndex++) {
            const std::string key = "AP" + std::to_string(apIndex + 1U);
            const int keyIndex = findHeaderIndex(headerColumns, key);
            apValues[apIndex] = parseDouble(values[keyIndex]);
        }

        row.dayNumber = civilToUnixDayNumber(year, static_cast<unsigned>(month), static_cast<unsigned>(day));
        row.ap3Hr = apValues;
        row.apAvg = parseDouble(values[apAvgIndex]);
        row.f107Obs = parseDouble(values[f107ObsIndex]);
        row.f107ObsCenter81 = parseDouble(values[f107CenterIndex]);
    } catch (const std::invalid_argument&) {
        return row;
    } catch (const std::out_of_range&) {
        return row;
    }

    parseOk = true;
    return row;
}
