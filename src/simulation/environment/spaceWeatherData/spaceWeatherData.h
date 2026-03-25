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

#ifndef MSIS_SPACE_WEATHER_H
#define MSIS_SPACE_WEATHER_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/EpochMsgPayload.h"
#include "architecture/msgPayloadDefC/SwDataMsgPayload.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief CelesTrak space-weather data publisher for :ref:`msisAtmosphere`. */
class SpaceWeatherData: public SysModel {
public:
    SpaceWeatherData();
    ~SpaceWeatherData();

    SpaceWeatherData(const SpaceWeatherData&) = delete;
    SpaceWeatherData& operator=(const SpaceWeatherData&) = delete;

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

    void loadSpaceWeatherFile(const std::string& fileName);

public:
    ReadFunctor<EpochMsgPayload> epochInMsg{};               //!< epoch input message
    std::vector<Message<SwDataMsgPayload>*> swDataOutMsgs{}; //!< 23-element MSIS SW output message vector
    BSKLogger bskLogger{};                                   //!< BSK logging helper

private:
    static constexpr uint64_t numSwMessages = 23U; //!< [count] number of MSIS weather channels
    static constexpr uint64_t numApPerDay = 8U;    //!< [count] number of three-hour Ap entries per day

    struct DailySpaceWeather {
        int64_t dayNumber{};                     //!< [day] days since Unix epoch
        std::array<double, numApPerDay> ap3Hr{}; //!< [-] AP1..AP8 values
        double apAvg{};                          //!< [-] daily Ap average
        double f107Obs{};                        //!< [sfu] daily observed F10.7
        double f107ObsCenter81{};                //!< [sfu] centered 81-day F10.7 average
    };

    std::vector<DailySpaceWeather> weatherData{}; //!< parsed weather table
    std::string loadedFileName{};                 //!< loaded CSV path

    void publishZeroState(uint64_t CurrentSimNanos);
    bool computeSwState(uint64_t CurrentSimNanos, std::array<double, numSwMessages>& swState);
    void getEpochState(double CurrentSimSeconds, int64_t& currentDay, double& secondsOfDay);

    static int64_t civilToUnixDayNumber(int year, unsigned month, unsigned day);
    static DailySpaceWeather parseWeatherLine(const std::string& line,
                                              const std::vector<std::string>& headerColumns,
                                              bool& parseOk);
};

#endif
