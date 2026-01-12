/*
 ISC License

 Copyright (c) 2025, Department of Engineering Cybernetics, NTNU, Norway

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

// Do brief documentation here about the file
/*! @brief Simplified brightness temperature models for solar system bodies
*/

#ifndef BRIGHTNESS_TEMPERATURE_SOLAR_SYSTEM_H
#define BRIGHTNESS_TEMPERATURE_SOLAR_SYSTEM_H

#include <string>
#include <cmath>
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/bskLogging.h"

/**
 * @brief The library: "getBrightnessTemperatureFromData" provides brightness temperature of celestial bodies in the solar system based on ITU-R P.372-17 (08/2024)
 *        link: https://www.itu.int/rec/R-REC-P.372
 *        This should be expanded for more celestial bodies as needed.
 * @param celestialBody Name of the celestial body ("Sun", "Earth", "Moon", "Galaxy")
 * @param frequency_Hz  Frequency in Hz [Hz]
 * @return              Brightness temperature in Kelvin [K]
 */
inline double getBrightnessTemperatureFromData(const std::string& celestialBody, double frequency_Hz) {
    static BSKLogger bskLogger;
    constexpr double T_0 = 290.0;                                                        // [K]  Reference temperature according to ITU-R P.372-17
    if (celestialBody == "Sun") {
        // Sun brightness temperature modeled after ITU-R P.372-17 Figure 10
        if (frequency_Hz < 5e7) {                                                        // ->   f < 50 MHz
            bskLogger.bskLog(BSK_ERROR, "brightnessTemperatureSolarSystem: Frequency out of range for Sun brightness temperature model");
            return 1e6;                                                                  // [K]  Return worst case value from the valid frequency range
        } else if (frequency_Hz >= 5e7 && frequency_Hz < 2e8) {                          // ->   50 MHz <= f < 200 MHz
            return 1e6;                                                                  // [K]  (ITU-R P.372-17, 4.1 Brightness temperature due to extra-terrestrial sources)
        } else if (frequency_Hz >= 2e8 && frequency_Hz < 1e9) {                          // ->   200 MHz <= f < 1 GHz
            return 5e5;                                                                  // [K]  (ITU-R P.372-17, 4.1 Brightness temperature due to extra-terrestrial sources)
        } else if (frequency_Hz >= 1e9 && frequency_Hz < 1.4e9) {                        // ->   1 GHz <= f < 1.4 GHz
            return pow(10, 5.4525) * pow((frequency_Hz * 1e-9), -1.6044);                // [K] (ITU-R P.372-17, Figure 10 "Quiet Sun")
        } else if (frequency_Hz >= 1.4e9 && frequency_Hz < 1e10) {                       // ->   1.4 GHz <= f < 10 GHz
            return pow(10, 4.8891) * pow((frequency_Hz * 1e-9), -0.78897);               // [K]  (ITU-R P.372-17, Figure 10 "Quiet Sun")
        } else if (frequency_Hz >= 1e10 && frequency_Hz < 1e11) {                        // ->   10 GHz <= f < 100 GHz
            return pow(10, 4.1985) * pow((frequency_Hz * 1e-9), -0.14);                  // [K]  (ITU-R P.372-17, Figure 10 "Quiet Sun")
        } else if (frequency_Hz >= 1e11) {                                               // ->   f >= 100 GHz
            return 5.0e3;                                                                // [K]  (ITU-R P.372-17, Figure 10 "Quiet Sun")
        } else {                                                                         // ->   Fallback case (should never be reached)
            bskLogger.bskLog(BSK_ERROR, "brightnessTemperatureSolarSystem: Frequency out of range for Sun brightness temperature model");
            return 1e6;                                                                  // [K]  Return worst case value for the frequency range
        }
    } else if (celestialBody == "Earth") {
        // Simplified model for Earth brightness temperature
        if (frequency_Hz >= 2e9) {                                                       // ->   Above 2 GHz
            return 300.0;                                                                // [K]  worst case value
        } else {
            return 0.0;                                                                  // [K]  Below 2 GHz Earth is neglected
        }
    } else if (celestialBody == "Moon") {
        if (frequency_Hz >= 2e9) {                                                       // ->   f >= 2 GHz
            return 280.0;                                                                // [K]  (ITU-R P.372-17, Section 4.1 "worst case, full moon")
        } else {
            return 0.0;                                                                  // [K]  Below 2 GHz moon is neglected (ITU-R P.372-17, Section 4.1)
        }
    } else if (celestialBody == "Galaxy") {
        if (frequency_Hz <= 1e7) {                                                       // ->   f <= 10 MHz
            bskLogger.bskLog(BSK_ERROR, "brightnessTemperatureSolarSystem: Invalid frequency for Galaxy brightness temperature model");
            return 864.0;                                                                // [K]  Return worst case for valid frequency range
        } else if (frequency_Hz > 1e7 && frequency_Hz <= 1e8) {                          // ->   100 MHz   >= f >   10 MHz
            double F_am_dB       = 52.0 - 23.0 * log10(frequency_Hz * 1e-6);             // [dB] (ITU-R P.372-17, eq: 15) -> 100MHz=864K ; 10MHz=2.3e5 ; 1MHz=4.6e6K
            double T_background  = (pow(10, (F_am_dB/10)) - 1) * T_0;                    // [K]  Convert Noise Power [dB] to Noise Temperature [K] -> 10^(F_am_dB/10) -1) * 290.0
            return (T_background > CMB_TEMPERATURE) ? T_background : CMB_TEMPERATURE;    // [K]  Ensure non-negative brightness temperature
        } else if (frequency_Hz > 1e8 && frequency_Hz <= 1.16e9) {                       // ->   1.16 GHz  >= f >   100 MHz
            return pow(10, 0.99125) * pow((frequency_Hz * 1e-9), -2.94474);              // [K]  (ITU-R P.372-17, Figure 10)
        } else if (frequency_Hz > 1.16e9) {                                              // ->   f > 1.16 GHz
            return CMB_TEMPERATURE;                                                      // [K]  Above 1.16 GHz, galactic contribution is negligible; only CMB remains (ITU-R P.372-17, Figure 10)
        } else {                                                                         // ->   Frequency out of range (This should never happen)
            bskLogger.bskLog(BSK_ERROR, "brightnessTemperatureSolarSystem: Frequency out of range for Galaxy brightness temperature model");
            return CMB_TEMPERATURE;                                                      // [K]  Return CMB value for invalid frequency
        }
    } else {                                                                             // ->   Celestial body not implemented
        bskLogger.bskLog(BSK_ERROR, "brightnessTemperatureSolarSystem: Celestial body not (yet) implemented");
        return CMB_TEMPERATURE;                                                          // [K]  Return CMB value for unimplemented celestial body
    }
}

#endif // BRIGHTNESS_TEMPERATURE_SOLAR_SYSTEM_H
