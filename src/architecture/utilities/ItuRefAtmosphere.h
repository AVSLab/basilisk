/*
 ISC License

 Copyright (c) 2025, Department of Engineering Cybernetics, NTNU

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


#ifndef ITU_ATMOSPHERE_H
#define ITU_ATMOSPHERE_H

#include <cmath>
#include <algorithm>

/*! @brief ITU-R P.835-2 Reference Standard Atmosphere Implementation
 *
 * This class implements the mean annual global reference atmosphere as specified
 * in ITU-R P.835-2 recommendation for calculating gaseous attenuation along Earth-space paths.
 * The class precomputes atmospheric properties at initialization and proviedes lookup values at runtime.
 */
class ItuAtmosphere {
public:
    ItuAtmosphere();
    ~ItuAtmosphere();

    /*! @brief Static methods for accessing atmospheric temperature during runtime
    *  @param altitude Altitude in [m]
    *  @return Temperature in [K]
    */
    static double getTempISA(double altitude);
    /*! @brief Static methods for accessing atmospheric pressure during runtime
    *  @param altitude Altitude in [m]
    *  @return Pressure in [hPa]
    */
    static double getPresISA(double altitude);
    /*! @brief Static methods for accessing atmospheric water vapor density during runtime
    *  @param altitude Altitude in [m]
    *  @return Water vapor density in [g/m3]
    */
    static double getWaterVapDensityISA(double altitude);

private:
    /*! @brief find the layer index for a given altitude
    *  @param altitude_m Altitude in [m]
    *  @return Layer index
    */
    size_t findLayerIndex(double altitude_m) const;
    /*! @brief Calculate geometric height from altitude
    *  @param altitude_m Altitude in [m]
    *  @return Geometric height in [km]
    */
    double geometricHeight(double altitude_m) const;
    /*! @brief Calculate the ISA temperature
    *  @param altitude_m Altitude in [m]
    *  @return Temperature in [K]
    */
    double calcTempISA(double altitude_m) const;
    /*! @brief Calculate the ISA pressure
    *  @param altitude_m Altitude in [m]
    *  @return Pressure in [hPa]
    */
    double calcPresISA(double altitude_m) const;
    /*! @brief Calculate the ISA water vapor density
    *  @param altitude_m Altitude in [m]
    *  @return Water vapor density in [g/m3]
    */
    double calcWaterVapDensityISA(double altitude_m) const;

public:

private:
    /*! @brief Pressure constant for barometric formula [K/km]
    *  derived from g * M / R
    *  where:
    *  R = 'gas constant'      -> 8.31446[J/(mol*K)]
    *  M = 'molar mass of air' -> 28.9645[g/mol]
    *  g = "gravity"          -> 9.80665[m/s2]
    *  Source: ITU-R P.835-7
    */
    static constexpr double itu_press_const = 34.1632; // [K/km]

     /*! @brief Atmosphere layer structure for piecewise temperature profile
     *  Defines layer boundaries and temperature gradients per source: ITU-R P.835-7
     */
    struct AtmosphereLayer {
        double h;        // Altitude Hi (km)
        double dT_dh;    // Temperature gradient Li (K/km)
    };

    /*! @brief Number of atmospheric layers in the reference atmosphere; Source: ITU-R P.835-7, Table 1 */
    static constexpr size_t NUM_LAYERS = 8;

    /*! @brief Mean annual global reference atmosphere layer data
    *  Source: ITU-R P.835-7, Table 1
    */
    static constexpr AtmosphereLayer ATMOSPHERE_DATA[NUM_LAYERS] = {
        {  0.0e3,   -6.5},  // [m, K/km] 0  - 11  km
        { 11.0e3,    0.0},  // [m, K/km] 11 - 20  km
        { 20.0e3,    1.0},  // [m, K/km] 20 - 32  km
        { 32.0e3,    2.8},  // [m, K/km] 32 - 47  km
        { 47.0e3,    0.0},  // [m, K/km] 47 - 51  km
        { 51.0e3,   -2.8},  // [m, K/km] 51 - 71  km
        { 71.0e3,   -2.0},  // [m, K/km] 71 - 85  km
        { 84.852e3,  0.0}   // [m, K/km] Above 85 km (gradient not used beyond this assume const temp)
    };

    /*! @brief Ground-level standard conditions (ITU-R P.835-7) */
    static constexpr double T0   = 288.15;   ///< Standard temperature at sea level [K] (eq. 5)
    static constexpr double P0   = 1013.25;  ///< Standard pressure at sea level [hPa] (eq. 5)
    static constexpr double rho0 = 7.5;      ///< Standard water vapor density at sea level [g/m³] (eq. 7)

    /*! @brief Scale heights for exponential profiles */
    static constexpr double h0_water = 2.0;    ///< [km] Water vapor scale height (ITU-R P.835-7 eq 6)
//    static constexpr double h0_dry   = 6.0;  ///< [km] Dry atmosphere scale height (ITU-R P.835-7 eq 9)
    static constexpr double R_EARTH_ITU = 6356.766; ///< [km] Earth's radius for ITU atmosphere calculations

    ItuAtmosphere(const ItuAtmosphere&)            = delete;
    ItuAtmosphere& operator=(const ItuAtmosphere&) = delete;

    /* Singleton instance */
    static const ItuAtmosphere& instance();
};
#endif /* ITU_ATMOSPHERE_H */
