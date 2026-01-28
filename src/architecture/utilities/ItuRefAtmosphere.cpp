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

#include "architecture/utilities/ItuRefAtmosphere.h"
#include <cmath>
#include <algorithm>

// Constructor
ItuAtmosphere::ItuAtmosphere() {
}

// Destructor
ItuAtmosphere::~ItuAtmosphere() {
}
const ItuAtmosphere& ItuAtmosphere::instance() {
    static ItuAtmosphere inst;
    return inst;
}

// Getters
double ItuAtmosphere::getTempISA(double altitude_m) {
    return instance().calcTempISA(altitude_m);
}

double ItuAtmosphere::getPresISA(double altitude_m) {
    return instance().calcPresISA(altitude_m);
}

double ItuAtmosphere::getWaterVapDensityISA(double altitude_m) {
    return instance().calcWaterVapDensityISA(altitude_m);
}

size_t ItuAtmosphere::findLayerIndex(double altitude_m) const {
    size_t layer_idx = NUM_LAYERS - 1;
    for (size_t i = 0; i < NUM_LAYERS - 1; ++i) {
        if (altitude_m >= ATMOSPHERE_DATA[i].h &&
            altitude_m <  ATMOSPHERE_DATA[i + 1].h)
        {
            layer_idx = i;
            break;
        }
    }
    return layer_idx;
}

double ItuAtmosphere::geometricHeight(double altitude_m) const {
    // For altitudes above 84.852 km, convert geopotential height to geometric height
    return (R_EARTH_ITU * altitude_m * 1e-3) / (R_EARTH_ITU - altitude_m*1e-3); // ITU-R P.835-7 eq 1b
}

/* Calculate Temperature according to ITU-R P.835-7
*/
double ItuAtmosphere::calcTempISA(double altitude_m) const {
    // Clamp above model validity (85 km)
    if (altitude_m >= ATMOSPHERE_DATA[NUM_LAYERS - 1].h) {
        // if h above 84.852 km
        double Z = ItuAtmosphere::geometricHeight(altitude_m); // [km] Geometric height
        if (Z < 91.0) {
            return 186.8673; // [K] Constant temperature between 84.852 km and 91 km (ITU-R P.835-7 eq 4a)
        } else if (Z < 100.0) {
            return 263.1905 - 76.3232 * std::sqrt(1 - std::pow((Z - 91.0)/(19.9429),2)); // [K] Linear increase between 91 km and 100 km (ITU-R P.835-7 eq 4b)
        } else {
            return 195.08134; // [K] Above 100 km, assume constant temperature
        }
    }

    // Find which layer the altitude falls into (table in header file)
    size_t layer_idx = ItuAtmosphere::findLayerIndex(altitude_m);

    // Compute temperature at layer base: T_(layer_idx-1)
    double Ti = T0;
    for (size_t i = 0; i < layer_idx; ++i) {
        double h0      = ATMOSPHERE_DATA[i].h;          // [m] Altitude at layer "i" base
        double h1      = ATMOSPHERE_DATA[i + 1].h;      // [m] Altitude at layer "i+1" base
        double dT_dh_i = ATMOSPHERE_DATA[i].dT_dh;      // [K/km] Temperature gradient in layer "i"
        Ti += dT_dh_i * (h1 - h0) * 1e-3;               // [K] Temperature at top of layer "layer_idx-1"
    }

    // Linear variation within that layer
    double Hi    = ATMOSPHERE_DATA[layer_idx].h;        // [m] Altitude at layer "layer_idx" base
    double dT_dh = ATMOSPHERE_DATA[layer_idx].dT_dh;    // [K/km] Temperature gradient in layer "layer_idx"

    return Ti + dT_dh * (altitude_m - Hi) * 1e-3;       // [](ITU-R P.835-2 eq 2)
}

/* Calculate Pressure according to ITU-R P.835-7
 * barometric formula for each atmospheric layer
 */
double ItuAtmosphere::calcPresISA(double altitude_m) const {
    // Find which layer the altitude falls into
    size_t layer_idx = ItuAtmosphere::findLayerIndex(altitude_m);
    double dT_dh = ATMOSPHERE_DATA[layer_idx].dT_dh;
    if (layer_idx == 0) {
        // 0 km < altitude < 11 km
        return 1013.25 * pow((288.15 / (288.15 - (dT_dh * altitude_m * 1e-3))), -itu_press_const/dT_dh);             // ITU-R P.835-7 eq 3a
    } else if (layer_idx == 1) {
        // 11 km < altitude < 20 km
        return 226.3226 * exp(-0.157688 * (altitude_m * 1e-3 - 11.0));                                               // ITU-R P.835-7 eq 3b
    } else if (layer_idx == 2) {
        // 20 km < altitude < 32 km
        return 54.7498 * pow((216.65 / (216.65 + (altitude_m * 1e-3 - 20.0))), -itu_press_const);                    // ITU-R P.835-7 eq 3c
    } else if (layer_idx == 3) {
        // 32 km < altitude < 47 km
        return 8.680422 * pow((228.65 / (228.65 + (dT_dh * (altitude_m * 1e-3 - 32.0)))), -itu_press_const/dT_dh);   // ITU-R P.835-7 eq 3d
    } else if (layer_idx == 4) {
        // 47 km < altitude < 51 km
        return 1.109106 * exp(-0.126226 * (altitude_m * 1e-3 - 47.0));                                               // ITU-R P.835-7 eq 3e
    } else if (layer_idx == 5) {
        // 51 km < altitude < 71 km
        return 0.6694167 * pow((270.65 / (270.65 - (dT_dh * (altitude_m * 1e-3 - 51.0)))), -itu_press_const/dT_dh);  // ITU-R P.835-7 eq 3f
    } else if (layer_idx == 6) {
        // 71 km < altitude < 85 km
        return 0.03956649 * pow((214.65 / (214.65 - (dT_dh * (altitude_m * 1e-3 - 71.0)))), -itu_press_const/dT_dh); // ITU-R P.835-7 eq 3g
    } else {
        // altitude >= 85 km
        double Z = ItuAtmosphere::geometricHeight(altitude_m); // [km] Geometric height
        if (Z < 100.0e3) {
            return exp(95.571899 - 4.011801*Z + 6.424731*Z*Z*1e-2 - 4.789660*Z*Z*Z*1e-4 + 1.340543*Z*Z*Z*Z*1e-6);    // ITU-R P.835-7 eq 5
        } else {
            return 0.0; // Pressure above 100 km is assumed zero
        }
    }
}

/* Calculate Water Vapor Density according to ITU-R P.835-7
 * Uses exponential decay model with different scale heights
 */
double ItuAtmosphere::calcWaterVapDensityISA(double altitude_m) const {
    // Calculate geometric height
    double Z = ItuAtmosphere::geometricHeight(altitude_m); // [km] Geometric height
    double rho = rho0 * std::exp(-Z / h0_water);           // [g/m3] ITU-R P.835-2 eq 6
    double T_Z = ItuAtmosphere::calcTempISA(altitude_m);   // [K]
    double e_z = (rho*T_Z)/(216.7);                        // [hPa] Partial pressure at height Z
    double p_z = ItuAtmosphere::calcPresISA(altitude_m);   // [hPa] Total pressure at height Z
    if (e_z/p_z < 2.0e-6) {
        // Use ITU-R 835-7 eq. 8
        rho = 2.0e-6 * (p_z * 216.7)/(T_Z); // [g/m3]
    }
    return rho;
}
