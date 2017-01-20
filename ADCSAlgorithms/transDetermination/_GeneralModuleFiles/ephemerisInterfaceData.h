/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _EPHEMERIS_INTERFACE_DATA_H_
#define _EPHEMERIS_INTERFACE_DATA_H_

#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief time correlation factor structure used to take vehicle time and convert
 it over to ephemeris time (TDB)
 */
typedef struct {
    double ephemerisTime;                /*!< [s] Ephemeris time associated with the vehicle time*/
    double vehicleClockTime;             /*!< [s] Vehicle time code converted over to seconds */
}TDBVehicleClockCorrelation;

/*! @brief Structure used to write ephemeris states out to other modules*/
typedef struct {
    double r_BdyZero_N[3];          /*!< [m] Position of orbital body*/
    double v_BdyZero_N[3];          /*!< [m/s] Velocity of orbital body*/
    double timeTag;                 /*!< [s] vehicle Time-tag for state*/
}EphemerisOutputData;

#endif
