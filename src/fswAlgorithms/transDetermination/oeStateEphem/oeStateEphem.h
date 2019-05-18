/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _OE_STATE_EPHEM_H_
#define _OE_STATE_EPHEM_H_

#include "messaging/static_messaging.h"
#include "fswMessages/TDBVehicleClockCorrelationFswMsg.h"
#include "transDetermination/oeStateEphem/oeStateEphem.h"
#include "simFswInterfaceMessages/ephemerisIntMsg.h"

#define MAX_OE_RECORDS 10
#define MAX_OE_COEFF 20

/*! \defgroup oeStateEphem
 @brief This module takes the TDB time, current object time and computes the state of the object
 * using the time corrected by TDB and the stored Chebyshev coefficients.
 *
 * If the time provided
 * is outside the specified range for which the stored Chebyshev coefficients are valid then
 * the position vectors rail high/low appropriately. 
 * More information can be found in the [PDF Description](Basilisk-oeStateEphem-20190426.pdf).
 *  @{
*/

/*! @brief Structure that defines the layout of an Ephemeris "record."  This is
           basically the set of coefficients for the ephemeris elements and
           the time factors associated with those coefficients
*/
typedef struct {
    uint32_t nChebCoeff;                  //!< [-] Number chebyshev coefficients loaded into record
    double ephemTimeMid;                  //!< [s] Ephemeris time (TDB) associated with the mid-point of the curve
    double ephemTimeRad;                  //!< [s] "Radius" of time that curve is valid for (half of total range
    double rPeriapCoeff[MAX_OE_COEFF];    //!< [-] Set of chebyshev coefficients for radius at periapses
    double eccCoeff[MAX_OE_COEFF];        //!< [-] Set of chebyshev coefficients for eccentrity
    double incCoeff[MAX_OE_COEFF];        //!< [-] Set of chebyshev coefficients for inclination
    double argPerCoeff[MAX_OE_COEFF];     //!< [-] Set of chebyshev coefficients for argument of periapses
    double RAANCoeff[MAX_OE_COEFF];       //!< [-] Set of chebyshev coefficients for right ascention of the ascending node
    double anomCoeff[MAX_OE_COEFF];       //!< [-] Set of chebyshev coefficients for true anomaly angle
    uint32_t anomalyFlag;                 //!< [-] Flag indicating if the anomaly angle is true (0), mean (1)
}ChebyOERecord;

/*! @brief Top level structure for the Chebyshev position ephemeris 
           fit system.  Allows the user to specify a set of chebyshev 
           coefficients and then use the input time to determine where 
           a given body is in space
*/
typedef struct {
    char stateFitOutMsgName[MAX_STAT_MSG_LENGTH]; //!< [-] The name of the output navigation message for pos/vel
    char clockCorrInMsgName[MAX_STAT_MSG_LENGTH]; //!< The name of the clock correlation message
    double muCentral;                             //!< [m3/s^2] Gravitational parameter for center of orbital elements
    ChebyOERecord ephArray[MAX_OE_RECORDS];       //!< [-] Array of Chebyshev records for ephemeris
    int32_t stateFitOutMsgId;                     //!< [-] The ID associated with the outgoing message
    int32_t clockCorrInMsgId;                     //!< [-] The ID associated with the incoming clock correlation
    uint32_t coeffSelector;                       //!< [-] Index in the ephArray that we are currently using
}OEStateEphemData;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_oeStateEphem(OEStateEphemData *configData, uint64_t moduleID);
    void CrossInit_oeStateEphem(OEStateEphemData *configData, uint64_t moduleID);
    void Update_oeStateEphem(OEStateEphemData *configData, uint64_t callTime,
        uint64_t moduleID);
    void Reset_oeStateEphem(OEStateEphemData *configData, uint64_t callTime,
                              uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
