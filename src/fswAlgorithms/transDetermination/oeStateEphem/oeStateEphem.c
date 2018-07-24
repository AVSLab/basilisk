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

#include "transDetermination/oeStateEphem/oeStateEphem.h"
#include "transDetermination/_GeneralModuleFiles/ephemerisUtilities.h"
#include "transDetermination/chebyPosEphem/chebyPosEphem.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "utilities/linearAlgebra.h"
#include "utilities/orbitalMotion.h"
#include "utilities/astroConstants.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/*! This method creates the output navigation message (translation only) for 
    the ephemeris model
 @return void
 @param ConfigData The configuration data associated with the ephemeris model
 */
void SelfInit_oeStateEphem(OEStateEphemData *ConfigData, uint64_t moduleID)
{
    ConfigData->stateFitOutMsgID = CreateNewMessage(ConfigData->stateFitOutMsgName,
        sizeof(EphemerisIntMsg), "EphemerisIntMsg", moduleID);
}

/*! This method initializes the input time correlation factor structure
 @return void
 @param ConfigData The configuration data associated with the ephemeris model
 */
void CrossInit_oeStateEphem(OEStateEphemData *ConfigData, uint64_t moduleID)
{

    ConfigData->clockCorrInMsgID = subscribeToMessage(
        ConfigData->clockCorrInMsgName, sizeof(TDBVehicleClockCorrelationFswMsg), moduleID);

}

/*! This method takes the chebyshev coefficients loaded for the position 
    estimator and computes the coefficients needed to estimate the time 
    derivative of that position vector (velocity).
 @return void
 @param ConfigData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Reset_oeStateEphem(OEStateEphemData *ConfigData, uint64_t callTime,
                         uint64_t moduleID)
{

}

/*! This method takes the current time and computes the state of the object
    using that time and the stored Chebyshev coefficients.  If the time provided 
    is outside the specified range, the position vectors rail high/low appropriately.
 @return void
 @param ConfigData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_oeStateEphem(OEStateEphemData *ConfigData, uint64_t callTime, uint64_t moduleID)
{

    double currentEphTime;
    uint64_t writeTime;
    uint32_t writeSize;
    double currentScaledValue;
    double meanAnom;
    double orbAnom;
    ChebyOERecord *currRec;
    int i;
    TDBVehicleClockCorrelationFswMsg localCorr;
    classicElements orbEl;
    
    memset(&localCorr, 0x0 ,sizeof(TDBVehicleClockCorrelationFswMsg));
    ReadMessage(ConfigData->clockCorrInMsgID, &writeTime, &writeSize,
                sizeof(TDBVehicleClockCorrelationFswMsg), &localCorr, moduleID);
    
    memset(&ConfigData->outputState, 0x0, sizeof(EphemerisIntMsg));
    
    currentEphTime = callTime*NANO2SEC;
    currentEphTime += localCorr.ephemerisTime - localCorr.vehicleClockTime;

    ConfigData->coeffSelector = 0;
    for(i=0; i<MAX_OE_RECORDS; i++)
    {
        if(fabs(currentEphTime - ConfigData->ephArray[i].ephemTimeMid) <=
            ConfigData->ephArray[i].ephemTimeRad)
        {
            ConfigData->coeffSelector = i;
            break;
        }
    }

    currRec = &(ConfigData->ephArray[ConfigData->coeffSelector]);
    currentScaledValue = (currentEphTime - currRec->ephemTimeMid)
        /currRec->ephemTimeRad;
    if(fabs(currentScaledValue) > 1.0)
    {
        currentScaledValue = currentScaledValue/fabs(currentScaledValue);
    }
    
    ConfigData->outputState.timeTag = callTime*NANO2SEC;
    orbEl.a = calculateChebyValue(currRec->semiMajorCoeff, currRec->nChebCoeff,
                                  currentScaledValue);
    orbEl.i = calculateChebyValue(currRec->incCoeff, currRec->nChebCoeff,
                                  currentScaledValue);
    orbEl.e = calculateChebyValue(currRec->eccCoeff, currRec->nChebCoeff,
                                  currentScaledValue);
    orbEl.omega = calculateChebyValue(currRec->argPerCoeff, currRec->nChebCoeff,
                                  currentScaledValue);
    orbEl.Omega = calculateChebyValue(currRec->RAANCoeff, currRec->nChebCoeff,
                                  currentScaledValue);
    orbEl.a = calculateChebyValue(currRec->semiMajorCoeff, currRec->nChebCoeff,
                                  currentScaledValue);
    meanAnom = calculateChebyValue(currRec->meanAnomCoeff, currRec->nChebCoeff,
                                   currentScaledValue);
    
    if(orbEl.a > 0.0)
    {
        orbAnom = M2E(meanAnom, orbEl.e);
        orbEl.f = E2f(orbAnom, orbEl.e);
    }
    else
    {
        orbAnom = N2H(meanAnom, orbEl.e);
        orbEl.f = H2f(orbAnom, orbEl.e);
    }
    
    while(orbEl.Omega < 0.0)
    {
        orbEl.Omega += 2.0*M_PI;
    }

    elem2rv(ConfigData->muCentral, &orbEl, ConfigData->outputState.r_BdyZero_N,
            ConfigData->outputState.v_BdyZero_N);

    WriteMessage(ConfigData->stateFitOutMsgID, callTime,
                 sizeof(EphemerisIntMsg), &ConfigData->outputState, moduleID);

    return;

}
