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
 @param configData The configuration data associated with the ephemeris model
 @param moduleID The module identification integer
 */
void SelfInit_oeStateEphem(OEStateEphemData *configData, uint64_t moduleID)
{
    configData->stateFitOutMsgID = CreateNewMessage(configData->stateFitOutMsgName,
        sizeof(EphemerisIntMsg), "EphemerisIntMsg", moduleID);
}

/*! This method initializes the input time correlation factor structure
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param moduleID The module identification integer
 */
void CrossInit_oeStateEphem(OEStateEphemData *configData, uint64_t moduleID)
{
    configData->clockCorrInMsgID = subscribeToMessage(
        configData->clockCorrInMsgName, sizeof(TDBVehicleClockCorrelationFswMsg), moduleID);
}

/*! This Reset method is empty
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Reset_oeStateEphem(OEStateEphemData *configData, uint64_t callTime,
                         uint64_t moduleID)
{

}

/*! This method takes the current time and computes the state of the object
    using that time and the stored Chebyshev coefficients.  If the time provided 
    is outside the specified range, the position vectors rail high/low appropriately.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Update_oeStateEphem(OEStateEphemData *configData, uint64_t callTime, uint64_t moduleID)
{
    double currentEphTime;
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    double currentScaledValue;
    double meanAnom;
    double orbAnom;
    ChebyOERecord *currRec;
    int i;
    TDBVehicleClockCorrelationFswMsg localCorr;
    memset(&localCorr, 0x0 ,sizeof(TDBVehicleClockCorrelationFswMsg));
    EphemerisIntMsg tmpOutputState;
    memset(&tmpOutputState, 0x0, sizeof(EphemerisIntMsg));
    classicElements orbEl;

    ReadMessage(configData->clockCorrInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(TDBVehicleClockCorrelationFswMsg), &localCorr, moduleID);
    
    currentEphTime = callTime*NANO2SEC;
    currentEphTime += localCorr.ephemerisTime - localCorr.vehicleClockTime;

    configData->coeffSelector = 0;
    for(i=0; i<MAX_OE_RECORDS; i++)
    {
        if(fabs(currentEphTime - configData->ephArray[i].ephemTimeMid) <=
            configData->ephArray[i].ephemTimeRad)
        {
            configData->coeffSelector = i;
            break;
        }
    }

    currRec = &(configData->ephArray[configData->coeffSelector]);
    currentScaledValue = (currentEphTime - currRec->ephemTimeMid)
        /currRec->ephemTimeRad;
    if(fabs(currentScaledValue) > 1.0)
    {
        currentScaledValue = currentScaledValue/fabs(currentScaledValue);
    }
    
    tmpOutputState.timeTag = callTime*NANO2SEC;
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

    elem2rv(configData->muCentral, &orbEl, tmpOutputState.r_BdyZero_N,
            tmpOutputState.v_BdyZero_N);

    WriteMessage(configData->stateFitOutMsgID, callTime,
                 sizeof(EphemerisIntMsg), &tmpOutputState,
                 moduleID);

    return;

}
