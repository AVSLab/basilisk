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

#include "transDetermination/chebyPosEphem/chebyPosEphem.h"
#include "transDetermination/_GeneralModuleFiles/ephemerisUtilities.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "utilities/linearAlgebra.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/*! This method creates the output navigation message (translation only) for 
    the ephemeris model
 @return void
 @param ConfigData The configuration data associated with the ephemeris model
 */
void SelfInit_chebyPosEphem(ChebyPosEphemData *ConfigData, uint64_t moduleID)
{
    ConfigData->posFitOutMsgID = CreateNewMessage(ConfigData->posFitOutMsgName,
        sizeof(EphemerisIntMsg), "EphemerisIntMsg", moduleID);
}

/*! This method initializes the input time correlation factor structure
 @return void
 @param ConfigData The configuration data associated with the ephemeris model
 */
void CrossInit_chebyPosEphem(ChebyPosEphemData *ConfigData, uint64_t moduleID)
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
void Reset_chebyPosEphem(ChebyPosEphemData *ConfigData, uint64_t callTime,
                         uint64_t moduleID)
{

    int i, j, k, n;
    ChebyEphemRecord *currRec;
    double tempCVec[MAX_CHEB_COEFF];
    memset(tempCVec, 0x0, MAX_CHEB_COEFF*sizeof(double));
    for(i=0; i< MAX_CHEB_RECORDS; i++)
    {
        currRec = &(ConfigData->ephArray[i]);
        n=currRec->nChebCoeff;
        for(k=0; k<3; k++)
        {
            memset(tempCVec, 0x0, MAX_CHEB_COEFF*sizeof(double));
            vCopy(&(currRec->posChebyCoeff[k*currRec->nChebCoeff]), currRec->nChebCoeff, tempCVec);
            for(j=n-2;j>=2;j--)
            {
                currRec->velChebyCoeff[k*n+j]=2*(j+1)*tempCVec[j+1];
                tempCVec[j - 1] += ((j+1)*tempCVec[j+1])/(j - 1);
            }
            currRec->velChebyCoeff[k*n+1] = 4.0*tempCVec[2];
            currRec->velChebyCoeff[k*n+0] = tempCVec[1];
            for(j=0; j<n; j++)
            {
                currRec->velChebyCoeff[k*n+j] *= 1.0/currRec->ephemTimeRad;
            }
        }
        
    }
    
}

/*! This method takes the current time and computes the state of the object
    using that time and the stored Chebyshev coefficients.  If the time provided 
    is outside the specified range, the position vectors rail high/low appropriately.
 @return void
 @param ConfigData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_chebyPosEphem(ChebyPosEphemData *ConfigData, uint64_t callTime, uint64_t moduleID)
{

    double currentEphTime;
    uint64_t writeTime;
    uint32_t writeSize;
    double currentScaledValue;
    ChebyEphemRecord *currRec;
    int i;
    TDBVehicleClockCorrelationFswMsg localCorr;
    
    ReadMessage(ConfigData->clockCorrInMsgID, &writeTime, &writeSize,
                sizeof(TDBVehicleClockCorrelationFswMsg), &localCorr, moduleID);
    
    memset(&ConfigData->outputState, 0x0, sizeof(EphemerisIntMsg));
    
    currentEphTime = callTime*NANO2SEC;
    currentEphTime += localCorr.ephemerisTime - localCorr.vehicleClockTime;
    
    ConfigData->coeffSelector = 0;
    for(i=0; i<MAX_CHEB_RECORDS; i++)
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
    
    for(i=0; i<3; i++)
    {
        ConfigData->outputState.r_BdyZero_N[i] = calculateChebyValue(
            &(currRec->posChebyCoeff[i*currRec->nChebCoeff]),
            currRec->nChebCoeff, currentScaledValue);
        ConfigData->outputState.v_BdyZero_N[i] = calculateChebyValue(
            &(currRec->velChebyCoeff[i*currRec->nChebCoeff]),
            currRec->nChebCoeff, currentScaledValue);
        
    }
    
    WriteMessage(ConfigData->posFitOutMsgID, callTime,
                 sizeof(EphemerisIntMsg), &ConfigData->outputState, moduleID);

    return;

}
