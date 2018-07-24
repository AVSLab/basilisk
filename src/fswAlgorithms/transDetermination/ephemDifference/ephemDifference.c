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

#include "transDetermination/ephemDifference/ephemDifference.h"
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
void SelfInit_ephemDifference(EphemDifferenceData *ConfigData, uint64_t moduleID)
{
    uint32_t i;
    for(i=0; i<ConfigData->ephBdyCount; i++)
    {
        ConfigData->changeBodies[i].ephOutMsgID = CreateNewMessage(
            ConfigData->changeBodies[i].ephOutMsgName,
            sizeof(EphemerisIntMsg), "EphemerisIntMsg", moduleID);
    }
}

/*! This method initializes the input time correlation factor structure
 @return void
 @param ConfigData The configuration data associated with the ephemeris model
 */
void CrossInit_ephemDifference(EphemDifferenceData *ConfigData, uint64_t moduleID)
{

    uint32_t i;
    for(i=0; i<ConfigData->ephBdyCount; i++)
    {
        ConfigData->changeBodies[i].ephInMsgID = subscribeToMessage(
                            ConfigData->changeBodies[i].ephInMsgName,
                            sizeof(EphemerisIntMsg), moduleID);
    }

    ConfigData->ephBaseInMsgID = subscribeToMessage(
        ConfigData->ephBaseInMsgName, sizeof(EphemerisIntMsg), moduleID);

}

/*! This method takes the chebyshev coefficients loaded for the position 
    estimator and computes the coefficients needed to estimate the time 
    derivative of that position vector (velocity).
 @return void
 @param ConfigData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Reset_ephemDifference(EphemDifferenceData *ConfigData, uint64_t callTime,
                         uint64_t moduleID)
{
    if(ConfigData->baseScale == 0.0)
    {
        ConfigData->baseScale = 1.0;
    }
}

/*! This method takes the current time and computes the state of the object
    using that time and the stored Chebyshev coefficients.  If the time provided 
    is outside the specified range, the position vectors rail high/low appropriately.
 @return void
 @param ConfigData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_ephemDifference(EphemDifferenceData *ConfigData, uint64_t callTime, uint64_t moduleID)
{

    
    uint64_t writeTime;
    uint32_t writeSize;
    uint32_t i;
    double posBase[3];
    double velBase[3];
    
    ReadMessage(ConfigData->ephBaseInMsgID, &writeTime, &writeSize,
                sizeof(EphemerisIntMsg), &ConfigData->baseEphem, moduleID);
    v3Scale(ConfigData->baseScale, ConfigData->baseEphem.r_BdyZero_N, posBase);
    v3Scale(ConfigData->baseScale, ConfigData->baseEphem.v_BdyZero_N, velBase);
    
    for(i=0; i<ConfigData->ephBdyCount; i++)
    {
        ReadMessage(ConfigData->changeBodies[i].ephInMsgID, &writeTime,
            &writeSize, sizeof(EphemerisIntMsg),
            &ConfigData->changeBodies[i].ephStore, moduleID);
        v3Subtract(ConfigData->changeBodies[i].ephStore.r_BdyZero_N,
                   posBase, ConfigData->changeBodies[i].ephStore.r_BdyZero_N);
        v3Subtract(ConfigData->changeBodies[i].ephStore.v_BdyZero_N,
                   velBase, ConfigData->changeBodies[i].ephStore.v_BdyZero_N);
        WriteMessage(ConfigData->changeBodies[i].ephOutMsgID, callTime,
            sizeof(EphemerisIntMsg), &ConfigData->changeBodies[i].ephStore,
            moduleID);
    }

    return;

}
