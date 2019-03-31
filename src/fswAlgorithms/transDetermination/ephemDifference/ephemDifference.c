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

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "transDetermination/ephemDifference/ephemDifference.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "utilities/linearAlgebra.h"

/*! @brief This method creates the output ephermeris messages for each body.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param moduleID The module identification integer
 */
void SelfInit_ephemDifference(EphemDifferenceData *configData, uint64_t moduleID)
{
    uint32_t i;
    for(i = 0; i < MAX_NUM_CHANGE_BODIES; i++)
    {
        if (strlen(configData->changeBodies[i].ephOutMsgName) != 0) {
            configData->changeBodies[i].ephOutMsgID = CreateNewMessage(
                                                                       configData->changeBodies[i].ephOutMsgName,
                                                                       sizeof(EphemerisIntMsg), "EphemerisIntMsg", moduleID);
            configData->ephBdyCount++;
        }
    }
}

/*! @brief This method subscribes to the body ephermeris messages which will be
    augmented relative to another base frame.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param moduleID The module identification integer
 */
void CrossInit_ephemDifference(EphemDifferenceData *configData, uint64_t moduleID)
{
    uint32_t i;
    for(i = 0; i < configData->ephBdyCount; i++)
    {
        configData->changeBodies[i].ephInMsgID = subscribeToMessage(
                            configData->changeBodies[i].ephInMsgName,
                            sizeof(EphemerisIntMsg), moduleID);
    }

    configData->ephBaseInMsgID = subscribeToMessage(configData->ephBaseInMsgName,
                                                    sizeof(EphemerisIntMsg),
                                                    moduleID);
}

/*! @brief This method resets the base scale.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Reset_ephemDifference(EphemDifferenceData *configData, uint64_t callTime,
                         uint64_t moduleID)
{
 
}

/*! @brief This method recomputes the body postions and velocities relative to
    the base body ephemeris and writes out updated ephermeris position and velocity
    for each body.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Update_ephemDifference(EphemDifferenceData *configData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    uint32_t i;
    EphemerisIntMsg tmpBaseEphem;
    EphemerisIntMsg tmpEphStore;
    memset(&tmpBaseEphem, 0x0, sizeof(EphemerisIntMsg));
    
    ReadMessage(configData->ephBaseInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(EphemerisIntMsg), (void *)&tmpBaseEphem, moduleID);
    
    
    for(i = 0; i < configData->ephBdyCount; i++)
    {
        memset(&tmpEphStore, 0x0, sizeof(EphemerisIntMsg));

        ReadMessage(configData->changeBodies[i].ephInMsgID, &timeOfMsgWritten,
                    &sizeOfMsgWritten, sizeof(EphemerisIntMsg), (void *)&tmpEphStore,
                    moduleID);
        
        v3Subtract(tmpEphStore.r_BdyZero_N,
                   tmpBaseEphem.r_BdyZero_N,
                   tmpEphStore.r_BdyZero_N);
        v3Subtract(tmpEphStore.v_BdyZero_N,
                   tmpBaseEphem.v_BdyZero_N,
                   tmpEphStore.v_BdyZero_N);
        tmpEphStore.timeTag = tmpBaseEphem.timeTag;
        
        WriteMessage(configData->changeBodies[i].ephOutMsgID, callTime,
                     sizeof(EphemerisIntMsg), &tmpEphStore,
                     moduleID);
    }
    return;
}
