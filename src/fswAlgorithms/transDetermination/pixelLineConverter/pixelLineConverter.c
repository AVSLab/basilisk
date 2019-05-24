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
#include "pixelLineConverter.h"
#include "simFswInterfaceMessages/circlesOpNavMsg.h"
#include "simFswInterfaceMessages/cameraConfigMsg.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "fswMessages/opnavFswMsg.h"
#include "utilities/linearAlgebra.h"

/*! This method transforms pixel, line, and diameter data into heading data for orbit determination or heading determination.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param moduleID The module identification integer
 */
void SelfInit_pixelLineConverter(PixelLineConvertData *configData, uint64_t moduleID)
{
    configData->stateOutMsgID = CreateNewMessage(configData->opNavOutMsgName,
                                                 sizeof(OpnavFswMsg),
                                                 "OpnavFsw",
                                                 moduleID);
}

/*! This method subscribes to the camera and circle messages
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param moduleID The module identification integer
 */
void CrossInit_pixelLineConverter(PixelLineConvertData *configData, uint64_t moduleID)
{
    configData->cameraConfigMsgID = subscribeToMessage(configData->cameraConfigMsgName,sizeof(CameraConfigMsg),moduleID);
    configData->circlesInMsgID = subscribeToMessage(configData->circlesInMsgName, sizeof(CirclesOpNavMsg),moduleID);
}

/*! This resets the module to original states.
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Reset_pixelLineConverter(PixelLineConvertData *configData, uint64_t callTime, uint64_t moduleID)
{

}

/*! This method reads in the camera and circle messages and extracts navigation data from them. It outputs the heading (norm and direction) to the celestial body identified in the body frame. 
 @return void
 @param configData The configuration data associated with the ephemeris model
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identification integer
 */
void Update_pixelLineConverter(PixelLineConvertData *configData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    CameraConfigMsg cameraSpecs;
    CirclesOpNavMsg circlesIn;
    OpnavFswMsg opNavMsgOut ;
    memset(&cameraSpecs, 0x0, sizeof(CameraConfigMsg));
    memset(&circlesIn, 0x0, sizeof(CirclesOpNavMsg));
    memset(&opNavMsgOut, 0x0, sizeof(OpnavFswMsg));

    /*! - read input messages */
    ReadMessage(configData->cameraConfigMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(CameraConfigMsg), &cameraSpecs, moduleID);
    ReadMessage(configData->circlesInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(CirclesOpNavMsg), &circlesIn, moduleID);

    /*! - map timeTag, position and velocity vector to output message */

    /*! - write output message */
    WriteMessage(configData->stateOutMsgID, callTime, sizeof(OpnavFswMsg),
                 &opNavMsgOut, moduleID);

    return;
}
