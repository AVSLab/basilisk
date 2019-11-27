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

#include "sensorInterfaces/IMUSensorData/imuComm.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <string.h>

/*! This method initializes the configData for theIMU sensor interface.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param configData The configuration data associated with the IMU sensor interface
 */
void SelfInit_imuProcessTelem(IMUConfigData *configData, int64_t moduleID)
{
    configData->bskPrint = _BSKPrint();
    /*! - Create output message for module */
    configData->OutputMsgID = CreateNewMessage(configData->OutputDataName,
        sizeof(IMUSensorBodyFswMsg), "IMUSensorBodyFswMsg", moduleID);

}

/*! This method performs the second stage of initialization for the IMU sensor
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param configData The configuration data associated with the IMU interface
 */
void CrossInit_imuProcessTelem(IMUConfigData *configData, int64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    VehicleConfigFswMsg LocalConfigData;

    /*! - Link the message ID for the incoming sensor data message to here */
    configData->SensorMsgID = subscribeToMessage(configData->InputDataName,
        sizeof(IMUSensorIntMsg), moduleID);
    configData->PropsMsgID = subscribeToMessage(configData->InputPropsName,
        sizeof(VehicleConfigFswMsg), moduleID);
    if(configData->PropsMsgID >= 0)
    {
        ReadMessage(configData->PropsMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                    sizeof(VehicleConfigFswMsg), (void*) &LocalConfigData, moduleID);
    }
    
}

/*! This method takes the raw sensor data from the coarse sun sensors and
 converts that information to the format used by the IMU nav.
 @return void
 @param configData The configuration data associated with the IMU interface
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_imuProcessTelem(IMUConfigData *configData, uint64_t callTime, int64_t moduleID)
{
    
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    IMUSensorIntMsg LocalInput;
    ReadMessage(configData->SensorMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(IMUSensorIntMsg), (void*) &LocalInput, moduleID);
    
    m33MultV3(RECAST3X3 configData->dcm_BP, LocalInput.DVFramePlatform,
              configData->LocalOutput.DVFrameBody);
    m33MultV3(RECAST3X3 configData->dcm_BP, LocalInput.AccelPlatform,
              configData->LocalOutput.AccelBody);
    m33MultV3(RECAST3X3 configData->dcm_BP, LocalInput.DRFramePlatform,
              configData->LocalOutput.DRFrameBody);
    m33MultV3(RECAST3X3 configData->dcm_BP, LocalInput.AngVelPlatform,
              configData->LocalOutput.AngVelBody);
    
    WriteMessage(configData->OutputMsgID, callTime, sizeof(IMUSensorBodyFswMsg),
                 (void*) &(configData->LocalOutput), moduleID);
    
    return;
}
