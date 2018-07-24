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

#include "sensorInterfaces/STSensorData/stComm.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <string.h>

/*! This method initializes the ConfigData for theST sensor interface.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the ST sensor interface
 */
void SelfInit_stProcessTelem(STConfigData *ConfigData, uint64_t moduleID)
{
    
    /*! - Create output message for module */
    ConfigData->OutputMsgID = CreateNewMessage(ConfigData->OutputDataName,
        sizeof(STAttFswMsg), "STAttFswMsg", moduleID);
    
}

/*! This method performs the second stage of initialization for the ST sensor
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the ST interface
 */
void CrossInit_stProcessTelem(STConfigData *ConfigData, uint64_t moduleID)
{
    uint64_t UnusedClockTime;
    uint32_t ReadSize;
    VehicleConfigFswMsg LocalConfigData;
    /*! Begin method steps */
    /*! - Link the message ID for the incoming sensor data message to here */
    ConfigData->SensorMsgID = subscribeToMessage(ConfigData->InputDataName,
        sizeof(STSensorIntMsg), moduleID);
    ConfigData->PropsMsgID = subscribeToMessage(ConfigData->InputPropsName,
        sizeof(VehicleConfigFswMsg), moduleID);
    if(ConfigData->PropsMsgID >= 0)
    {
        ReadMessage(ConfigData->PropsMsgID, &UnusedClockTime, &ReadSize,
                    sizeof(VehicleConfigFswMsg), (void*) &LocalConfigData, moduleID);
    }
    
}

/*! This method takes the raw sensor data from the star tracker and
 converts that information to the format used by the ST nav.
 @return void
 @param ConfigData The configuration data associated with the ST interface
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_stProcessTelem(STConfigData *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    
    uint64_t UnusedClockTime;
    uint32_t ReadSize;
    double dcm_CN[3][3];            /* dcm, inertial to case frame */
    double dcm_BN[3][3];            /* dcm, inertial to body frame */
    STSensorIntMsg LocalInput;
    ReadMessage(ConfigData->SensorMsgID, &UnusedClockTime, &ReadSize,
                sizeof(STSensorIntMsg), (void*) &LocalInput, moduleID);
    EP2C(LocalInput.qInrtl2Case, dcm_CN);
    m33MultM33(RECAST3X3 ConfigData->dcm_BP, dcm_CN, dcm_BN);
    C2MRP(dcm_BN, ConfigData->LocalOutput.MRP_BdyInrtl);
    ConfigData->LocalOutput.timeTag = LocalInput.timeTag;
    WriteMessage(ConfigData->OutputMsgID, callTime, sizeof(STAttFswMsg),
                 (void*) &(ConfigData->LocalOutput), moduleID);
    
    return;
}
