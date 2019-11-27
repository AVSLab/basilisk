/*
 ISC License

 Copyright (c) 2019, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "sensorInterfaces/TAMSensorData/tamComm.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "utilities/linearAlgebra.h"
#include <string.h>
#include <math.h>

/*! This method initializes the configData for the TAM sensor interface.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param configData The configuration data associated with the TAM sensor interface
 */
void SelfInit_tamProcessTelem(tamConfigData *configData, int64_t moduleID)
{
    configData->bskPrint = _BSKPrint();
    /*! - Create output message for module */
    configData->tamOutMsgID = CreateNewMessage(configData->tamOutMsgName,
        sizeof(TAMSensorBodyFswMsg), "TAMSensorBodyFswMsg", moduleID);
}

/*! This method performs the second stage of initialization for the TAM sensor
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param configData The configuration data associated with the TAM interface
 */
void CrossInit_tamProcessTelem(tamConfigData *configData, int64_t moduleID)
{
    /*! - Link the message ID for the incoming sensor data message to here */
    configData->tamSensorMsgID = subscribeToMessage(configData->tamInMsgName,
        sizeof(TAMSensorIntMsg), moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the guidance module
 */
void Reset_tamProcessTelem(tamConfigData* configData, uint64_t callTime, int64_t moduleID)
{
    if (fabs(m33Determinant(RECAST3X3 configData->dcm_BS) - 1.0) > 1e-10) {
        _printMessage(configData->bskPrint, MSG_WARNING, "dcm_BS is set to zero values.");
    }

    return;
}

/*! This method takes the sensor data from the magnetometers and
 converts that information to the format used by the TAM nav.
 @return void
 @param configData The configuration data associated with the TAM interface
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_tamProcessTelem(tamConfigData *configData, uint64_t callTime, int64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    TAMSensorIntMsg LocalInput;

    memset(&LocalInput, 0x0, sizeof(TAMSensorIntMsg));

    ReadMessage(configData->tamSensorMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(TAMSensorIntMsg), (void*) &LocalInput, moduleID);

    m33MultV3(RECAST3X3 configData->dcm_BS, LocalInput.tam_S,
              configData->tamLocalOutput.tam_B);

    /*! - Write aggregate output into output message */
    WriteMessage(configData->tamOutMsgID, callTime,    sizeof(TAMSensorBodyFswMsg),
                (void*) & (configData->tamLocalOutput), moduleID);
    
    return;
}
