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

/*
    simpleInstrumentController Module

 */

#include "fswAlgorithms/sensorInterfaces/simpleInstrumentController/simpleInstrumentController.h"
#include "architecture/utilities/linearAlgebra.h"
#include <stdio.h>

/*!
    This method initializes the output messages for this module.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_simpleInstrumentController(simpleInstrumentControllerConfig *configData, int64_t moduleID)
{
    printf("simpleInstrumentController selfInit started\n");
    configData->imaged = 0;
    DeviceStatusMsg_C_init(&configData->deviceStatusOutMsg);
    printf("simpleInstrumentController selfInit started\n");
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_simpleInstrumentController(simpleInstrumentControllerConfig *configData, uint64_t callTime, int64_t moduleID)
{
    configData->imaged = 0;
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_simpleInstrumentController(simpleInstrumentControllerConfig *configData, uint64_t callTime, int64_t moduleID)
{
    printf("simpleInstrumentController update started\n");
    unsigned int status; //!< Data status to be written to deviceStatus msg
    double sigma_BR_norm; //!< Norm of sigma_BR

    /* Local copies of the msg buffers*/
    AccessMsgPayload accessInMsgBuffer;  //!< local copy of input message buffer
    AttGuidMsgPayload attGuidInMsgBuffer;  //!< local copy of output message buffer
    DeviceStatusMsgPayload deviceStatusOutMsgBuffer;  //!< local copy of output message buffer

    // zero output buffer
    deviceStatusOutMsgBuffer = DeviceStatusMsg_C_zeroMsgPayload();

    // read in the input messages
    accessInMsgBuffer = AccessMsg_C_read(&configData->locationAccessInMsg);
    attGuidInMsgBuffer = AttGuidMsg_C_read(&configData->attGuidInMsg);

    printf("1\n");

    // Compute the norm of the attitude error
    sigma_BR_norm = v3Norm(attGuidInMsgBuffer.sigma_BR);

    // If the target has not been imaged
    if (!configData->imaged) {
        /* If the attitude error is less than the tolerance and the groundLocation is accessible, turn on the instrument and
        set the imaged indicator to 1*/
        if ((sigma_BR_norm <= configData->attErrTolerance) && (accessInMsgBuffer.hasAccess)) {
            deviceStatusOutMsgBuffer.deviceStatus = 1;
            configData->imaged = 1;
            // Otherwise, turn off the instrument
        } else {
            deviceStatusOutMsgBuffer.deviceStatus = 0;
        }
    }
    else {
        deviceStatusOutMsgBuffer.deviceStatus = 0;
    }

    printf("2\n");

    printf("%s\n", configData->deviceStatusOutMsg);
    printf("%i\n", deviceStatusOutMsgBuffer.deviceStatus);

//    printf("%p\n", &configData->deviceStatusOutMsg);
//    printf("%p\n", &deviceStatusOutMsgBuffer);

    // write to the output messages
    DeviceStatusMsg_C_write(&deviceStatusOutMsgBuffer, &(configData->deviceStatusOutMsg), moduleID, callTime);

    printf("simpleInstrumentController update ended\n");

    return;
}
