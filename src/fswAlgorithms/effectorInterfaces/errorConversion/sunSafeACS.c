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

#include "effectorInterfaces/errorConversion/sunSafeACS.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simulation/utilities/rigidBodyKinematics.h"
#include <string.h>
#include <math.h>

/*! This method initializes the configData for the sun safe ACS control.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param configData The configuration data associated with the sun safe control
 */
void SelfInit_sunSafeACS(sunSafeACSConfig *configData, int64_t moduleID)
{
    configData->bskPrint = _BSKPrint();
    /*! - Create output message for module */
    configData->thrData.outputMsgID = CreateNewMessage(
        configData->thrData.outputDataName, sizeof(THRArrayOnTimeCmdIntMsg),
        "THRArrayOnTimeCmdIntMsg", moduleID);

}

/*! This method performs the second stage of initialization for the sun safe ACS
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param configData The configuration data associated with the sun safe ACS control
 */
void CrossInit_sunSafeACS(sunSafeACSConfig *configData, int64_t moduleID)
{
    /*! - Get the control data message ID*/
    configData->inputMsgID = subscribeToMessage(configData->inputControlName,
        sizeof(CmdTorqueBodyIntMsg), moduleID);

}

/*! This method takes the estimated body-observed sun vector and computes the
 current attitude/attitude rate errors to pass on to control.
 @return void
 @param configData The configuration data associated with the sun safe ACS control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_sunSafeACS(sunSafeACSConfig *configData, uint64_t callTime,
    int64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    CmdTorqueBodyIntMsg cntrRequest;

    /*! - Read the input parsed CSS sensor data message*/
    ReadMessage(configData->inputMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(CmdTorqueBodyIntMsg), (void*) &(cntrRequest), moduleID);
    computeSingleThrustBlock(&(configData->thrData), callTime,
                             &cntrRequest, moduleID);

    return;
}
