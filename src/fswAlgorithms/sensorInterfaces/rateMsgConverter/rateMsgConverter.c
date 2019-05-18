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
    Rate Converter message

    Note:   this module reads in a message of type ImuSensorBodyFswMsg, extracts the body rate vector information,
            and adds this info to a msg of type NavAttIntMsg.
    Author: Hanspeter Schaub
    Date:   June 30, 2018
 
 */

#include <string.h>
#include "rateMsgConverter.h"
#include "simulation/utilities/linearAlgebra.h"

/*! This method initializes the configData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The Basilisk module identifier
 */
void SelfInit_rateMsgConverter(rateMsgConverterConfig *configData, uint64_t moduleID)
{
    configData->navRateOutMsgID = CreateNewMessage(configData->navRateOutMsgName,
                                                   sizeof(NavAttIntMsg),
                                                   "NavAttIntMsg",
                                                   moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The Basilisk module identifier
 */
void CrossInit_rateMsgConverter(rateMsgConverterConfig *configData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    configData->imuRateInMsgID = subscribeToMessage(configData->imuRateInMsgName,
                                                    sizeof(IMUSensorBodyFswMsg),
                                                    moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The Basilisk module identifier
 */
void Reset_rateMsgConverter(rateMsgConverterConfig *configData, uint64_t callTime, uint64_t moduleID)
{
    return;
}

/*! This method performs a time step update of the module.
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The Basilisk module identifier
 */
void Update_rateMsgConverter(rateMsgConverterConfig *configData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    IMUSensorBodyFswMsg inMsg;
    NavAttIntMsg outMsg;
    
    /*! - read in the message of type IMUSensorBodyFswMsg */
    memset(&inMsg, 0x0, sizeof(inMsg));
    ReadMessage(configData->imuRateInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten,
                sizeof(IMUSensorBodyFswMsg), (void*) &inMsg, moduleID);
    
    /*! - create a zero message of type NavAttIntMsg which has the rate vector from the input message */
    memset(&outMsg, 0x0, sizeof(outMsg));
    v3Copy(inMsg.AngVelBody, outMsg.omega_BN_B);
    
    /*! - write output message */
    WriteMessage(configData->navRateOutMsgID, callTime, sizeof(NavAttIntMsg),
                 (void*) &outMsg, moduleID);
    
    return;
}
