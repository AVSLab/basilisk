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

/* modify the path to reflect the new module names */
#include "rateMsgConverter.h"
#include "simulation/utilities/linearAlgebra.h"
#include <string.h>


/*! This method initializes the ConfigData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_rateMsgConverter(rateMsgConverterConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->navRateOutMsgID = CreateNewMessage(ConfigData->navRateOutMsgName,
                                               sizeof(NavAttIntMsg),
                                               "NavAttIntMsg",          /* add the output structure name */
                                               moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_rateMsgConverter(rateMsgConverterConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->imuRateInMsgID = subscribeToMessage(ConfigData->imuRateInMsgName,
                                                sizeof(IMUSensorBodyFswMsg),
                                                moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_rateMsgConverter(rateMsgConverterConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    return;
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_rateMsgConverter(rateMsgConverterConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t            clockTime;
    uint32_t            readSize;
    IMUSensorBodyFswMsg inMsg;

    /*! Begin method steps*/
    /*! - Read the input messages */
    ReadMessage(ConfigData->imuRateInMsgID, &clockTime, &readSize,
                sizeof(IMUSensorBodyFswMsg), (void*) &(inMsg), moduleID);

    /*
     create output message
     */
    memset(&ConfigData->outMsg, 0x0, sizeof(ConfigData->outMsg));
    v3Copy(inMsg.AngVelBody, ConfigData->outMsg.omega_BN_B);

    /*
     store the output message 
     */
    WriteMessage(ConfigData->navRateOutMsgID, callTime, sizeof(NavAttIntMsg),   /* update module name */
                 (void*) &(ConfigData->outMsg), moduleID);

    return;
}
