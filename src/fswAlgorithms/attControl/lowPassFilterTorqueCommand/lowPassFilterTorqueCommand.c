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
    Control Torque Low Pass Filter Module
 
 */

/* modify the path to reflect the new module names */
#include "attControl/lowPassFilterTorqueCommand/lowPassFilterTorqueCommand.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include "fswUtilities/fswDefinitions.h"
#include "math.h"



/*! This method initializes the ConfigData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_lowPassFilterTorqueCommand(lowPassFilterTorqueCommandConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(CmdTorqueBodyIntMsg),
                                                "CmdTorqueBodyIntMsg",
                                                moduleID);

}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_lowPassFilterTorqueCommand(lowPassFilterTorqueCommandConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputMsgID = subscribeToMessage(ConfigData->inputDataName,
                                                sizeof(CmdTorqueBodyIntMsg),
                                                moduleID);

}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the MRP steering control
 */
void Reset_lowPassFilterTorqueCommand(lowPassFilterTorqueCommandConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    int i;

    ConfigData->reset  = BOOL_TRUE;         /* reset the first run flag */

    for (i=0;i<NUM_LPF;i++) {
        v3SetZero(ConfigData->Lr[i]);
        v3SetZero(ConfigData->LrF[i]);
    }
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param ConfigData The configuration data associated with the MRP Steering attitude control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_lowPassFilterTorqueCommand(lowPassFilterTorqueCommandConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    uint64_t    clockTime;
    uint32_t    readSize;
    double      v3[3];                      /*!<      3d vector sub-result */
    int         i;

    /*! Begin method steps*/
    /*! - Read the input messages */
    ReadMessage(ConfigData->inputMsgID, &clockTime, &readSize,
                sizeof(CmdTorqueBodyIntMsg), (void*) &(ConfigData->Lr[0]), moduleID);

    /*
        check if the filter states must be reset
     */
    if (ConfigData->reset) {
        /* populate the filter history with 1st input */
        for (i=1;i<NUM_LPF;i++) {
            v3Copy(ConfigData->Lr[0], ConfigData->Lr[i]);
        }

        /* zero the history of filtered outputs */
        for (i=0;i<NUM_LPF;i++) {
            v3SetZero(ConfigData->LrF[i]);
        }

        /* compute h times the prewarped critical filter frequency */
        ConfigData->hw = tan(ConfigData->wc * ConfigData->h / 2.0)*2.0;

        /* determine 1st order low-pass filter coefficients */
        ConfigData->a[0] = 2.0 + ConfigData->hw;
        ConfigData->a[1] = 2.0 - ConfigData->hw;
        ConfigData->b[0] = ConfigData->hw;
        ConfigData->b[1] = ConfigData->hw;

        /* turn off first run flag */
        ConfigData->reset = BOOL_FALSE;

    }
    
    /*
        regular filter run
     */

    v3SetZero(ConfigData->LrF[0]);
    for (i=0;i<NUM_LPF;i++) {
        v3Scale(ConfigData->b[i], ConfigData->Lr[i], v3);
        v3Add(v3, ConfigData->LrF[0], ConfigData->LrF[0]);
    }
    for (i=1;i<NUM_LPF;i++) {
        v3Scale(ConfigData->a[i], ConfigData->LrF[i], v3);
        v3Add(v3, ConfigData->LrF[0], ConfigData->LrF[0]);
    }
    v3Scale(1.0/ConfigData->a[0], ConfigData->LrF[0], ConfigData->LrF[0]);


    /* reset the filter state history */
    for (i=1;i<NUM_LPF;i++) {
        v3Copy(ConfigData->Lr[NUM_LPF-1-i],  ConfigData->Lr[NUM_LPF-i]);
        v3Copy(ConfigData->LrF[NUM_LPF-1-i], ConfigData->LrF[NUM_LPF-i]);
    }

    /*
        store the output message 
     */
    v3Copy(ConfigData->LrF[0], ConfigData->controlOut.torqueRequestBody);
    
    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(CmdTorqueBodyIntMsg),
                 (void*) &(ConfigData->controlOut), moduleID);

    return;
}
