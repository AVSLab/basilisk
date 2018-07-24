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
    Thruster RW Momentum Management
 
 */

/* modify the path to reflect the new module names */
#include "attControl/thrMomentumManagement/thrMomentumManagement.h"

/* update this include to reflect the required module input messages */
#include "attControl/MRP_Steering/MRP_Steering.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
#include <string.h>


/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */
#include "simulation/utilities/linearAlgebra.h"


/*! This method initializes the ConfigData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_thrMomentumManagement(thrMomentumManagementConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->deltaHOutMsgID = CreateNewMessage(ConfigData->deltaHOutMsgName,
                                               sizeof(CmdTorqueBodyIntMsg),
                                               "CmdTorqueBodyIntMsg",          /* add the output structure name */
                                               moduleID);

}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_thrMomentumManagement(thrMomentumManagementConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the other message IDs */
    ConfigData->rwConfInMsgID = subscribeToMessage(ConfigData->rwConfigDataInMsgName,
                                                  sizeof(RWArrayConfigFswMsg), moduleID);
    ConfigData->rwSpeedsInMsgID = subscribeToMessage(ConfigData->rwSpeedsInMsgName,
                                                     sizeof(RWSpeedIntMsg), moduleID);
    ConfigData->vehicleConfigDataInMsgID = subscribeToMessage(ConfigData->vehicleConfigDataInMsgName,
                                                              sizeof(VehicleConfigFswMsg), moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_thrMomentumManagement(thrMomentumManagementConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    VehicleConfigFswMsg   sc;                 /*!< spacecraft configuration message */
    uint64_t clockTime;
    uint32_t readSize;

    ReadMessage(ConfigData->vehicleConfigDataInMsgID, &clockTime, &readSize,
                sizeof(VehicleConfigFswMsg), (void*) &(sc), moduleID);

    ReadMessage(ConfigData->rwConfInMsgID, &clockTime, &readSize,
                sizeof(RWArrayConfigFswMsg), &(ConfigData->rwConfigParams), moduleID);

    ConfigData->initRequest = 1;
    v3SetZero(ConfigData->Delta_H_B);
    memset(&(ConfigData->controlOut), 0x0, sizeof(CmdTorqueBodyIntMsg));
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_thrMomentumManagement(thrMomentumManagementConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t            clockTime;
    uint32_t            readSize;
    RWSpeedIntMsg      rwSpeedMsg;         /*!< Reaction wheel speed estimates */
    double              hs;                 /*!< net RW cluster angularl momentum magnitude */
    double              hs_B[3];            /*!< RW angular momentum */
    double              vec3[3];            /*!< temp vector */
    int i;


    if (ConfigData->initRequest == 1) {

        /*! - Read the input messages */
        ReadMessage(ConfigData->rwSpeedsInMsgID, &clockTime, &readSize,
                    sizeof(RWSpeedIntMsg), (void*) &(rwSpeedMsg), moduleID);

        /* compute net RW momentum magnitude */
        v3SetZero(hs_B);
        for (i=0;i<ConfigData->rwConfigParams.numRW;i++) {
            v3Scale(ConfigData->rwConfigParams.JsList[i]*rwSpeedMsg.wheelSpeeds[i],&ConfigData->rwConfigParams.GsMatrix_B[i*3],vec3);
            v3Add(hs_B, vec3, hs_B);
        }
        hs = v3Norm(hs_B);

        if (hs < ConfigData->hs_min) {
            /* Momentum dumping not required */
            v3SetZero(ConfigData->Delta_H_B);
        } else {
            v3Scale(-(hs - ConfigData->hs_min)/hs, hs_B, ConfigData->Delta_H_B);
        }
        ConfigData->initRequest = 0;


        /*
         store the output message 
         */
        v3Copy(ConfigData->Delta_H_B, ConfigData->controlOut.torqueRequestBody);

        WriteMessage(ConfigData->deltaHOutMsgID, callTime, sizeof(CmdTorqueBodyIntMsg),
                     (void*) &(ConfigData->controlOut), moduleID);

    }

    return;
}
