/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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
    FSW MODULE Template
 
 */

/* modify the path to reflect the new module names */
#include "rwConfigData/rwConfigData.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"
#include <string.h>

/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */


/*! This method initializes the ConfigData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_rwConfigData(rwConfigData *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->rwParamsOutMsgID = CreateNewMessage(ConfigData->rwParamsOutMsgName,
                                                    sizeof(RWConfigParams), "RWConfigParams", moduleID);
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_rwConfigData(rwConfigData *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->vehConfigInMsgID = subscribeToMessage(ConfigData->vehConfigInMsgName,
                                                              sizeof(vehicleConfigData), moduleID);
    
    if(strlen(ConfigData->rwConstellationInMsgName) > 0) {
        ConfigData->rwConstellationInMsgID = subscribeToMessage(ConfigData->rwConstellationInMsgName,
                                                       sizeof(RWConstellation), moduleID);
    } else {
        ConfigData->rwConfigParamsOut.numRW = 0;
        ConfigData->rwConstellationInMsgID = -1;
    }

}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_rwConfigData(rwConfigData *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    int i;
    uint64_t clockTime;
    uint32_t readSize;
    vehicleConfigData   sc;                 /*!< spacecraft configuration message */
    RWConstellation localRWConst;
    
    ReadMessage(ConfigData->vehConfigInMsgID, &clockTime, &readSize,
                sizeof(vehicleConfigData), (void*) &(sc), moduleID);
    
    ConfigData->rwConfigParamsOut.numRW = ConfigData->rwConstellation.numRW;
    localRWConst.numRW =ConfigData->rwConstellation.numRW;
    
    for(i=0; i<ConfigData->rwConfigParamsOut.numRW; i=i+1)
    {
        ConfigData->rwConfigParamsOut.JsList[i] = ConfigData->rwConstellation.reactionWheels[i].Js;
        localRWConst.reactionWheels[i].Js = ConfigData->rwConstellation.reactionWheels[i].Js;
        m33MultV3(RECAST3X3 sc.BS,
                  ConfigData->rwConstellation.reactionWheels[i].gsHat_S,
                  &ConfigData->rwConfigParamsOut.GsMatrix_B[i*3]);
        m33MultV3(RECAST3X3 sc.BS,
                  ConfigData->rwConstellation.reactionWheels[i].gsHat_S,
                  localRWConst.reactionWheels[i].gsHat_S);
    }
    /*! - Write output RW config data to the messaging system*/
    WriteMessage(ConfigData->rwParamsOutMsgID, 0, sizeof(RWConfigParams),
                 &(ConfigData->rwConfigParamsOut), moduleID);
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_rwConfigData(rwConfigData *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! Nothing done in this method.  Make sure this is still true!*/
    return;
}
