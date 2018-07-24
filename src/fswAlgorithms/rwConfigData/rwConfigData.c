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
    FSW MODULE Template
 
 */

/* modify the path to reflect the new module names */
#include "rwConfigData/rwConfigData.h"
#include "simulation/utilities/linearAlgebra.h"
#include "simFswInterfaceMessages/macroDefinitions.h"
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
void SelfInit_rwConfigData(rwConfigData_Config *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->rwParamsOutMsgID = CreateNewMessage(ConfigData->rwParamsOutMsgName,
                                                    sizeof(RWArrayConfigFswMsg), "RWArrayConfigFswMsg", moduleID);
    
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_rwConfigData(rwConfigData_Config *ConfigData, uint64_t moduleID)
{
    /*! - Read vehicle config data, convert RW info from S to B and write it in the output mesage */
    /*! - NOTE: it is important that this initialization takes place in CrossInit and not Reset.
     When Reset call takes place in all the modules, this RW message should already be available.*/
    ConfigData->vehConfigInMsgID = subscribeToMessage(ConfigData->vehConfigInMsgName,
                                                              sizeof(VehicleConfigFswMsg), moduleID);
    int i;
    uint64_t clockTime;
    uint32_t readSize;
    VehicleConfigFswMsg   sc;                 /*!< spacecraft configuration message */
    
    ReadMessage(ConfigData->vehConfigInMsgID, &clockTime, &readSize,
                sizeof(VehicleConfigFswMsg), (void*) &(sc), moduleID);
    
    if(strlen(ConfigData->rwConstellationInMsgName) > 0)
    {
        ConfigData->rwConstellationInMsgID = subscribeToMessage(ConfigData->rwConstellationInMsgName,
                                                          sizeof(RWConstellationFswMsg), moduleID);
        ReadMessage(ConfigData->rwConstellationInMsgID, &clockTime, &readSize, sizeof(RWConstellationFswMsg),
            &ConfigData->rwConstellation, moduleID);
    }
    
    ConfigData->rwConfigParamsOut.numRW = ConfigData->rwConstellation.numRW;
    
    for(i=0; i<ConfigData->rwConfigParamsOut.numRW; i=i+1)
    {
        ConfigData->rwConfigParamsOut.JsList[i] = ConfigData->rwConstellation.reactionWheels[i].Js;
        ConfigData->rwConfigParamsOut.uMax[i] = ConfigData->rwConstellation.reactionWheels[i].uMax;
        v3Copy(ConfigData->rwConstellation.reactionWheels[i].gsHat_B, &ConfigData->rwConfigParamsOut.GsMatrix_B[i*3]);
    }
    /*! - Write output RW config data to the messaging system*/
    WriteMessage(ConfigData->rwParamsOutMsgID, 0, sizeof(RWArrayConfigFswMsg),
                 &(ConfigData->rwConfigParamsOut), moduleID);

}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_rwConfigData(rwConfigData_Config *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_rwConfigData(rwConfigData_Config *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! Nothing done in this method.  Make sure this is still true!*/
    return;
}
