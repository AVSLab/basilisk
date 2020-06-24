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
#include "rwConfigData.h"
#include <string.h>

/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */


/*! This method initializes the configData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The ID associated with the configData
 */
void SelfInit_rwConfigData(rwConfigData_Config *configData, int64_t moduleID)
{
    /*! - Create output message for module */
    configData->rwParamsOutMsgID = CreateNewMessage(configData->rwParamsOutMsgName,
                                                    sizeof(RWArrayConfigFswMsg), "RWArrayConfigFswMsg", moduleID);
    
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param configData The configuration data associated with this module
 @param moduleID The ID associated with the configData
 */
void CrossInit_rwConfigData(rwConfigData_Config *configData, int64_t moduleID)
{
    /*! - Read vehicle config data, convert RW info from S to B and write it in the output mesage */
    /*! - NOTE: it is important that this initialization takes place in CrossInit and not Reset.
     When Reset call takes place in all the modules, this RW message should already be available.*/
    configData->vehConfigInMsgID = subscribeToMessage(configData->vehConfigInMsgName,
                                                              sizeof(VehicleConfigFswMsg), moduleID);

    configData->rwConstellationInMsgID = -1;
    if(strlen(configData->rwConstellationInMsgName) > 0)
    {
        configData->rwConstellationInMsgID = subscribeToMessage(configData->rwConstellationInMsgName,
                                                          sizeof(RWConstellationFswMsg), moduleID);
    }


}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void Reset_rwConfigData(rwConfigData_Config *configData, uint64_t callTime, int64_t moduleID)
{
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    int i;

    if(configData->rwConstellationInMsgID >= 0)
    {
        ReadMessage(configData->rwConstellationInMsgID, &timeOfMsgWritten, &sizeOfMsgWritten, sizeof(RWConstellationFswMsg),
                    (void *) &configData->rwConstellation, moduleID);
    }
    configData->rwConfigParamsOut.numRW = configData->rwConstellation.numRW;

    for(i=0; i<configData->rwConfigParamsOut.numRW; i=i+1)
    {
        configData->rwConfigParamsOut.JsList[i] = configData->rwConstellation.reactionWheels[i].Js;
        configData->rwConfigParamsOut.uMax[i] = configData->rwConstellation.reactionWheels[i].uMax;
        v3Copy(configData->rwConstellation.reactionWheels[i].gsHat_B, &configData->rwConfigParamsOut.GsMatrix_B[i*3]);
    }

    /*! - Write output RW config data to the messaging system*/
    WriteMessage(configData->rwParamsOutMsgID, callTime, sizeof(RWArrayConfigFswMsg),
                 &(configData->rwConfigParamsOut), moduleID);

}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void Update_rwConfigData(rwConfigData_Config *configData, uint64_t callTime, int64_t moduleID)
{
    /*! Nothing done in this method.  Make sure this is still true!*/
    return;
}
