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

#include "ADCSUtilities/ADCSAlgorithmMacros.h"
#include "vehicleConfigData/vehicleConfigData.h"
#include "SimCode/utilities/linearAlgebra.h"
#include <string.h>

/*! This method initializes the ConfigData for the veh config algorithm.  
    It initializes the output message in the messaging system.
 @return void
 @param ConfigData The configuration data associated with the vehcle config interface
 */
void SelfInit_vehicleConfigData(VehConfigInputData *ConfigData, uint64_t moduleID)
{

    double SB[3][3];
    double halfInertia[3][3];
    vehicleConfigData localConfigData;
    
    memset(&localConfigData, 0x0, sizeof(vehicleConfigData));

    ConfigData->outputPropsID = CreateNewMessage(
        ConfigData->outputPropsName, sizeof(vehicleConfigData),
        "vehicleConfigData", moduleID);
    m33Transpose(RECAST3X3 ConfigData->BS, SB);
    
    m33MultV3(RECAST3X3 ConfigData->BS, ConfigData->CoM_S,
        localConfigData.CoM_B);
    m33Copy(RECAST3X3 ConfigData->BS, RECAST3X3 localConfigData.BS);
    m33MultM33(RECAST3X3 ConfigData->BS, RECAST3X3 ConfigData->ISCPntB_S,
        halfInertia);
    m33MultM33t(halfInertia, RECAST3X3 ConfigData->BS,
        RECAST3X3 localConfigData.ISCPntB_B);
    
    WriteMessage(ConfigData->outputPropsID, 0, sizeof(vehicleConfigData),
        &localConfigData, moduleID);
    

}

/*! This method performs the second stage of initialization for the vehicle config 
    data interface.  No operations are performed here currently.
 @return void
 @param ConfigData The configuration data associated with the veh config interface
 */
void CrossInit_vehicleConfigData(VehConfigInputData *ConfigData, uint64_t moduleID)
{
}

/*! There are no runtime operations performed by the vehicle configuration 
    module.
 @return void
 @param ConfigData The configuration data associated with the veh config module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_vehicleConfigData(VehConfigInputData *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    
    return;
}
