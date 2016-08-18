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
    Thruster RW Momentum Management
 
 */

/* modify the path to reflect the new module names */
#include "attControl/thrMomentumManagement/thrMomentumManagement.h"

/* update this include to reflect the required module input messages */
#include "attControl/MRP_Steering/MRP_Steering.h"
#include "effectorInterfaces/_GeneralModuleFiles/rwSpeedData.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"



/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */
#include "SimCode/utilities/linearAlgebra.h"
//#include "SimCode/utilities/rigidBodyKinematics.h"
//#include "SimCode/utilities/astroConstants.h"
//#include "vehicleConfigData/ADCSAlgorithmMacros.h"


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
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(vehControlOut),
                                               "vehControlOut",          /* add the output structure name */
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
    ConfigData->inputRWConfID = subscribeToMessage(ConfigData->inputRWConfigData,
                                                  sizeof(RWConstellation), moduleID);
    ConfigData->inputRWSpeedsID = subscribeToMessage(ConfigData->inputRWSpeedsName,
                                                     sizeof(RWSpeedData), moduleID);
    ConfigData->inputVehicleConfigDataID = subscribeToMessage(ConfigData->inputVehicleConfigDataName,
                                                              sizeof(vehicleConfigData), moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_thrMomentumManagement(thrMomentumManagementConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    RWConstellation localRWData;
    vehicleConfigData   sc;                 /*!< spacecraft configuration message */
    uint64_t clockTime;
    uint32_t readSize;
    int i;

    ReadMessage(ConfigData->inputVehicleConfigDataID, &clockTime, &readSize,
                sizeof(vehicleConfigData), (void*) &(sc), moduleID);
    ReadMessage(ConfigData->inputRWConfID, &clockTime, &readSize,
                sizeof(RWConstellation), &localRWData, moduleID);
    ConfigData->numRW = localRWData.numRW;

    for(i=0; i<ConfigData->numRW; i=i+1)
    {
        ConfigData->JsList[i] = localRWData.reactionWheels[i].Js;
        m33MultV3(RECAST3X3 sc.BS,
                  localRWData.reactionWheels[i].gsHat_S,
                  &ConfigData->GsMatrix[i*3]);
    }

    ConfigData->status = DUMPING_OFF;
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
    RWSpeedData         wheelSpeeds;        /*!< Reaction wheel speed estimates */
    double              hs;                 /*!< net RW cluster angularl momentum magnitude */
    double              hs_B[3];            /*!< RW angular momentum */
    double              vec3[3];            /*!< temp vector */
    int i;


    /*! - Read the input messages */
    ReadMessage(ConfigData->inputRWSpeedsID, &clockTime, &readSize,
                sizeof(RWSpeedData), (void*) &(wheelSpeeds), moduleID);

    /* compute net RW momentum magnitude */
    v3SetZero(hs_B);
    for (i=0;i<ConfigData->numRW;i++) {
        v3Scale(ConfigData->JsList[i],&ConfigData->GsMatrix[i*3],vec3);
        v3Add(hs_B, vec3, hs_B);
    }
    hs = v3Norm(hs_B);

    if (hs < ConfigData->hs_min || ConfigData->status == DUMPING_COMPLETED ) {
        /* Momentum dumping completed or not required */
        ConfigData->status = DUMPING_COMPLETED;
        v3SetZero(ConfigData->Delta_H_B);
    } else {
        if (ConfigData->status == DUMPING_OFF) {
            /* turn on momentum dumping */
            ConfigData->status = DUMPING_ON;
            v3Scale((hs - ConfigData->hs_min)/hs*1.1, hs_B, ConfigData->Delta_H_B);
        }
    }


    /*
     store the output message 
     */
    v3Copy(ConfigData->Delta_H_B, ConfigData->controlOut.torqueRequestBody);

    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehControlOut),
                 (void*) &(ConfigData->controlOut), moduleID);


    return;
}
