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
    /*! - Get the control data message ID*/
    ConfigData->inputRWSpeedsID = subscribeToMessage(ConfigData->inputRWSpeedsName,
                                                     sizeof(RWSpeedData), moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_thrMomentumManagement(thrMomentumManagementConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    ConfigData->dummy = 0.0;              /* reset any required variables */
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
    double              Lr[3];              /*!< [unit] variable description */
    RWSpeedData         wheelSpeeds;        /*!< Reaction wheel speed estimates */


    /*! Begin method steps*/
    /*! - Read the input messages */
    ReadMessage(ConfigData->inputRWSpeedsID, &clockTime, &readSize,
                sizeof(RWSpeedData), (void*) &(wheelSpeeds), moduleID);



    /*
        Add the module specific code
     */
    v3Copy(ConfigData->inputVector, Lr);
    ConfigData->dummy += 1.0;
    Lr[0] += ConfigData->dummy;

    /*
     store the output message 
     */
    v3Copy(Lr, ConfigData->controlOut.torqueRequestBody);

    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehControlOut),
                 (void*) &(ConfigData->controlOut), moduleID);


    return;
}
