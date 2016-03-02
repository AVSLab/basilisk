/*
    SUB_MODULE Template
 
 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved

 */

/* modify the path to reflect the new module names */
#include "_moduleTemplate/subModuleTemplate/subModuleTemplate.h"

/* update this include to reflect the required module input messages */
#include "attControl/MRP_Steering/MRP_Steering.h"



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
void SelfInit_subModuleTemplate(subModuleTemplateConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputMsgID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(subModuleOut),
                                               "subModuleOut",          /* add the output structure name */
                                               moduleID);

}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_subModuleTemplate(subModuleTemplateConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputMsgID = subscribeToMessage(ConfigData->inputDataName,
                                                sizeof(subModuleOut),
                                                moduleID);

}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the module
 */
void Reset_subModuleTemplate(subModuleTemplateConfig *ConfigData)
{
    ConfigData->dummy = 0;              /* reset any required variables */
}

/*! Add a description of what this main Update() routine does for this module
 @return void
 @param ConfigData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_subModuleTemplate(subModuleTemplateConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    uint64_t            clockTime;
    uint32_t            readSize;
    double              Lr[3];              /*!< [unit] variable description */


    /*! Begin method steps*/
    /*! - Read the input messages */
    ReadMessage(ConfigData->inputMsgID, &clockTime, &readSize,
                sizeof(vehControlOut), (void*) &(ConfigData->inputVector));



    /*
        Add the module specific code
     */
    v3Copy(ConfigData->inputVector, Lr);
    ConfigData->dummy = 2.0;

    /*
     store the output message 
     */
    v3Copy(Lr, ConfigData->subModuleOut.outputVector);                      /* populate the output message */

    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(subModuleOut),   /* update module name */
                 (void*) &(ConfigData->subModuleOut), moduleID);

    return;
}
