/*
    SUB_MODULE Template
 
 * University of Colorado, Autonomous Vehicle Systems (AVS) Lab
 * Unpublished Copyright (c) 2012-2015 University of Colorado, All Rights Reserved

 */

/* modify the path to reflect the new module names */
#include " moduleTemplate/ subModuleTemplate/subModuleTemplate.h"


/*
 Pull in supprt files from other modules.  Be sure to use the absolute path.
 */
//#include "Utilities/rigidBodyKinematics.h"


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
        sizeof(subModuleOut), "subModuleOut", moduleID);

}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_subModuleTemplate(subModuleTemplateConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
//    ConfigData->inputGuidID = subscribeToMessage(ConfigData->inputGuidName,
//                                                 sizeof(attGuidOut), moduleID);

}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the MRP steering control
 */
void Reset_subModuleTemplate(subModuleTemplateConfig *ConfigData)
{
    ConfigData->dummy = 0;              /* reset any required variables */
}

/*! This method takes the attitude and rate errors relative to the Reference frame, as well as
    the reference frame angular rates and acceleration, and computes the required control torque Lr.
 @return void
 @param ConfigData The configuration data associated with the MRP Steering attitude control
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_subModuleTemplate(subModuleTemplateConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
//    uint64_t            clockTime;
//    uint32_t            readSize;
//    double              variable;           /*!< [unit] variable description */


    /*! Begin method steps*/
    /*! - Read the input messages */
//    ReadMessage(ConfigData->inputMsgID, &clockTime, &readSize,
//                sizeof(attGuidOut), (void*) &(guidCmd));



    /*
        Add the module specific code
     */


    /*
     store the output message 
     */
//    v3Copy(Lr, ConfigData->controlOut.torqueRequestBody);
//    
//    WriteMessage(ConfigData->outputMsgID, callTime, sizeof(vehControlOut),
//                 (void*) &(ConfigData->controlOut), moduleID);

    return;
}
