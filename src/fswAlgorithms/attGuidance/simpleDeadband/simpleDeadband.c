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
    Attitude Tracking simple Module
 
 */

/* modify the path to reflect the new module names */
#include "attGuidance/simpleDeadband/simpleDeadband.h"
#include <string.h>
#include <math.h>
#include "fswUtilities/fswDefinitions.h"
#include "simFswInterfaceMessages/macroDefinitions.h"

/* update this include to reflect the required module input messages */
#include "attGuidance/attTrackingError/attTrackingError.h"



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
void SelfInit_simpleDeadband(simpleDeadbandConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputGuidID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(AttGuidFswMsg),
                                               "AttGuidFswMsg",
                                               moduleID);
    ConfigData->wasControlOff = 1;
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_simpleDeadband(simpleDeadbandConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputGuidID = subscribeToMessage(ConfigData->inputGuidName,
                                                sizeof(AttGuidFswMsg),
                                                moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the MRP steering control
 */
void Reset_simpleDeadband(simpleDeadbandConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    ConfigData->wasControlOff = 1;
}

/*! This method parses the input data, checks if the deadband needs to be applied and outputs
 the guidance command with simples either zeroed (control OFF) or left unchanged (control ON)
 @return void
 @param ConfigData The configuration data associated with the attitude tracking simple module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_simpleDeadband(simpleDeadbandConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! - Read the input message and set it as the output by default */
    uint64_t    clockTime;
    uint32_t    readSize;
    ReadMessage(ConfigData->inputGuidID, &clockTime, &readSize,
                sizeof(AttGuidFswMsg), (void*) &(ConfigData->attGuidOut), moduleID);
    
    /*! - Evaluate average simple in attitude and rates */
    ConfigData->attError = 4.0 * atan(v3Norm(ConfigData->attGuidOut.sigma_BR));
    ConfigData->rateError = v3Norm(ConfigData->attGuidOut.omega_BR_B);
    
    /*! - Check whether control should be ON or OFF */
    applyDBLogic_simpleDeadband(ConfigData);
    
    /*! - Write output guidance message and update module knowledge of control status*/
    WriteMessage(ConfigData->outputGuidID, callTime, sizeof(AttGuidFswMsg),
                 (void*) &(ConfigData->attGuidOut), moduleID);
    return;
}


/*! This method applies a two-level deadbanding logic (according to the current average simple compared with the set threshold)
 and decides whether control should be switched ON/OFF or not.
 @return void
 @param ConfigData The configuration data associated with the attitude tracking simple module
 */
void applyDBLogic_simpleDeadband(simpleDeadbandConfig *ConfigData)
{
    uint32_t areErrorsBelowUpperThresh = (ConfigData->attError < ConfigData->outerAttThresh && ConfigData->rateError < ConfigData->outerRateThresh);
    uint32_t areErrorsBelowLowerThresh = (ConfigData->attError < ConfigData->innerAttThresh && ConfigData->rateError < ConfigData->innerRateThresh);
    
    if (areErrorsBelowUpperThresh)
    {
        if ((areErrorsBelowLowerThresh == 1) || ((areErrorsBelowLowerThresh == 0) && ConfigData->wasControlOff))
        {
            /* Set simples to zero in order to turn off control */
            v3SetZero(ConfigData->attGuidOut.sigma_BR);
            v3SetZero(ConfigData->attGuidOut.omega_BR_B);
            ConfigData->wasControlOff = 1;
        } else {
            ConfigData->wasControlOff = 0;
        }
    } else { ConfigData->wasControlOff = 0; }
}




