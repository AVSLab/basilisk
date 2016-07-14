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
    Attitude Tracking Error Module
 
 */

/* modify the path to reflect the new module names */
#include "attGuidance/errorDeadband/errorDeadband.h"
#include <string.h>
#include <math.h>
#include "ADCSUtilities/ADCSDefinitions.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"

/* update this include to reflect the required module input messages */
#include "attGuidance/attTrackingError/attTrackingError.h"



/*
 Pull in support files from other modules.  Be sure to use the absolute path relative to Basilisk directory.
 */
#include "SimCode/utilities/linearAlgebra.h"
#include "SimCode/utilities/rigidBodyKinematics.h"


/*! This method initializes the ConfigData for this module.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with this module
 */
void SelfInit_errorDeadband(errorDeadbandConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    ConfigData->outputGuidID = CreateNewMessage(ConfigData->outputDataName,
                                               sizeof(attGuidOut),
                                               "attGuidOut",
                                               moduleID);
    v3Set(-1, -1, -1, ConfigData->sigma_BR);
}

/*! This method performs the second stage of initialization for this module.
 It's primary function is to link the input messages that were created elsewhere.
 @return void
 @param ConfigData The configuration data associated with this module
 */
void CrossInit_errorDeadband(errorDeadbandConfig *ConfigData, uint64_t moduleID)
{
    /*! - Get the control data message ID*/
    ConfigData->inputGuidID = subscribeToMessage(ConfigData->inputGuidName,
                                                sizeof(attGuidOut),
                                                moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param ConfigData The configuration data associated with the MRP steering control
 */
void Reset_errorDeadband(errorDeadbandConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    v3Set(-1, -1, -1, ConfigData->sigma_BR);

}

/*! This method parses the input data, checks if the deadband needs to be applied and outputs
 the guidance command with errors either zeroed (control OFF) or left unchanged (control ON)
 @return void
 @param ConfigData The configuration data associated with the attitude tracking error module
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_errorDeadband(errorDeadbandConfig *ConfigData, uint64_t callTime, uint64_t moduleID)
{
    /*! - Read the input message and set it as the output by default */
    uint64_t    clockTime;
    uint32_t    readSize;
    ReadMessage(ConfigData->inputGuidID, &clockTime, &readSize,
                sizeof(attGuidOut), (void*) &(ConfigData->attGuidOut), moduleID);
    
    /*! - Evaluate average error in attitude and rates */
    double attError;
    double rateError;
    attError = 4 * atan(v3Norm(ConfigData->attGuidOut.sigma_BR));
    rateError = v3Norm(ConfigData->attGuidOut.omega_BR_B);
    ConfigData->error = sqrt(attError * attError + rateError * rateError);
    
    /*! - Check whether control should be ON or OFF */
    applyDBLogic_errorDeadband(ConfigData);
    
    /*! - Write output guidance message and update module knowledge of control status*/
    WriteMessage(ConfigData->outputGuidID, callTime, sizeof(attGuidOut),
                 (void*) &(ConfigData->attGuidOut), moduleID);
    v3Copy(ConfigData->attGuidOut.sigma_BR, ConfigData->sigma_BR);
    v3Copy(ConfigData->attGuidOut.omega_BR_B, ConfigData->omega_BR_B);
    return;
}


/*! This method applies a two-level deadbanding logic (according to the current average error compared with the set threshold)
 and decides whether control should be switched ON/OFF or not.
 @return void
 @param ConfigData The configuration data associated with the attitude tracking error module
 */
void applyDBLogic_errorDeadband(errorDeadbandConfig *ConfigData)
{
    if (ConfigData->error < ConfigData->outerThresh)
    {
        if (ConfigData->error < ConfigData->innerThresh || (ConfigData->error > ConfigData->innerThresh && wasControlOff_errorDeadband(ConfigData)))
        {
            /* Set errors to zero in order to turn off control */
            v3SetZero(ConfigData->attGuidOut.sigma_BR);
            v3SetZero(ConfigData->attGuidOut.omega_BR_B);
        }
    }
}

/*! This method checks the previous status of the deadband. Returns 1 if control was OFF (errors were set to zero in the previous step)
 or 0 if control was ON (errors were passed to control as non-zero). The reason of this function is that a two-level deadbanding needs
 to keep track of the direction in which the upper and lower limits of the deadband are crossed (i.e. whether the Control was OFF or ON).
 @return uint32_t
 @param ConfigData The configuration data associated with the attitude tracking error module
 */
uint32_t wasControlOff_errorDeadband(errorDeadbandConfig *ConfigData)
{
    double zeroVec[3];
    v3SetZero(zeroVec);
    return v3IsEqual(ConfigData->sigma_BR, zeroVec, 1E-12) && v3IsEqual(ConfigData->omega_BR_B, zeroVec, 1E-12);
}


