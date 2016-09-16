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

#include "attDetermination/SunlineUKF/sunlineUKF.h"
#include "attDetermination/_GeneralModuleFiles/UKFUtilities.h"
#include "SimCode/utilities/linearAlgebra.h"
#include "ADCSUtilities/ADCSAlgorithmMacros.h"
#include "vehicleConfigData/vehicleConfigData.h"
#include <string.h>

/*! This method initializes the ConfigData for theCSS WLS estimator.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the CSS WLS estimator
 */
void SelfInit_sunlineUKF(SunlineUKFConfig *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - Create output message for module */
    /*ConfigData->OutputMsgID = CreateNewMessage(ConfigData->OutputDataName,
        sizeof(CSSWlsEstOut), "CSSWlsEstOut", moduleID);*/
    
}

/*! This method performs the second stage of initialization for the CSS sensor
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the CSS interface
 */
void CrossInit_sunlineUKF(SunlineUKFConfig *ConfigData, uint64_t moduleID)
{
    /*int32_t i;
    vehicleConfigData localConfigData;
    uint64_t writeTime;
    uint32_t writeSize;*/
    /*! - Loop over the number of sensors and find IDs for each one */
    /*ConfigData->InputMsgID = subscribeToMessage(ConfigData->InputDataName,
        MAX_NUM_CSS_SENSORS*sizeof(CSSOutputData), moduleID);
    ConfigData->InputPropsID = subscribeToMessage(ConfigData->InputPropsName,
        sizeof(vehicleConfigData), moduleID);
    ReadMessage(ConfigData->InputPropsID, &writeTime, &writeSize,
                sizeof(vehicleConfigData), &localConfigData, moduleID);
    for(i=0; i<MAX_NUM_CSS_SENSORS; i = i+1)
    {
        m33MultV3(RECAST3X3 localConfigData.BS, ConfigData->CSSData[i].nHatStr,
                  ConfigData->CSSData[i].nHatBdy);
    }*/
    
}

/*! This method takes the parsed CSS sensor data and outputs an estimate of the
 sun vector in the ADCS body frame
 @return void
 @param ConfigData The configuration data associated with the CSS estimator
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_sunlineUKF(SunlineUKFConfig *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    
    return;
}
