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

#include "sensorInterfaces/CSSSensorData/cssComm.h"
#include "messaging/static_messaging.h"
#include <string.h>
#include <stdio.h> 

/*! This method initializes the configData for theCSS sensor interface.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param configData The configuration data associated with the CSS sensor interface
 @param moduleID The ID associated with the configData
 */
void SelfInit_cssProcessTelem(CSSConfigData *configData, int64_t moduleID)
{
    /*! - Create output message for module */
    configData->OutputMsgID = CreateNewMessage(configData->OutputDataName,
        sizeof(CSSArraySensorIntMsg), "CSSArraySensorIntMsg", moduleID);
}

/*! This method performs the second stage of initialization for the CSS sensor
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param configData The configuration data associated with the CSS interface
 @param moduleID The ID associated with the configData
 */
void CrossInit_cssProcessTelem(CSSConfigData *configData, int64_t moduleID)
{
    configData->SensorMsgID = subscribeToMessage(
        configData->SensorListName, sizeof(CSSArraySensorIntMsg), moduleID);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.
 @return void
 @param configData The configuration data associated with the guidance module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void Reset_cssProcessTelem(CSSConfigData *configData, uint64_t callTime, int64_t moduleID)
{
    /*! - Check to make sure that number of sensors is less than the max and warn if none are set*/
    if(configData->NumSensors > MAX_NUM_CSS_SENSORS)
    {
        char info[MAX_LOGGING_LENGTH];
        sprintf(info, "The configured number of CSS sensors exceeds the maximum, %d > %d! Changing the number of sensors to the max.", configData->NumSensors, MAX_NUM_CSS_SENSORS);
        _bskLog(configData->bskLogger, BSK_WARNING, info);
        configData->NumSensors = MAX_NUM_CSS_SENSORS;
    }
    else if (configData->NumSensors == 0)
    {
        _bskLog(configData->bskLogger, BSK_WARNING, "There are zero CSS configured!");
    }
    
    if (configData->MaxSensorValue == 0)
    {
        _bskLog(configData->bskLogger, BSK_WARNING, "Max CSS sensor value configured to zero! CSS sensor values will be normalized by zero, inducing faux saturation!");
    }
    
    memset(configData->InputValues.CosValue, 0x0, configData->NumSensors*sizeof(double));

    return;
}


/*! This method takes the raw sensor data from the coarse sun sensors and
 converts that information to the format used by the CSS nav.
 @return void
 @param configData The configuration data associated with the CSS interface
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The ID associated with the configData
 */
void Update_cssProcessTelem(CSSConfigData *configData, uint64_t callTime,
    int64_t moduleID)
{
    uint32_t i, j;
    uint64_t timeOfMsgWritten;
    uint32_t sizeOfMsgWritten;
    double InputValues[MAX_NUM_CSS_SENSORS]; /* [-] Current measured CSS value for the constellation of CSS sensor */
    double ChebyDiffFactor, ChebyPrev, ChebyNow, ChebyLocalPrev, ValueMult; /* Parameters used for the Chebyshev Recursion Forumula */
    CSSArraySensorIntMsg OutputBuffer;
    
    memset(&OutputBuffer, 0x0, sizeof(CSSArraySensorIntMsg));
    
    ReadMessage(configData->SensorMsgID, &timeOfMsgWritten, &sizeOfMsgWritten, sizeof(CSSArraySensorIntMsg),
                (void*) (InputValues), moduleID);
    
    /*! - Loop over the sensors and compute data
         -# Check appropriate range on sensor and calibrate
         -# If Chebyshev polynomials are configured:
             - Seed polynominal computations
             - Loop over polynominals to compute estimated correction factor
             - Output is base value plus the correction factor
         -# If sensor output range is incorrect, set output value to zero
     */
    for(i=0; i<configData->NumSensors; i++)
    {
        OutputBuffer.CosValue[i] = (float) InputValues[i]/configData->MaxSensorValue; /* Scale Sensor Data */
        
        /* Seed the polynomial computations */
        ValueMult = 2.0*OutputBuffer.CosValue[i];
        ChebyPrev = 1.0;
        ChebyNow = OutputBuffer.CosValue[i];
        ChebyDiffFactor = 0.0;
        ChebyDiffFactor = configData->ChebyCount > 0 ? ChebyPrev*configData->KellyCheby[0] : ChebyDiffFactor; /* if only first order correction */
        ChebyDiffFactor = configData->ChebyCount > 1 ? ChebyNow*configData->KellyCheby[1] + ChebyDiffFactor : ChebyDiffFactor; /* if higher order (> first) corrections */
        
        /* Loop over remaining polynomials and add in values */
        for(j=2; j<configData->ChebyCount; j = j+1)
        {
            ChebyLocalPrev = ChebyNow;
            ChebyNow = ValueMult*ChebyNow - ChebyPrev;
            ChebyPrev = ChebyLocalPrev;
            ChebyDiffFactor += configData->KellyCheby[j]*ChebyNow;
        }
        
        OutputBuffer.CosValue[i] = OutputBuffer.CosValue[i] + ChebyDiffFactor;
        
        if(OutputBuffer.CosValue[i] > 1.0)
        {
            OutputBuffer.CosValue[i] = 1.0;
        }
        else if(OutputBuffer.CosValue[i] < 0.0)
        {
            OutputBuffer.CosValue[i] = 0.0;
        }
    }
    
    /*! - Write aggregate output into output message */
    WriteMessage(configData->OutputMsgID, callTime,
                 sizeof(CSSArraySensorIntMsg), (void*) &OutputBuffer,
                 moduleID);
    
    return;
}
