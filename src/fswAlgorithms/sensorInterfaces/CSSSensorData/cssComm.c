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

/*! This method initializes the ConfigData for theCSS sensor interface.
 It checks to ensure that the inputs are sane and then creates the
 output message
 @return void
 @param ConfigData The configuration data associated with the CSS sensor interface
 */
void SelfInit_cssProcessTelem(CSSConfigData *ConfigData, uint64_t moduleID)
{
    
    /*! - Check to make sure that number of sensors is less than the max*/
    if(ConfigData->NumSensors > MAX_NUM_CSS_SENSORS)
    {
        return; /* Throw ugly FSW error/crash here */
    }
    /*! - Create output message for module */
    ConfigData->OutputMsgID = CreateNewMessage(ConfigData->OutputDataName,
        sizeof(CSSArraySensorIntMsg), "CSSArraySensorIntMsg", moduleID);
    
}

/*! This method performs the second stage of initialization for the CSS sensor
 interface.  It's primary function is to link the input messages that were
 created elsewhere.
 @return void
 @param ConfigData The configuration data associated with the CSS interface
 */
void CrossInit_cssProcessTelem(CSSConfigData *ConfigData, uint64_t moduleID)
{
    
    /*! Begin method steps */
    /*! - If num sensors is past max, quit*/
    if(ConfigData->NumSensors > MAX_NUM_CSS_SENSORS)
    {
        return; /* Throw ugly FSW error/crash here */
    }
    
    ConfigData->SensorMsgID = subscribeToMessage(
        ConfigData->SensorListName, sizeof(CSSArraySensorIntMsg), moduleID);
}

/*! This method takes the raw sensor data from the coarse sun sensors and
 converts that information to the format used by the CSS nav.
 @return void
 @param ConfigData The configuration data associated with the CSS interface
 @param callTime The clock time at which the function was called (nanoseconds)
 */
void Update_cssProcessTelem(CSSConfigData *ConfigData, uint64_t callTime,
    uint64_t moduleID)
{
    uint32_t i, j;
    uint64_t ClockTime;
    uint32_t ReadSize;
    double InputValues[MAX_NUM_CSS_SENSORS];
    double ChebyDiffFactor, ChebyPrev, ChebyNow, ChebyLocalPrev, ValueMult;
    CSSArraySensorIntMsg OutputBuffer;
    
    /*! Begin method steps*/
    /*! - Check for correct values in NumSensors and MaxSensorValue */
    if(ConfigData->NumSensors >= MAX_NUM_CSS_SENSORS ||
       ConfigData->MaxSensorValue <= 0.0)
    {
        return; /* Throw ugly FSW error/crash here */
    }
    memset(&OutputBuffer, 0x0, sizeof(CSSArraySensorIntMsg));
    ReadMessage(ConfigData->SensorMsgID, &ClockTime, &ReadSize, sizeof(CSSArraySensorIntMsg),
                (void*) (InputValues), moduleID);
    /*! - Loop over the sensors and compute data
     -# Check appropriate range on sensor and calibrate
     -# If Chebyshev polynomials are configured:
     - Seed polynominal computations
     - Loop over polynominals to compute estimated correction factor
     - Output is base value plus the correction factor
     -# If range is incorrect, set output value to zero */
    for(i=0; i<ConfigData->NumSensors; i++)
    {
        if(InputValues[i] < ConfigData->MaxSensorValue && InputValues[i] >= 0.0)
        {
            /* Scale sensor */
            OutputBuffer.CosValue[i] = (float) InputValues[i]/
            ConfigData->MaxSensorValue;
            /* Seed the polynomial computations*/
            ValueMult = 2.0*OutputBuffer.CosValue[i];
            ChebyPrev = 1.0;
            ChebyNow = OutputBuffer.CosValue[i];
            ChebyDiffFactor = 0.0;
            ChebyDiffFactor = ConfigData->ChebyCount > 0 ?
            ChebyPrev*ConfigData->KellyCheby[0] : ChebyDiffFactor;
            ChebyDiffFactor = ConfigData->ChebyCount > 1 ?
            ChebyNow*ConfigData->KellyCheby[1] +
            ChebyDiffFactor : ChebyDiffFactor;
            /*Loop over remaining polynomials and add in values*/
            for(j=2; j<ConfigData->ChebyCount; j = j+1)
            {
                ChebyLocalPrev = ChebyNow;
                ChebyNow = ValueMult*ChebyNow - ChebyPrev;
                ChebyPrev = ChebyLocalPrev;
                ChebyDiffFactor += ConfigData->KellyCheby[j]*ChebyNow;
            }
            OutputBuffer.CosValue[i] = OutputBuffer.CosValue[i] + ChebyDiffFactor;
        }
        else
        {
            OutputBuffer.CosValue[i] = 0.0;
        }
    }
    /*! - Write aggregate output into output message */
    WriteMessage(ConfigData->OutputMsgID, callTime,
                 sizeof(CSSArraySensorIntMsg), (void*) &OutputBuffer,
                 moduleID);
    
    return;
}
