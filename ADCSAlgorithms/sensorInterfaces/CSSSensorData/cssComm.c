
#include "sensorInterfaces/CSSSensorData/cssComm.h"
#include <string.h>

/*! This method initializes the ConfigData for theCSS sensor interface.
    It checks to ensure that the inputs are sane and then creates the 
    output message
    @return void
    @param ConfigData The configuration data associated with the CSS sensor interface
*/
void SelfInit_cssProcessTelem(CSSConfigData *ConfigData)
{

   /*! - Check to make sure that number of sensors is less than the max*/
   if(ConfigData->NumSensors >= MAX_NUM_CSS_SENSORS)
   {
      return; /* Throw ugly FSW error/crash here */
   }
   /*! - Create output message for module */
   ConfigData->OutputMsgID = CreateNewMessage(ConfigData->OutputDataName, 
      sizeof(CSSOutputData)*MAX_NUM_CSS_SENSORS);

}

/*! This method performs the second stage of initialization for the CSS sensor
    interface.  It's primary function is to link the input messages that were 
    created elsewhere.
    @return void
    @param ConfigData The configuration data associated with the CSS interface
*/
void CrossInit_cssProcessTelem(CSSConfigData *ConfigData)
{
   uint32_t i;

   /*! Begin method steps */
   /*! - If num sensors is past max, quit*/
   if(ConfigData->NumSensors >= MAX_NUM_CSS_SENSORS)
   {
      return; /* Throw ugly FSW error/crash here */
   }
   /*! - Loop over the number of sensors and find IDs for each one */
   for(i=0; i<ConfigData->NumSensors; i++)
   {
      ConfigData->SensorMsgIDs[i] = FindMessageID(ConfigData->SensorList[i].SensorMsgName);
   }
}

/*! This method takes the raw sensor data from the coarse sun sensors and 
    converts that information to the format used by the CSS nav.  
    @return void
    @param ConfigData The configuration data associated with the CSS interface
    @param callTime The clock time at which the function was called (nanoseconds)
*/
void Update_cssProcessTelem(CSSConfigData *ConfigData, uint64_t callTime)
{
   uint32_t i, j;
   uint64_t ClockTime;
   uint32_t ReadSize;
   double InputValues[MAX_NUM_CSS_SENSORS];
   double ChebyDiffFactor, ChebyPrev, ChebyNow, ChebyLocalPrev, ValueMult;
   CSSOutputData OutputBuffer[MAX_NUM_CSS_SENSORS];

   /*! Begin method steps*/
   /*! - Check for correct values in NumSensors and MaxSensorValue */
   if(ConfigData->NumSensors >= MAX_NUM_CSS_SENSORS || 
      ConfigData->MaxSensorValue <= 0.0)
   {
      return; /* Throw ugly FSW error/crash here */
   }
   memset(OutputBuffer, 0x0, MAX_NUM_CSS_SENSORS*sizeof(CSSOutputData));
   /*! - Loop over the sensors and compute data
          -# Read the message associated with the sensor.
          -# Check appropriate range on sensor and calibrate
          -# If Chebyshev polynomials are configured:
             - Seed polynominal computations
             - Loop over polynominals to compute estimated correction factor
             - Output is base value plus the correction factor
          -# If range is incorrect, set output value to zero */
   for(i=0; i<ConfigData->NumSensors; i++)
   {
      ReadMessage(ConfigData->SensorMsgIDs[i], &ClockTime, &ReadSize, 
         sizeof(double), (void*) &(InputValues[i]));
      if(InputValues[i] < ConfigData->MaxSensorValue && InputValues[i] >= 0.0)
      {
         /* Scale sensor */
         OutputBuffer[i].CosValue = (float) InputValues[i]/
            ConfigData->MaxSensorValue;
         /* Seed the polynomial computations*/
         ValueMult = 2.0*OutputBuffer[i].CosValue;
         ChebyPrev = 1.0;
         ChebyNow = OutputBuffer[i].CosValue;
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
         OutputBuffer[i].CosValue = OutputBuffer[i].CosValue + ChebyDiffFactor;
      }
      else
      {
         OutputBuffer[i].CosValue = 0.0;
      }
   }
   /*! - Write aggregate output into output message */
   WriteMessage(ConfigData->OutputMsgID, callTime, 
       MAX_NUM_CSS_SENSORS*sizeof(CSSOutputData), (void*) OutputBuffer);

   return;
}
