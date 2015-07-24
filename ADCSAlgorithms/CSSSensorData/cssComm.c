
#include "cssComm.h"

/*! This method initializes the ConfigData for theCSS sensor interface.
    It checks to ensure that the inputs are sane and then creates the 
    output message
    @return void
    @param ConfigData The configuration data associated with the CSS sensor interface
*/
void SelfInit_cssProcessTelem(void *ConfigData)
{

   /*! Begin method steps */
   CSSConfigData *ConfigPtr;
   ConfigPtr = (CSSConfigData *) ConfigData;

   /*! - Check to make sure that number of sensors is less than the max*/
   if(ConfigPtr->NumSensors >= MAX_NUM_CSS_SENSORS)
   {
      return; /* Throw ugly FSW error/crash here */
   }
   /*! - Create output message for module */
   ConfigPtr->OutputMsgID = CreateNewMessage(ConfigPtr->OutputDataName, 
      sizeof(CSSOutputData)*ConfigPtr->NumSensors);

}

/*! This method performs the second stage of initialization for the CSS sensor
    interface.  It's primary function is to link the input messages that were 
    created elsewhere.
    @return void
    @param ConfigData The configuration data associated with the CSS interface
*/
void CrossInit_cssProcessTelem(void *ConfigData)
{
   CSSConfigData *ConfigPtr;
   uint32_t i;
   ConfigPtr = (CSSConfigData *) ConfigData;

   /*! Begin method steps */
   /*! - If num sensors is past max, quit*/
   if(ConfigPtr->NumSensors >= MAX_NUM_CSS_SENSORS)
   {
      return; /* Throw ugly FSW error/crash here */
   }
   /*! - Loop over the number of sensors and find IDs for each one */
   for(i=0; i<ConfigPtr->NumSensors; i++)
   {
      ConfigPtr->SensorMsgIDs[i] = FindMessageID(ConfigPtr->SensorList[i].SensorMsgName);
   }
}

/*! This method takes the raw sensor data from the coarse sun sensors and 
    converts that information to the format used by the CSS nav.  
    @return void
    @param ConfigData The configuration data associated with the CSS interface
*/
void Update_cssProcessTelem(void *ConfigData)
{
   uint32_t i;
   CSSConfigData *ConfigPtr;
   uint64_t ClockTime;
   uint32_t ReadSize;
   double InputValues[MAX_NUM_CSS_SENSORS];
   CSSOutputData OutputBuffer[MAX_NUM_CSS_SENSORS];

   /*! Begin method steps*/
   ConfigPtr = (CSSConfigData *) ConfigData;
   /*! - Check for correct values in NumSensors and MaxSensorValue */
   if(ConfigPtr->NumSensors >= MAX_NUM_CSS_SENSORS || 
      ConfigPtr->MaxSensorValue <= 0.0)
   {
      return; /* Throw ugly FSW error/crash here */
   }
   /*! - Loop over the sensors and compute data
          -# Read the message associated with the sensor.
          -# Check appropriate range on sensor and calibrate
          -# If range is incorrect, set output value to zero */
   for(i=0; i<ConfigPtr->NumSensors; i++)
   {
      ReadMessage(ConfigPtr->SensorMsgIDs[i], &ClockTime, &ReadSize, 
         sizeof(double), (void*) &(InputValues[i]));
      if(InputValues[i] < ConfigPtr->MaxSensorValue && InputValues[i] >= 0.0)
      {
         OutputBuffer[i].CosValue = (float) InputValues[i]/
            ConfigPtr->MaxSensorValue;
      }
      else
      {
         OutputBuffer[i].CosValue = 0.0;
      }
   }
   /*! - Write aggregate output into output message */
   WriteMessage(ConfigPtr->OutputMsgID, 0, 
      ConfigPtr->NumSensors*sizeof(CSSOutputData), (void*) OutputBuffer);

   return;
}
