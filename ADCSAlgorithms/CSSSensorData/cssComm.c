
#include "cssComm.h"

void SelfInit_cssProcessTelem(void *ConfigData)
{

   CSSConfigData *ConfigPtr;
   ConfigPtr = (CSSConfigData *) ConfigData;
   if(ConfigPtr->NumSensors >= MAX_NUM_CSS_SENSORS)
   {
      return; /* Throw ugly FSW error/crash here */
   }
   ConfigPtr->OutputMsgID = CreateNewMessage(ConfigPtr->OutputDataName, 
      sizeof(CSSOutputData)*ConfigPtr->NumSensors);

}

void CrossInit_cssProcessTelem(void *ConfigData)
{
   CSSConfigData *ConfigPtr;
   uint32_t i;
   ConfigPtr = (CSSConfigData *) ConfigData;
   if(ConfigPtr->NumSensors >= MAX_NUM_CSS_SENSORS)
   {
      return; /* Throw ugly FSW error/crash here */
   }
   for(i=0; i<ConfigPtr->NumSensors; i++)
   {
      ConfigPtr->SensorMsgIDs[i] = FindMessageID(ConfigPtr->SensorList[i].SensorMsgName);
   }
}

/*
* Function: cssProcessTelem
* Purpose: Processes CSS measurements and aggregate all sensors for output
* Inputs:
*   ConfigData = CSS configuration data as formless pointer
* Outputs:
*   Scaled cos-value outputs from the sun sensors
*/
void Update_cssProcessTelem(void *ConfigData)
{
   uint32_t i;
   CSSConfigData *ConfigPtr;
   uint64_t ClockTime;
   uint32_t ReadSize;
   double InputValues[MAX_NUM_CSS_SENSORS];
   CSSOutputData OutputBuffer[MAX_NUM_CSS_SENSORS];

   ConfigPtr = (CSSConfigData *) ConfigData;
   if(ConfigPtr->NumSensors >= MAX_NUM_CSS_SENSORS || 
      ConfigPtr->MaxSensorValue <= 0.0)
   {
      return; /* Throw ugly FSW error/crash here */
   }
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
   WriteMessage(ConfigPtr->OutputMsgID, 0, 
      ConfigPtr->NumSensors*sizeof(CSSOutputData), (void*) OutputBuffer);

   return;
}
