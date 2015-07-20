
#ifndef _CSS_COMM_H_
#define _CSS_COMM_H_

#define MAX_NUM_CSS_SENSORS 32

#include "messaging/static_messaging.h"

typedef struct {
   float CosValue;
}CSSOutputData;

typedef struct {
   char SensorMsgName[MAX_STAT_MSG_LENGTH];
}SensorMsgNameCarrier;

typedef struct {
   uint32_t  NumSensors;
   SensorMsgNameCarrier SensorList[MAX_NUM_CSS_SENSORS];
   char OutputDataName[MAX_STAT_MSG_LENGTH];
   int32_t SensorMsgIDs[MAX_NUM_CSS_SENSORS];
   int32_t OutputMsgID;
   float InputValues[MAX_NUM_CSS_SENSORS];
   float MaxSensorValue;
}CSSConfigData;

#ifdef __cplusplus
   extern "C" {
#endif

void SelfInit_cssProcessTelem(void *ConfigData);
void CrossInit_cssProcessTelem(void *ConfigData);
void Update_cssProcessTelem(void *ConfigData);

#ifdef __cplusplus
   }
#endif

#endif
