
#ifndef _SUN_SAFE_ACS_H_
#define _SUN_SAFE_ACS_H_

#include "messaging/static_messaging.h"
#include "attControl/vehControlOut.h"
#include "effectorInterfaces/errorConversion/vehEffectorOut.h"
#include <stdint.h>
#include <stdlib.h>
#include "effectorInterfaces/errorConversion/dvAttEffect.h"

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sun-safe attitude control routine.
 This algorithm is intended to be incredibly simple and robust*/
typedef struct {
    ThrustGroupData thrData;  /*!< Collection of thruster configuration data*/
    char inputControlName[MAX_STAT_MSG_LENGTH]; /*!< -- The name of the Input message*/
    int32_t inputMsgID;      /*!< -- ID for the incoming guidance errors*/
}sunSafeACSConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_sunSafeACS(sunSafeACSConfig *ConfigData);
    void CrossInit_sunSafeACS(sunSafeACSConfig *ConfigData);
    void Update_sunSafeACS(sunSafeACSConfig *ConfigData, uint64_t callTime);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
