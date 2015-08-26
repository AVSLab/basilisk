
#ifndef _SUN_SAFE_CONTROL_H_
#define _SUN_SAFE_CONTROL_H_

#include "messaging/static_messaging.h"
#include "attControl/vehControlOut.h"
#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sun-safe attitude control routine.
 This algorithm is intended to be incredibly simple and robust*/
typedef struct {
    double K;           /*!< r/s2 The proportional gain applied to att errors*/
    double P;           /*!< 1/s The derivative gain applied to att rate errors*/
    char outputDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    char inputGuidName[MAX_STAT_MSG_LENGTH]; /*!< The name of the Input message*/
    int32_t outputMsgID;     /*!< -- ID for the outgoing body accel requests*/
    int32_t inputMsgID;      /*!< -- ID for the incoming guidance errors*/
    vehControlOut controlOut; /*!< -- Control output requests for suSafe*/
}sunSafeControlConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_sunSafeControl(sunSafeControlConfig *ConfigData);
    void CrossInit_sunSafeControl(sunSafeControlConfig *ConfigData);
    void Update_sunSafeControl(sunSafeControlConfig *ConfigData, uint64_t callTime);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
