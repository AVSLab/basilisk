
#ifndef _SUN_SAFE_POINT_H_
#define _SUN_SAFE_POINT_H_

#include "messaging/static_messaging.h"
#include "attDetermination/CSSEst/cssWlsEst.h"
#include "attGuidance/attGuidOut.h"
#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sun-safe attitude guidance routine.
 This algorithm is intended to be incredibly simple and robust*/
typedef struct {
    char outputDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    char inputSunVecName[MAX_STAT_MSG_LENGTH]; /*!< The name of the Input message*/
    char inputIMUDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the incoming IMU message*/
    double minUnitMag;       /*!< -- The minimally acceptable sun body unit vector*/
    double sunAngleErr;      /*!< r  The current error between cmd and obs sun angle*/
    double sunMnvrVec[3];    /*!< -- The eigen axis that we want to rotate on to get sun*/
    double sHatBdyCmd[3];    /*!< -- Desired body vector to point at the sun*/
    int32_t outputMsgID;     /*!< -- ID for the outgoing body estimate message*/
    int32_t inputMsgID;      /*!< -- ID for the incoming CSS sensor message*/
    int32_t imuMsgID;      /*!< -- ID for the incoming CSS sensor message*/
    attGuidOut attOut;       /*!< -- The output data that we compute*/
}sunSafePointConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_sunSafePoint(sunSafePointConfig *ConfigData);
    void CrossInit_sunSafePoint(sunSafePointConfig *ConfigData);
    void Update_sunSafePoint(sunSafePointConfig *ConfigData, uint64_t callTime);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
