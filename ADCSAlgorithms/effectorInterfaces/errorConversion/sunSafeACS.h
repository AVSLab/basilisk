
#ifndef _SUN_SAFE_ACS_H_
#define _SUN_SAFE_ACS_H_

#include "messaging/static_messaging.h"
#include "attControl/vehControlOut.h"
#include "effectorInterfaces/errorConversion/vehEffectorOut.h"
#include <stdint.h>
#include <stdlib.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

typedef struct {
    double onTime;              /*!< s   The requested on time for this thruster*/
    uint32_t thrustIndex;       /*!< --  The actual thruster index associated with on-time*/
}effPairs;

/*! @brief Top level structure for the sun-safe attitude control routine.
 This algorithm is intended to be incredibly simple and robust*/
typedef struct {
    uint32_t maxNumCmds;        /*!< -- The max number of thruster commands to output*/
    uint32_t numThrusters;      /*!< -- Number of thrusters available in the system*/
    double minThrustRequest;    /*!< -- The minimum allowable thrust to output nonzero on-time for*/
    double thrOnMap[3*MAX_NUM_EFFECTORS]; /*!< -- Mapping between on-times and torque requests*/
    vehEffectorOut cmdRequests; /*!< -- The array of command requests sent to effectors*/
    char outputDataName[MAX_STAT_MSG_LENGTH]; /*!< -- The name of the output message*/
    char inputControlName[MAX_STAT_MSG_LENGTH]; /*!< -- The name of the Input message*/
    int32_t outputMsgID;     /*!< -- ID for the outgoing body accel requests*/
    int32_t inputMsgID;      /*!< -- ID for the incoming guidance errors*/
}sunSafeACSConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_sunSafeACS(sunSafeACSConfig *ConfigData);
    void CrossInit_sunSafeACS(sunSafeACSConfig *ConfigData);
    void Update_sunSafeACS(sunSafeACSConfig *ConfigData, uint64_t callTime);
    void effectorVSort(effPairs *Input, effPairs *Output, size_t dim);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
