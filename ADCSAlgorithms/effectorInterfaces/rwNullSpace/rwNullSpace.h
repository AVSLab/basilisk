
#ifndef _RW_NULL_SPACE_H_
#define _RW_NULL_SPACE_H_

#include "messaging/static_messaging.h"
#include "attControl/_GeneralModuleFiles/vehControlOut.h"
#include "effectorInterfaces/errorConversion/vehEffectorOut.h"
#include "effectorInterfaces/_GeneralModuleFiles/rwSpeedData.h"
#include <stdint.h>
#include <stdlib.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sun-safe attitude control routine.
 This algorithm is intended to be incredibly simple and robust*/
typedef struct {
    char inputRWCommands[MAX_STAT_MSG_LENGTH]; /*!< -- The name of the Input message*/
	char inputRWSpeeds[MAX_STAT_MSG_LENGTH];   /*!< (-) The name of the input RW speeds*/
	char outputControlName[MAX_STAT_MSG_LENGTH]; /*!< (-) The name of the output message*/
	double GsMatrix[3*MAX_NUM_EFFECTORS]; /*!< (-) The spin axis matrix used to find null space*/
	double GsInverse[MAX_NUM_EFFECTORS * MAX_NUM_EFFECTORS];    /*!< (-) Pseudo-inverse of the spin axis matrix*/
	double OmegaGain;           /*!< (-) The gain factor applied to the RW speeds*/
	uint32_t numWheels;         /*!< (-) The number of reaction wheels we have*/
    int32_t inputRWCmdsID;      /*!< -- ID for the incoming RW commands*/
	int32_t inputSpeedsID;      /*!< (-) ID for the incoming RW speed measure*/
	int32_t outputMsgID;     /*!< (-) ID for the outgoing RW commands*/
}rwNullSpaceConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_rwNullSpace(rwNullSpaceConfig *ConfigData, uint64_t moduleID);
    void CrossInit_rwNullSpace(rwNullSpaceConfig *ConfigData, uint64_t moduleID);
    void Update_rwNullSpace(rwNullSpaceConfig *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
