
#ifndef _DV_ATT_EFFECT_H_
#define _DV_ATT_EFFECT_H_

#include "messaging/static_messaging.h"
#include "attControl/_GeneralModuleFiles/vehControlOut.h"
#include "effectorInterfaces/errorConversion/vehEffectorOut.h"
#include <stdint.h>
#include <stdlib.h>

#define MAX_NUM_THR_GROUPS 4

/*! \addtogroup ADCSAlgGroup
 * @{
 */

typedef struct {
    double onTime;              /*!< s   The requested on time for this thruster*/
    uint32_t thrustIndex;       /*!< -  The actual thruster index associated with on-time*/
}effPairs;

/*! @brief Sub structure that contains all of the configuration data and output 
    information for a single thruster group.  There can be several thruster 
    groups available in a single control scheme.
*/
typedef struct {
    double nomThrustOn;          /*!< s The nominal thruster on-time for effectors*/
    uint32_t maxNumCmds;         /*!< - The maximum number of commands to output*/
    uint32_t numEffectors;       /*!< - The number of effectors we have access to*/
    double minThrustRequest;     /*!< - The minimum allowable on-time for a thruster*/
    double thrOnMap[3*MAX_EFF_CNT]; /*!< - Mapping between on-times and torque requests*/
    char outputDataName[MAX_STAT_MSG_LENGTH]; /*!< - The name of the output message*/
    char outputMsgID;            /*!< - ID for the outgoing command messages*/
    vehEffectorOut cmdRequests; /*!< - The array of command requests sent to effectors*/
}ThrustGroupData;

/*! @brief Top level structure for the DV attitude effector management algorithm.  
   This algorithm is used to control both the RCS and DV thrusters when 
   executing a trajectory adjustment.*/
typedef struct {
    char inputControlName[MAX_STAT_MSG_LENGTH]; /*!< - The name of the Input message*/
    int32_t inputMsgID;      /*!< - ID for the incoming guidance errors*/
    uint32_t numThrGroups;   /*!< - Count on the number of thrusters groups available*/
    ThrustGroupData thrGroups[MAX_NUM_THR_GROUPS]; /*!< - Thruster grouping container*/
}dvAttEffectConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_dvAttEffect(dvAttEffectConfig *ConfigData, uint64_t moduleID);
    void CrossInit_dvAttEffect(dvAttEffectConfig *ConfigData, uint64_t moduleID);
    void Update_dvAttEffect(dvAttEffectConfig *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    void effectorVSort(effPairs *Input, effPairs *Output, size_t dim);
    void computeSingleThrustBlock(ThrustGroupData *thrData, uint64_t callTime,
                                  vehControlOut *contrReq, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
