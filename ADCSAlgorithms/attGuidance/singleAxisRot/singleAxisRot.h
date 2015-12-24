
#ifndef _SINGLE_AXIS_ROT_H_
#define _SINGLE_AXIS_ROT_H_

#include "messaging/static_messaging.h"
#include "attGuidance/_GeneralModuleFiles/attGuidOut.h"
#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the nominal single axis rotation guidance*/
typedef struct {
    char outputDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    char inputAttCmdName[MAX_STAT_MSG_LENGTH];/*<! The name of the incoming att cmd message*/
    double rotVector[3];     /*!< (r/s) The vector we are rotating about*/
	uint64_t mnvrStartTime;  /*! (ns) The time that the attitude maneuver started*/
    int32_t outputMsgID;     /*!< (-) ID for the outgoing body command message*/
    int32_t inputAttID;       /*!< (-) ID for the incoming attitude command message*/
    attCmdOut attCmd;       /*!< (-) Output attitude command data to send*/
}singleAxisRotConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_singleAxisRot(singleAxisRotConfig *ConfigData,
        uint64_t moduleID);
    void CrossInit_singleAxisRot(singleAxisRotConfig *ConfigData,
        uint64_t moduleID);
    void Update_singleAxisRot(singleAxisRotConfig *ConfigData,
    uint64_t callTime, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
