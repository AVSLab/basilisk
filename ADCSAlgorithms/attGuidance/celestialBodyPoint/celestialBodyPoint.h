
#ifndef _CELESTIAL_BODY_POINT_H_
#define _CELESTIAL_BODY_POINT_H_

#include "messaging/static_messaging.h"
#include "attGuidance/_GeneralModuleFiles/attGuidOut.h"
#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the nominal delta-V guidance*/
typedef struct {
    char outputDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    char inputNavDataName[MAX_STAT_MSG_LENGTH]; /*<! The name of the incoming attitude command*/
    char inputCelMessName[MAX_STAT_MSG_LENGTH];/*<! The name of the celestial body message*/
    char inputSecMessName[MAX_STAT_MSG_LENGTH];/*<! The name of the secondary body to constrain point*/
    double TPoint2Bdy[9];     /*!< (-) transformation from burn frame to body*/
	double singularityThresh; /*!< (r) Threshold for when to fix constraint axis*/
	double prevConstraintAxis[3]; /*!< (-) Previous setting for constraint axis*/
	uint32_t prevAvail;           /*!< (-) Flag indicating whether the previous constraint axis is populated*/
    int32_t outputMsgID;     /*!< (-) ID for the outgoing body estimate message*/
    int32_t inputNavID;      /*!< (-) ID for the incoming IMU data message*/
    int32_t inputCelID;       /*!< (-) ID for the incoming mass properties message*/
    int32_t inputSecID;       /*!< (-) ID for the secondary constraint message*/
    attCmdOut attCmd;       /*!< (-) Output attitude command data to send*/
}celestialBodyPointConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_celestialBodyPoint(celestialBodyPointConfig *ConfigData,
        uint64_t moduleID);
    void CrossInit_celestialBodyPoint(celestialBodyPointConfig *ConfigData,
        uint64_t moduleID);
    void Update_celestialBodyPoint(celestialBodyPointConfig *ConfigData,
    uint64_t callTime, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
