
#ifndef _DV_GUIDANCE_POINT_H_
#define _DV_GUIDANCE_POINT_H_

#include "messaging/static_messaging.h"
#include "attGuidance/_GeneralModuleFiles/attGuidOut.h"
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"
#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the nominal delta-V guidance*/
typedef struct {
    char outputDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    char inputNavDataName[MAX_STAT_MSG_LENGTH]; /*<! The name of the incoming attitude command*/
    char inputMassPropName[MAX_STAT_MSG_LENGTH];/*<! The name of the mass properties message*/
    double dvInrtlCmd[3];    /*!< (m) The inertial DV we are going to execute*/
    double dvRotAxis[3];     /*! < (-) Rotation vector for the DV maneuver */
    double dvRotMag;         /*! < (r/s) Magnitude of the mnvr rotation vector*/
    double Tburn2Bdy[9];     /*!< (-) transformation from burn frame to body*/
    double dvMag;            /*!< (m/s) Magnitude of the requested deltaV*/
    double dvInit[3];        /*!< (m/s) DV reading off the accelerometers at burn start*/
    uint64_t burnStartTime;  /*!< (ns) Vehicle clock time to start the burn at*/
    uint32_t burnExecuting;  /*!< (-) Flag indicating whether the burn is in progress or not*/
    uint32_t burnComplete;   /*!< (-) Flag indicating that burn has completed successfully*/
    int32_t outputMsgID;     /*!< (-) ID for the outgoing body estimate message*/
    int32_t inputNavID;      /*!< (-) ID for the incoming IMU data message*/
    int32_t inputMPID;       /*!< (-) ID for the incoming mass properties message*/
    attCmdOut attCmd;       /*!< (-) Output attitude command data to send*/
}dvGuidanceConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_dvGuidance(dvGuidanceConfig *ConfigData, uint64_t moduleID);
    void CrossInit_dvGuidance(dvGuidanceConfig *ConfigData, uint64_t moduleID);
    void Update_dvGuidance(dvGuidanceConfig *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
