
#ifndef _HILL_POINT_
#define _HILL_POINT_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/attGuidOut.h"

/* Required module input messages */
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"
#include "SimCode/environment/spice/spice_planet_state.h"

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */



typedef struct {
    
    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!<        The name of the output message*/
    int32_t outputMsgID;                            /*!< (-)    ID for the outgoing message */
    char inputNavDataName[MAX_STAT_MSG_LENGTH];     /*<!        The name of the incoming attitude command*/
    int32_t inputNavID;                             /*!< (-)    ID for the incoming IMU data message*/
    char inputCelMessName[MAX_STAT_MSG_LENGTH];     /*<!        The name of the celestial body message */
    int32_t inputCelID;                             /*!< (-)    ID for the planet input message */
    
    /*  copy of the output message */
    attRefOut attRefOut;

}hillPointConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_hillPoint(hillPointConfig *ConfigData, uint64_t moduleID);
    void CrossInit_hillPoint(hillPointConfig *ConfigData, uint64_t moduleID);
    void Update_hillPoint(hillPointConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_hillPoint(hillPointConfig *ConfigData);

    void computeHillPointingReference(hillPointConfig *ConfigData, NavStateOut navData, SpicePlanetState primPlanet,
                                      double sigma_RN[3],
                                      double omega_RN_N[3],
                                      double domega_RN_N[3]);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
