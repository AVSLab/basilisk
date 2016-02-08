
#ifndef _VELOCITY_POINT_
#define _VELOCITY_POINT_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/attGuidOut.h"

/* Required module input messages */
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"
#include "SimCode/environment/spice/spice_planet_state.h"
#include "SimCode/utilities/orbitalMotion.h"
/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */


typedef struct {
    
    /* declare module private variables */
    double mu;                                      /*!< #TEMP  Planet gravitational parameter */
    classicElements oe;                             /*!<        structure for the Orbit Elements set */
   
    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!<        The name of the output message */
    int32_t outputMsgID;                            /*!< (-)    ID for the outgoing message */
    char inputNavDataName[MAX_STAT_MSG_LENGTH];     /*<!        The name of the incoming attitude command*/
    int32_t inputNavID;                             /*!< (-)    ID for the incoming IMU data message*/
    char inputCelMessName[MAX_STAT_MSG_LENGTH];     /*<!        The name of the celestial body message */
    int32_t inputCelID;                             /*!< (-)    ID for the planet input message */
    
    /*  copy of the output message */
    attRefOut attRefOut;

}velocityPointConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_velocityPoint(velocityPointConfig *ConfigData, uint64_t moduleID);
    void CrossInit_velocityPoint(velocityPointConfig *ConfigData, uint64_t moduleID);
    void Update_velocityPoint(velocityPointConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_velocityPoint(velocityPointConfig *ConfigData);

    void computeVelocityPointingReference(velocityPointConfig *ConfigData,
                                          double r_BN_N[3],
                                          double v_BN_N[3],
                                          double celBdyPositonVector[3],
                                          double celBdyVelocityVector[3],
                                          double sigma_RN[3],
                                          double omega_RN_N[3],
                                          double domega_RN_N[3]);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
