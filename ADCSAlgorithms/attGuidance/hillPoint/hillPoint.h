
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
    unsigned int        i_r;                        /* body axis index that lines up with i_r */
    unsigned int        i_theta;                    /* body axis index that lines up with i_theta */
    unsigned int        i_h;                        /* body axis index that lines up with i_h */
    int                 i_rSign;                    /* sign of the i_r axis alignment */
    int                 i_thetaSign;                /* sign of the i_theta axis alignment */
    int                 i_hSign;                    /* sign of the i_h axis alignment */
    int                 o_spin;                     /* orbit frame axis about which to spin */
    unsigned int        b_spin;                     /* principal body frame axis about which to spin */
    double              omega_spin;                 /* desired spin rate */
} OrbitFrameStates;

typedef struct {
    /* declare module private variables */
    OrbitFrameStates OrbitFrameStates;              /*!<        Nadir pointing states */
    uint64_t priorTime;                             /*!< [ns]   Last time the guidance module is called */

    
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
