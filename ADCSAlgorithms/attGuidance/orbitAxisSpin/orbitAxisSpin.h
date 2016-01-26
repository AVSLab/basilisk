
#ifndef _HILL_SPIN_
#define _HILL_SPIN_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/attGuidOut.h"

/*! \addtogroup ADCSAlgGroup
 * @{
 */

typedef struct {
    /* declare module private variables */
    int o_spin;                                     /*!< [0,1,2]   Orbit axis around which to spin */
    double omega_spin;                              /*!< [rad/sec] Desired spinning rate */
    double phi_spin;                              /*!< [rad]     Initial  spin angle */
    uint64_t priorTime;                             /*!< [ns]   Last time the guidance module is called */

    
    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!<        The name of the output message*/
    int32_t outputMsgID;                            /*!< (-)    ID for the outgoing message */
    char inputNavDataName[MAX_STAT_MSG_LENGTH];     /*<!        The name of the incoming attitude command*/
    int32_t inputNavID;                             /*!< (-)    ID for the incoming IMU data message*/
    char inputCelMessName[MAX_STAT_MSG_LENGTH];     /*<!        The name of the celestial body message */
    int32_t inputCelID;                             /*!< (-)    ID for the planet input message */
    char inputPointRefName[MAX_STAT_MSG_LENGTH];     /*<!       The name of the celestial body message */
    int32_t inputRefID;                             /*!< (-)    ID for the planet input message */
    
    /*  copy of the output message */
    attRefOut attRefOut;

}orbitAxisSpinConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_orbitAxisSpin(orbitAxisSpinConfig *ConfigData, uint64_t moduleID);
    void CrossInit_orbitAxisSpin(orbitAxisSpinConfig *ConfigData, uint64_t moduleID);
    void Update_orbitAxisSpin(orbitAxisSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_orbitAxisSpin(orbitAxisSpinConfig *ConfigData);

    void computeorbitAxisSpinReference(orbitAxisSpinConfig *ConfigData,
                                  double r_BN_N[3],
                                  double v_BN_N[3],
                                  double celBdyPositonVector[3],
                                  double celBdyVelocityVector[3],
                                  double sigma_R0N[3],
                                  double omega_R0N_N[3],
                                  double domega_R0N_N[3],
                                  int    integrateFlag,
                                  double dt,
                                  double sigma_RN[3],
                                  double omega_RN_N[3],
                                  double domega_RN_N[3]);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
