
#ifndef _orbitAxis_SPIN_
#define _orbitAxis_SPIN_

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
    /* declare module private variables */
    int     o_spin;                                  /*!< [0,1,2]   Orbit axis around which to spin */
    double  omega_spin;                              /*!< [rad/sec] Desired spinning rate */
    double  phi_spin;                                /*!< [rad]     Current  spin angle */
    double  phi_spin_0;                              /*!< [rad]     Initial  spin angle */
    double  dt;                                      /*!< [rad]     Module update time */
    
    int     b_spin;                                  /*!< [0,1,2]   Body axis around which to spin */
    double  sigma_BN[3];                             /*!<           MRP from body B-frame to inertial */
    
    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!<        The name of the output message*/
    int32_t outputMsgID;                            /*!< (-)    ID for the outgoing message */
    char inputRefName[MAX_STAT_MSG_LENGTH];         /*!< The name of the guidance reference Input message */
    int32_t inputRefID;                             /*!< ID for the incoming guidance reference message */
    
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
    void computeOrbitAxisSpinReference(orbitAxisSpinConfig *ConfigData,
                                  double sigma_R0N[3],
                                  double omega_R0N_N[3],
                                  double domega_R0N_N[3],
                                  int    integrateFlag,
                                  double dt,
                                  double sigma_RN[3],
                                  double omega_RN_N[3],
                                  double domega_RN_N[3]);
    
    double computeInitialSpinAngle(orbitAxisSpinConfig *ConfigData,
                                   double sigma_ON[3]);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
