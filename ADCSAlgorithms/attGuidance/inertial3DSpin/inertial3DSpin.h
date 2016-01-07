
#ifndef _INERTIAL3D_SPIN_
#define _INERTIAL3D_SPIN_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/attGuidOut.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    double sigma_R0N[3];                            /*!<        MRP from inertial frame N to original Reference frame R0
                                                                at initial time t0 */
    double sigma_R0R[3];                            /*!<        MRP from corrected reference frame to original reference frame R0
                                                                This is the same as [BcB] going from primary body frame B
                                                                to the corrected body frame Bc */
    double *sigma_BcB;                              /*!<        MRP from primary body frame B to corrected body frame Bc */
    double omega_RN_N[3];                           /*!< [r/s]  angular velocity vector of R relative to inertial N
                                                                in N-frame components */
    double sigma_RN[3];                             /*!<        MRP from inertial frame N to corrected reference frame R */
    uint64_t priorTime;                             /*!< [ns]   Last time the guidance module is called */

    
    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!< The name of the output message*/
    char inputNavName[MAX_STAT_MSG_LENGTH];         /*!< The name of the Navigation Input message */
    int32_t outputMsgID;                            /*!< ID for the outgoing message */
    int32_t inputNavID;                             /*!< ID for the incoming navigation message */


    attGuidOut attGuidOut;                          /*!< -- copy of the output message */

}inertial3DSpinConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t moduleID);
    void CrossInit_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t moduleID);
    void Update_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_inertial3DSpin(inertial3DSpinConfig *ConfigData);

    void computeInertialSpinAttitudeError(double sigma_BN[3],
                                          double omega_BN_B[3],
                                          inertial3DSpinConfig *ConfigData,
                                          int    integrateFlag,
                                          double dt,
                                          double sigma_BR[3],
                                          double omega_BR_B[3],
                                          double omega_RN_B[3],
                                          double domega_RN_B[3]);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
