
#ifndef _ATT_TRACKING_ERROR_
#define _ATT_TRACKING_ERROR_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/attGuidOut.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    double sigma_R0R[3];                            /*!< MRP from corrected reference frame to original reference frame R0
                                                         This is the same as [BcB] going from primary body frame B
                                                         to the corrected body frame Bc */
    double *sigma_BcB;                              /*!< MRP from primary body frame B to corrected body frame Bc */

    
    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!< The name of the output message*/
    char inputRefName[MAX_STAT_MSG_LENGTH];         /*!< The name of the guidance reference Input message */
    char inputNavName[MAX_STAT_MSG_LENGTH];         /*!< The name of the navigation Input message */
    int32_t outputMsgID;                            /*!< ID for the outgoing message */
    int32_t inputRefID;                             /*!< ID for the incoming guidance reference message */
    int32_t inputNavID;                             /*!< ID for the incoming navigation message */


    attGuidOut attGuidOut;                          /*!< copy of the output message */

}attTrackingErrorConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_attTrackingError(attTrackingErrorConfig *ConfigData, uint64_t moduleID);
    void CrossInit_attTrackingError(attTrackingErrorConfig *ConfigData, uint64_t moduleID);
    void Update_attTrackingError(attTrackingErrorConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_attTrackingError(attTrackingErrorConfig *ConfigData);

    void computeAttitudeError(double sigma_BN[3],
                              double omega_BN_B[3],
                              double sigma_R0N[3],
                              double omega_RN_N[3],
                              double domega_RN_N[3],
                              attTrackingErrorConfig *ConfigData,
                              double sigma_BR[3],
                              double omega_BR_B[3],
                              double omega_RN_B[3],
                              double domega_RN_B[3]);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
