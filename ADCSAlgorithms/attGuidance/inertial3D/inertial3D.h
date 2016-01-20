
#ifndef _INERTIAL3D_
#define _INERTIAL3D_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/attGuidOut.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    double sigma_RN[3];                             /*!<        MRP from inertial frame N to corrected reference frame R */
    uint64_t priorTime;                             /*!<        [ns]   Last time the guidance module is called */
    
    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!<        The name of the output message */
    int32_t outputMsgID;                            /*!<        ID for the outgoing message */
    
    /* copy of the output message */
    attRefOut attRefOut;

}inertial3DConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_inertial3D(inertial3DConfig *ConfigData, uint64_t moduleID);
    void CrossInit_inertial3D(inertial3DConfig *ConfigData, uint64_t moduleID);
    void Update_inertial3D(inertial3DConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_inertial3D(inertial3DConfig *ConfigData);

    void computeInertialPointingReference(inertial3DConfig *ConfigData,
                                          double sigma_RN[3],
                                          double omega_RN_N[3],
                                          double domega_RN_N[3]);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
