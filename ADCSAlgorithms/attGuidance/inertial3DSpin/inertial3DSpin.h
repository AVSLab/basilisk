
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
    double dummy;                                   /*!< [units] sample module variable declaration */
    double dumVector[3];                            /*!< [units] sample vector variable */

    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!< The name of the output message*/
    char inputDataName[MAX_STAT_MSG_LENGTH];        /*!< The name of the Input message*/
    int32_t outputMsgID;                            /*!< ID for the outgoing message */
    int32_t inputMsgID;                             /*!< ID for the incoming message */

    double  inputVector[3];                         /*!< [units]  vector description */

    attGuidOut attGuidOut;                          /*!< -- copy of the output message */

}inertial3DSpinConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t moduleID);
    void CrossInit_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t moduleID);
    void Update_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_inertial3DSpin(inertial3DSpinConfig *ConfigData);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
