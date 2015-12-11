
#ifndef _SUB_MODULE_TEMPLATE_H_
#define _SUB_MODULE_TEMPLATE_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/subModuleOut.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    double dummy;                                   /*!< [rad/sec] sample module variable declaration */

    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!< The name of the output message*/
    char inputDataName[MAX_STAT_MSG_LENGTH];        /*!< The name of the Input message*/
    int32_t outputMsgID;                            /*!< [] ID for the outgoing message */
    int32_t inputMsgID;                             /*!< [] ID for the incoming message */

    subModuleOut subModuleOut;                      /*!< -- copy of the output message */

}subModuleTemplateConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_subModuleTemplate(subModuleTemplateConfig *ConfigData, uint64_t moduleID);
    void CrossInit_subModuleTemplate(subModuleTemplateConfig *ConfigData, uint64_t moduleID);
    void Update_subModuleTemplate(subModuleTemplateConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_subModuleTemplate(subModuleTemplateConfig *ConfigData);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
