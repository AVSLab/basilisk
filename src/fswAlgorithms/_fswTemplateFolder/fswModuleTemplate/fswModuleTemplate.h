/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef _FSW_MODULE_TEMPLATE_FSW_MSG_H_
#define _FSW_MODULE_TEMPLATE_FSW_MSG_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/fswModuleTemplateFswMsg.h"



/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    double dummy;                                   /*!< [units] sample module variable declaration */
    double dumVector[3];                            /*!< [units] sample vector variable */

    /* declare module IO interfaces */
    char dataOutMsgName[MAX_STAT_MSG_LENGTH];       /*!< The name of the output message*/
    int32_t dataOutMsgID;                           /*!< ID for the outgoing message */
    char dataInMsgName[MAX_STAT_MSG_LENGTH];        /*!< The name of the Input message*/
    int32_t dataInMsgID;                            /*!< ID for the incoming message */

    double  inputVector[3];                         /*!< [units]  vector description */

    FswModuleTemplateFswMsg fswModuleOut;           /*!< -- copy of the output message */

}fswModuleTemplateConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_fswModuleTemplate(fswModuleTemplateConfig *ConfigData, uint64_t moduleID);
    void CrossInit_fswModuleTemplate(fswModuleTemplateConfig *ConfigData, uint64_t moduleID);
    void Update_fswModuleTemplate(fswModuleTemplateConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_fswModuleTemplate(fswModuleTemplateConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
