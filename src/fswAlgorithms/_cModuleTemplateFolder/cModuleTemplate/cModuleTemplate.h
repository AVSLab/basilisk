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

#ifndef _FSW_MODULE_TEMPLATE_H_
#define _FSW_MODULE_TEMPLATE_H_

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/CModuleTemplateMsg_C.h"



/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    double dummy;                                   //!< [units] sample module variable declaration
    double dumVector[3];                            //!< [units] sample vector variable

    /* declare module IO interfaces */
    CModuleTemplateMsg_C dataOutMsg;              //!< sample output message
    CModuleTemplateMsg_C dataInMsg;               //!< sample input message

    double  inputVector[3];                         //!< [units]  vector description
    BSKLogger *bskLogger;                           //!< BSK Logging
}cModuleTemplateConfig;

#ifdef __cplusplus
extern "C" {
#endif

    void SelfInit_cModuleTemplate(cModuleTemplateConfig *configData, int64_t moduleID);
    void Update_cModuleTemplate(cModuleTemplateConfig *configData, uint64_t callTime, int64_t moduleID);
    void Reset_cModuleTemplate(cModuleTemplateConfig *configData, uint64_t callTime, int64_t moduleID);

#ifdef __cplusplus
}
#endif


#endif
