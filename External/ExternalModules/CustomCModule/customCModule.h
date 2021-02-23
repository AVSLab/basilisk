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

#ifndef _CUSTOM_C_MODULE_MSG_H_
#define _CUSTOM_C_MODULE_MSG_H_

#include <stdint.h>
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/CustomModuleMsg_C.h"

#define TESTHMM 10
/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    double dummy;                                   //!< [units] sample module variable declaration
    double dumVector[3];                            //!< [units] sample vector variable

    /* declare module IO interfaces */
    CustomModuleMsg_C dataOutMsg;              //!< sample output message
    CustomModuleMsg_C dataInMsg;               //!< sample input message

    double  inputVector[3];                         //!< [units]  vector description
    BSKLogger *bskLogger;                           //!< BSK Logging
} customCModuleConfig;

#ifdef __cplusplus
extern "C" {
#endif

    void SelfInit_customCModule(customCModuleConfig *configData, int64_t moduleID);
    void Update_customCModule(customCModuleConfig *configData, uint64_t callTime, int64_t moduleID);
    void Reset_customCModule(customCModuleConfig *configData, uint64_t callTime, int64_t moduleID);

#ifdef __cplusplus
}
#endif


#endif
