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

#ifndef _HILL_STATE_CONVERTER_H_
#define _HILL_STATE_CONVERTER_H_

//  Standard lib imports
#include <stdint.h>

//  Support imports
#include "messaging/static_messaging.h"
#include "simulation/utilities/bskLogging.h"
#include "utilities/orbitalMotion.h"

//  Message type imports
#include "simulation/simFswInterfaceMessages/navTransIntMsg.h"
#include "fswMessages/hillRelStateFswMsg.h"



/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module IO interfaces */
    char hillStateOutMsgName[MAX_STAT_MSG_LENGTH];       //!< The name of the output message
    int32_t hillStateOutMsgID;                           //!< ID for the outgoing message
    HillRelStateFswMsg hillStateOutMsg;
    char chiefStateInMsgName[MAX_STAT_MSG_LENGTH];        //!< The name of the Input message
    int32_t chiefStateInMsgID;                            //!< ID for the incoming message
    NavTransIntMsg chiefStateInMsg;
    char depStateInMsgName[MAX_STAT_MSG_LENGTH];        //!< The name of the Input message
    int32_t depStateInMsgID;                            //!< ID for the incoming message
    NavTransIntMsg depStateInMsg;

    BSKLogger *bskLogger;                           //!< BSK Logging
}hillStateConverterConfig;

#ifdef __cplusplus
extern "C" {
#endif

    void SelfInit_hillStateConverter(hillStateConverterConfig *configData, int64_t moduleID);
    void CrossInit_hillStateConverter(hillStateConverterConfig *configData, int64_t moduleID);
    void Update_hillStateConverter(hillStateConverterConfig *configData, uint64_t callTime, int64_t moduleID);
    void Reset_hillStateConverter(hillStateConveterConfig *configData, uint64_t callTime, int64_t moduleID);

#ifdef __cplusplus
}
#endif


#endif
