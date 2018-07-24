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

#ifndef _SIMPLE_DEADBAND_
#define _SIMPLE_DEADBAND_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "fswMessages/attGuidFswMsg.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    double innerAttThresh;              /*!< inner limit for sigma (attitude) errors */
    double outerAttThresh;              /*!< outer limit for sigma (attitude) errors */
    double innerRateThresh;             /*!< inner limit for omega (rate) errors */
    double outerRateThresh;             /*!< outer limit for omega (rate) errors */
    uint32_t wasControlOff;             /*!< boolean variable to keep track of the last Control status (ON/OFF) */
    double attError;                    /*!< current scalar attitude error */
    double rateError;                   /*!< current scalar rate error */
    
    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];        /*!< The name of the output message*/
    char inputGuidName[MAX_STAT_MSG_LENGTH];         /*!< The name of the guidance reference Input message */
    int32_t outputGuidID;                            /*!< ID for the outgoing message */
    int32_t inputGuidID;                             /*!< ID for the incoming guidance reference message */

    AttGuidFswMsg attGuidOut;                       /*!< copy of the output message */

}simpleDeadbandConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_simpleDeadband(simpleDeadbandConfig *ConfigData, uint64_t moduleID);
    void CrossInit_simpleDeadband(simpleDeadbandConfig *ConfigData, uint64_t moduleID);
    void Update_simpleDeadband(simpleDeadbandConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_simpleDeadband(simpleDeadbandConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void applyDBLogic_simpleDeadband(simpleDeadbandConfig *ConfigData);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
