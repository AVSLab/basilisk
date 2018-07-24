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

#ifndef _DV_EXECUTE_GUIDANCE_H_
#define _DV_EXECUTE_GUIDANCE_H_

#include "messaging/static_messaging.h"
#include "simFswInterfaceMessages/navTransIntMsg.h"
#include "simFswInterfaceMessages/thrArrayOnTimeCmdIntMsg.h"
#include "fswMessages/dvBurnCmdFswMsg.h"
#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

typedef struct {
    uint32_t burnExecuting;    /*!< [-] Flag indicating whether burn is executing*/
    uint32_t burnComplete;     /*!< [-] Flag indicating whether the burn is complete */
}dvExecutionData;

/*! @brief Top level structure for the nominal delta-V guidance*/
typedef struct {
    char outputDataName[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the output message*/
    char inputNavDataName[MAX_STAT_MSG_LENGTH]; /*<! [-] The name of the incoming attitude command*/
    char inputBurnDataName[MAX_STAT_MSG_LENGTH];/*<! [-] Input message that configures the vehicle burn*/
    char outputThrName[MAX_STAT_MSG_LENGTH]; /*!< [-] Output thruster message name */
    double dvInit[3];        /*!< (m/s) DV reading off the accelerometers at burn start*/
    uint32_t burnExecuting;  /*!< (-) Flag indicating whether the burn is in progress or not*/
    uint32_t burnComplete;   /*!< (-) Flag indicating that burn has completed successfully*/
    int32_t outputMsgID;     /*!< (-) ID for the outgoing body estimate message*/
    int32_t outputThrID;     /*!< [-] ID for the outgoing thruster command message*/
    int32_t inputNavID;      /*!< (-) ID for the incoming IMU data message*/
    int32_t inputBurnCmdID;  /*!< [-] ID for the incoming burn command data*/
}dvExecuteGuidanceConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_dvExecuteGuidance(dvExecuteGuidanceConfig *ConfigData, uint64_t moduleID);
    void CrossInit_dvExecuteGuidance(dvExecuteGuidanceConfig *ConfigData, uint64_t moduleID);
    void Update_dvExecuteGuidance(dvExecuteGuidanceConfig *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
