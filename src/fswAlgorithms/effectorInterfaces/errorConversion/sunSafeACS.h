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

#ifndef _SUN_SAFE_ACS_H_
#define _SUN_SAFE_ACS_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include <stdlib.h>
#include "effectorInterfaces/errorConversion/dvAttEffect.h"
#include "../_GeneralModuleFiles/thrustGroupData.h"
#include "simFswInterfaceMessages/cmdTorqueBodyIntMsg.h"
#include "simFswInterfaceMessages/thrArrayOnTimeCmdIntMsg.h"

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sun-safe attitude control routine.
 This algorithm is intended to be incredibly simple and robust*/
typedef struct {
    ThrustGroupData thrData;  /*!< Collection of thruster configuration data*/
    char inputControlName[MAX_STAT_MSG_LENGTH]; /*!< -- The name of the Input message*/
    int32_t inputMsgID;      /*!< -- ID for the incoming guidance errors*/
}sunSafeACSConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_sunSafeACS(sunSafeACSConfig *ConfigData, uint64_t moduleID);
    void CrossInit_sunSafeACS(sunSafeACSConfig *ConfigData, uint64_t moduleID);
    void Update_sunSafeACS(sunSafeACSConfig *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
