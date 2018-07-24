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

#ifndef _RW_NULL_SPACE_H_
#define _RW_NULL_SPACE_H_

#include "messaging/static_messaging.h"
#include "fswMessages/vehicleConfigFswMsg.h"
#include "simFswInterfaceMessages/rwSpeedIntMsg.h"
#include "simFswInterfaceMessages/rwArrayTorqueIntMsg.h"
#include "fswMessages/rwAvailabilityFswMsg.h"
#include "fswMessages/rwConstellationFswMsg.h"
#include <stdint.h>
#include <stdlib.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sun-safe attitude control routine.
 This algorithm is intended to be incredibly simple and robust*/
typedef struct {
    char inputRWCommands[MAX_STAT_MSG_LENGTH]; /*!< -- The name of the Input message*/
	char inputRWSpeeds[MAX_STAT_MSG_LENGTH];   /*!< (-) The name of the input RW speeds*/
    char inputRWConfigData[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the RWA configuration message*/
	char outputControlName[MAX_STAT_MSG_LENGTH]; /*!< (-) The name of the output message*/
	double GsInverse[MAX_EFF_CNT * MAX_EFF_CNT];    /*!< (-) Pseudo-inverse of the spin axis matrix*/
	double OmegaGain;           /*!< (-) The gain factor applied to the RW speeds*/
	uint32_t numWheels;         /*!< (-) The number of reaction wheels we have*/
    int32_t inputRWCmdsID;      /*!< -- ID for the incoming RW commands*/
	int32_t inputSpeedsID;      /*!< (-) ID for the incoming RW speed measure*/
    int32_t inputRWConfID;      /*!< [-] ID for the incoming RWA configuration data*/
	int32_t outputMsgID;     /*!< (-) ID for the outgoing RW commands*/
}rwNullSpaceConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_rwNullSpace(rwNullSpaceConfig *ConfigData, uint64_t moduleID);
    void CrossInit_rwNullSpace(rwNullSpaceConfig *ConfigData, uint64_t moduleID);
    void Update_rwNullSpace(rwNullSpaceConfig *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    void Reset_rwNullSpace(rwNullSpaceConfig *ConfigData, uint64_t callTime,
                            uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
