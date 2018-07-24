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

#ifndef _RATE_IMU_TO_NAV_CONVERTER_H_
#define _RATE_IMU_TO_NAV_CONVERTER_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "simFswInterfaceMessages/navAttIntMsg.h"
#include "fswAlgorithms/fswMessages/imuSensorBodyFswMsg.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {

    /* declare module IO interfaces */
    char navRateOutMsgName[MAX_STAT_MSG_LENGTH];       /*!< The name of the navAttIntMsg output message*/
    int32_t navRateOutMsgID;                           /*!< ID for the outgoing message */
    char imuRateInMsgName[MAX_STAT_MSG_LENGTH];        /*!< The name of the imuSensorBody Input message*/
    int32_t imuRateInMsgID;                            /*!< ID for the incoming message */

    NavAttIntMsg outMsg;                               /*!< -- copy of the output message */

}rateMsgConverterConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_rateMsgConverter(rateMsgConverterConfig *ConfigData, uint64_t moduleID);
    void CrossInit_rateMsgConverter(rateMsgConverterConfig *ConfigData, uint64_t moduleID);
    void Update_rateMsgConverter(rateMsgConverterConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_rateMsgConverter(rateMsgConverterConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
