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

#ifndef _SUN_SAFE_POINT_H_
#define _SUN_SAFE_POINT_H_

#include "messaging/static_messaging.h"
#include "fswMessages/attGuidFswMsg.h"
#include "simFswInterfaceMessages/navAttIntMsg.h"
#include <stdint.h>

/*! \defgroup sunSafePoint
 * @{
 */

/*! @brief Top level structure for the sun-safe attitude guidance routine.
 This algorithm is intended to be incredibly simple and robust*/
typedef struct {
    char attGuidanceOutMsgName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    char sunDirectionInMsgName[MAX_STAT_MSG_LENGTH]; /*!< The name of the Input message*/
    char imuInMsgName[MAX_STAT_MSG_LENGTH]; /*!< The name of the incoming IMU message*/
    double minUnitMag;       /*!< -- The minimally acceptable norm of sun body vector*/
    double sunAngleErr;      /*!< rad The current error between cmd and obs sun angle*/
    double smallAngle;       /*!< rad An angle value that specifies what is near 0 or 180 degrees */
    double eHat180_B[3];     /*!< -- Eigen axis to use if commanded axis is 180 from sun axis */
    double sunMnvrVec[3];    /*!< -- The eigen axis that we want to rotate on to get sun*/
    double sHatBdyCmd[3];    /*!< -- Desired body vector to point at the sun*/
    double omega_RN_B[3];    /*!< -- Desired body rate vector if no sun direction is available */
    double sunAxisSpinRate;  /*!< r/s Desired constant spin rate about sun heading vector */
    int32_t attGuidanceOutMsgID;/*!< -- ID for the outgoing body estimate message*/
    int32_t sunDirectionInMsgID;/*!< -- ID for the incoming CSS sensor message*/
    int32_t imuInMsgID;        /*!< -- ID for the incoming IMU sensor message*/
    AttGuidFswMsg attGuidanceOutBuffer;   /*!< -- The output data that we compute*/
}sunSafePointConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_sunSafePoint(sunSafePointConfig *configData, uint64_t moduleID);
    void CrossInit_sunSafePoint(sunSafePointConfig *configData, uint64_t moduleID);
    void Update_sunSafePoint(sunSafePointConfig *configData, uint64_t callTime,
        uint64_t moduleID);
    void Reset_sunSafePoint(sunSafePointConfig *configData, uint64_t callTime, uint64_t moduleID);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
