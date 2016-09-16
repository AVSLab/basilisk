/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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

#ifndef _SUNLINE_UKF_H_
#define _SUNLINE_UKF_H_

#include "messaging/static_messaging.h"
#include "sensorInterfaces/CSSSensorData/cssComm.h"
#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the CSS weighted least squares estimator.
 Used to estimate the sun state in the vehicle body frame*/
typedef struct {
//    SingleCSSConfig CSSData[MAX_NUM_CSS_SENSORS]; /*!< -- The config data for the estimator*/
    char OutputDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the output message*/
    char InputDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the Input message*/
    char InputPropsName[MAX_STAT_MSG_LENGTH]; /*!< [-] The name of the mass props message*/
    uint32_t numActiveCss;   /*!< -- Number of currently active CSS sensors*/
    uint32_t UseWeights;     /*!< -- Flag indicating whether or not to use weights for least squares*/
    double SensorUseThresh;  /*!< -- Threshold below which we discount sensors*/
    //CSSWlsEstOut OutputData; /*!< -- Unit vector to the Sun in the spacecraft body frame*/
    int32_t OutputMsgID;     /*!< -- ID for the outgoing body estimate message*/
    int32_t InputMsgID;      /*!< -- ID for the incoming CSS sensor message*/
    int32_t InputPropsID;    /*!< [-] ID for the incoming mass properties message*/
}SunlineUKFConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_sunlineUKF(SunlineUKFConfig *ConfigData, uint64_t moduleID);
    void CrossInit_sunlineUKF(SunlineUKFConfig *ConfigData, uint64_t moduleID);
    void Update_sunlineUKF(SunlineUKFConfig *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
