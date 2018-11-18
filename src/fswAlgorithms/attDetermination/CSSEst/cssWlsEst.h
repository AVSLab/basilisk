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

#ifndef _CSS_WLS_EST_H_
#define _CSS_WLS_EST_H_

#include "messaging/static_messaging.h"
#include "simFswInterfaceMessages/navAttIntMsg.h"
#include "fswMessages/cssConfigFswMsg.h"
#include "fswMessages/cssUnitConfigFswMsg.h"
#include <stdint.h>
#include "simFswInterfaceMessages/cssArraySensorIntMsg.h"

/*! \addtogroup ADCSAlgGroup
 * @{
 */





/*! @brief Top level structure for the CSS weighted least squares estimator.
 Used to estimate the sun state in the vehicle body frame*/
typedef struct {
    char cssDataInMsgName[MAX_STAT_MSG_LENGTH];         /*!< The name of the css sensor input message*/
    char cssConfigInMsgName[MAX_STAT_MSG_LENGTH];       /*!< The name of the css configuration input message*/
    char navStateOutMsgName[MAX_STAT_MSG_LENGTH];       /*!< The name of the navigation output message*/
    char cssWLSFiltResOutMsgName[MAX_STAT_MSG_LENGTH];  /*!< [-] Name of the CSS filter data out message*/
    uint32_t numActiveCss;                              /*!< -- Number of currently active CSS sensors*/
    uint32_t useWeights;                                /*!< -- Flag indicating whether or not to use weights for least squares*/
    uint32_t priorSignalAvailable;                      /*!< -- Flag indicating if a recent prior heading estimate is available */
    double dOld[3];                                     /*!< -- prior sun heading estimate */
    double sensorUseThresh;                             /*!< -- Threshold below which we discount sensors*/
    uint64_t priorTime;                                 /*!< [ns] Last time the attitude control is called */
    CSSConfigFswMsg cssConfigInBuffer;                  /*!< -- CSS constellation configuration message buffer */
    NavAttIntMsg sunlineOutBuffer;                      /*!< -- Nav message*/
    int32_t cssDataInMsgID;                             /*!< -- ID for the incoming CSS sensor message*/
    int32_t cssConfigInMsgID;                           /*!< -- ID for the incoming CSS configuration message*/
    int32_t navStateOutMsgId;                            /*!< -- ID for the outgoing body estimate message*/
    int32_t cssWlsFiltResOutMsgId;                       /*!< [-] norm of the output residuals for CSS*/
}CSSWLSConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_cssWlsEst(CSSWLSConfig *ConfigData, uint64_t moduleID);
    void CrossInit_cssWlsEst(CSSWLSConfig *ConfigData, uint64_t moduleID);
    void Update_cssWlsEst(CSSWLSConfig *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    void Reset_cssWlsEst(CSSWLSConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    int computeWlsmn(int numActiveCss, double *H, double *W,
                     double *y, double x[3]);
    void computeWlsResiduals(double *cssMeas, CSSConfigFswMsg *cssConfig,
                             double *wlsEst, double *cssResiduals);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
