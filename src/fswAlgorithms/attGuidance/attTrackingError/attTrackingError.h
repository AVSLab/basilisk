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

#ifndef _ATT_TRACKING_ERROR_
#define _ATT_TRACKING_ERROR_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "simFswInterfaceMessages/navAttIntMsg.h"
#include "fswMessages/attGuidFswMsg.h"
#include "fswMessages/attRefFswMsg.h"
#include "simulation/utilities/bskPrint.h"



/*!@brief Data structure for module to compute the attitude tracking error between the spacecraft attitude and the reference.
 */
typedef struct {
    /* declare module private variables */
    double sigma_R0R[3];                            //!< MRP from corrected reference frame to original reference frame R0. This is the same as [BcB] going from primary body frame B to the corrected body frame Bc
    char outputDataName[MAX_STAT_MSG_LENGTH];       //!< The name of the output message
    char inputRefName[MAX_STAT_MSG_LENGTH];         //!< The name of the guidance reference Input message
    char inputNavName[MAX_STAT_MSG_LENGTH];         //!< The name of the navigation Input message
    int32_t outputMsgID;                            //!< ID for the outgoing message
    int32_t inputRefID;                             //!< ID for the incoming guidance reference message
    int32_t inputNavID;                             //!< ID for the incoming navigation message
    BSKPrint *bskPrint;
}attTrackingErrorConfig;

#ifdef __cplusplus
extern "C" {
#endif

    void SelfInit_attTrackingError(attTrackingErrorConfig *configData, int64_t moduleID);
    void CrossInit_attTrackingError(attTrackingErrorConfig *configData, int64_t moduleID);
    void Update_attTrackingError(attTrackingErrorConfig *configData, uint64_t callTime, int64_t moduleID);
    void Reset_attTrackingError(attTrackingErrorConfig *configData, uint64_t callTime, int64_t moduleID);
    void computeAttitudeError(double sigma_R0R[3], NavAttIntMsg nav, AttRefFswMsg ref, AttGuidFswMsg *attGuidOut);

#ifdef __cplusplus
}
#endif


#endif
