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

#ifndef _MEAN_OE_FEEDBACK_H_
#define _MEAN_OE_FEEDBACK_H_

#include <stdint.h>

#include "messaging/static_messaging.h"
#include "simFswInterfaceMessages/cmdForceInertialIntMsg.h"
#include "simFswInterfaceMessages/navTransIntMsg.h"
#include "simulation/utilities/bskLogging.h"
#include "simulation/utilities/orbitalMotion.h"

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module IO interfaces */
    // in
    char ChiefTransInMsgName[MAX_STAT_MSG_LENGTH];
    int32_t ChiefTransInMsgID;
    char DeputyTransInMsgName[MAX_STAT_MSG_LENGTH];
    int32_t DeputyTransInMsgID;
    // out
    char ForceOutMsgName[MAX_STAT_MSG_LENGTH];
    int32_t ForceOutMsgID;
    // Lyapunov Gain (6*6)
    double K[36];
    // target mean orital element difference
    double target_oe_mean[6];
    // parameters
    uint8_t oe_type;  // 0: classic, 1: equinoctial
    double mu;        // [m^3/s^2]
    double req;       // [m]
    double J2;        // []
} meanOEFeedbackConfig;

#ifdef __cplusplus
extern "C" {
#endif
void SelfInit_meanOEFeedback(meanOEFeedbackConfig *configData, int64_t moduleID);
void CrossInit_meanOEFeedback(meanOEFeedbackConfig *configData, int64_t moduleID);
void Update_meanOEFeedback(meanOEFeedbackConfig *configData, uint64_t callTime, int64_t moduleID);
void Reset_meanOEFeedback(meanOEFeedbackConfig *configData, uint64_t callTime, int64_t moduleID);
#ifdef __cplusplus
}
#endif

#endif
