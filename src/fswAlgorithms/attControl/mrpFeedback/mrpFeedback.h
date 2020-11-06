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

#ifndef _MRP_FEEDBACK_CONTROL_H_
#define _MRP_FEEDBACK_CONTROL_H_

#include "messaging/static_messaging.h"
#include "fswMessages/rwArrayConfigFswMsg.h"
#include "simFswInterfaceMessages/rwSpeedIntMsg.h"
#include "fswMessages/rwAvailabilityFswMsg.h"
#include "simulation/utilities/bskLogging.h"
#include "architecture/messaging2/cMsgCInterface/VehicleConfigMsg_C.h"
#include "architecture/messaging2/cMsgCInterface/AttGuidMsg_C.h"
#include "architecture/messaging2/cMsgCInterface/CmdTorqueBodyMsg_C.h"
#include <stdint.h>


/*! @brief Data configuration structure for the MRP feedback attitude control routine. */
typedef struct {
    double K;                           //!< [rad/sec] Proportional gain applied to MRP errors
    double P;                           //!< [N*m*s]   Rate error feedback gain applied
    double Ki;                          //!< [N*m]     Integration feedback error on rate error
    double integralLimit;               //!< [N*m]     Integration limit to avoid wind-up issue
    uint64_t priorTime;                 //!< [ns]      Last time the attitude control is called
    double z[3];                        //!< [rad]     integral state of delta_omega
    double int_sigma[3];                //!< [s]       integral of the MPR attitude error
    double knownTorquePntB_B[3];        //!< [N*m]     known external torque in body frame vector components

    double ISCPntB_B[9];                //!< [kg m^2]  Spacecraft Inertia
    RWArrayConfigFswMsg rwConfigParams; //!< [-] struct to store message containing RW config parameters in body B frame

    /* declare module IO interfaces */
    char rwParamsInMsgName[MAX_STAT_MSG_LENGTH];        //!< The name of the RW array configuration input message
    int32_t rwParamsInMsgId;                            //!< [-] ID for the RWArrayConfigFswMsg ingoing message

    char rwAvailInMsgName[MAX_STAT_MSG_LENGTH];         //!< [-] The name of the RWs availability message
    int32_t rwAvailInMsgId;                             //!< [-] ID for the incoming  RWs availability data

    char inputRWSpeedsName[MAX_STAT_MSG_LENGTH];        //!< [-] The name for the reaction wheel speeds message
    int32_t rwSpeedsInMsgId;                            //!< [-] ID for the reaction wheel speeds message

    CmdTorqueBodyMsg_C cmdTorqueOutMsg;                 //!< output message
    AttGuidMsg_C guidInMsg;                             //!< input message
    VehicleConfigMsg_C vehConfigInMsg;                  //!< input message

    BSKLogger *bskLogger;                               //!< BSK Logging
}mrpFeedbackConfig;

#ifdef __cplusplus
extern "C" {
#endif

    void SelfInit_mrpFeedback(mrpFeedbackConfig *configData, int64_t moduleID);
    void CrossInit_mrpFeedback(mrpFeedbackConfig *configData, int64_t moduleID);
    void Update_mrpFeedback(mrpFeedbackConfig *configData, uint64_t callTime, int64_t moduleID);
    void Reset_mrpFeedback(mrpFeedbackConfig *configData, uint64_t callTime, int64_t moduleID);


#ifdef __cplusplus
}
#endif


#endif
