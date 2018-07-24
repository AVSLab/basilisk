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

#ifndef _RATE_SERVO_FULL_NONLINEAR_
#define _RATE_SERVO_FULL_NONLINEAR_

#include "messaging/static_messaging.h"
#include "fswMessages/attGuidFswMsg.h"
#include "fswMessages/vehicleConfigFswMsg.h"
#include "fswMessages/rwArrayConfigFswMsg.h"
#include "fswMessages/rwAvailabilityFswMsg.h"
#include "fswMessages/rateCmdFswMsg.h"
#include "simFswInterfaceMessages/rwSpeedIntMsg.h"
#include "simFswInterfaceMessages/cmdTorqueBodyIntMsg.h"
#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*!@brief Data structure for the module that uses the attiude steering message and
 determine the ADCS control torque vector.

 */

typedef struct {
    /* declare module private variables */
    double K1;                          /*!< [rad/sec] Proportional gain applied to MRP errors */
    double K3;                          /*!< [rad/sec] Cubic gain applied to MRP error in steering saturation function */
    double omega_max;                   /*!< [rad/sec] Maximum rate command of steering control */
    double P;                           /*!< [N*m*s]   Rate error feedback gain applied  */
    double Ki;                          /*!< [N*m]     Integration feedback error on rate error  */
    double integralLimit;               /*!< [N*m]     Integration limit to avoid wind-up issue */
    uint64_t priorTime;                 /*!< [ns]      Last time the attitude control is called */
    double z[3];                        /*!< [rad]     integral state of delta_omega */
    double knownTorquePntB_B[3];        /*!< [N*m]     known external torque in body frame vector components */

    
    double ISCPntB_B[9];                /*!< [kg m^2] Spacecraft Inertia */
    RWArrayConfigFswMsg rwConfigParams; /*!< [-] struct to store message containing RW config parameters in body B frame */

    /* declare module IO interfaces */
    char rwParamsInMsgName[MAX_STAT_MSG_LENGTH];        /*!< The name of the RWArrayConfigFswMsg input message*/
    int32_t rwParamsInMsgID;                            /*!< [-] ID for the RWArrayConfigFswMsg ingoing message */
    char vehConfigInMsgName[MAX_STAT_MSG_LENGTH];
    int32_t vehConfigInMsgID;
    char rwAvailInMsgName[MAX_STAT_MSG_LENGTH];         /*!< [-] The name of the RWs availability message*/
    int32_t rwAvailInMsgID;                             /*!< [-] ID for the incoming  RWs availability data*/
    
    char outputDataName[MAX_STAT_MSG_LENGTH];   /*!< The name of the output message*/
    int32_t outputMsgID;                        /*!< [] ID for the outgoing body accel requests*/
    char inputGuidName[MAX_STAT_MSG_LENGTH];    /*!< The name of the Input message*/
    int32_t inputGuidID;                        /*!< [] ID for the incoming guidance errors*/
    char inputRWSpeedsName[MAX_STAT_MSG_LENGTH];/*!< [] The name for the reaction wheel speeds message */
    int32_t inputRWSpeedsID;                    /*!< [] The ID for the reaction wheel speeds message*/
    char inputRateSteeringName[MAX_STAT_MSG_LENGTH];  /*!< [] the name of the steering law message */
    int32_t inputRateSteeringID;                /*!< [] ID for the incoming steering law message */
    
    CmdTorqueBodyIntMsg controlOut;             /*!< [] Control output requests */
}rateServoFullNonlinearConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_rateServoFullNonlinear(rateServoFullNonlinearConfig *ConfigData, uint64_t moduleID);
    void CrossInit_rateServoFullNonlinear(rateServoFullNonlinearConfig *ConfigData, uint64_t moduleID);
    void Update_rateServoFullNonlinear(rateServoFullNonlinearConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_rateServoFullNonlinear(rateServoFullNonlinearConfig *ConfigData, uint64_t callTime, uint64_t moduleID);

    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
