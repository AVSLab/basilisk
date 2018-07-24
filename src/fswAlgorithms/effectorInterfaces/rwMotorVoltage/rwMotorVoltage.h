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

#ifndef _RW_MOTOR_VOLTAGE_H_
#define _RW_MOTOR_VOLTAGE_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "fswMessages/rwAvailabilityFswMsg.h"
#include "simFswInterfaceMessages/rwSpeedIntMsg.h"
#include "simFswInterfaceMessages/rwArrayTorqueIntMsg.h"
#include "simFswInterfaceMessages/rwArrayVoltageIntMsg.h"
#include "fswMessages/rwArrayConfigFswMsg.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*!@brief Data structure for module to compute the RW motor voltage from the command torque.

 The module
 [PDF Description](Basilisk-rwMotorVoltage-20170113.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
 */

typedef struct {
    /* declare module private variables */
    double VMin;                                    /*!< [V]    minimum voltage below which the torque is zero */
    double VMax;                                    /*!< [V]    maximum output voltage */
    double K;                                       /*!< [V/Nm] torque tracking gain for closed loop control.*/
    double rwSpeedOld[MAX_EFF_CNT];                 /*!< [r/s]  the RW spin rates from the prior control step */
    uint64_t priorTime;                             /*!< [ns]   Last time the module control was called */
    int    resetFlag;                               /*!< []     Flag indicating that a module reset occured */

    /* declare module IO interfaces */
    char voltageOutMsgName[MAX_STAT_MSG_LENGTH];    /*!< The name of the voltage output message*/
    int32_t voltageOutMsgID;                        /*!< ID for the outgoing voltage message */
    
    char torqueInMsgName[MAX_STAT_MSG_LENGTH];      /*!< The name of the Input torque message*/
    int32_t torqueInMsgID;                          /*!< ID for the incoming torque message */
    char rwParamsInMsgName[MAX_STAT_MSG_LENGTH];     /*!< The name of the RWArrayConfigFswMsg input message*/
    int32_t rwParamsInMsgID;                         /*!< [-] ID for the RWArrayConfigFswMsg ingoing message */
    char inputRWSpeedsInMsgName[MAX_STAT_MSG_LENGTH];/*!< [] The name for the reaction wheel speeds message. Must be provided to enable speed tracking loop */
    int32_t inputRWSpeedsInMsgID;                    /*!< [] The ID for the reaction wheel speeds message. If negative, no speed tracking*/
    char rwAvailInMsgName[MAX_STAT_MSG_LENGTH];      /*!< [-] The name of the RWs availability message*/
    int32_t rwAvailInMsgID;                          /*!< [-] ID for the incoming  RWs availability data*/

    RWArrayConfigFswMsg rwConfigParams;                  /*!< [-] struct to store message containing RW config parameters in body B frame */
    RWArrayVoltageIntMsg voltageOut;                /*!< -- copy of the output message */

}rwMotorVoltageConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_rwMotorVoltage(rwMotorVoltageConfig *ConfigData, uint64_t moduleID);
    void CrossInit_rwMotorVoltage(rwMotorVoltageConfig *ConfigData, uint64_t moduleID);
    void Update_rwMotorVoltage(rwMotorVoltageConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_rwMotorVoltage(rwMotorVoltageConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
