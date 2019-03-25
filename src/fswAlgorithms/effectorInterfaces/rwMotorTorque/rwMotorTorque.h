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

#ifndef _RW_MOTOR_TORQUE_H_
#define _RW_MOTOR_TORQUE_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "simFswInterfaceMessages/rwSpeedIntMsg.h"
#include "simFswInterfaceMessages/rwArrayTorqueIntMsg.h"
#include "fswMessages/rwAvailabilityFswMsg.h"
#include "fswMessages/rwArrayConfigFswMsg.h"
#include "simFswInterfaceMessages/cmdTorqueBodyIntMsg.h"


/*! \defgroup rwMotorTorque
 * @brief This module maps a desired torque to control the spacecraft, and maps it to the available wheels using a minimum norm inverse fit.
 * More information can be found in the [PDF Description](Basilisk-rwMotorTorque-20190320.pdf).
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    double   controlAxes_B[3*3];        /*!< [-] array of the control unit axes */
    uint32_t numControlAxes;            /*!< [-] counter indicating how many orthogonal axes are controlled */
    uint32_t numAvailRW;                /*!< [-] number of reaction wheels available */
    RWArrayConfigFswMsg rwConfigParams;     /*!< [-] struct to store message containing RW config parameters in body B frame */
    double GsMatrix_B[3*MAX_EFF_CNT];   /*!< [-] The RW spin axis matrix in body frame components */
    double CGs[3][MAX_EFF_CNT];

    /* declare module IO interfaces */
    char     outputDataName[MAX_STAT_MSG_LENGTH];   /*!< The name of the output message*/
    int32_t  outputMsgID;                           /*!< ID for the outgoing message */
    char inputVehControlName[MAX_STAT_MSG_LENGTH];  /*!< The name of the vehicle control (Lr) Input message*/
    int32_t  inputVehControlID;                     /*!< ID for the incoming Lr control message */
    
    char rwParamsInMsgName[MAX_STAT_MSG_LENGTH];    /*!< The name of the RWArrayConfigFswMsg input message*/
    int32_t rwParamsInMsgID;                        /*!< [-] ID for the RWArrayConfigFswMsg ingoing message */
    char rwAvailInMsgName[MAX_STAT_MSG_LENGTH];     /*!< The name of the RWs availability message*/
    int32_t rwAvailInMsgID;                         /*!< [-] ID for the incoming  RWs availability data */
    
    RWArrayTorqueIntMsg rwMotorTorques;            /*!< [-] struct to store the output message */

}rwMotorTorqueConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_rwMotorTorque(rwMotorTorqueConfig *ConfigData, uint64_t moduleID);
    void CrossInit_rwMotorTorque(rwMotorTorqueConfig *ConfigData, uint64_t moduleID);
    void Update_rwMotorTorque(rwMotorTorqueConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_rwMotorTorque(rwMotorTorqueConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
