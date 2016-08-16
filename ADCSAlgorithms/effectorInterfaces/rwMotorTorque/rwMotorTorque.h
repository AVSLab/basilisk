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

#ifndef _RW_MOTOR_TORQUE_H_
#define _RW_MOTOR_TORQUE_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/vehControlOut.h"
#include "effectorInterfaces/_GeneralModuleFiles/rwSpeedData.h"
#include "effectorInterfaces/_GeneralModuleFiles/rwDeviceStates.h"
#include "effectorInterfaces/errorConversion/vehEffectorOut.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    double   controlAxes_B[3*3];                    /*!< []      array of the control unit axes */
    double   gsHat_B[MAX_EFF_CNT][3];               /*!< []      local copy of the rw torque axis */
    uint32_t numOfAxesToBeControlled;               /*!< []      counter indicating how many orthogonal axes are controlled */
    uint32_t numRW;                               /*!< []      The number of RWs installed on vehicle */
    int wheelsAvailability[MAX_EFF_CNT];

    /* declare module IO interfaces */
    char     outputDataName[MAX_STAT_MSG_LENGTH];   /*!< The name of the output message*/
    int32_t  outputMsgID;                           /*!< ID for the outgoing message */
    char inputVehControlName[MAX_STAT_MSG_LENGTH];  /*!< The name of the vehicle control (Lr) Input message*/
    int32_t  inputVehControlID;                     /*!< ID for the incoming Lr control message */
    char inputRWConfigDataName[MAX_STAT_MSG_LENGTH];/*!< The name of the RWA cluster Input message*/
    int32_t  inputRWConfID;                   /*!< [-] ID for the incoming RW configuration data*/
    char inputVehicleConfigDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the Input message*/
    int32_t inputVehicleConfigDataID;               /*!< [] ID for the incoming static vehicle data */
    char inputRWsAvailDataName[MAX_STAT_MSG_LENGTH]; /*!< The name of the RWs availability message*/
    int32_t inputRWsAvailID; /*!< [-] ID for the incoming  RWs availability data*/
    vehicleConfigData   sc;                         /*!< spacecraft configuration message */
    vehEffectorOut rwMotorTorques;                  /*!< -- copy of the output message */

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
