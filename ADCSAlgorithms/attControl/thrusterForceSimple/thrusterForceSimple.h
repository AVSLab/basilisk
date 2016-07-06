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

#ifndef _THRUSTER_FORCE_SIMPLE_H_
#define _THRUSTER_FORCE_SIMPLE_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/vehControlOut.h"
#include "effectorInterfaces/errorConversion/vehEffectorOut.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    uint32_t ignoreBodyAxis1;                       /*!< []      Boolean flag indicating if Lr(1) should be ignored */
    uint32_t ignoreBodyAxis2;                       /*!< []      Boolean flag indicating if Lr(2) should be ignored */
    uint32_t ignoreBodyAxis3;                       /*!< []      Boolean flag indicating if Lr(3) should be ignored */
    uint32_t ignoreBodyAxis[3];                     /*!< []      array of boolean flags to ignore body axis */
    uint32_t flipLrSign;                            /*!< []      Boolean flag indicating if Lr or -Lr should be used */
    uint32_t numThrusters;                          /*!< []      The number of thrusters available on vehicle */
    double   D[3][MAX_EFF_CNT];                     /*!< [m]     mapping matrix from thruster forces to body torque */
    double   Gt[3][MAX_EFF_CNT];                    /*!< []      matrix containing the thrust direction unit vectors gHat_t_i */
    double   DTDDTinv[MAX_EFF_CNT][3];              /*!< [1/m]   mapping matrix from command torque Lr to thruster force sets */
    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!< The name of the output message*/
    int32_t outputMsgID;                            /*!< ID for the outgoing message */
    char inputVehControlName[MAX_STAT_MSG_LENGTH];  /*!< The name of the vehicle control (Lr) Input message*/
    char inputThrusterConfName[MAX_STAT_MSG_LENGTH];/*!< The name of the thruster cluster Input message*/
    int32_t inputVehControlID;                      /*!< ID for the incoming Lr control message */
    int32_t inputThrusterConfID;                    /*!< [-] ID for the incoming Thruster configuration data*/

    double  Lr_B[3];                                /*!< [Nm]    commanded ADCS control torque */

    vehEffectorOut thrusterForceOut;                /*!< -- copy of the output message */

}thrusterForceSimpleConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_thrusterForceSimple(thrusterForceSimpleConfig *ConfigData, uint64_t moduleID);
    void CrossInit_thrusterForceSimple(thrusterForceSimpleConfig *ConfigData, uint64_t moduleID);
    void Update_thrusterForceSimple(thrusterForceSimpleConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_thrusterForceSimple(thrusterForceSimpleConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
