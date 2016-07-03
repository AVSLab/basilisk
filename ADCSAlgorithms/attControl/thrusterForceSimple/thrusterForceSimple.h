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
    double dummy;                                   /*!< [units] sample module variable declaration */
    double dumVector[3];                            /*!< [units] sample vector variable */

    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!< The name of the output message*/
    int32_t outputMsgID;                            /*!< ID for the outgoing message */
    char inputDataName[MAX_STAT_MSG_LENGTH];        /*!< The name of the Input message*/
    int32_t inputMsgID;                             /*!< ID for the incoming message */

    double  inputVector[3];                         /*!< [units]  vector description */

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
