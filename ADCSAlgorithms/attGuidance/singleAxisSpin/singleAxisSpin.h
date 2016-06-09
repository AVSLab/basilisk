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

#ifndef _singleAxis_SPIN_
#define _singleAxis_SPIN_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/attGuidOut.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* Declare module private variables */
    double      sigma_R0N[3];                           /*!< base MRP attitude set*/
    double      rotVector[3];                           /*!< (r/s) The vector we are rotating about*/
    uint64_t    mnvrStartTime;                          /*!< (ns) The time that the attitude maneuver started*/
    
    /* Declare module IO interfaces */
    char        outputDataName[MAX_STAT_MSG_LENGTH];    /*!< The name of the output message*/
    int32_t     outputMsgID;                            /*!< ID for the outgoing message */
    
    /* Output attitude reference data to send */
    attRefOut attRefOut;
}singleAxisSpinConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_singleAxisSpin(singleAxisSpinConfig *ConfigData, uint64_t moduleID);
    void CrossInit_singleAxisSpin(singleAxisSpinConfig *ConfigData, uint64_t moduleID);
    void Update_singleAxisSpin(singleAxisSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_singleAxisSpin(singleAxisSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    
    void computeSingleAxisSpinReference(singleAxisSpinConfig *ConfigData, uint64_t callTime);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
