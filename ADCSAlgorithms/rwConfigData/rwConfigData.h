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

#ifndef _FSW_MODULE_TEMPLATE_H_
#define _FSW_MODULE_TEMPLATE_H_

#include "messaging/static_messaging.h"
#include "vehicleConfigData/vehicleConfigData.h"
#include <stdint.h>


/*! \addtogroup ADCSAlgGroup
 * @{
 */

typedef struct{
    double GsMatrix_B[3*MAX_EFF_CNT];   /*!< [-] The RW spin axis matrix in body frame components */
    double JsList[MAX_EFF_CNT];         /*!< [kgm2] he spin axis inertia for RWs*/
    uint32_t numRW;                     /*!< [-] The number of reaction wheels available on vehicle */
}RWConfigParams;

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    RWConstellation rwConstellation;
    /* declare module IO interfaces */
    char rwConstellationInMsgName[MAX_STAT_MSG_LENGTH];  /*!< The name of the RWConstellation input message*/
    int32_t rwConstellationInMsgID;                      /*!< [-] ID for the RWConstellation incoming message */
    char rwParamsOutMsgName[MAX_STAT_MSG_LENGTH];        /*!< The name of the RWConfigParams output message*/
    int32_t rwParamsOutMsgID;                            /*!< [-] ID for the RWConfigParams outgoing message */
    char vehConfigInMsgName[MAX_STAT_MSG_LENGTH];        /*!< The name of the vehicle config data input message*/
    int32_t vehConfigInMsgID;                            /*!< [-] ID for the vehicle config data incoming message */
    RWConfigParams  rwConfigParamsOut; /* struct for the ouput */

}rwConfigData;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_rwConfigData(rwConfigData*ConfigData, uint64_t moduleID);
    void CrossInit_rwConfigData(rwConfigData *ConfigData, uint64_t moduleID);
    void Update_rwConfigData(rwConfigData *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_rwConfigData(rwConfigData *ConfigData, uint64_t callTime, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
