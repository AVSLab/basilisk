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

#ifndef _RW_CONFIG_DATA_H_
#define _RW_CONFIG_DATA_H_

#include "messaging/static_messaging.h"
#include "fswMessages/vehicleConfigFswMsg.h"
#include "fswMessages/rwArrayConfigFswMsg.h"
#include "fswMessages/rwConstellationFswMsg.h"
#include <stdint.h>


/*! \addtogroup ADCSAlgGroup
 * @{
 */



/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    RWConstellationFswMsg rwConstellation; /* struct to populate input RW config parameters in structural S frame */
    RWArrayConfigFswMsg  rwConfigParamsOut; /* struct to populate ouput RW config parameters in body B frame */
    /* declare module IO interfaces */
    char rwConstellationInMsgName[MAX_STAT_MSG_LENGTH];  /*!< The name of the RWConstellationFswMsg input message*/
    int32_t rwConstellationInMsgID;                      /*!< [-] ID for the RWConstellationFswMsg incoming message */
    char rwParamsOutMsgName[MAX_STAT_MSG_LENGTH];        /*!< The name of the RWArrayConfigFswMsg output message*/
    int32_t rwParamsOutMsgID;                            /*!< [-] ID for the RWArrayConfigFswMsg outgoing message */
    char vehConfigInMsgName[MAX_STAT_MSG_LENGTH];        /*!< The name of the vehicle config data input message*/
    int32_t vehConfigInMsgID;                            /*!< [-] ID for the vehicle config data incoming message */

}rwConfigData_Config;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_rwConfigData(rwConfigData_Config*ConfigData, uint64_t moduleID);
    void CrossInit_rwConfigData(rwConfigData_Config *ConfigData, uint64_t moduleID);
    void Update_rwConfigData(rwConfigData_Config *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_rwConfigData(rwConfigData_Config *ConfigData, uint64_t callTime, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
