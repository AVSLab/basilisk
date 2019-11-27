/*
 ISC License

 Copyright (c) 2019, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _TAM_COMM_H_
#define _TAM_COMM_H_

#include "messaging/static_messaging.h"
#include "fswMessages/vehicleConfigFswMsg.h"
#include "fswMessages/tamSensorBodyFswMsg.h"
#include "simFswInterfaceMessages/tamSensorIntMsg.h"
#include "simulation/utilities/bskLog.h"


/*! module configuration message definition */
typedef struct {
    double dcm_BS[9];                         //!< [T] Row - Sensor to Body DCM
    char tamInMsgName[MAX_STAT_MSG_LENGTH];   //!< [-] The name of the TAM interface input message
    char tamOutMsgName[MAX_STAT_MSG_LENGTH];  //!< [-] The name of the TAM interface output message
    int32_t tamSensorMsgID;                   //!< [-] TAM sensor IDs tied to the input name
    int32_t tamOutMsgID;                      //!< [-] TAM message ID for the output port
    TAMSensorBodyFswMsg tamLocalOutput;       //!< [-] TAM output data structure
    BSKLogger *bskLogger;                       //!< BSK Logging
}tamConfigData;

#ifdef __cplusplus
extern "C" {
#endif

    void SelfInit_tamProcessTelem(tamConfigData *configData, int64_t moduleID);
    void CrossInit_tamProcessTelem(tamConfigData *configData, int64_t moduleID);
    void Update_tamProcessTelem(tamConfigData *configData, uint64_t callTime, int64_t moduleID);
    void Reset_tamProcessTelem(tamConfigData* configData, uint64_t callTime, int64_t moduleID);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
