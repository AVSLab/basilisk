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

#ifndef _SPACECRAFTPOINTING_H_
#define _SPACECRAFTPOINTING_H_

#include "messaging/static_messaging.h"
#include "fswMessages/attGuidFswMsg.h"
#include "fswMessages/attRefFswMsg.h"
#include "simFswInterfaceMessages/navAttIntMsg.h"
#include "simFswInterfaceMessages/navTransIntMsg.h"
#include <stdint.h>

/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the spacecraft pointing module.*/
typedef struct {
    char attReferenceOutMsgName[MAX_STAT_MSG_LENGTH];   /*!< The name of the output message */
    char chiefPositionInMsgName[MAX_STAT_MSG_LENGTH];   /*!< The name of the Input message of the chief */
    char deputyPositionInMsgName[MAX_STAT_MSG_LENGTH];  /*!< The name of the Input message of the deputy */
    double alignmentVector_B[3];                        /*!< Vector within the B-frame that points to antenna */
    double sigma_BA[3];                                 /*!< -- MRP of B-frame with respect to A-frame */
    double old_sigma_RN[3];                             /*!< -- MRP of previous timestep */
    double old_omega_RN_N[3];                           /*!< -- Omega of previous timestep */
    int i;                                              /*!< -- Flag used to set incorrect numerical answers to zero */
    int32_t attReferenceOutMsgID;                       /*!< -- ID for the outgoing reference message */
    int32_t chiefPositionInMsgID;                       /*!< -- ID for the incoming chief position message */
    int32_t deputyPositionInMsgID;                      /*!< -- ID for the incoming deputy position message */
    uint64_t priorTime;                                 /*!< [ns] Last time the attitude control is called */
    AttRefFswMsg attReferenceOutBuffer;
}spacecraftPointingConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_spacecraftPointing(spacecraftPointingConfig *ConfigData, uint64_t moduleID);
    void CrossInit_spacecraftPointing(spacecraftPointingConfig *ConfigData, uint64_t moduleID);
    void Update_spacecraftPointing(spacecraftPointingConfig *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    void Reset_spacecraftPointing(spacecraftPointingConfig *ConfigData, uint64_t callTime, uint64_t moduleID);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
