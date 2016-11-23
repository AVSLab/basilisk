/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _INERTIAL3D_SPIN_
#define _INERTIAL3D_SPIN_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/attGuidOut.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    double sigma_RN[3];
    double omega_spin[3];
    uint64_t priorTime;                              /*!< [ns] last time the guidance module is called */
    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];        /*!< Name of the outgoing guidance reference message */
    int32_t outputMsgID;                             /*!< [-] ID for the outgoing guidance reference message */
    char inputRefName[MAX_STAT_MSG_LENGTH];          /*!< The name of the input guidance reference message */
    int32_t inputRefID;                              /*!< [-] ID for the incoming guidance reference message */
    attRefOut attRefOut;                             /*!< [-] structure for the output data */
}inertial3DSpinConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t moduleID);
    void CrossInit_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t moduleID);
    void Update_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_inertial3DSpin(inertial3DSpinConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void computeReference_inertial3DSpin(inertial3DSpinConfig *ConfigData,
                                         double omega_R0N_N[3],
                                         double domega_R0N_N[3],
                                         double omega_RR0_R[3],
                                         double dt);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif
