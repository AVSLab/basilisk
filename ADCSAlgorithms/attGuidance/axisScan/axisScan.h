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

#ifndef _AXIS_SCAN_
#define _AXIS_SCAN_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "../_GeneralModuleFiles/attGuidOut.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */



typedef struct {
    /* Declare module private variables */
    double      theta0;                                     /* 2nd Euler Angle offset */
    double      psi0;                                       /* 3rd Euler Angle offset */
    double      psiDot;                                     /* Scanning rate */
    double      sigma_UN[3];                                /* offset from inertial N to initial scanning reference U */
    uint64_t    mnvrStartTime;                              /*!< (ns) The time that the attitude maneuver started*/
    
    /* Declare module IO interfaces */
    char        outputDataName[MAX_STAT_MSG_LENGTH];        /*!< The name of the output message*/
    int32_t     outputMsgID;                                /*!< ID for the outgoing message */
    char        inputRefName[MAX_STAT_MSG_LENGTH];          /*!< The name of the guidance reference Input message */
    int32_t     inputRefID;                                 /*!< ID for the incoming guidance reference message */
    
    /* Output attitude reference data to send */
    attRefOut   attRefOut;
}axisScanConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_axisScan(axisScanConfig *ConfigData, uint64_t moduleID);
    void CrossInit_axisScan(axisScanConfig *ConfigData, uint64_t moduleID);
    void Reset_axisScan(axisScanConfig *ConfigData, uint64_t moduleID);
    void Update_axisScan(axisScanConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    
    void initializeScanReference(axisScanConfig *ConfigData, double sigma_R0N[3]);
    void computeAxisScanReference(axisScanConfig *ConfigData,
                                  double sigma_R0N[3],
                                  double omegaa_R0N_N[3],
                                  double domega_R0N_N[3],
                                  uint64_t callTime);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
