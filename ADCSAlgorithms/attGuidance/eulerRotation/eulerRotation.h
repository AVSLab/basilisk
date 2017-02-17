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

#ifndef _EULER_ROTATION_
#define _EULER_ROTATION_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "fswMessages/eulerAngleMessage.h"
#include "fswMessages/attRefFswMsg.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* Declare module private variables */
    double angleSet[3];                              /*!< [-] current euler angle 321 set with respect to the input reference */
    double angleRates[3];                            /*!< [rad/s] euler angle 321 rates */
    double cmdSet[3];
    double cmdRates[3];
    double priorCmdSet[3];
    double priorCmdRates[3];
    uint64_t priorTime;                              /*!< [ns] last time the guidance module is called */
    double dt;                                       /*!< [s] integration time-step */
    //AttRefFswMsg inputRef;
    
    /* Declare module IO interfaces */
    char        outputDataName[MAX_STAT_MSG_LENGTH];        /*!< The name of the output message containing the Reference */
    int32_t     outputMsgID;                                /*!< [-] ID for the outgoing Reference message */
    char        outputEulerSetName[MAX_STAT_MSG_LENGTH];    /*!< The name of the output message containing the current Euler Angle set */
    int32_t     outputEulerSetID;                           /*!< [-] ID for the outgoing Euler Angle Set message */
    char        outputEulerRatesName[MAX_STAT_MSG_LENGTH];  /*!< The name of the output message containing the current Euler Angle set */
    int32_t     outputEulerRatesID;                              /*!< [-] ID for the outgoing Euler Angle Set message */
    char        inputRefName[MAX_STAT_MSG_LENGTH];          /*!< The name of the guidance reference Input message */
    int32_t     inputRefID;                                 /*!< [-] ID for the incoming guidance reference message */
    
    char        inputEulerSetName[MAX_STAT_MSG_LENGTH];     /*!< The name of the incoming message containing the desired EA set */
    int32_t     inputEulerSetID;                            /*!< [-] ID for the incoming EA set message */
    char        inputEulerRatesName[MAX_STAT_MSG_LENGTH];   /*!< The name of the incoming message containing the desired EA rates */
    int32_t     inputEulerRatesID;                          /*!< [-] ID for the incoming EA rates message */
    
    
    /* Output attitude reference data to send */
    AttRefFswMsg attRefOut;                                /*!< [-] structure for the Reference output data */
    EulerAngleMessage      eulerSetOut;                     /*!< [-] structure for the Euler Set output data */
    EulerAngleMessage      eulerRatesOut;                   /*!< [-] structure for the Euler Set output data */
}eulerRotationConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_eulerRotation(eulerRotationConfig *ConfigData, uint64_t moduleID);
    void CrossInit_eulerRotation(eulerRotationConfig *ConfigData, uint64_t moduleID);
    void Reset_eulerRotation(eulerRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Update_eulerRotation(eulerRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    
    void writeOutputMessages(eulerRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void checkRasterCommands(eulerRotationConfig *ConfigData);
    void computeTimeStep(eulerRotationConfig *ConfigData, uint64_t callTime);
    void computeEuler321_Binv_derivative(double angleSet[3], double angleRates[3], double B_inv_deriv[3][3]);
    void computeEulerRotationReference(eulerRotationConfig *ConfigData,
                                       double sigma_R0N[3],
                                       double omega_R0N_N[3],
                                       double domega_R0N_N[3]);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
