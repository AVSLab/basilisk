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

#ifndef _MRP_ROTATION_
#define _MRP_ROTATION_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "fswMessages/attStateFswMsg.h"
#include "fswMessages/attRefFswMsg.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* Declare module private variables */
    double mrpSet[3];                           /*!< [-] current MRP attitude coordinate set with respect to the input reference */
    double omega_RR0_R[3];                      /*!< [rad/s] angular velocity vector relative to input reference */
    double cmdSet[3];                           /*!< [] commanded initial MRP set with respect to input reference */
    double cmdRates[3];                         /*!< [rad/s] commanded constant angular velocity vector */
    double priorCmdSet[3];                      /*!< [] prior commanded MRP set */
    double priorCmdRates[3];                    /*!< [rad/s] prior commanded angular velocity vector */
    uint64_t priorTime;                         /*!< [ns] last time the guidance module is called */
    double dt;                                  /*!< [s] integration time-step */
    
    /* Declare module IO interfaces */
    char        attRefOutMsgName[MAX_STAT_MSG_LENGTH];      /*!< The name of the output message containing the Reference */
    int32_t     attRefOutMsgID;                             /*!< [-] ID for the outgoing Reference message */
    char        attitudeOutMsgName[MAX_STAT_MSG_LENGTH];    /*!< The name of the output message containing the current MRP and rate set */
    int32_t     attitudeOutMsgID;                           /*!< [-] ID for the outgoing MRP angles and rates Set message */
    char        attRefInMsgName[MAX_STAT_MSG_LENGTH];       /*!< The name of the guidance reference Input message */
    int32_t     attRefInMsgID;                              /*!< [-] ID for the incoming guidance reference message */
    
    char        desiredAttInMsgName[MAX_STAT_MSG_LENGTH];   /*!< The name of the incoming message containing the desired EA set */
    int32_t     desiredAttInMsgID;                          /*!< [-] ID for the incoming EA set message */

    
    /* Output attitude reference data to send */
    AttRefFswMsg   attRefOut;                               /*!< [-] structure for the Reference output data */
    AttStateFswMsg attStateOut;                             /*!< [-] structure for the attitude reference output data */
}mrpRotationConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_mrpRotation(mrpRotationConfig *ConfigData, uint64_t moduleID);
    void CrossInit_mrpRotation(mrpRotationConfig *ConfigData, uint64_t moduleID);
    void Reset_mrpRotation(mrpRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Update_mrpRotation(mrpRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    
    void writeOutputMessages(mrpRotationConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void checkRasterCommands(mrpRotationConfig *ConfigData);
    void computeTimeStep(mrpRotationConfig *ConfigData, uint64_t callTime);
    void computeMRPRotationReference(mrpRotationConfig *ConfigData,
                                       double sigma_R0N[3],
                                       double omega_R0N_N[3],
                                       double domega_R0N_N[3]);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
