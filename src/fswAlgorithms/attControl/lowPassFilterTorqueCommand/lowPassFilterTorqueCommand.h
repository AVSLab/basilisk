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

#ifndef _LOW_PASS_FILTER_TORQUE_COMMAND_
#define _LOW_PASS_FILTER_TORQUE_COMMAND_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "simFswInterfaceMessages/cmdTorqueBodyIntMsg.h"


/*! \addtogroup ADCSAlgGroup
 * @{
 */

#define NUM_LPF       2                             /*            number of states to track, including current state */


/*!@brief Data structure for the algorithm Module that applies a low pass filter to the
 attitude control torque command.

 The module
 [PDF Description](AVS-Sim-LowPassFilterControlTorque-20160108.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
 */

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* declare module private variables */
    double   h;                                     /*!< [s]      filter time step (assumed to be fixed */
    double   wc;                                    /*!< [rad/s]  continuous filter cut-off frequency */
    double   hw;                                    /*!< [rad]    h times the prewarped discrete time cut-off frequency */
    double   a[NUM_LPF];                            /*!<          filter coefficients for output */
    double   b[NUM_LPF];                            /*!<          filter coefficients for input */
    double   Lr[NUM_LPF][3];                        /*!< [Nm]     prior torque command */
    double   LrF[NUM_LPF][3];                       /*!< [Nm]     prior filtered torque command */
    int      reset;                                 /*!<          flag indicating the filter being started up */

    /* declare module IO interfaces */
    char outputDataName[MAX_STAT_MSG_LENGTH];       /*!< The name of the output message*/
    char inputDataName[MAX_STAT_MSG_LENGTH];        /*!< The name of the Input message*/
    int32_t outputMsgID;                            /*!< [] ID for the outgoing filtered torque message */
    int32_t inputMsgID;                             /*!< [] ID for the commanded torque message */

    CmdTorqueBodyIntMsg controlOut;                /*!< -- Control output message */

}lowPassFilterTorqueCommandConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_lowPassFilterTorqueCommand(lowPassFilterTorqueCommandConfig *ConfigData, uint64_t moduleID);
    void CrossInit_lowPassFilterTorqueCommand(lowPassFilterTorqueCommandConfig *ConfigData, uint64_t moduleID);
    void Update_lowPassFilterTorqueCommand(lowPassFilterTorqueCommandConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_lowPassFilterTorqueCommand(lowPassFilterTorqueCommandConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
