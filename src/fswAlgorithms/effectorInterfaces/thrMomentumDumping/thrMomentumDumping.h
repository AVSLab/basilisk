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

#ifndef _THR_MOMENTUM_DUMPING_H_
#define _THR_MOMENTUM_DUMPING_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "fswMessages/vehicleConfigFswMsg.h"
#include "fswMessages/thrArrayConfigFswMsg.h"
#include "fswMessages/thrArrayCmdForceFswMsg.h"
#include "simFswInterfaceMessages/thrArrayOnTimeCmdIntMsg.h"

/*! \addtogroup ADCSAlgGroup
 * @{
 */


/*!@brief Data structure for module to implement RW momentum dumping.

 The module
 [PDF Description](Basilisk-thrMomentumDumping-20160820.pdf)
 contains further information on this module's function,
 how to run it, as well as testing.
 */

typedef struct {
    /* declare module private variables */
    int32_t     thrDumpingCounter;                      /*!<        counter to specify after how many contro period a thruster
                                                                    firing should occur. */
    double      Delta_p[MAX_EFF_CNT];                   /*!<        vector of desired total thruster impulses */
    double      thrOnTimeRemaining[MAX_EFF_CNT];        /*!<        vector of remaining thruster on times */
    uint64_t    priorTime;                              /*!< [ns]   Last time the attitude control is called */
    int         numThrusters;                           /*!<        number of thrusters installed */
    double      thrMaxForce[MAX_EFF_CNT];               /*!< [N]    vector of maximum thruster forces */
    double      thrMinFireTime;                         /*!< [s]    smallest thruster firing time */

    /* declare module public variables */
    int         maxCounterValue;                        /*!<        this variable must be set to a non-zero value, indicating
                                                                    how many control periods to wait until the thrusters
                                                                    fire again to dump RW momentum  */

    /* declare module IO interfaces */
    char thrusterOnTimeOutMsgName[MAX_STAT_MSG_LENGTH];
    int32_t thrusterOnTimeOutMsgID;
    char thrusterImpulseInMsgName[MAX_STAT_MSG_LENGTH];
    int32_t thrusterImpulseInMsgID;
    char thrusterConfInMsgName[MAX_STAT_MSG_LENGTH];/*!< The name of the thruster cluster Input message*/
    int32_t  thrusterConfInMsgID;                   /*!< [-] ID for the incoming Thruster configuration data*/


    THRArrayOnTimeCmdIntMsg thrOnTimeOut;          /*!< -- copy of the output message */

}thrMomentumDumpingConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_thrMomentumDumping(thrMomentumDumpingConfig *ConfigData, uint64_t moduleID);
    void CrossInit_thrMomentumDumping(thrMomentumDumpingConfig *ConfigData, uint64_t moduleID);
    void Update_thrMomentumDumping(thrMomentumDumpingConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    void Reset_thrMomentumDumping(thrMomentumDumpingConfig *ConfigData, uint64_t callTime, uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif
