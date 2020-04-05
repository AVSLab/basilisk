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

#ifndef _SPACECRAFT_RECONFIG_H_
#define _SPACECRAFT_RECONFIG_H_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "simFswInterfaceMessages/navTransIntMsg.h"
#include "../../fswMessages/thrArrayConfigFswMsg.h"
#include "fswMessages/attRefFswMsg.h"
#include "simFswInterfaceMessages/thrArrayOnTimeCmdIntMsg.h"
#include "simulation/utilities/bskLogging.h"

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    uint8_t flag;        // 0:not scheduled yet, 1:not burned yet, 2:already burned
    double t;            // when to burn [s]
    double thrustOnTime; // thrust on duration time [s]
    double sigma_RN[3];  // target attitude
}spacecraftReconfigConfigBurnInfo;

typedef struct {
    /* declare module IO interfaces */
    // in
    char chiefTransInMsgName[MAX_STAT_MSG_LENGTH];
    int32_t chiefTransInMsgID;
    char deputyTransInMsgName[MAX_STAT_MSG_LENGTH];
    int32_t deputyTransInMsgID;
    char thrustConfigInMsgName[MAX_STAT_MSG_LENGTH];
    int32_t thrustConfigInMsgID;
    char attRefInMsgName[MAX_STAT_MSG_LENGTH];
    int32_t attRefInMsgID;
    // out
    char attRefOutMsgName[MAX_STAT_MSG_LENGTH];
    int32_t attRefOutMsgID;
    char onTimeOutMsgName[MAX_STAT_MSG_LENGTH];
    int32_t onTimeOutMsgID;
    // central body gravity constant
    double mu;  // [m^3/s^2]
    // attitude control margin time (time necessary to change sc's attitude)
    double attControlTime; // [s]
    // target classic orital element difference
    double targetClassicOED[6]; // SMA should be normalized
    // burn scheduling reset period
    double resetPeriod; // [s]
    // deputy SC mass
    double scMassDeputy; //[kg]
    // timer
    double tCurrent; // [s]
    uint64_t prevCallTime; // [ns]
    // thrust control
    uint8_t thrustOnFlag;
    spacecraftReconfigConfigBurnInfo dvArray[3];
}spacecraftReconfigConfig;

#ifdef __cplusplus
extern "C" {
#endif
    void SelfInit_spacecraftReconfig(spacecraftReconfigConfig *configData, int64_t moduleID);
    void CrossInit_spacecraftReconfig(spacecraftReconfigConfig *configData, int64_t moduleID);
    void Update_spacecraftReconfig(spacecraftReconfigConfig *configData, uint64_t callTime, int64_t moduleID);
    void Reset_spacecraftReconfig(spacecraftReconfigConfig *configData, uint64_t callTime, int64_t moduleID);
#ifdef __cplusplus
}
#endif

#endif
