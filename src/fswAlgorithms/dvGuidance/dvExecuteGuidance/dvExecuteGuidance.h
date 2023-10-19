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

#ifndef _DV_EXECUTE_GUIDANCE_H_
#define _DV_EXECUTE_GUIDANCE_H_

#include "cMsgCInterface/NavTransMsg_C.h"
#include "cMsgCInterface/THRArrayOnTimeCmdMsg_C.h"
#include "cMsgCInterface/DvBurnCmdMsg_C.h"
#include "cMsgCInterface/DvExecutionDataMsg_C.h"

#include "architecture/utilities/bskLogging.h"
#include <stdint.h>



/*! @brief Top level structure for the execution of a Delta-V maneuver */
typedef struct {
    NavTransMsg_C navDataInMsg; /*!< [-] navigation input message that includes dv accumulation info */
    DvBurnCmdMsg_C burnDataInMsg;/*!< [-] commanded burn input message */
    THRArrayOnTimeCmdMsg_C thrCmdOutMsg; /*!< [-] thruster command on time output message */
    DvExecutionDataMsg_C burnExecOutMsg; /*!< [-] burn execution output message */
    double dvInit[3];        /*!< [m/s] DV reading off the accelerometers at burn start*/
    uint32_t burnExecuting;  /*!< [-] Flag indicating whether the burn is in progress or not*/
    uint32_t burnComplete;   /*!< [-] Flag indicating that burn has completed successfully*/
    double burnTime;          /*!< [s] Burn time to be used for telemetry*/
    uint64_t prevCallTime;   /*!< [-] Call time register for computing total burn time*/
    double minTime;           /*!< [s] Minimum count of burn time allowed to elapse*/
    double maxTime;           /*!< [s] Maximum count of burn time allowed to elapse*/
    double defaultControlPeriod; /*!< [s] Default control period used for first call*/

    BSKLogger *bskLogger;   //!< BSK Logging
}dvExecuteGuidanceConfig;


#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_dvExecuteGuidance(dvExecuteGuidanceConfig *configData, int64_t moduleID);
    void Update_dvExecuteGuidance(dvExecuteGuidanceConfig *configData, uint64_t callTime,
        int64_t moduleID);
    void Reset_dvExecuteGuidance(dvExecuteGuidanceConfig *configData, uint64_t callTime,
                       int64_t moduleID);

#ifdef __cplusplus
}
#endif


#endif
