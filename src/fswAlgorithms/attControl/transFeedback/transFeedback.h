/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef TRANSFEEDBACK_H
#define TRANSFEEDBACK_H

#include <stdint.h>
#include "cMsgCInterface/TransRefMsg_C.h"
#include "cMsgCInterface/SCStatesMsg_C.h"
#include "cMsgCInterface/VehicleConfigMsg_C.h"
#include "cMsgCInterface/CmdForceInertialMsg_C.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief This is a module that computes the translational tracking error and uses a PD controller to output the required forces.
 */
typedef struct {

    /* declare module IO interfaces */
    TransRefMsg_C transRefInMsg;  //!< input msg reference translation
    SCStatesMsg_C scStateInMsg;  //!< input msg spacecraft state
    VehicleConfigMsg_C vehConfigInMsg;  //!< vehicle configuration input message
    CmdForceInertialMsg_C cmdForceOutMsg;  //!< commanded spacecraft external control force output message

    double K[3][3];  //!< [kg/s^2] Proportional gain applied to translational errors
    double P[3][3];  //!< [kg/s] Rate error feedback gain applied to translational errors
    double massSC;  //!< [kg] Spacecraft mass
    double knownForcePntC_N[3];  //!< [N] Known external force in inertial frame vector components

    BSKLogger *bskLogger;  //!< BSK Logging
}transFeedbackConfig;

#ifdef __cplusplus
extern "C" {
#endif
    void SelfInit_transFeedback(transFeedbackConfig *configData, int64_t moduleID);
    void Update_transFeedback(transFeedbackConfig *configData, uint64_t callTime, int64_t moduleID);
    void Reset_transFeedback(transFeedbackConfig *configData, uint64_t callTime, int64_t moduleID);

#ifdef __cplusplus
}
#endif

#endif
