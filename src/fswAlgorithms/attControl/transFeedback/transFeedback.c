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


#include "fswAlgorithms/attControl/transFeedback/transFeedback.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/macroDefinitions.h"

#include <string.h>
#include <math.h>

/*!
    This method initializes the output messages for this module.

 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_transFeedback(transFeedbackConfig  *configData, int64_t moduleID)
{
    CmdForceInertialMsg_C_init(&configData->cmdForceOutMsg);
}


/*! This method performs a complete reset of the module.  Local module variables that retain
    time varying states between function calls are reset to their default values.
    Check if required input messages are connected.

 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_transFeedback(transFeedbackConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // check if the required message has not been connected
    if (!TransRefMsg_C_isLinked(&configData->transRefInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: transFeedback.transRefInMsg was not connected.");
    }
    if (!SCStatesMsg_C_isLinked(&configData->scStateInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: transFeedback.scStateInMsg was not connected.");
    }
    if (!VehicleConfigMsg_C_isLinked(&configData->vehConfigInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: transFeedback.vehConfigInMsg was not connected.");
    }

    /*! - zero and read in vehicle configuration message */
    VehicleConfigMsgPayload sc;
    sc = VehicleConfigMsg_C_read(&configData->vehConfigInMsg);
    /*! - copy over spacecraft inertia tensor */
    configData->massSC = sc.massSC;
}


/*! Add a description of what this main Update() routine does for this module

 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_transFeedback(transFeedbackConfig *configData, uint64_t callTime, int64_t moduleID)
{
    TransRefMsgPayload transRefInMsg;  //!< local copy of message buffer
    SCStatesMsgPayload scStateInMsg;  //!< local copy of message buffer
    VehicleConfigMsgPayload vehConfigInMsg;  //!< local copy of message buffer
    CmdForceInertialMsgPayload controlOut;  //!< local copy of message buffer

    double deltaX[3];
    double deltaXdot[3];
    double Fr[3];
    double v1[3];
    double v2[3];
    double v3[3];

    /*! zero the output message */
    controlOut = CmdForceInertialMsg_C_zeroMsgPayload();

    /*! read in the input messages */
    transRefInMsg = TransRefMsg_C_read(&configData->transRefInMsg);
    scStateInMsg = SCStatesMsg_C_read(&configData->scStateInMsg);

    /*! calculate the state errors*/
    v3Subtract(scStateInMsg.r_CN_N, transRefInMsg.r_RN_N, deltaX);
    v3Subtract(scStateInMsg.v_CN_N, transRefInMsg.v_RN_N, deltaXdot);

    /*! calculate the control forces */
    m33MultV3(configData->K, deltaX, v1);
    m33MultV3(configData->P, deltaXdot, v2);
    v3Scale(-1.0, v1, v1);
    v3Scale(-1.0, v2, v2);
    v3Scale(configData->massSC, transRefInMsg.a_RN_N, v3);
    v3Add(v1, v2, Fr);
    v3Add(Fr, v3, Fr);
    v3Add(Fr, configData->knownForcePntC_N, Fr);

    /*! set the output message and write it*/
    v3Copy(Fr, controlOut.forceRequestInertial);
    CmdForceInertialMsg_C_write(&controlOut, &configData->cmdForceOutMsg, moduleID, callTime);
}
