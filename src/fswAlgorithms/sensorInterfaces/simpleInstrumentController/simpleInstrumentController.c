/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

/*
    simpleInstrumentController Module

 */

#include "fswAlgorithms/sensorInterfaces/simpleInstrumentController/simpleInstrumentController.h"
#include "architecture/utilities/linearAlgebra.h"
#include <stdio.h>

/*!
    This method initializes the output messages for this module.

 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_simpleInstrumentController(simpleInstrumentControllerConfig *configData, int64_t moduleID)
{
    configData->imaged = 0;
    configData->controllerStatus = 1;
    configData->constraintStartTime = 0.0;
    configData->constraintsActive = 0;

    DeviceCmdMsg_C_init(&configData->deviceCmdOutMsg);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.

 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_simpleInstrumentController(simpleInstrumentControllerConfig *configData, uint64_t callTime, int64_t moduleID)
{
    // check if the required message has not been connected
    if (!AccessMsg_C_isLinked(&configData->locationAccessInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: simpleInstrumentController.locationAccessInMsg wasn't connected.");
    }
    if (!AttGuidMsg_C_isLinked(&configData->attGuidInMsg)) {
        _bskLog(configData->bskLogger, BSK_ERROR, "Error: simpleInstrumentController.attGuidInMsg wasn't connected.");
    }

    // reset the imaged variable to zero
    configData->imaged = 0;
    configData->constraintsActive = 0;
    configData->constraintStartTime = 0.0;
}

/*! Add a description of what this main Update() routine does for this module

 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_simpleInstrumentController(simpleInstrumentControllerConfig *configData, uint64_t callTime, int64_t moduleID)
{
    double sigma_BR_norm; //!< Norm of sigma_BR
    double omega_BR_norm; //!< Norm of omega_BR

    /* Local copies of the msg buffers*/
    AccessMsgPayload accessInMsgBuffer;  //!< local copy of input message buffer
    AttGuidMsgPayload attGuidInMsgBuffer;  //!< local copy of output message buffer
    DeviceStatusMsgPayload deviceStatusInMsgBuffer; //!< local copy of input message buffer
    DeviceCmdMsgPayload deviceCmdOutMsgBuffer;  //!< local copy of output message buffer

    // zero output buffer
    deviceCmdOutMsgBuffer = DeviceCmdMsg_C_zeroMsgPayload();

    // read in the input messages
    accessInMsgBuffer = AccessMsg_C_read(&configData->locationAccessInMsg);
    attGuidInMsgBuffer = AttGuidMsg_C_read(&configData->attGuidInMsg);

    // read in the device cmd message if it is connected
    if (DeviceStatusMsg_C_isLinked(&configData->deviceStatusInMsg)) {
        deviceStatusInMsgBuffer = DeviceStatusMsg_C_read(&configData->deviceStatusInMsg);
        configData->controllerStatus = deviceStatusInMsgBuffer.deviceStatus;
    }

    // Compute the norms of the attitude and rate errors
    sigma_BR_norm = v3Norm(attGuidInMsgBuffer.sigma_BR);
    omega_BR_norm = v3Norm(attGuidInMsgBuffer.omega_BR_B);

    // If the controller is active
    if (configData->controllerStatus) {
        // If the target has not been imaged
        if (!configData->imaged) {
            unsigned int constraintsMet =
                (sigma_BR_norm <= configData->attErrTolerance)
                && (!configData->useRateTolerance || (omega_BR_norm <= configData->rateErrTolerance))
                && (accessInMsgBuffer.hasAccess);

            if (constraintsMet) {
                // Default: immediate imaging
                if (!configData->useDurationImaging) {
                    deviceCmdOutMsgBuffer.deviceCmd = 1;
                    configData->imaged = 1;
                }
                // Duration-based imaging
                else {
                    if (!configData->constraintsActive) {
                        configData->constraintsActive = 1;
                        configData->constraintStartTime = callTime;  // current sim time
                    }

                    if (configData->acquisitionTime < 0.0) {
                        // Negative acquisitionTime is invalid; cap to zero
                        configData->acquisitionTime = 0.0;
                        _bskLog(configData->bskLogger, BSK_WARNING,
                            "simpleInstrumentController: acquisitionTime is negative and has been set to zero.");
                    }

                    if (configData->allowedTime < 0.0) {
                        // Negative allowedTime is invalid; cap to zero
                        configData->allowedTime = 0.0;
                        _bskLog(configData->bskLogger, BSK_WARNING,
                            "simpleInstrumentController: allowedTime is negative and has been set to zero.");
                    }

                    double elapsedTime = callTime - configData->constraintStartTime;

                    // Determine the effective time to image: cannot exceed allowedTime
                    double effectiveImageTime = (configData->acquisitionTime > configData->allowedTime)
                                                ? configData->allowedTime
                                                : configData->acquisitionTime;

                    if (elapsedTime >= effectiveImageTime) {
                        // If full effective duration passed, set imaged
                        if (configData->acquisitionTime <= configData->allowedTime) {
                            configData->imaged = 1;  // Success
                            deviceCmdOutMsgBuffer.deviceCmd = 1;
                        } else {
                            // Failed because required time > allowed duration
                            configData->imaged = 0;
                            deviceCmdOutMsgBuffer.deviceCmd = 0;
                            configData->controllerStatus = 0; // Disable further attempts
                        }
                        configData->constraintsActive = 0;  // Reset timer
                    }
                }
            } else {
                // Reset if constraints break
                configData->constraintsActive = 0;
            }
        }
    }

    // write to the output messages
    DeviceCmdMsg_C_write(&deviceCmdOutMsgBuffer, &(configData->deviceCmdOutMsg), moduleID, callTime);

    return;
}
