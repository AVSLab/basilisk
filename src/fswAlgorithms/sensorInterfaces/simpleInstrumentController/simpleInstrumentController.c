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
#include <math.h>
#include <stdio.h>

/*!
    This method initializes the output messages for this module.

 @param configData The configuration data associated with this module
 @param moduleID The module identifier
 */
void SelfInit_simpleInstrumentController(simpleInstrumentControllerConfig* configData, int64_t moduleID)
{
    (void)moduleID;
    configData->imaged = 0;
    configData->controllerStatus = 1;
    configData->constraintStartTime = 0.0; // [ns]
    configData->elapsedTime = 0.0;         // [ns]
    configData->constraintsActive = 0;
    configData->durationStarted = 0;
    configData->lastCallTime = 0;           // [ns]
    configData->resetTime = 0;              // [ns]
    configData->illuminationIntegral = 0.0; // [ns]
    configData->previousIllumination = 1.0; // [-]
    configData->captureIllumination = 1.0;  // [-]
    configData->captureTime = 0;            // [ns]

    DeviceCmdMsg_C_init(&configData->deviceCmdOutMsg);
}

/*! This method performs a complete reset of the module.  Local module variables that retain
 time varying states between function calls are reset to their default values.

 @param configData The configuration data associated with the module
 @param callTime [ns] time the method is called
 @param moduleID The module identifier
*/
void Reset_simpleInstrumentController(simpleInstrumentControllerConfig* configData, uint64_t callTime, int64_t moduleID)
{
    // check if the required message has not been connected
    if (!AccessMsg_C_isLinked(&configData->locationAccessInMsg)) {
        _bskError(configData->bskLogger, "Error: simpleInstrumentController.locationAccessInMsg wasn't connected.");
    }
    if (!AttGuidMsg_C_isLinked(&configData->attGuidInMsg)) {
        _bskError(configData->bskLogger, "Error: simpleInstrumentController.attGuidInMsg wasn't connected.");
    }
    if (configData->useIlluminationConstraint && !EclipseMsg_C_isLinked(&configData->illuminationInMsg)) {
        _bskError(configData->bskLogger, "Error: simpleInstrumentController.illuminationInMsg wasn't connected.");
    }
    if (!isfinite(configData->illuminationThreshold) || configData->illuminationThreshold < 0.0 ||
        configData->illuminationThreshold > 1.0) {
        _bskError(configData->bskLogger, "Error: simpleInstrumentController.illuminationThreshold must be in [0, 1].");
    }
    if (!isfinite(configData->acquisitionTime) || !isfinite(configData->allowedTime)) {
        _bskError(configData->bskLogger, "Error: simpleInstrumentController duration settings must be finite.");
    }

    // reset the imaged variable to zero
    configData->imaged = 0;
    configData->constraintsActive = 0;
    configData->constraintStartTime = 0.0; // [ns]
    configData->elapsedTime = 0.0;         // [ns]
    configData->durationStarted = 0;
    configData->lastCallTime = callTime;    // [ns]
    configData->resetTime = callTime;       // [ns]
    configData->illuminationIntegral = 0.0; // [ns]
    configData->previousIllumination = 1.0; // [-]
    configData->captureIllumination = 1.0;  // [-]
    configData->captureTime = 0;            // [ns]
    DeviceCmdMsgPayload command = DeviceCmdMsg_C_zeroMsgPayload();
    DeviceCmdMsg_C_write(&command, &configData->deviceCmdOutMsg, moduleID, callTime);
}

/*! Evaluate acquisition constraints and issue at most one image command per attempt.

 @param configData The configuration data associated with the module
 @param callTime The clock time at which the function was called (nanoseconds)
 @param moduleID The module identifier
*/
void Update_simpleInstrumentController(simpleInstrumentControllerConfig* configData,
                                       uint64_t callTime,
                                       int64_t moduleID)
{
    double sigma_BR_norm; //!< Norm of sigma_BR
    double omega_BR_norm; //!< Norm of omega_BR

    /* Local copies of the msg buffers*/
    AccessMsgPayload accessInMsgBuffer;             //!< local copy of input message buffer
    AttGuidMsgPayload attGuidInMsgBuffer;           //!< local copy of output message buffer
    DeviceStatusMsgPayload deviceStatusInMsgBuffer; //!< local copy of input message buffer
    DeviceCmdMsgPayload deviceCmdOutMsgBuffer;      //!< local copy of output message buffer

    // zero output buffer
    deviceCmdOutMsgBuffer = DeviceCmdMsg_C_zeroMsgPayload();

    // read in the input messages
    accessInMsgBuffer = AccessMsg_C_isWritten(&configData->locationAccessInMsg)
                            ? AccessMsg_C_read(&configData->locationAccessInMsg)
                            : AccessMsg_C_zeroMsgPayload();
    attGuidInMsgBuffer = AttGuidMsg_C_isWritten(&configData->attGuidInMsg)
                             ? AttGuidMsg_C_read(&configData->attGuidInMsg)
                             : AttGuidMsg_C_zeroMsgPayload();

    // read in the device cmd message if it is connected
    if (DeviceStatusMsg_C_isLinked(&configData->deviceStatusInMsg)) {
        deviceStatusInMsgBuffer = DeviceStatusMsg_C_read(&configData->deviceStatusInMsg);
        configData->controllerStatus = deviceStatusInMsgBuffer.deviceStatus;
    }

    // Compute the norms of the attitude and rate errors
    sigma_BR_norm = v3Norm(attGuidInMsgBuffer.sigma_BR);
    omega_BR_norm = v3Norm(attGuidInMsgBuffer.omega_BR_B);

    // If non-zero time tolerances are provided, require callTime to fall inside
    // [imagingTime - timeToleranceLower, imagingTime + timeToleranceUpper].
    int timeControl = 1;
    if (configData->timeToleranceLower > 0) {
        if (callTime < configData->imagingTime && configData->imagingTime - callTime > configData->timeToleranceLower) {
            timeControl = 0;
        }
    }
    if (configData->timeToleranceUpper > 0) {
        if (callTime > configData->imagingTime && callTime - configData->imagingTime > configData->timeToleranceUpper) {
            timeControl = 0;
        }
    }
    unsigned int captureReady = 1;
    if (DeviceStatusMsg_C_isLinked(&configData->captureReadyInMsg)) {
        captureReady = DeviceStatusMsg_C_isWritten(&configData->captureReadyInMsg) &&
                       DeviceStatusMsg_C_read(&configData->captureReadyInMsg).deviceStatus;
    }
    double illumination = 1.0; // [-]
    if (EclipseMsg_C_isLinked(&configData->illuminationInMsg) &&
        EclipseMsg_C_isWritten(&configData->illuminationInMsg)) {
        illumination = EclipseMsg_C_read(&configData->illuminationInMsg).illuminationFactor;
    }
    unsigned int fresh = 1;
    if (configData->requireFreshMessages) {
        uint64_t guidanceTime = AttGuidMsg_C_timeWritten(&configData->attGuidInMsg);     // [ns]
        uint64_t accessTime = AccessMsg_C_timeWritten(&configData->locationAccessInMsg); // [ns]
        fresh = AttGuidMsg_C_isWritten(&configData->attGuidInMsg) && guidanceTime > configData->resetTime &&
                guidanceTime <= callTime && callTime - guidanceTime <= configData->maximumGuidanceAge &&
                AccessMsg_C_isWritten(&configData->locationAccessInMsg) && accessTime <= callTime &&
                callTime - accessTime <= configData->maximumAccessAge;
        if (EclipseMsg_C_isLinked(&configData->illuminationInMsg)) {
            uint64_t illuminationTime = EclipseMsg_C_timeWritten(&configData->illuminationInMsg); // [ns]
            fresh = fresh && EclipseMsg_C_isWritten(&configData->illuminationInMsg) && illuminationTime <= callTime &&
                    callTime - illuminationTime <= configData->maximumAccessAge;
        }
    }
    unsigned int constraintsMet =
        configData->controllerStatus && fresh && (sigma_BR_norm <= configData->attErrTolerance) &&
        (!configData->useRateTolerance || omega_BR_norm <= configData->rateErrTolerance) &&
        accessInMsgBuffer.hasAccess && isfinite(illumination) &&
        (!configData->useIlluminationConstraint ||
         (EclipseMsg_C_isWritten(&configData->illuminationInMsg) && illumination >= configData->illuminationThreshold));

    if (!configData->imaged && constraintsMet) {
        if (!configData->useDurationImaging) {
            if (timeControl && captureReady) {
                deviceCmdOutMsgBuffer.deviceCmd = 1;
                configData->imaged = 1;
                configData->captureTime = callTime;             // [ns]
                configData->captureIllumination = illumination; // [-]
            }
        } else {
            if (configData->acquisitionTime < 0.0) {
                configData->acquisitionTime = 0.0; // [ns]
                _bskLog(configData->bskLogger,
                        BSK_WARNING,
                        "simpleInstrumentController: acquisitionTime is negative and has been set to zero.");
            }
            if (configData->allowedTime < 0.0) {
                configData->allowedTime = 0.0; // [ns]
                _bskLog(configData->bskLogger,
                        BSK_WARNING,
                        "simpleInstrumentController: allowedTime is negative and has been set to zero.");
            }
            if (!configData->durationStarted) {
                configData->durationStarted = 1;
                configData->constraintStartTime = (double)callTime; // [ns]
                configData->elapsedTime = 0.0;                      // [ns]
                configData->illuminationIntegral = 0.0;             // [ns]
            } else if (configData->constraintsActive && callTime >= configData->lastCallTime) {
                double interval = (double)(callTime - configData->lastCallTime); // [ns]
                configData->elapsedTime += interval;
                configData->illuminationIntegral += 0.5 * (configData->previousIllumination + illumination) * interval;
            }
            configData->constraintsActive = 1;
            configData->previousIllumination = illumination; // [-]
            if (configData->elapsedTime >= configData->acquisitionTime &&
                configData->acquisitionTime <= configData->allowedTime &&
                configData->elapsedTime <= configData->allowedTime && timeControl && captureReady) {
                deviceCmdOutMsgBuffer.deviceCmd = 1;
                configData->imaged = 1;
                configData->captureTime = callTime; // [ns]
                configData->captureIllumination = configData->elapsedTime > 0.0
                                                      ? configData->illuminationIntegral / configData->elapsedTime
                                                      : illumination; // [-]
                configData->constraintsActive = 0;
                configData->durationStarted = 0;
            } else if (configData->elapsedTime >= configData->allowedTime) {
                configData->elapsedTime = configData->allowedTime; // [ns]
                configData->controllerStatus = 0;
                configData->constraintsActive = 0;
                configData->durationStarted = 0;
            }
        }
    } else if (!configData->imaged) {
        configData->constraintsActive = 0;
        if (!configData->useCumulativeHold && (configData->controllerStatus || configData->durationStarted)) {
            configData->durationStarted = 0;
            configData->elapsedTime = 0.0;          // [ns]
            configData->illuminationIntegral = 0.0; // [ns]
        }
    }
    configData->lastCallTime = callTime; // [ns]

    // write to the output messages
    DeviceCmdMsg_C_write(&deviceCmdOutMsgBuffer, &(configData->deviceCmdOutMsg), moduleID, callTime);

    return;
}
