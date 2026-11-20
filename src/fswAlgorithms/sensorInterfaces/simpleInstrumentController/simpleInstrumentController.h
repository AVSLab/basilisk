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

#ifndef _SIMPLE_INSTRUMENT_CONTROLLER_H_
#define _SIMPLE_INSTRUMENT_CONTROLLER_H_

#include "architecture/utilities/bskLogging.h"
#include <stdint.h>

#include "cMsgCInterface/AccessMsg_C.h"
#include "cMsgCInterface/AttGuidMsg_C.h"
#include "cMsgCInterface/DeviceCmdMsg_C.h"
#include "cMsgCInterface/DeviceStatusMsg_C.h"
#include "cMsgCInterface/EclipseMsg_C.h"

/*! @brief Pointing, access, illumination, and acquisition-duration instrument controller. */
typedef struct
{
    /* User configurable variables */
    double attErrTolerance;            //!< Normalized MRP attitude error tolerance
    unsigned int useRateTolerance;     //!< Flag to enable rate error tolerance
    double rateErrTolerance;           //!< Rate error tolerance in rad/s
    unsigned int imaged;               //!< Indicator for whether or not the image has already been captured
    unsigned int controllerStatus;     //!< dictates whether or not the controller should be running
    double constraintStartTime;        //!< time when constraints were first satisfied [nanoseconds]
    unsigned int useDurationImaging;   //!< flag to enable duration-based imaging
    unsigned int useCumulativeHold;    //!< Accumulate valid sampled intervals instead of requiring a continuous hold
    double acquisitionTime;            //!< (optional) Duration that constraints must be satisfied [nanoseconds]
    double allowedTime;                //!< (optional) Maximum allowed time for imaging [nanoseconds]
    double elapsedTime;                //!< Elapsed valid-constraint duration for duration-based imaging [nanoseconds]
    unsigned int constraintsActive;    //!< Whether the previous sample qualified, so an invalid gap is never counted
    unsigned int durationStarted;      //!< Whether this attempt has started, including a paused cumulative hold
    uint64_t lastCallTime;             //!< Previous update time, used to measure the actual interval length [ns]
    uint64_t resetTime;                //!< Time of the most recent reset [ns]
    unsigned int requireFreshMessages; //!< Require written guidance after reset and current access/illumination inputs
    uint64_t maximumGuidanceAge;       //!< Maximum guidance-message age when freshness checking is enabled [ns]
    uint64_t maximumAccessAge; //!< Maximum access/illumination-message age when freshness checking is enabled [ns]
    unsigned int useIlluminationConstraint; //!< Require target illumination at or above illuminationThreshold
    double illuminationThreshold;           //!< Minimum illuminated fraction in [0, 1] [-]
    double illuminationIntegral;            //!< Sum of mean interval illumination times valid interval length [ns]
    double previousIllumination;            //!< Previous valid endpoint for trapezoidal interval averaging [-]
    double
        captureIllumination; //!< Frozen mean illumination when the image is commanded; instantaneous for zero hold [-]
    uint64_t captureTime;    //!< Time of the last image command [ns]
    uint64_t timeToleranceLower; //!< Optional lower time tolerance for imaging [nanoseconds]; zero disables lower bound
    uint64_t timeToleranceUpper; //!< Optional upper time tolerance for imaging [nanoseconds]; zero disables upper bound
    uint64_t imagingTime;        //!< Desired imaging time [nanoseconds]

    /* declare module IO interfaces */
    AccessMsg_C locationAccessInMsg;     //!< Ground or spacecraft target access input message
    AttGuidMsg_C attGuidInMsg;           //!< attitude guidance input message
    DeviceStatusMsg_C deviceStatusInMsg; //!< (optional) device status input message
    DeviceStatusMsg_C captureReadyInMsg; //!< Optional capture readiness, e.g. available storage; does not reset a hold
    EclipseMsg_C
        illuminationInMsg; //!< Optional target illumination input; required when illumination checking is enabled
    DeviceCmdMsg_C deviceCmdOutMsg; //!< device status output message

    BSKLogger* bskLogger; //!< BSK Logging
} simpleInstrumentControllerConfig;

#ifdef __cplusplus
extern "C"
{
#endif

    void SelfInit_simpleInstrumentController(simpleInstrumentControllerConfig* configData, int64_t moduleID);
    void Update_simpleInstrumentController(simpleInstrumentControllerConfig* configData,
                                           uint64_t callTime,
                                           int64_t moduleID);
    void Reset_simpleInstrumentController(simpleInstrumentControllerConfig* configData,
                                          uint64_t callTime,
                                          int64_t moduleID);

#ifdef __cplusplus
}
#endif

#endif //_SIMPLE_INSTRUMENT_CONTROLLER_H_
