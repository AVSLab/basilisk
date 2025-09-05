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

#ifndef _AL_MEAN_OE_FEEDBACK_H_
#define _AL_MEAN_OE_FEEDBACK_H_

#include <stdint.h>

#include "cMsgCInterface/CmdForceInertialMsg_C.h"
#include "cMsgCInterface/EphemerisMsg_C.h"
#include "cMsgCInterface/NavTransMsg_C.h"
#include "cMsgCInterface/NavAttMsg_C.h"
#include "cMsgCInterface/AttRefMsg_C.h"

#include "architecture/utilities/bskLogging.h"

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    NavTransMsg_C chiefTransInMsg;      //!< chief orbit input message
    NavTransMsg_C deputyTransInMsg;     //!< deputy orbit input message
    NavAttMsg_C deputyAttNavInMsg;      //!< input msg measured attitude
    EphemerisMsg_C celBodyInMsg;


    CmdForceInertialMsg_C forceOutMsg;  //!< deputy control force output message
    AttRefMsg_C attRefOutMsg; 


    double K[36];               //!< Lyapunov Gain (6*6)
    double mu;                  //!< [m^3/s^2] gravitational constant
    double req;                 //!< [m] equatorial planet radius
    double J2;                  //!< [] J2 planet oblateness parameter

     double targetDiffOeMean[6];     //!< target mean orbital element difference
    uint8_t varying_target;         //!< if true, use formation guidance parameters to compute targetDiffOeMean on the fly

    double max_thrust;  //!< [N] maximum thrust magnitude

    // modification to include formation guidance 
    // if this is true, then formation guidance will be use instead of the target difference 
    double phase_angle;  // [rad], desired angular separation between the deputy and chief spacecraft along the reference orbit for formation guidance; used to maintain a specified phase offset in the relative motion
    double ring_height;  // [m], height of the formation ring above the reference plane
    double ring_diameter;  // [m]

    // thrust locations parameters
    double windowDeg;
    int enableNodeWindows;
    int enableApsesWindows;
    
    BSKLogger *bskLogger;       //!< BSK Logging
} attitudeMOEFeedbackConfig;

#ifdef __cplusplus
extern "C" {
#endif
void SelfInit_attitudeMOEFeedback(attitudeMOEFeedbackConfig *configData, int64_t moduleID);
void Update_attitudeMOEFeedback(attitudeMOEFeedbackConfig *configData, uint64_t callTime, int64_t moduleID);
void Reset_attitudeMOEFeedback(attitudeMOEFeedbackConfig *configData, uint64_t callTime, int64_t moduleID);
#ifdef __cplusplus
}
#endif

#endif
