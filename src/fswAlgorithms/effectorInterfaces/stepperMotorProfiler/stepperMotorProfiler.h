/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef _STEPPERMOTORPROFILER_
#define _STEPPERMOTORPROFILER_

#include <stdint.h>
#include <stdbool.h>
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/MotorStepCommandMsg_C.h"
#include "cMsgCInterface/StepperMotorMsg_C.h"
#include "cMsgCInterface/PrescribedMotionMsg_C.h"
#include "cMsgCInterface/HingedRigidBodyMsg_C.h"

/*! @brief Top level structure for the sub-module routines. */
typedef struct {
    /* User-configured parameters (required) */
    double rotAxis_M[3];                                   //!< Stepper motor rotation axis
    double thetaInit;                                      //!< [rad] Initial motor angle
    double stepAngle;                                      //!< [rad] Angle the stepper motor moves through for a single step
    double stepTime;                                       //!< [s] Time required for a single motor step (constant)
    double thetaDDotMax;                                   //!< [rad/s^2] Maximum angular acceleration of the stepper motor
    double r_FM_M[3];                                      //!< [m] Position of the F frame origin with respect to the M frame origin in M frame components (fixed)
    double rPrime_FM_M[3];                                 //!< [m/s] B frame time derivative of r_FM_M in M frame components (fixed)
    double rPrimePrime_FM_M[3];                            //!< [m/s^2] B frame time derivative of rPrime_FM_M in M frame components (fixed)

    /* Other prescribed parameters */
    double sigma_FM[3];                                    //!< MRP attitude of frame F with respect to frame M
    double omega_FM_F[3];                                  //!< [rad/s] Angular velocity of frame F wrt frame M in F frame components
    double omegaPrime_FM_F[3];                             //!< [rad/s^2] B frame time derivative of omega_FM_F in F frame components

    /* Step parameters */
    int stepsCommanded;                                    //!< [steps] Number of commanded steps
    int stepCount;                                         //!< [steps] Current motor step count (number of steps taken)

    /* Motor angle parameters */
    double maneuverThetaInit;                              //!< [rad] Initial motor angle
    double intermediateThetaInit;                          //!< [rad] Motor angle at the start of a new maneuver
    double theta;                                          //!< [rad] Current motor angle
    double thetaDotInit;                                   //!< [rad/s] Initial motor angle rate
    double thetaDot;                                       //!< [rad/s] Current motor angle rate
    double thetaDDot;                                      //!< [rad/s^2] Current motor angular acceleration
    double intermediateThetaRef;                           //!< [rad] Motor angle at the end of each step
    double thetaDotRef;                                    //!< [rad/s] Reference angle rate

    /* Temporal parameters */
    double tInit;                                          //!< [s] Simulation time at the beginning of the maneuver
    double previousWrittenTime;                            //!< [ns] Time the last input message was written
    double ts;                                             //!< [s] The simulation time halfway through the maneuver (switch time for ang accel)
    double tf;                                             //!< [s] Simulation time when the maneuver is finished

    /* Boolean parameters */
    bool completion;                                       //!< Boolean designating a fully completed maneuver
    bool stepComplete;                                     //!< Boolean designating a completed step
    bool newMsg;                                           //!< Boolean designating a new message was written

    /* Constant parameters */
    double a;                                              //!< Parabolic constant for the first half of a step
    double b;                                              //!< Parabolic constant for the second half of a step

    BSKLogger *bskLogger;                                  //!< BSK Logging

    /* Messages */
    MotorStepCommandMsg_C motorStepCommandInMsg;           //!< Input msg for the number of commanded motor step counts
    StepperMotorMsg_C stepperMotorOutMsg;                  //!< Output msg for the stepper motor information
    HingedRigidBodyMsg_C hingedRigidBodyOutMsg;            //!< Output msg for the spinning body module
    PrescribedMotionMsg_C prescribedMotionOutMsg;          //!< Output msg for the spinning body prescribed states

}StepperMotorProfilerConfig;

#ifdef __cplusplus
extern "C" {
#endif
    void SelfInit_stepperMotorProfiler(StepperMotorProfilerConfig *configData, int64_t moduleID);                     //!< Method for module initialization
    void Reset_stepperMotorProfiler(StepperMotorProfilerConfig *configData, uint64_t callTime, int64_t moduleID);     //!< Method for module reset
    void Update_stepperMotorProfiler(StepperMotorProfilerConfig *configData, uint64_t callTime, int64_t moduleID);    //!< Method for module time update
#ifdef __cplusplus
}
#endif

#endif
