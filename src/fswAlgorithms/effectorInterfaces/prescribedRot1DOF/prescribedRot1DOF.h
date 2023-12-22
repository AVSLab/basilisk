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

#ifndef _PRESCRIBEDROT1DOF_
#define _PRESCRIBEDROT1DOF_

#include <stdint.h>
#include <stdbool.h>
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/HingedRigidBodyMsg_C.h"
#include "cMsgCInterface/PrescribedMotionMsg_C.h"

/*! @brief Top level structure for the sub-module routines. */
typedef struct {

    /* User-configurable module variables */
    bool coastOption;                                           //!< Boolean variable used for selecting an optional coast period during the rotation
    double tRamp;                                               //!< [s] Ramp time used for the coast option maneuver
    double thetaDDotMax;                                        //!< [rad/s^2] Maximum angular acceleration of the spinning body
    double rotAxis_M[3];                                        //!< Spinning body rotation axis expressed in M frame components
    double r_FM_M[3];                                           //!< [m] Spinning body position relative to the Mount frame expressed in M frame components (fixed)
    double rPrime_FM_M[3];                                      //!< [m/s] B frame time derivative of r_FM_M expressed in M frame components (fixed)
    double rPrimePrime_FM_M[3];                                 //!< [m/s^2] B frame time derivative of rPrime_FM_M expressed in M frame components (fixed)
    double omega_FM_F[3];                                       //!< [rad/s] Spinning body angular velocity relative to the Mount frame expressed in F frame components
    double omegaPrime_FM_F[3];                                  //!< [rad/s^2] B frame time derivative of omega_FM_F expressed in F frame components
    double sigma_FM[3];                                         //!< Spinning body MRP attitude with respect to frame M

    /* Coast option variables */
    double theta_tr;                                            //!< [rad] Angle at the end of the first ramp segment
    double theta_tc;                                            //!< [rad] Angle at the end of the coast segment
    double thetaDot_tr;                                         //!< [rad/s] Angle rate at the end of the first ramp segment
    double thetaDot_tc;                                         //!< [rad/s] Angle rate at the end of the coast segment
    double tr;                                                  //!< [s] The simulation time at the end of the first ramp segment
    double tc;                                                  //!< [s] The simulation time at the end of the coast period

    /* Non-coast option variables */
    double ts;                                                  //!< [s] The simulation time halfway through the rotation

    /* Shared module variables */
    bool convergence;                                           //!< Boolean variable is true when the rotation is complete
    double tInit;                                               //!< [s] Simulation time at the beginning of the rotation
    double thetaInit;                                           //!< [rad] Initial spinning body rotation angle from M to F frame about rotAxis_M
    double thetaDotInit;                                        //!< [rad/s] Initial spinning body angle rate
    double thetaRef;                                            //!< [rad] Reference angle from frame M to frame F about rotAxis_M
    double theta;                                               //!< [rad] Current angle
    double thetaDot;                                            //!< [rad/s] Current angle rate
    double thetaDDot;                                           //!< [rad/s^2] Current angular acceleration
    double tf;                                                  //!< [s] Simulation time when the rotation is finished
    double a;                                                   //!< Parabolic constant for the first half of the rotation
    double b;                                                   //!< Parabolic constant for the second half of the rotation
    BSKLogger *bskLogger;                                       //!< BSK Logging

    /* Messages */
    HingedRigidBodyMsg_C spinningBodyInMsg;                     //!< Input msg for the spinning body reference angle and angle rate
    HingedRigidBodyMsg_C spinningBodyOutMsg;                    //!< Output msg for the spinning body angle and angle rate
    PrescribedMotionMsg_C prescribedMotionOutMsg;               //!< Output msg for the spinning body prescribed states

}PrescribedRot1DOFConfig;

#ifdef __cplusplus
extern "C" {
#endif
    void SelfInit_prescribedRot1DOF(PrescribedRot1DOFConfig *configData, int64_t moduleID);                     //!< Method for module initialization
    void Reset_prescribedRot1DOF(PrescribedRot1DOFConfig *configData, uint64_t callTime, int64_t moduleID);     //!< Method for module reset
    void Update_prescribedRot1DOF(PrescribedRot1DOFConfig *configData, uint64_t callTime, int64_t moduleID);    //!< Method for module time update
#ifdef __cplusplus
}
#endif

#endif
