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

#ifndef _PRESCRIBEDTRANS_
#define _PRESCRIBEDTRANS_

#include <stdint.h>
#include <stdbool.h>
#include "architecture/utilities/bskLogging.h"
#include "cMsgCInterface/PrescribedMotionMsg_C.h"
#include "cMsgCInterface/PrescribedTransMsg_C.h"

/*! @brief Top level structure for the sub-module routines. */
typedef struct {

    /* User-configurable module variables */
    double transAccelMax;                                       //!< [m/s^2] Maximum translational acceleration
    double transAxis_M[3];                                      //!< Axis along the direction of translation expressed in M frame components
    double r_FM_M[3];                                           //!< [m] Translational body position relative to the Mount frame expressed in M frame components
    double rPrime_FM_M[3];                                      //!< [m/s] B frame time derivative of r_FM_M expressed in M frame components
    double rPrimePrime_FM_M[3];                                 //!< [m/s^2] B frame time derivative of rPrime_FM_M expressed in M frame components
    double omega_FM_F[3];                                       //!< [rad/s] Translational body angular velocity relative to the Mount frame expressed in F frame components (fixed)
    double omegaPrime_FM_F[3];                                  //!< [rad/s^2] B frame time derivative of omega_FM_F expressed in F frame components (fixed)
    double sigma_FM[3];                                         //!< Translational body MRP attitude with respect to frame M

    /* Private variables */
    bool convergence;                                           //!< Boolean variable is true when the rotation is complete
    double tInit;                                               //!< [s] Simulation time at the beginning of the translation
    double transPosInit;                                        //!< [m] Initial translational body position from M to F frame origin along transAxis_M
    double transVelInit;                                        //!< [m/s] Initial translational body velocity
    double transPosRef;                                         //!< [m] Reference translational body position from M to F frame origin along transAxis_M
    double transPos;                                            //!< [m] Current translational body position along transAxis_M
    double transVel;                                            //!< [m] Current translational body velocity along transAxis_M
    double transAccel;                                          //!< [m] Current translational body acceleration along transAxis_M
    double ts;                                                  //!< [s] The simulation time halfway through the translation
    double tf;                                                  //!< [s] The simulation time when the rotation is complete
    double a;                                                   //!< Parabolic constant for the first half of the translation
    double b;                                                   //!< Parabolic constant for the second half of the translation
    BSKLogger *bskLogger;                                       //!< BSK Logging

    /* Messages */
    PrescribedTransMsg_C prescribedTransInMsg;                  //!< Input msg for the translational reference position and velocity
    PrescribedMotionMsg_C prescribedMotionOutMsg;               //!< Output msg for the translational body prescribed states

}PrescribedTransConfig;

#ifdef __cplusplus
extern "C" {
#endif
    void SelfInit_prescribedTrans(PrescribedTransConfig *configData, int64_t moduleID);                     //!< Method for module initialization
    void Reset_prescribedTrans(PrescribedTransConfig *configData, uint64_t callTime, int64_t moduleID);     //!< Method for module reset
    void Update_prescribedTrans(PrescribedTransConfig *configData, uint64_t callTime, int64_t moduleID);    //!< Method for module time update
#ifdef __cplusplus
}
#endif

#endif
