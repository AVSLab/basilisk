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

#ifndef prescribedMotionSimMsg_h
#define prescribedMotionSimMsg_h


 /*! @brief Structure used to define the individual prescribed motion state effector data message*/
typedef struct {
    double r_PM_M[3];                          //!< [m] position vector from the M frame origin to the P frame origin in M frame components
    double rPrime_PM_M[3];                     //!< [m/s] B frame time derivative of r_PM_M
    double rPrimePrime_PM_M[3];                //!< [m/s^2] B frame time derivative of rPrime_PM_M
    double omega_PM_P[3];                      //!< [rad/s] Angular velocity of the P frame wrt the M frame in P frame components
    double omegaPrime_PM_P[3];                 //!< [rad/s^2] B frame time derivative of omega_PM_P
    double sigma_PM[3];                        //!< MRP attitude parameters for the P frame relative to the M frame
}PrescribedMotionMsgPayload;


#endif /* prescribedMotionSimMsg_h */
