/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef CONSTRAINT_DYN_EFFECTOR_MSG_H
#define CONSTRAINT_DYN_EFFECTOR_MSG_H

/*! @brief Structure used to Constraint Dynamic Effector output message */
typedef struct {
    double Fc_N[3]; //!< [N] Constraint force applied in Inertial frame
    double L1_B1[3]; //!< [N.m] Constraint torque applied on s/c 1 in B1 frame
    double L2_B2[3]; //!< [N.m] Constraint torque applied on s/c 2 in B2 frame
    double psi_N[3]; //!< [m] Direction constraint violation
    double Fc_mag_filtered; //!< [N] Magnitude of filtered constraint force applied in Inertial frame
    double L1_mag_filtered; //!< [N.m] Magnitude of filtered constraint torque on s/c 1 applied in B1 frame
    double L2_mag_filtered; //!< [N.m] Magnitude of filtered constraint torque on s/c 2 applied in B2 frame
}ConstDynEffectorMsgPayload;

#endif
