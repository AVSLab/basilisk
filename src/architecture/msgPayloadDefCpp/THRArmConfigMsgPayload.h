/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef THR_ARM_CONFIG_MESSAGE_H
#define THR_ARM_CONFIG_MESSAGE_H

#include <array>
#include <vector>

/*! @brief Structure used by the messaging system to communicate details about the spacecraft system configuration
for spacecraft with thruster arms.*/
typedef struct
//@cond DOXYGEN_IGNORE
THRArmConfigMsgPayload
//@endcond
{
    double hubMass;                       //!< [kg] mass of the spacecraft's hub
    std::vector<double> r_BcB_B;          //!< [m] position vector from the hub body frame origin to the hub center of mass (body frame)

    std::vector<int> bodyArmIdx;          //!< [-] index for which arm a body belongs to
    std::vector<int> bodyJointIdx;        //!< [-] local joint index of the closest parent joint for each body
    std::vector<double> bodyMass;         //!< [kg] mass of each arm body
    std::vector<double> r_LcP_P;          //!< [m] position vector from the parent joint to the body center of mass (parent joint's frame), column-major order

    std::vector<int> armJointCount;       //!< [-] list of the number of hinged joints in each thruster arm
    std::vector<double> r_CP_P;           //!< [m] position vector from the parent joint to the child joint(parent joint's frame), column-major order
    std::vector<double> shat_P;           //!< [-] spin axis unit vector of a child joint (parent joint's frame), column-major order
    std::vector<double> dcm_C0P;          //!< [-] direction cosine matrix from the child joint frame zero angle orientation to the parent joint frame, column-major order

    std::vector<int> armTreeIdx;          //!< [-] index for which kinematic tree each thruster arm belongs to
    std::vector<int> thrArmIdx;           //!< [-] index for which arm a thruster belongs to
    std::vector<int> thrArmJointIdx;      //!< [-] local joint index of the closest parent joint for each thruster
    std::vector<double> r_TP_P;           //!< [m] position vector from the closest parent joint to the thruster (parent joint's frame), column-major order
    std::vector<double> fhat_P;           //!< [-] force axis unit vector of a thruster (parent joint's frame), column-major order
} THRArmConfigMsgPayload;

#endif
