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

#ifndef TWO_HINGE_DAMPER_H
#define TWO_HINGE_DAMPER_H

#include <stdint.h>

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/ScalarJointStateMsgPayload.h"
#include "architecture/msgPayloadDefC/SingleActuatorMsgPayload.h"
#include "architecture/utilities/bskLogging.h"

/**
 * @brief Generalized damping for a two-hinge spherical pendulum.
 *
 * For the rod direction obtained by a phi rotation followed by a theta
 * rotation, isotropic Cartesian damping at the bob gives
 *
 *     Q_phi   = -c cos(theta)^2 phiDot
 *     Q_theta = -c thetaDot.
 *
 * The output messages are intended for motors on the corresponding MuJoCo
 * joints, which apply each torque as an internal parent-child pair.
 */
class TwoHingeDamper : public SysModel {
public:
    /**
     * @brief Validate the input message links and the damping coefficient.
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    void Reset(uint64_t CurrentSimNanos) override;

    /**
     * @brief Write both hinge damping torques for the current joint state.
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    void UpdateState(uint64_t CurrentSimNanos) override;

    double dampingCoeff{0.0};  //!< [N*m*s] Cartesian damping coefficient times rod length squared

    ReadFunctor<ScalarJointStateMsgPayload> thetaInMsg;     //!< [rad] theta joint position
    ReadFunctor<ScalarJointStateMsgPayload> phiDotInMsg;    //!< [rad/s] phi joint rate
    ReadFunctor<ScalarJointStateMsgPayload> thetaDotInMsg;  //!< [rad/s] theta joint rate

    Message<SingleActuatorMsgPayload> phiTorqueOutMsg;    //!< [N*m] phi-joint damping torque
    Message<SingleActuatorMsgPayload> thetaTorqueOutMsg;  //!< [N*m] theta-joint damping torque

    BSKLogger bskLogger;  //!< module logger
};

#endif
