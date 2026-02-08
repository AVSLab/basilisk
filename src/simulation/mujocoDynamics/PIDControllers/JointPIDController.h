/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef MUJOCO_JOINT_PID_CONTROLLER
#define MUJOCO_JOINT_PID_CONTROLLER

#include "architecture/msgPayloadDefC/ScalarJointStateMsgPayload.h"
#include "architecture/msgPayloadDefC/SingleActuatorMsgPayload.h"
#include "simulation/mujocoDynamics/_GeneralModuleFiles/PIDController.h"

/**
 * @brief PID controller for scalar joints (rotational or translational).
 *
 * Implements the required virtual methods for reading joint state and writing actuator input.
 */
class JointPIDController : public PIDController<ScalarJointStateMsgPayload, ScalarJointStateMsgPayload, SingleActuatorMsgPayload>
{
protected:
    /**
     * @brief Read the measured position from the joint state payload.
     * @param i Joint state message payload.
     * @return The joint position (radians or meters).
     */
    double readMeasuredPosition(const ScalarJointStateMsgPayload& i) const override
    {
        return i.state;
    }

    /**
     * @brief Read the measured velocity from the joint state payload.
     * @param i Joint state message payload.
     * @return The joint velocity (radians/sec or meters/sec).
     */
    double readMeasuredVelocity(const ScalarJointStateMsgPayload& i) const override
    {
        return i.state;
    }

    /**
     * @brief Write the computed actuator input to the output payload.
     * @param o Actuator message payload.
     * @param val Value to write.
     */
    void writeOutput(SingleActuatorMsgPayload& o, double val) override
    {
        o.input = val;
    }
};

#endif
