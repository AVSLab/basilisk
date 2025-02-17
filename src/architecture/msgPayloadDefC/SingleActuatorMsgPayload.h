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

#ifndef SINGLE_ACTUATOR_MESSAGE_H
#define SINGLE_ACTUATOR_MESSAGE_H

/** Represents the control input for a MuJoCo actuator that takes a
 * single input.
 *
 * This message is used to represent the control input value to a
 * MuJoCo actuator. For actuators applied to pin (1DOF revolute) joints,
 * this represents the torque magnitude applied on said joint. For
 * actuators applied to a slider (1DOF translational) joint, this represents
 * the force magnitude applied on the joint. For actuators applied at sites,
 * the actuator will apply a force and torque in the site reference frame,
 * where the force vector will be `input*gear[0:3]` and the torque vector
 * will be `input*gear[3:6]`, where `gear` is an attribute of the actuator.
 * Essentially, `gear` defines the direction of both the force and torque
 * and the `input` in this message defines the magnitude of both. Other actuators
 * in MuJoCo are supported and controled with this message.
 */
typedef struct {
    double input; //!< [-] Real valued input
} SingleActuatorMsgPayload;

#endif
