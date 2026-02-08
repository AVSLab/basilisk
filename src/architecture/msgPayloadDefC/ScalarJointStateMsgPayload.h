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

#ifndef SCALAR_JOINT_STATE_MESSAGE_H
#define SCALAR_JOINT_STATE_MESSAGE_H

/** Represents the state of a scalar joint.
 *
 * This joint may be rotational or translational. For rotational joints,
 * the state is given in radians. For translational joints, it is given
 * in meters.
 *
 * This message can be used to represent the current state of a joint, or
 * to indicate the desired state when constraining the state of a joint.
 */
typedef struct {
    double state; //!< [rad] or [m] State of the joint
} ScalarJointStateMsgPayload;

#endif
