/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef MJ_NONACTUATOR_FORCES_MESSAGE_H
#define MJ_NONACTUATOR_FORCES_MESSAGE_H

#include "architecture/utilities/macroDefinitions.h"

/*! @brief This structure is used in the messaging system to communicate what the nonactuator forces for
the full spacecraft system are currently from Mujoco.*/
typedef struct {
    double baseTransForces[3];                               //!< [N] non-actuator translational forces on the base
    double baseRotForces[3];                                 //!< [Nm] non-actuator rotational forces on the base
    double jointForces[MAX_EFF_CNT];                         //!< [Varies] non-actuator forces on the joints
}MJNonActuatorForcesMsgPayload;


#endif
