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

#ifndef MJSCENE_STATE_MESSAGE_H
#define MJSCENE_STATE_MESSAGE_H

#include <Eigen/Dense>

/** Use to represent the state of an MJScene in generalized coordinates */
typedef struct
//@cond DOXYGEN_IGNORE
MJSceneStateMsgPayload
//@endcond
{
    Eigen::VectorXd qpos; //!< [-] Position of the scene in generalized coordinates
    Eigen::VectorXd qvel; //!< [-] Velocity of the scene in generalized coordinates
    Eigen::VectorXd act;  //!< [-] State values of stateful actuators

} MJSceneStateMsgPayload;

#endif
