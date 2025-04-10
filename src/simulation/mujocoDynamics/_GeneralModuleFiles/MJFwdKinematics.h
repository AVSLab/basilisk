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

#ifndef MJ_FWD_KINEMATICS_H
#define MJ_FWD_KINEMATICS_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"

#include "MJScene.h"

/**
 * @brief Model that computes the forward kinematics of an `MJScene`
 *
 * The state of an `MJScene` is given by the minimal coordinates of the system
 * (i.e. joint angles). A 'forward kinematics' computation uses these
 * minimal coordinates to compute the position, velocity, attitude, and
 * angular velocity of all bodies, frames, etc. of the system.
 */
class MJFwdKinematics : public SysModel
{
public:
    /**
     * @brief Constructor for MJFwdKinematics.
     * @param scene Reference to an MJScene object.
     */
    MJFwdKinematics(MJScene& scene) : scene(scene) {}


    /**
     * @brief Computes the forward kinematics for the given scene.
     * @param scene Reference to an MJScene object.
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    static void fwdKinematics(MJScene& scene, uint64_t CurrentSimNanos);

    /**
     * @brief Computes the forward kinematics for the scene associated with this model.
     *
     * @param CurrentSimNanos Current simulation time in nanoseconds.
     */
    void UpdateState(uint64_t CurrentSimNanos) { fwdKinematics(this->scene, CurrentSimNanos); };

protected:
    MJScene& scene; ///< Reference to the MJScene object.
};

#endif
