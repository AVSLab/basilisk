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

#include "MJFwdKinematics.h"

#include "architecture/utilities/macroDefinitions.h"

#include "MJScene.h"

#include <iostream>

void MJFwdKinematics::fwdKinematics(MJScene& scene, uint64_t CurrentSimNanos)
{
    if (!scene.areKinematicsStale()) return;

    mjModel* model = scene.getMujocoModel();
    mjData* data = scene.getMujocoData();

    // Compute the forward kinematics, which will update the
    // accelerations and actuator state derivatives
    // position-dependent
    mj_fwdPosition(model, data);
    mj_fwdVelocity(model, data);

    scene.writeFwdKinematicsMessages(CurrentSimNanos);
}
