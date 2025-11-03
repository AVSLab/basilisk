/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "simulation/mujocoDynamics/MJSystemMassMatrix/MJSystemMassMatrix.h"
#include "architecture/utilities/macroDefinitions.h"
#include <iostream>
#include <cstring>
#include <mujoco/mujoco.h>
#include <iomanip>

void MJSystemMassMatrix::Reset(uint64_t CurrentSimNanos)
{
    if (!scene) {
        bskLogger.bskLog(BSK_ERROR, "MJSystemMassMatrix: scene pointer not set!");
    }

    const mjModel* model = scene->getMujocoModel();
    if (!model) {
        bskLogger.bskLog(BSK_ERROR, "MJSystemMassMatrix: MuJoCo model not available in Reset()");
    }

    // extract the DOF dimensions
    this->nDOF = static_cast<std::size_t>(model->nv);

    // determine the joint types in the system
    this->jointTypes.clear();
    this->jointTypes.reserve(static_cast<std::size_t>(model->njnt));
    for (int j = 0; j < model->njnt; ++j) {
        const int jt = model->jnt_type[j];
        this->jointTypes.push_back(static_cast<std::size_t>(jt));
    }

    // determine the number of spacecraft in the system and store the starting joint index for each spacecraft
    // a new spacecraft is assumed to be added for each kinematic tree
    this->nSC = 0;
    this->scStartIdx.clear();
    for (int b = 1; b < model->nbody; ++b) {
        if (model->body_parentid[b] == 0) {
            const int jntCount = model->body_jntnum[b];
            if (jntCount > 0) {
                const int rootJ = model->body_jntadr[b];
                this->scStartIdx.push_back(static_cast<std::size_t>(rootJ));
                ++this->nSC;
            }
        }
    }


    // verify that all free joints are at the root of a kinematic tree
    for (int j = 0; j < model->njnt; ++j) {
        if (model->jnt_type[j] == mjJNT_FREE) {
            const int b = model->jnt_bodyid[j];

            const bool bodyIsTreeRoot = (model->body_parentid[b] == 0);
            const bool jointIsFirstOnBody = (j == model->body_jntadr[b]);

            if (!bodyIsTreeRoot || !jointIsFirstOnBody) {
                bskLogger.bskLog(BSK_ERROR,
                    "MJSystemMassMatrix: Free joint j=%d on body b=%d must be the first joint on a tree root body.", j, b);
            }
        }
    }
}

void MJSystemMassMatrix::UpdateState(uint64_t CurrentSimNanos)
{
    const mjModel* model = scene->getMujocoModel();
    const mjData*  data  = scene->getMujocoData();

    // always zero the output message buffers before assigning values
    MJSysMassMatrixMsgPayload payload = this->massMatrixOutMsg.zeroMsgPayload;

    // Build dense M matrix from MuJoCo
    const std::size_t NN = (this->nDOF) * this->nDOF;
    std::vector<mjtNum> Mdense(NN, mjtNum(0));
    mj_fullM(model, Mdense.data(), data->qM);


    // write to the output message
    payload.nSC = static_cast<int>(this->nSC);
    payload.scStartIdx.reserve(this->scStartIdx.size());
    for (auto idx : this->scStartIdx) {
        payload.scStartIdx.push_back(static_cast<int>(idx));
    }
    payload.jointTypes.reserve(this->jointTypes.size());
    for (auto jt : this->jointTypes) {
        payload.jointTypes.push_back(static_cast<int>(jt));
    }
    payload.massMatrix.assign(Mdense.begin(), Mdense.end());
    this->massMatrixOutMsg.write(&payload, this->moduleID, CurrentSimNanos);
}
