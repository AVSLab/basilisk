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


#include "simulation/mujocoDynamics/MJJointReactionForces/MJJointReactionForces.h"
#include <iostream>
#include <cstring>
#include <mujoco/mujoco.h>
#include <iomanip>

namespace {
void fillForcePayload(std::vector<double>& forces,
                             const mjtNum* forceData,
                             std::size_t nDOF)
{
    forces.resize(nDOF);
    if constexpr (std::is_same_v<mjtNum, double>) {
        forces.assign(forceData, forceData + nDOF);
    } else {
        forces.resize(nDOF);
        std::transform(forceData, forceData + nDOF, forces.begin(),
                       [](mjtNum x) { return static_cast<double>(x); });
    }
}
}

void MJJointReactionForces::Reset(uint64_t CurrentSimNanos)
{
    if (!scene) {
        bskLogger.bskLog(BSK_ERROR, "MJJointReactionForces: scene pointer not set!");
    }

    const mjModel* model = scene->getMujocoModel();
    if (!model) {
        bskLogger.bskLog(BSK_ERROR, "MJJointReactionForces: MuJoCo model not available in Reset()");
    }

    // extract the DOF dimensions
    this->nDOF = static_cast<std::size_t>(model->nv);

    // extract the static joint information
    std::vector<int> bodyTreeIdx(model->nbody, -1);
    int nextTreeIdx = 0;
    for (int b = 1; b < model->nbody; ++b) {
        int root = b;
        while (root > 0 && model->body_parentid[root] != 0) {
            root = model->body_parentid[root];
        }
        if (bodyTreeIdx[root] == -1) {
            bodyTreeIdx[root] = nextTreeIdx;
            nextTreeIdx++;
        }

        bodyTreeIdx[b] = bodyTreeIdx[root];
    }

    this->jointTreeIdx.clear();
    this->jointParentBodyIdx.clear();
    this->jointTypes.clear();
    this->jointDOFStart.clear();
    for (int j = 0; j < model->njnt; ++j) {
        int parentBody = model->jnt_bodyid[j];
        this->jointParentBodyIdx.push_back(parentBody);

        int treeIdx = (parentBody>= 0) ? bodyTreeIdx[parentBody] : -1;
        this->jointTreeIdx.push_back(treeIdx);

        int jt = model->jnt_type[j];
        this->jointTypes.push_back(jt);

        int dofStart = model->jnt_dofadr[j];
        this->jointDOFStart.push_back(dofStart);
    }

    // verify that all free joints are at the root of a kinematic tree
    for (int j = 0; j < model->njnt; ++j) {
        if (model->jnt_type[j] == mjJNT_FREE) {
            const int b = model->jnt_bodyid[j];

            const bool bodyIsTreeRoot = (model->body_parentid[b] == 0);
            const bool jointIsFirstOnBody = (j == model->body_jntadr[b]);

            if (!bodyIsTreeRoot || !jointIsFirstOnBody) {
                bskLogger.bskLog(BSK_ERROR,
                    "MJJointReactionForces: Free joint j=%d on body b=%d must be the first joint on a tree root body.", j, b);
            }
        }
    }

}

void MJJointReactionForces::UpdateState(uint64_t CurrentSimNanos)
{
    const mjData* data = scene->getMujocoData();

    // always zero the output message buffers before assigning values
    MJJointReactionsMsgPayload reactionForcesOut = this->reactionForcesOutMsg.zeroMsgPayload;

    // populate the static msg fields
    reactionForcesOut.jointTreeIdx = this->jointTreeIdx;
    reactionForcesOut.jointParentBodyIdx = this->jointParentBodyIdx;
    reactionForcesOut.jointTypes = this->jointTypes;
    reactionForcesOut.jointDOFStart = this->jointDOFStart;

    // populate the reaction forces and torques on each joint DOF
    fillForcePayload(reactionForcesOut.biasForces, data->qfrc_bias, this->nDOF);
    fillForcePayload(reactionForcesOut.passiveForces, data->qfrc_passive, this->nDOF);
    if (data->nefc > 0) {
        fillForcePayload(reactionForcesOut.constraintForces, data->qfrc_constraint, this->nDOF);
    } else {
        reactionForcesOut.constraintForces.assign(this->nDOF, 0.0);
    }
    fillForcePayload(reactionForcesOut.appliedForces, data->qfrc_applied, this->nDOF);
    fillForcePayload(reactionForcesOut.actuatorForces, data->qfrc_actuator, this->nDOF);

    // write to the output messages
    this->reactionForcesOutMsg.write(&reactionForcesOut, this->moduleID, CurrentSimNanos);
}
