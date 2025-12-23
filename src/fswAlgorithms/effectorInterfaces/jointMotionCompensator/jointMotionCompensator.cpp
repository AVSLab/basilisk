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


#include "fswAlgorithms/effectorInterfaces/jointMotionCompensator/jointMotionCompensator.h"
#include <iostream>
#include <cstring>

void JointMotionCompensator::Reset(uint64_t CurrentSimNanos)
{
    // check that required input messages are connected
    if (!this->massMatrixInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "JointMotionCompensator.massMatrixInMsg was not linked.");
    }
    if (!this->reactionForcesInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "JointMotionCompensator.reactionForcesInMsg was not linked.");
    }
    if (this->jointTorqueInMsgs.empty()) {
        bskLogger.bskLog(BSK_ERROR, "JointMotionCompensator.jointTorqueInMsgs vector is empty.");
    } else {
        for (size_t i = 0; i < this->jointTorqueInMsgs.size(); ++i) {
            if (!this->jointTorqueInMsgs[i].isLinked()) {
                bskLogger.bskLog(BSK_ERROR, "JointMotionCompensator.jointTorqueInMsgs[%zu] was not linked.", i);
            }
        }
    }

    this->treeInfoInitialized = false;
}

void JointMotionCompensator::UpdateState(uint64_t CurrentSimNanos)
{
    MJSysMassMatrixMsgPayload massMatrixIn = this->massMatrixInMsg();
    MJJointReactionsMsgPayload reactionForcesIn = this->reactionForcesInMsg();

    // extract tree info if not done already
    if (!this->treeInfoInitialized) {
        int numKinematicTrees = 0;
        this->treeMap.clear();
        int nextHingeIdx = 0;

        for (std::size_t joint=0; joint<reactionForcesIn.jointTreeIdx.size(); ++joint) {
            const int tree      = reactionForcesIn.jointTreeIdx[joint];
            const int jointType = reactionForcesIn.jointTypes[joint];

            TreeInfo& info = this->treeMap[tree];

            if (info.freeJointIdx < 0) {
                ++numKinematicTrees;
                // check that first joint in tree is free
                if (jointType != 0) {
                    bskLogger.bskLog(BSK_ERROR, "JointMotionCompensator: first joint in kinematic tree %d is not a free joint.", tree);
                }
                info.freeJointIdx = static_cast<int>(joint);
            } else {
                // subsequent joints are hinges
                if (jointType != 3) {
                    bskLogger.bskLog(BSK_ERROR, "JointMotionCompensator: joint %zu in kinematic tree %d is not a hinged joint.", joint, tree);
                }
                info.hingeJointIdxs.push_back(static_cast<int>(joint));
                info.hingeGlobalIdxs.push_back(nextHingeIdx);
                ++nextHingeIdx;
            }
        }

        if (this->numHingedJoints != nextHingeIdx) {
            bskLogger.bskLog(BSK_ERROR, "JointMotionCompensator: numHingedJoints does not match the number of hinge joints found in the kinematic trees.");
        }
        if (this->numSpacecraft != numKinematicTrees) {
            bskLogger.bskLog(BSK_ERROR, "JointMotionCompensator: numSpacecraft does not match the number of kinematic trees found in the system.");
        }
        this->treeInfoInitialized = true;
    }

    // loop through the motor torque input messages and store the values
    Eigen::VectorXd jointTorques(this->numHingedJoints);
    jointTorques.setZero();
    for (size_t i=0; i<this->jointTorqueInMsgs.size(); ++i) {
        SingleActuatorMsgPayload jointTorqueIn = this->jointTorqueInMsgs[i]();
        jointTorques(i) = jointTorqueIn.input;
    }

    // compute the hub torques needed to negate the reaction torques induced by moving joints
    Eigen::VectorXd hubTorques(this->numSpacecraft * 3);
    hubTorques.setZero();
    const int nDOF = static_cast<int>(reactionForcesIn.biasForces.size());
    Eigen::VectorXd nonActuatorForces(nDOF);
    nonActuatorForces.setZero();
    for (int k = 0; k < nDOF; ++k) {
        nonActuatorForces(k) = reactionForcesIn.passiveForces[k] + reactionForcesIn.constraintForces[k] +
                               reactionForcesIn.appliedForces[k] - reactionForcesIn.biasForces[k];
    }

    for (const auto& [treeId, info] : this->treeMap) {
        const int scIdx = treeId;
        const int freeJointIdx = info.freeJointIdx;
        const auto& hingeJointIdxs = info.hingeJointIdxs;
        const auto& hingeGlobalIdxs = info.hingeGlobalIdxs;
        const int nHingeJoints = static_cast<int>(hingeJointIdxs.size());

        if (nHingeJoints == 0) {
            continue; // no hinged joints in this tree
        }

        std::vector<int> dofIdx;
        dofIdx.reserve(6 + nHingeJoints);

        const int freeStart = reactionForcesIn.jointDOFStart[freeJointIdx];

        for (int i = 0; i < 6; ++i) {
            dofIdx.push_back(freeStart + i);
        }
        for (int hJointIdx : hingeJointIdxs) {
            const int hingeStart = reactionForcesIn.jointDOFStart[hJointIdx];
            dofIdx.push_back(hingeStart); // hinge joint torque DOF
        }

        const int nTreeDOF = static_cast<int>(dofIdx.size());
        Eigen::MatrixXd Mfull(nTreeDOF, nTreeDOF);
        Mfull.setZero();
        const auto& massMatrixData = massMatrixIn.massMatrix;
        for (int r = 0; r < nTreeDOF; ++r) {
            const int rowIdx = dofIdx[r];
            for (int c = 0; c < nTreeDOF; ++c) {
                const int colIdx = dofIdx[c];
                Mfull(r, c) = massMatrixData[rowIdx * nDOF + colIdx];
            }
        }

        Eigen::Matrix3d Mtt  = Mfull.block<3,3>(0, 0);
        Eigen::Matrix3d Mrt  = Mfull.block<3,3>(3, 0);
        Eigen::MatrixXd Mtth = Mfull.block(0, 6, 3, nHingeJoints);
        Eigen::MatrixXd Mrth = Mfull.block(3, 6, 3, nHingeJoints);
        Eigen::MatrixXd Mtht = Mfull.block(6, 0, nHingeJoints, 3);
        Eigen::MatrixXd Mthth= Mfull.block(6, 6, nHingeJoints, nHingeJoints);

        // Build the non-actuator force vectors
        Eigen::Vector3d baseTransBias;
        baseTransBias.setZero();
        Eigen::Vector3d baseRotBias;
        baseRotBias.setZero();
        Eigen::VectorXd jointBias(nHingeJoints);
        jointBias.setZero();
        Eigen::VectorXd treeMotorTorque(nHingeJoints);
        treeMotorTorque.setZero();
        for (int i = 0; i < 3; ++i) {
            baseTransBias(i) = nonActuatorForces(dofIdx[i]);
            baseRotBias(i) = nonActuatorForces(dofIdx[3 + i]);
        }
        for (int i = 0; i < nHingeJoints; ++i) {
            jointBias(i) = nonActuatorForces(dofIdx[6 + i]);
            const int globalHingeIdx = hingeGlobalIdxs[i];
            treeMotorTorque(i) = jointTorques(globalHingeIdx);
        }

        // compute the hub torque
        Eigen::LDLT<Eigen::Matrix3d> ldltMtt(Mtt);
        Eigen::Vector3d temp1 = ldltMtt.solve(baseTransBias);
        Eigen::MatrixXd temp2 = ldltMtt.solve(Mtth);
        Eigen::MatrixXd temp3 = Mthth - Mtht*temp2;
        Eigen::VectorXd thetaDDot = temp3.ldlt().solve(jointBias + treeMotorTorque - Mtht*temp1);
        Eigen::Vector3d rDDot = temp1 - temp2*thetaDDot;
        Eigen::Vector3d treeHubTorque = -baseRotBias + Mrt*rDDot + Mrth*thetaDDot;

        // store the hub torque for this spacecraft
        hubTorques.segment<3>(scIdx * 3) = treeHubTorque;

    }

    // Apply saturation if uMax is set
    if (!this->uMax.empty()) {
        if (static_cast<int>(this->uMax.size()) != this->numSpacecraft * 3) {
            bskLogger.bskLog(BSK_ERROR, "JointMotionCompensator: size of uMax does not match 3 x number of spacecraft.");
        }
        for (int i = 0; i < this->numSpacecraft * 3; ++i) {
            hubTorques(i) = std::max(-this->uMax[i], std::min(this->uMax[i], hubTorques(i)));
        }
    }
    // write to the output messages
    for (int sc = 0; sc < this->numSpacecraft; ++sc) {
        for (int axis = 0; axis < 3; ++axis) {
            SingleActuatorMsgPayload hubTorqueOut;
            hubTorqueOut.input = hubTorques(sc * 3 + axis);
            this->hubTorqueOutMsgs[sc * 3 + axis]->write(&hubTorqueOut, this->moduleID, CurrentSimNanos);
        }
    }
}

void JointMotionCompensator::setUMax(std::vector<double> var)
{
    // Check each entry is non-negative
    for (size_t i = 0; i < var.size(); ++i) {
        if (var[i] < 0.0) {
            bskLogger.bskLog(BSK_ERROR, "JointMotionCompensator: uMax[%zu] is negative.", i);
        }
    }

    this->uMax = var;
}

void JointMotionCompensator::addSpacecraft()
{
    // increase the number of spacecraft by 1
    this->numSpacecraft ++;

    // add an output message for each principal axis of the new spacecraft
    this->hubTorqueOutMsgs.push_back(new Message<SingleActuatorMsgPayload>());
    this->hubTorqueOutMsgs.push_back(new Message<SingleActuatorMsgPayload>());
    this->hubTorqueOutMsgs.push_back(new Message<SingleActuatorMsgPayload>());
}

void JointMotionCompensator::addHingedJoint()
{
    // increase the number of hinged joints by 1
    this->numHingedJoints ++;

    // add an input message for the new hinged joint
    this->jointTorqueInMsgs.push_back(ReadFunctor<SingleActuatorMsgPayload>());
}
