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


#include "fswAlgorithms/effectorInterfaces/hingedJointArrayMotor/hingedJointArrayMotor.h"
#include <iostream>
#include <cstring>
#include <algorithm>

void HingedJointArrayMotor::Reset(uint64_t CurrentSimNanos [[maybe_unused]])
{
    // check that required input messages are connected
    if (!this->massMatrixInMsg.isLinked()) {
        bskLogger.bskError("HingedJointArrayMotor.massMatrixInMsg was not linked.");
    }
    if (!this->reactionForcesInMsg.isLinked()) {
        bskLogger.bskError("HingedJointArrayMotor.reactionForcesInMsg was not linked.");
    }
    if (!this->desJointStatesInMsg.isLinked()) {
        bskLogger.bskError("HingedJointArrayMotor.desJointStatesInMsg was not linked.");
    }
    if (this->jointStatesInMsgs.empty()) {
        bskLogger.bskError("HingedJointArrayMotor.jointStatesInMsgs vector is empty.");
    } else {
        if (this->jointStatesInMsgs.size() != static_cast<std::size_t>(this->numHingedJoints)) {
            bskLogger.bskError("HingedJointArrayMotor.jointStatesInMsgs size does not match numHingedJoints.");
        }
        for (std::size_t i = 0; i < this->jointStatesInMsgs.size(); ++i) {
            if (!this->jointStatesInMsgs[i].isLinked()) {
                bskLogger.bskError("HingedJointArrayMotor.jointStatesInMsgs[%zu] was not linked.", i);
            }
        }
    }
    if (this->jointStateDotsInMsgs.empty()) {
        bskLogger.bskError("HingedJointArrayMotor.jointStateDotsInMsgs vector is empty.");
    } else {
        if (this->jointStateDotsInMsgs.size() != static_cast<std::size_t>(this->numHingedJoints)) {
            bskLogger.bskError("HingedJointArrayMotor.jointStateDotsInMsgs size does not match numHingedJoints.");
        }
        for (std::size_t i = 0; i < this->jointStateDotsInMsgs.size(); ++i) {
            if (!this->jointStateDotsInMsgs[i].isLinked()) {
                bskLogger.bskError("HingedJointArrayMotor.jointStateDotsInMsgs[%zu] was not linked.", i);
            }
        }
    }

    // Check that the gains have been set properly
    if (this->Ktheta.rows() != this->numHingedJoints || this->Ktheta.cols() != this->numHingedJoints) {
        bskLogger.bskError("HingedJointArrayMotor Ktheta gains matrix does not have dimensions of %d x %d.", this->numHingedJoints, this->numHingedJoints);
    }
    if (this->Ptheta.rows() != this->numHingedJoints || this->Ptheta.cols() != this->numHingedJoints) {
        bskLogger.bskError("HingedJointArrayMotor Ptheta gains matrix does not have dimensions of %d x %d.", this->numHingedJoints, this->numHingedJoints);
    }


    this->treeInfoInitialized = false;

}

void HingedJointArrayMotor::UpdateState(uint64_t CurrentSimNanos)
{
    MJSysMassMatrixMsgPayload massMatrixIn = this->massMatrixInMsg();
    MJJointReactionsMsgPayload reactionForcesIn = this->reactionForcesInMsg();
    JointArrayStateMsgPayload desJointStatesIn = this->desJointStatesInMsg();

    // Extract the information on the kinematic trees if not already done
    if (!this->treeInfoInitialized) {
        this->numKinematicTrees = 0;
        this->treeMap.clear();
        int nextHingeIdx = 0;
        for (std::size_t joint = 0; joint < reactionForcesIn.jointTreeIdx.size(); ++joint) {
            const int tree = reactionForcesIn.jointTreeIdx[joint];
            const int jointType = reactionForcesIn.jointTypes[joint];

            TreeInfo& info = this->treeMap[tree];

            if (info.freeJointIdx < 0) {
                // first time seeing this tree
                ++this->numKinematicTrees;
                // check that first joint in tree is free
                if (jointType != 0) {
                    bskLogger.bskError("HingedJointArrayMotor: first joint in kinematic tree %d is not a free joint.", tree);
                }
                info.freeJointIdx = static_cast<int>(joint);
            } else {
                // subsequent joints must be hinges
                if (jointType != 3) {
                    bskLogger.bskError("HingedJointArrayMotor: joint %zu in kinematic tree %d is not a hinged joint.", joint, tree);
                }
                info.hingeJointIdxs.push_back(static_cast<int>(joint));
                info.hingeGlobalIdxs.push_back(nextHingeIdx);
                ++nextHingeIdx;
            }
        }
        if (this->numHingedJoints != nextHingeIdx) {
            bskLogger.bskError("HingedJointArrayMotor: numHingedJoints does not match the number of hinge joints found in the kinematic trees.");
        }
        this->treeInfoInitialized = true;
    }

    // Loop through the joint states and joint state dot input messages
    Eigen::VectorXd jointStates(this->numHingedJoints);
    jointStates.setZero();
    Eigen::VectorXd jointStateDots(this->numHingedJoints);
    jointStateDots.setZero();
    for (size_t i = 0; i < this->jointStatesInMsgs.size(); ++i) {
        ScalarJointStateMsgPayload jointStateIn = this->jointStatesInMsgs[i]();
        ScalarJointStateMsgPayload jointStateDotIn = this->jointStateDotsInMsgs[i]();
        const Eigen::Index jointIndex = static_cast<Eigen::Index>(i);
        jointStates(jointIndex) = jointStateIn.state;
        jointStateDots(jointIndex) = jointStateDotIn.state;

    }

    // Compute the motor torques for each hinged joint
    Eigen::VectorXd uH(this->numHingedJoints);
    uH.setZero();
    const int nDOF = static_cast<int>(reactionForcesIn.biasForces.size());
    Eigen::VectorXd nonActuatorForces(nDOF);
    nonActuatorForces.setZero();
    for (int k = 0; k < nDOF; ++k) {
        const size_t dofIndex = static_cast<size_t>(k);
        nonActuatorForces(k) = reactionForcesIn.passiveForces[dofIndex]
                               + reactionForcesIn.constraintForces[dofIndex]
                               + reactionForcesIn.appliedForces[dofIndex]
                               - reactionForcesIn.biasForces[dofIndex];
    }

    constexpr int nTransDOF = 3; // number of translational DOFs for free joint base
    for (const auto& [treeId, info] : this->treeMap) {
        const int freeJointIdx = info.freeJointIdx;
        const auto& hingeJointIdxs = info.hingeJointIdxs;
        const auto& hingeGlobalIdxs = info.hingeGlobalIdxs;
        const int nHingeJoints = static_cast<int>(hingeJointIdxs.size());

        if (nHingeJoints == 0) {
            continue; // no hinged joints in this tree
        }

        // Extract the joint DOF indicies for this tree
        std::vector<int> dofIdx;
        dofIdx.reserve(static_cast<size_t>(nTransDOF + nHingeJoints));

        const int freeStart = reactionForcesIn.jointDOFStart[static_cast<size_t>(freeJointIdx)];
        for (int i = 0; i < nTransDOF; ++i) {
            dofIdx.push_back(freeStart + i);
        }

        for (int mhJointIdx : hingeJointIdxs) {
            const int hingeStart = reactionForcesIn.jointDOFStart[static_cast<size_t>(mhJointIdx)];
            dofIdx.push_back(hingeStart); // hinged joints have 1 DOF
        }

        // Build the mass matrix for this kinematic tree
        const int nTreeDOF = static_cast<int>(dofIdx.size());
        Eigen::MatrixXd Mfull(nTreeDOF, nTreeDOF);
        Mfull.setZero();
        const auto& massMatrixData = massMatrixIn.massMatrix;

        for (int r = 0; r < nTreeDOF; ++r) {
            const int rowIdx = dofIdx[static_cast<size_t>(r)];
            for (int c = 0; c < nTreeDOF; ++c) {
                const int colIdx = dofIdx[static_cast<size_t>(c)];
                Mfull(r, c) = massMatrixData[static_cast<size_t>(rowIdx * nDOF + colIdx)];
            }
        }

        // Extract the submatrices
        Eigen::Matrix3d Mtt = Mfull.block<3, 3>(0, 0);
        Eigen::Matrix3Xd Mtth = Mfull.block(0, nTransDOF, 3, nHingeJoints);
        Eigen::MatrixX3d Mtht = Mfull.block(nTransDOF, 0, nHingeJoints, 3);
        Eigen::MatrixXd Mthth = Mfull.block(nTransDOF, nTransDOF, nHingeJoints, nHingeJoints);

        // Build the non-actuator force vectors
        Eigen::Vector3d baseTransBias;
        baseTransBias.setZero();
        Eigen::VectorXd jointBias(nHingeJoints);
        jointBias.setZero();
        for (int i = 0; i < nTransDOF; ++i) {
            baseTransBias(i) = nonActuatorForces(dofIdx[static_cast<size_t>(i)]);
        }
        for (int i = 0; i < nHingeJoints; ++i) {
            jointBias(i) = nonActuatorForces(dofIdx[static_cast<size_t>(nTransDOF + i)]);
        }

        // Build the gain matrices for the tree
        Eigen::MatrixXd Ktheta_tree(nHingeJoints, nHingeJoints);
        Ktheta_tree.setZero();
        Eigen::MatrixXd Ptheta_tree(nHingeJoints, nHingeJoints);
        Ptheta_tree.setZero();
        for (int i = 0; i < nHingeJoints; ++i) {
            const int gi = hingeGlobalIdxs[static_cast<size_t>(i)];
            for (int j = 0; j < nHingeJoints; ++j) {
                const int gj = hingeGlobalIdxs[static_cast<size_t>(j)];
                Ktheta_tree(i, j) = this->Ktheta(gi, gj);
                Ptheta_tree(i, j) = this->Ptheta(gi, gj);
            }
        }

        // Compute desired joint accelerations for this tree
        Eigen::VectorXd theta_ddot_des(nHingeJoints);
        theta_ddot_des.setZero();
        Eigen::VectorXd theta(nHingeJoints), thetaDot(nHingeJoints), theta_des(nHingeJoints), thetaDot_des(nHingeJoints), thetaDDot_des(nHingeJoints);
        theta.setZero();
        thetaDot.setZero();
        theta_des.setZero();
        thetaDot_des.setZero();
        thetaDDot_des.setZero();
        for (int i = 0; i < nHingeJoints; ++i) {
            const int jointIdx = hingeGlobalIdxs[static_cast<size_t>(i)];
            const size_t jointPosition = static_cast<size_t>(jointIdx);
            theta(i) = jointStates[jointIdx];
            thetaDot(i) = jointStateDots[jointIdx];
            theta_des(i) = desJointStatesIn.states[jointPosition];
            thetaDot_des(i) = desJointStatesIn.stateDots[jointPosition];
            thetaDDot_des(i) = desJointStatesIn.stateDDots[jointPosition];
        }

        auto wrap = [](double a){ return std::atan2(std::sin(a), std::cos(a)); };
        Eigen::VectorXd e(nHingeJoints), eDot(nHingeJoints);
        e.setZero();
        eDot.setZero();
        for (int i = 0; i < nHingeJoints; ++i) {
            e(i) = wrap(theta(i) - theta_des(i));
            eDot(i) = thetaDot(i) - thetaDot_des(i);
        }
        theta_ddot_des = thetaDDot_des - Ktheta_tree * e - Ptheta_tree * eDot;

        // Use Schur complement to compute the motor torques
        Eigen::Vector3d rhs = -Mtth * theta_ddot_des + baseTransBias;
        Eigen::LDLT<Eigen::Matrix3d> ldlt(Mtt);
        Eigen::Vector3d lambda = ldlt.solve(rhs);
        Eigen::VectorXd uH_tree = (Mthth * theta_ddot_des) + (Mtht*lambda) - jointBias;

        for (int i = 0; i < nHingeJoints; ++i) {
            const int hingedIdx = hingeGlobalIdxs[static_cast<size_t>(i)];
            uH(hingedIdx) = uH_tree(i);
        }

    }

    // Apply torque limits if specified
    if (!this->uMax.empty()) {
        if (static_cast<int>(this->uMax.size()) != this->numHingedJoints) {
            bskLogger.bskError("HingedJointArrayMotor: size of uMax does not match numHingedJoints.");
        }
        for (int i = 0; i < this->numHingedJoints; ++i) {
            const size_t jointIndex = static_cast<size_t>(i);
            uH(i) = std::max(-this->uMax[jointIndex], std::min(this->uMax[jointIndex], uH(i)));
        }
    }

    // write to the output messages
    for (int i=0; i < this->numHingedJoints; ++i) {
        const size_t jointIndex = static_cast<size_t>(i);
        SingleActuatorMsgPayload motorTorquesOutMsg = this->motorTorquesOutMsgs[jointIndex]->zeroMsgPayload;
        motorTorquesOutMsg.input = uH(i);
        this->motorTorquesOutMsgs[jointIndex]->write(&motorTorquesOutMsg, this->moduleID, CurrentSimNanos);
    }
}

void HingedJointArrayMotor::setKtheta(std::vector<double> var)
{
    const std::size_t N = var.size();
    const int m = static_cast<int>(std::llround(std::sqrt(static_cast<double>(N))));

    using MatMap = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;
    MatMap Kmap(var.data(), m, m);

    this->Ktheta = Kmap;
}

void HingedJointArrayMotor::setPtheta(std::vector<double> var)
{
    const std::size_t N = var.size();
    const int m = static_cast<int>(std::llround(std::sqrt(static_cast<double>(N))));

    using MatMap = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;
    MatMap Pmap(var.data(), m, m);

    this->Ptheta = Pmap;
}

void HingedJointArrayMotor::setUMax(std::vector<double> var)
{
    this->uMax = var;
}

void HingedJointArrayMotor::addHingedJoint()
{
    // increase the number of hinged joints by 1
    this->numHingedJoints++;

    // add a new input message reader for the new hinged joint
    this->jointStatesInMsgs.push_back(ReadFunctor<ScalarJointStateMsgPayload>());
    this->jointStateDotsInMsgs.push_back(ReadFunctor<ScalarJointStateMsgPayload>());

    // add a new output message for the new hinged joint
    this->motorTorquesOutMsgs.push_back(new Message<SingleActuatorMsgPayload>());
}
