/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#include "fswAlgorithms/effectorInterfaces/thrJointCompensation/thrJointCompensation.h"
#include <iostream>
#include <cstring>

void ThrJointCompensation::Reset(uint64_t CurrentSimNanos [[maybe_unused]])
{
    // check that required input messages are connected
    if (!this->armConfigInMsg.isLinked()) {
        bskLogger.bskError("ThrJointCompensation.armConfigInMsg was not linked.");
    }
    if (!this->massMatrixInMsg.isLinked()) {
        bskLogger.bskError("ThrJointCompensation.massMatrixInMsg was not linked.");
    }
    if (!this->reactionForcesInMsg.isLinked()) {
        bskLogger.bskError("ThrJointCompensation.reactionForcesInMsg was not linked.");
    }
    if (this->jointStatesInMsgs.empty()) {
        bskLogger.bskError("ThrJointCompensation.jointStatesInMsgs vector is empty.");
    } else {
        if (this->jointStatesInMsgs.size() != this->numHingedJoints) {
            bskLogger.bskError("ThrJointCompensation.jointStatesInMsgs size does not match numHingedJoints.");
        }
        for (std::size_t i = 0; i < this->jointStatesInMsgs.size(); ++i) {
            if (!this->jointStatesInMsgs[i].isLinked()) {
                bskLogger.bskError("ThrJointCompensation.jointStatesInMsgs[%zu] was not linked.", i);
            }
        }
    }
    if (this->thrForcesInMsgs.empty()) {
        bskLogger.bskError("ThrJointCompensation.thrForcesInMsgs vector is empty.");
    } else {
        if (this->thrForcesInMsgs.size() != this->numThrusters) {
            bskLogger.bskError("ThrJointCompensation.thrForcesInMsgs size does not match numThrusters.");
        }
        for (std::size_t i = 0; i < this->thrForcesInMsgs.size(); ++i) {
            if (!this->thrForcesInMsgs[i].isLinked()) {
                bskLogger.bskError("ThrJointCompensation.thrForcesInMsgs[%zu] was not linked.", i);
            }
        }
    }

    this->treeInfoInitialized = false;

    // load the static armConfig data
    THRArmConfigMsgPayload cfg = this->armConfigInMsg();
    this->kinCfg.thrArmIdx = cfg.thrArmIdx;
    this->kinCfg.thrArmJointIdx = cfg.thrArmJointIdx;
    this->kinCfg.armTreeIdx = cfg.armTreeIdx;
    this->kinCfg.armJointCount = cfg.armJointCount;
    this->kinCfg.r_CP_P = cfg.r_CP_P;
    this->kinCfg.r_TP_P = cfg.r_TP_P;
    this->kinCfg.shat_P = cfg.shat_P;
    this->kinCfg.fhat_P = cfg.fhat_P;
    this->kinCfg.dcm_C0P = cfg.dcm_C0P;

    // resize the joint pose struct
    size_t nFlat = 0;
    for (int cnt : this->kinCfg.armJointCount) {
        if (cnt < 0) {
            bskLogger.bskError("ThrJointCompensation: arm joint counts cannot be negative.");
        }
        nFlat += static_cast<size_t>(cnt);
    }
    this->jointPoseFlat.resize(nFlat);
    for (auto& jp : this->jointPoseFlat) {
        jp.dcm_CB.setIdentity();
        jp.r_CB_B.setZero();
    }
    this->kinCfg.armJointStart.assign(this->kinCfg.armJointCount.size(), 0);
    this->kinCfg.armHingeGlobalIdx.assign(nFlat, 0);
}

void ThrJointCompensation::UpdateState(uint64_t CurrentSimNanos)
{
    MJSysMassMatrixMsgPayload massMatrixIn = this->massMatrixInMsg();
    MJJointReactionsMsgPayload reactionForcesIn = this->reactionForcesInMsg();

    // Extract the information on the kinematic trees if not already done
    if (!this->treeInfoInitialized) {
        this->numKinematicTrees = 0;
        this->treeMap.clear();
        this->hingeToTree.assign(this->numHingedJoints, -1);
        size_t nextHingeIdx = 0;
        for (std::size_t joint = 0; joint < reactionForcesIn.jointTreeIdx.size(); ++joint) {
            const int tree = reactionForcesIn.jointTreeIdx[joint];
            const int jointType = reactionForcesIn.jointTypes[joint];

            TreeInfo& info = this->treeMap[tree];

            if (info.freeJointIdx < 0) {
                // first time seeing this tree
                ++this->numKinematicTrees;
                // check that first joint in tree is free
                if (jointType != 0) {
                    bskLogger.bskError("ThrJointCompensation: first joint in kinematic tree %d is not a free joint.", tree);
                }
                info.freeJointIdx = static_cast<int>(joint);
            } else {
                // subsequent joints must be hinges
                if (jointType != 3) {
                    bskLogger.bskError("ThrJointCompensation: joint %zu in kinematic tree %d is not a hinged joint.", joint, tree);
                }
                info.hingeJointIdxs.push_back(joint);
                info.hingeGlobalIdxs.push_back(nextHingeIdx);
                this->hingeToTree[nextHingeIdx] = tree;
                ++nextHingeIdx;
            }
        }
        if (this->numHingedJoints != nextHingeIdx) {
            bskLogger.bskError("ThrJointCompensation: numHingedJoints does not match the number of hinge joints found in the kinematic trees.");
        }

        if (this->kinCfg.armTreeIdx.size() != this->kinCfg.armJointCount.size()) {
            bskLogger.bskError("ThrJointCompensation: armTreeIdx size (%zu) does not match armJointCount size (%zu).",
                this->kinCfg.armTreeIdx.size(), this->kinCfg.armJointCount.size());
        }

        const size_t nArms = this->kinCfg.armJointCount.size();
        this->kinCfg.armJointStart.assign(nArms, 0);
        size_t nFlat = 0;
        for (size_t a = 0; a < nArms; ++a) {
            this->kinCfg.armJointStart[a] = nFlat;
            const int cnt = this->kinCfg.armJointCount[a];
            if (cnt < 0) {
                bskLogger.bskError("ThrJointCompensation: arm joint counts cannot be negative.");
            }
            nFlat += static_cast<size_t>(cnt);
        }

        this->kinCfg.armHingeGlobalIdx.assign(nFlat, 0);
        std::unordered_map<int, size_t> treeCursor;
        treeCursor.reserve(this->treeMap.size());
        for (const auto& [treeId, info] : this->treeMap) {
            treeCursor[treeId] = 0;
        }
        for (size_t a = 0; a < nArms; ++a) {
            const int treeId = this->kinCfg.armTreeIdx[a];
            const int cnt    = this->kinCfg.armJointCount[a];
            const size_t start = this->kinCfg.armJointStart[a];

            auto itTree = this->treeMap.find(treeId);
            if (itTree == this->treeMap.end()) {
                bskLogger.bskError("ThrJointCompensation: treeId %d for arm %zu was not found in joint reaction tree data.",
                    treeId, a);
            }
            const auto& hinges = itTree->second.hingeGlobalIdxs;
            size_t& cur = treeCursor[treeId];
            const size_t jointCount = static_cast<size_t>(cnt);
            if (cur + jointCount > hinges.size()) {
                bskLogger.bskError("ThrJointCompensation: arm %zu expects %d hinge joints in tree %d, but only %zu are available (cursor=%zu).",
                    a, cnt, treeId, hinges.size(), cur);
            }
            for (size_t j = 0; j < jointCount; ++j) {
                this->kinCfg.armHingeGlobalIdx[start + j] = hinges[cur++];
            }
        }

        this->treeInfoInitialized = true;
    }

    // Loop through the joint states input messages
    const Eigen::Index numHingedJointsEigen = static_cast<Eigen::Index>(this->numHingedJoints);
    Eigen::VectorXd jointStates(numHingedJointsEigen);
    jointStates.setZero();
    for (size_t i = 0; i < this->jointStatesInMsgs.size(); ++i) {
        ScalarJointStateMsgPayload jointStateIn = this->jointStatesInMsgs[i]();
        jointStates(static_cast<Eigen::Index>(i)) = jointStateIn.state;

    }

    // Loop through the thruster force input messages
    Eigen::VectorXd thrForces(static_cast<Eigen::Index>(this->numThrusters));
    thrForces.setZero();
    for (size_t i = 0; i < this->thrForcesInMsgs.size(); ++i) {
        SingleActuatorMsgPayload thrForceIn = this->thrForcesInMsgs[i]();
        thrForces(static_cast<Eigen::Index>(i)) = thrForceIn.input;
    }

    // Determine current system configuration
    const size_t nArms = this->kinCfg.armJointCount.size();
    for (size_t arm = 0; arm < nArms; ++arm) {

        const size_t start = this->kinCfg.armJointStart[arm];
        const size_t count = static_cast<size_t>(this->kinCfg.armJointCount[arm]);

        for (size_t jLocal = 0; jLocal < count; ++jLocal) {

            const size_t kFlat = start + jLocal;
            Eigen::Matrix3d dcm_PB;
            Eigen::Vector3d r_PB_B;
            if (jLocal == 0) {
                dcm_PB = Eigen::Matrix3d::Identity();
                r_PB_B = Eigen::Vector3d::Zero();
            } else {
                const size_t kFlatPrior = kFlat - 1;
                dcm_PB = this->jointPoseFlat[kFlatPrior].dcm_CB;
                r_PB_B = this->jointPoseFlat[kFlatPrior].r_CB_B;
            }

            Eigen::Vector3d r_CP_P(
                this->kinCfg.r_CP_P[3*kFlat + 0],
                this->kinCfg.r_CP_P[3*kFlat + 1],
                this->kinCfg.r_CP_P[3*kFlat + 2]
            );

            Eigen::Vector3d shat_P(
                this->kinCfg.shat_P[3*kFlat + 0],
                this->kinCfg.shat_P[3*kFlat + 1],
                this->kinCfg.shat_P[3*kFlat + 2]
            );

            Eigen::Matrix3d dcm_C0P;
            const double* p = &this->kinCfg.dcm_C0P[9*kFlat];
            dcm_C0P << p[0], p[3], p[6],
                    p[1], p[4], p[7],
                    p[2], p[5], p[8];

            const size_t hGlobal = this->kinCfg.armHingeGlobalIdx[kFlat];
            double Phi = jointStates(static_cast<Eigen::Index>(hGlobal));

            Eigen::Matrix3d sTilde;
            sTilde <<     0, -shat_P(2),  shat_P(1),
                shat_P(2),      0, -shat_P(0),
                -shat_P(1),  shat_P(0),     0;

            Eigen::Matrix3d dcm_CC0 =
                Eigen::Matrix3d::Identity() * std::cos(Phi)
                - std::sin(Phi) * sTilde
                + (1 - std::cos(Phi)) * (shat_P * shat_P.transpose());

            Eigen::Matrix3d dcm_CP = dcm_CC0 * dcm_C0P;

            Eigen::Vector3d r_CB_B = r_PB_B + dcm_PB.transpose() * r_CP_P;
            Eigen::Matrix3d dcm_CB =  dcm_CP * dcm_PB;

            this->jointPoseFlat[kFlat].dcm_CB = dcm_CB;
            this->jointPoseFlat[kFlat].r_CB_B = r_CB_B;
        }
    }

    // Determine the forces and torques resulting from the thrusters
    std::unordered_map<int, Eigen::Vector3d> treeForce_B;
    std::unordered_map<int, Eigen::Vector3d> treeTorque_B;
    for (const auto& [treeId, info] : this->treeMap) {
        treeForce_B[treeId]  = Eigen::Vector3d::Zero();
        treeTorque_B[treeId] = Eigen::Vector3d::Zero();
    }
    Eigen::VectorXd tauJoint = Eigen::VectorXd::Zero(numHingedJointsEigen);

    const size_t nThr = this->kinCfg.thrArmIdx.size();
    for (size_t i = 0; i < nThr; ++i){
        const int armValue = this->kinCfg.thrArmIdx[i];
        const int jointValue = this->kinCfg.thrArmJointIdx[i];
        if (armValue < 0 || jointValue < 0) {
            bskLogger.bskError("ThrJointCompensation: thruster arm and joint indices cannot be negative.");
        }
        const size_t arm = static_cast<size_t>(armValue);
        const size_t jLocal = static_cast<size_t>(jointValue);
        const size_t kFlat = this->kinCfg.armJointStart[arm] + jLocal;

        const Eigen::Matrix3d& dcm_CB = this->jointPoseFlat[kFlat].dcm_CB;
        const Eigen::Vector3d& r_CB_B = this->jointPoseFlat[kFlat].r_CB_B;

        Eigen::Vector3d fhat_P(
            this->kinCfg.fhat_P[3*i + 0],
            this->kinCfg.fhat_P[3*i + 1],
            this->kinCfg.fhat_P[3*i + 2]
        );
        const double Fcmd = thrForces(static_cast<Eigen::Index>(i));
        const Eigen::Vector3d f_B = dcm_CB.transpose() * (fhat_P * Fcmd);

        Eigen::Vector3d r_TP_P(
            this->kinCfg.r_TP_P[3*i + 0],
            this->kinCfg.r_TP_P[3*i + 1],
            this->kinCfg.r_TP_P[3*i + 2]
        );
        const Eigen::Vector3d r_TB_B = r_CB_B + dcm_CB.transpose() * r_TP_P;
        const size_t hAttachGlobal = this->kinCfg.armHingeGlobalIdx[kFlat];
        const int treeId = this->hingeToTree[hAttachGlobal];

        treeForce_B[treeId]  += f_B;
        treeTorque_B[treeId] += r_TB_B.cross(f_B);

        const size_t start = this->kinCfg.armJointStart[arm];
        for (size_t j = 0; j <= jLocal; ++j)
        {
            const size_t kJ = start + j;
            const size_t hGlobal = this->kinCfg.armHingeGlobalIdx[kJ];
            const Eigen::Matrix3d& dcm_JB = this->jointPoseFlat[kJ].dcm_CB;
            const Eigen::Vector3d& r_JB_B = this->jointPoseFlat[kJ].r_CB_B;

            Eigen::Vector3d shat_P(
                this->kinCfg.shat_P[3*kJ + 0],
                this->kinCfg.shat_P[3*kJ + 1],
                this->kinCfg.shat_P[3*kJ + 2]
            );

            const Eigen::Vector3d s_B = dcm_JB.transpose() * shat_P;
            const Eigen::Vector3d r_TJ_B = r_TB_B - r_JB_B;
            tauJoint(static_cast<Eigen::Index>(hGlobal)) += s_B.dot(r_TJ_B.cross(f_B));
        }
    }

    // Calculate the motor torque needed to prevent joint motion
    Eigen::VectorXd uH(numHingedJointsEigen);
    uH.setZero();
    const size_t nDOF = reactionForcesIn.biasForces.size();
    Eigen::VectorXd nonActuatorForces(static_cast<Eigen::Index>(nDOF));
    nonActuatorForces.setZero();
    for (size_t k = 0; k < nDOF; ++k) {
        nonActuatorForces(static_cast<Eigen::Index>(k)) = reactionForcesIn.passiveForces[k]
                                                        + reactionForcesIn.constraintForces[k]
                                                        + reactionForcesIn.appliedForces[k]
                                                        - reactionForcesIn.biasForces[k];
    }

    constexpr size_t nBaseDOF = 6;
    constexpr Eigen::Index nBaseDOFEigen = static_cast<Eigen::Index>(nBaseDOF);
    for (const auto& [treeId, info] : this->treeMap) {
        const int freeJointIdx = info.freeJointIdx;
        const auto& hingeJointIdxs = info.hingeJointIdxs;
        const auto& hingeGlobalIdxs = info.hingeGlobalIdxs;
        const size_t nHingeJoints = hingeJointIdxs.size();
        const Eigen::Index nHingeJointsEigen = static_cast<Eigen::Index>(nHingeJoints);

        if (nHingeJoints == 0) {
            continue; // no hinged joints in this tree
        }

        std::vector<size_t> dofIdx;
        dofIdx.reserve(nBaseDOF + nHingeJoints);

        const int freeStartValue = reactionForcesIn.jointDOFStart[static_cast<size_t>(freeJointIdx)];
        if (freeStartValue < 0) {
            bskLogger.bskError("ThrJointCompensation: free-joint degree-of-freedom index cannot be negative.");
        }
        const size_t freeStart = static_cast<size_t>(freeStartValue);
        for (size_t i = 0; i < nBaseDOF; ++i) {
            dofIdx.push_back(freeStart + i);
        }
        for (size_t mhJointIdx : hingeJointIdxs) {
            const int hingeStart = reactionForcesIn.jointDOFStart[mhJointIdx];
            if (hingeStart < 0) {
                bskLogger.bskError("ThrJointCompensation: hinge degree-of-freedom index cannot be negative.");
            }
            dofIdx.push_back(static_cast<size_t>(hingeStart)); // hinged joints have 1 DOF
        }

        const size_t nTreeDOF = dofIdx.size();
        const Eigen::Index nTreeDOFEigen = static_cast<Eigen::Index>(nTreeDOF);
        Eigen::MatrixXd Mfull(nTreeDOFEigen, nTreeDOFEigen);
        Mfull.setZero();
        const auto& massMatrixData = massMatrixIn.massMatrix;

        for (size_t r = 0; r < nTreeDOF; ++r) {
            const size_t rowIdx = dofIdx[r];
            for (size_t c = 0; c < nTreeDOF; ++c) {
                const size_t colIdx = dofIdx[c];
                Mfull(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
                    massMatrixData[rowIdx * nDOF + colIdx];
            }
        }

        Eigen::MatrixXd Mbase = Mfull.block(0, 0, nBaseDOFEigen, nBaseDOFEigen);
        Eigen::MatrixXd Mthb = Mfull.block(nBaseDOFEigen, 0, nHingeJointsEigen, nBaseDOFEigen);

        Eigen::VectorXd baseBias(nBaseDOFEigen);
        for (size_t i = 0; i < nBaseDOF; ++i) {
            baseBias(static_cast<Eigen::Index>(i)) =
                nonActuatorForces(static_cast<Eigen::Index>(dofIdx[i]));
        }

        Eigen::VectorXd baseThr(nBaseDOFEigen);
        baseThr.setZero();
        baseThr.segment<3>(0) = treeForce_B[treeId];
        baseThr.segment<3>(3) = treeTorque_B[treeId];
        Eigen::VectorXd baseAccel = Mbase.ldlt().solve(baseThr + baseBias);

        Eigen::VectorXd jointBias(nHingeJointsEigen);
        for (size_t i = 0; i < nHingeJoints; ++i) {
            jointBias(static_cast<Eigen::Index>(i)) =
                nonActuatorForces(static_cast<Eigen::Index>(dofIdx[nBaseDOF + i]));
        }

        Eigen::VectorXd tauThrTree(nHingeJointsEigen);
        tauThrTree.setZero();
        for (size_t i = 0; i < nHingeJoints; ++i) {
            const Eigen::Index hGlobal = static_cast<Eigen::Index>(hingeGlobalIdxs[i]);
            tauThrTree(static_cast<Eigen::Index>(i)) = tauJoint(hGlobal);
        }
        Eigen::VectorXd uH_tree = (Mthb * baseAccel) - jointBias - tauThrTree;

        for (size_t i = 0; i < nHingeJoints; ++i) {
            const Eigen::Index hGlobal = static_cast<Eigen::Index>(hingeGlobalIdxs[i]);
            uH(hGlobal) = uH_tree(static_cast<Eigen::Index>(i));
        }
    }

    // Apply torque limits if specified
    if (!this->uMax.empty()) {
        if (this->uMax.size() != this->numHingedJoints) {
            bskLogger.bskError("ThrJointCompensation: size of uMax does not match numHingedJoints.");
        }
        for (size_t i = 0; i < this->numHingedJoints; ++i) {
            const Eigen::Index eigenIndex = static_cast<Eigen::Index>(i);
            uH(eigenIndex) = std::max(-this->uMax[i], std::min(this->uMax[i], uH(eigenIndex)));
        }
    }

    // Write to the output messages
    for (size_t i = 0; i < this->numHingedJoints; ++i) {
        SingleActuatorMsgPayload motorTorquesOutMsg = this->motorTorquesOutMsgs[i]->zeroMsgPayload;
        motorTorquesOutMsg.input = uH(static_cast<Eigen::Index>(i));
        this->motorTorquesOutMsgs[i]->write(&motorTorquesOutMsg, this->moduleID, CurrentSimNanos);
    }
}

void ThrJointCompensation::setUMax(std::vector<double> var)
{
    this->uMax = var;
}

void ThrJointCompensation::addHingedJoint()
{
    // increase the number of hinged joints by 1
    this->numHingedJoints++;

    // add a new input message reader for the new hinged joint
    this->jointStatesInMsgs.push_back(ReadFunctor<ScalarJointStateMsgPayload>());

    // add a new output message for the new hinged joint
    this->motorTorquesOutMsgs.push_back(new Message<SingleActuatorMsgPayload>());
}

void ThrJointCompensation::addThruster()
{
    // increase the number of thrusters by 1
    this->numThrusters++;

    // add a new input message reader for the new thruster
    this->thrForcesInMsgs.push_back(ReadFunctor<SingleActuatorMsgPayload>());
}
