/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "generalSingleBodyStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <string>
#include <Eigen/Dense>

GeneralSingleBodyStateEffector::GeneralSingleBodyStateEffector() {
    this->nameOfBetaState = "generalBodyBeta" + std::to_string(effectorID);
    this->nameOfBetaDotState = "generalBodyBetaDot" + std::to_string(effectorID);
    effectorID++;
}

GeneralSingleBodyStateEffector::~GeneralSingleBodyStateEffector() {
    this->clearDOFs();
}

uint64_t GeneralSingleBodyStateEffector::effectorID = 1;

void GeneralSingleBodyStateEffector::Reset(uint64_t currentSimNanos) {
    if (!eigenIsValidInertiaMatrix(this->IPntGc_G)) {
        bskLogger.bskError("GeneralSingleBodyStateEffector: effector inertia tensor is invalid.");
    }
    if (!eigenIsRotationMatrix(this->dcm_G0B)) {
        bskLogger.bskError("GeneralSingleBodyStateEffector: effector hub-relative attitude DCM is invalid.");
    }
    for (const auto& dof : this->jointDOFList) {
        if (!eigenIsUnitVector(dof->axis_G)) {
            bskLogger.bskError("GeneralSingleBodyStateEffector: effector degree of freedom axis must be a unit vector.");
        }
    }
}

void GeneralSingleBodyStateEffector::linkInStates(DynParamManager& statesIn) {
}

void GeneralSingleBodyStateEffector::registerStates(DynParamManager& states) {
}

void GeneralSingleBodyStateEffector::UpdateState(uint64_t currentSimNanos) {
}

void GeneralSingleBodyStateEffector::computeGeneralBodyInertialStates() {
}

void GeneralSingleBodyStateEffector::writeOutputStateMessages(uint64_t currentSimNanos) {
}

void GeneralSingleBodyStateEffector::updateEffectorMassProps(double integTime) {
}

void GeneralSingleBodyStateEffector::updateContributions(double integTime,
                                                        BackSubMatrices & backSubContr,
                                                        Eigen::MRPd sigma_BN,
                                                        Eigen::Vector3d omega_BN_B,
                                                        Eigen::Vector3d g_N) {
}

void GeneralSingleBodyStateEffector::computeDerivatives(double integTime,
                                                       Eigen::Vector3d rDDot_BN_N,
                                                       Eigen::Vector3d omegaDot_BN_B,
                                                       Eigen::MRPd sigma_BN) {
}

void GeneralSingleBodyStateEffector::updateEnergyMomContributions(double integTime,
                                                                 Eigen::Vector3d & rotAngMomPntCContr_B,
                                                                 double & rotEnergyContr,
                                                                 Eigen::Vector3d omega_BN_B) {
}

void GeneralSingleBodyStateEffector::addRotDOF(std::shared_ptr<DOF> newDOF) {
    if (!newDOF) {
        bskLogger.bskError("A null DOF object was given. Provide a valid DOF object for each degree of freedom.");
    }
    if (this->isDOFAdded(newDOF)) {
        bskLogger.bskError("This DOF was already added. Create a separate DOF object for each degree of freedom.");
    }
    if (this->numRotDOF >= 3) {
        bskLogger.bskError("General body rotational degrees of freedom cannot exceed 3.");
    }

    this->numRotDOF++;
    newDOF->type = DOF::Type::ROTATION;
    this->addDOF(newDOF);

    // Add required message types
    this->spinningBodyRefInMsg.push_back(ReadFunctor<HingedRigidBodyMsgPayload>());
    this->motorTorqueInMsg.push_back(ReadFunctor<ArrayMotorTorqueMsgPayload>());
    this->spinningBodyOutMsgs.push_back(new Message<HingedRigidBodyMsgPayload>);
}

void GeneralSingleBodyStateEffector::addTransDOF(std::shared_ptr<DOF> newDOF) {
    if (!newDOF) {
        bskLogger.bskError("A null DOF object was given. Provide a valid DOF object for each degree of freedom.");
    }
    if (this->isDOFAdded(newDOF)) {
        bskLogger.bskError("This DOF was already added. Create a separate DOF object for each degree of freedom.");
    }
    if (this->numTransDOF >= 3) {
        bskLogger.bskError("General body translational degrees of freedom cannot exceed 3.");
    }

    this->numTransDOF++;
    newDOF->type = DOF::Type::TRANSLATION;
    this->addDOF(newDOF);

    // Add required message types
    this->translatingBodyRefInMsgs.push_back(ReadFunctor<LinearTranslationRigidBodyMsgPayload>());
    this->motorForceInMsg.push_back(ReadFunctor<ArrayMotorForceMsgPayload>());
    this->translatingBodyOutMsgs.push_back(new Message<LinearTranslationRigidBodyMsgPayload>);
}

void GeneralSingleBodyStateEffector::addDOF(std::shared_ptr<DOF> newDOF) {
    this->numDOF++;
    newDOF->index = this->numDOF - 1;
    this->jointDOFList.push_back(newDOF);

    // Resize required matrices
    this->TMat.conservativeResize(6, this->TMat.cols() + 1);
    this->TPrimeMat.conservativeResize(6, this->TPrimeMat.cols() + 1);
    this->UMat.conservativeResize(6, this->UMat.cols() + 1);
    this->ABeta.conservativeResize(this->ABeta.rows() + 1, 3);
    this->BBeta.conservativeResize(this->BBeta.rows() + 1, 3);
    this->CBeta.conservativeResize(this->CBeta.rows() + 1);
}

bool GeneralSingleBodyStateEffector::isDOFAdded(const std::shared_ptr<DOF>& newDOF) const {
    return std::find(this->jointDOFList.begin(), this->jointDOFList.end(), newDOF) != this->jointDOFList.end();
}

void GeneralSingleBodyStateEffector::clearDOFs() {
    this->numDOF = 0;
    this->numRotDOF = 0;
    this->numTransDOF = 0;
    this->jointDOFList.clear();
    this->betaInitList.clear();
    this->betaDotInitList.clear();

    this->spinningBodyRefInMsg.clear();
    this->motorTorqueInMsg.clear();
    this->translatingBodyRefInMsgs.clear();
    this->motorForceInMsg.clear();
    for (auto* rotMsg : this->spinningBodyOutMsgs) delete rotMsg;
    this->spinningBodyOutMsgs.clear();
    for (auto* transMsg : this->translatingBodyOutMsgs) delete transMsg;
    this->translatingBodyOutMsgs.clear();

    this->TMat.resize(6, 0);
    this->TPrimeMat.resize(6, 0);
    this->UMat.resize(6, 0);
    this->ABeta.resize(0, 3);
    this->BBeta.resize(0, 3);
    this->CBeta.resize(0);
}
