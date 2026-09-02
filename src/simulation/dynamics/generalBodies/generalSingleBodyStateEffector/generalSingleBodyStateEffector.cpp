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

const Eigen::Matrix<double, 3, 6> transMap =
        (Eigen::Matrix<double, 3, 6>() <<
                1, 0, 0,  0, 0, 0,
                0, 1, 0,  0, 0, 0,
                0, 0, 1,  0, 0, 0
        ).finished();

const Eigen::Matrix<double, 3, 6> rotMap =
        (Eigen::Matrix<double, 3, 6>() <<
                0, 0, 0,  1, 0, 0,
                0, 0, 0,  0, 1, 0,
                0, 0, 0,  0, 0, 1
        ).finished();

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
    this->inertialPositionProperty = statesIn.getPropertyReference(this->propName_inertialPosition);
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->propName_inertialVelocity);
}

void GeneralSingleBodyStateEffector::registerStates(DynParamManager& states) {
    this->betaState = states.registerState(this->numDOF, 1, this->nameOfBetaState);
    this->betaDotState = states.registerState(this->numDOF, 1, this->nameOfBetaDotState);

    Eigen::VectorXd betaInit = Eigen::VectorXd::Zero(this->numDOF);
    Eigen::VectorXd betaDotInit = Eigen::VectorXd::Zero(this->numDOF);
    for (auto& dof : this->jointDOFList) {
        betaInit(dof->index, 0) = dof->betaInit;
        betaDotInit(dof->index, 0) = dof->betaDotInit;
    }
    this->betaState->setState(betaInit);
    this->betaDotState->setState(betaDotInit);
}

void GeneralSingleBodyStateEffector::UpdateState(uint64_t currentSimNanos) {
    // Read input messages
    uint64_t rotIdx{};
    uint64_t transIdx{};
    for (auto& dof : this->jointDOFList) {
        if (dof->type == DOF::Type::ROTATION) {
            if (this->spinningBodyRefInMsg[rotIdx].isLinked()) {
                HingedRigidBodyMsgPayload spinningBodyBuffer = this->spinningBodyRefInMsg[rotIdx]();
                dof->betaRef = spinningBodyBuffer.theta;
                dof->betaDotRef = spinningBodyBuffer.thetaDot;
            }
            if (this->motorTorqueInMsg[rotIdx].isLinked()) {
                ArrayMotorTorqueMsgPayload motorTorqueBuffer = this->motorTorqueInMsg[rotIdx]();
                dof->u = motorTorqueBuffer.motorTorque[0];
            }
            rotIdx++;
        } else {
            if (this->translatingBodyRefInMsgs[transIdx].isLinked()) {
                LinearTranslationRigidBodyMsgPayload translatingBodyBuffer = this->translatingBodyRefInMsgs[transIdx]();
                dof->betaRef = translatingBodyBuffer.rho;
                dof->betaDotRef = translatingBodyBuffer.rhoDot;
            }
            if (this->motorForceInMsg[rotIdx].isLinked()) {
                ArrayMotorForceMsgPayload motorForceBuffer = this->motorForceInMsg[rotIdx]();
                dof->f = motorForceBuffer.motorForce[0];
            }
            transIdx++;
        }
    }

    this->computeGeneralBodyInertialStates();
    this->writeOutputStateMessages(currentSimNanos);
}

void GeneralSingleBodyStateEffector::computeGeneralBodyInertialStates() {
    // Inertial attitude
    Eigen::Matrix3d dcm_GB = this->jointDOFList.at(this->numDOF - 1)->dcm_GB;
    Eigen::Matrix3d dcm_GN = dcm_GB * this->dcm_BN;
    this->sigma_GN = eigenMRPd2Vector3d(eigenC2MRP(dcm_GN));

    // Inertial angular velocity
    Eigen::Vector3d omega_GB_B = rotMap * this->TMat * this->betaDot;
    Eigen::Vector3d omega_GN_B = omega_GB_B + this->omega_BN_B;
    this->omega_GN_G = dcm_GB * omega_GN_B;

    // Inertial position
    Eigen::Vector3d r_GcG_B = dcm_GB.transpose() * this->r_GcG_G;
    Eigen::Vector3d r_GcB_B = r_GcG_B + this->r_GB_B;
    this->r_GcN_N = this->dcm_BN.transpose() * r_GcB_B + (Eigen::Vector3d)*this->inertialPositionProperty;

    // Inertial velocity
    Eigen::Matrix3d omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    Eigen::Matrix3d rTilde_GcG_B = eigenTilde(r_GcG_B);
    Eigen::Vector3d rPrime_GcB_B = (transMap - rTilde_GcG_B * rotMap) * this->TMat * this->betaDot + transMap * this->TPrimeMat * this->beta;
    Eigen::Vector3d rDot_GcB_B = rPrime_GcB_B + omegaTilde_BN_B * r_GcB_B;
    this->v_GcN_N = this->dcm_BN.transpose() * rDot_GcB_B + (Eigen::Vector3d)*this->inertialVelocityProperty;
}

void GeneralSingleBodyStateEffector::writeOutputStateMessages(uint64_t currentSimNanos) {
    // Write the config log message if it is linked
    if (this->generalSingleBodyConfigLogOutMsg.isLinked()) {
        SCStatesMsgPayload configLogMsg = this->generalSingleBodyConfigLogOutMsg.zeroMsgPayload;
        eigenVector3d2CArray(this->r_GcN_N, configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_GcN_N, configLogMsg.v_BN_N);
        eigenMatrixXd2CArray(this->sigma_GN, configLogMsg.sigma_BN);
        eigenMatrixXd2CArray(this->omega_GN_G, configLogMsg.omega_BN_B);
        this->generalSingleBodyConfigLogOutMsg.write(&configLogMsg, this->moduleID, currentSimNanos);
    }

    // Write the joint state messages
    uint64_t rotIdx{};
    uint64_t transIdx{};
    for (auto& dof : this->jointDOFList) {
        if (dof->type == DOF::Type::ROTATION) {
            if (this->spinningBodyOutMsgs[rotIdx]->isLinked()) {
                HingedRigidBodyMsgPayload spinningBodyBuffer = this->spinningBodyOutMsgs[rotIdx]->zeroMsgPayload;
                spinningBodyBuffer.theta = dof->beta;
                spinningBodyBuffer.thetaDot = dof->betaDot;
                this->spinningBodyOutMsgs[rotIdx]->write(&spinningBodyBuffer, this->moduleID, currentSimNanos);
            }
            rotIdx++;
        } else {
            if (this->translatingBodyOutMsgs[transIdx]->isLinked()) {
                LinearTranslationRigidBodyMsgPayload translatingBodyBuffer = this->translatingBodyOutMsgs[transIdx]->zeroMsgPayload;
                translatingBodyBuffer.rho = dof->beta;
                translatingBodyBuffer.rhoDot = dof->betaDot;
                this->translatingBodyOutMsgs[transIdx]->write(&translatingBodyBuffer, this->moduleID, currentSimNanos);
            }
            transIdx++;
        }
    }
}

void GeneralSingleBodyStateEffector::updateEffectorMassProps(double integTime) {
    // Set effProps.mEff
    this->effProps.mEff = this->mass;

    // Get current state
    this->beta = this->betaState->getState();
    this->betaDot = this->betaDotState->getState();

    // Set beta and betaDot for each DOF
    for (auto& dof : this->jointDOFList) {
        dof->beta = this->beta[dof->index];
        dof->betaDot = this->betaDot[dof->index];
    }

    // Compute general body attitudes
    for (auto& dof : this->jointDOFList) {
        Eigen::Vector3d prv_GG0{Eigen::Vector3d::Zero()};
        if (dof->type == DOF::Type::ROTATION) {
            prv_GG0 = dof->beta * dof->axis_G;
        } else {
            prv_GG0 = dof->screwConstant * dof->beta * dof->axis_G;
        }
        double prv_GG0_array[3];
        eigenVector3d2CArray(prv_GG0, prv_GG0_array);

        double dcm_GG0_array[3][3];
        PRV2C(prv_GG0_array, dcm_GG0_array);
        Eigen::Matrix3d dcm_GG0 = c2DArray2EigenMatrix3d(dcm_GG0_array);

        if (dof->index == 0) {
            dof->dcm_GB = dcm_GG0 * this->dcm_G0B;
        } else {
            dof->dcm_GB = dcm_GG0 * this->jointDOFList.at(dof->index - 1)->dcm_GB;
        }
    }

    // Compute joint and general body position vectors relative to hub frame
    for (auto& dof : this->jointDOFList) {
        Eigen::Vector3d r_GG0_G{Eigen::Vector3d::Zero()};
        if (dof->type == DOF::Type::TRANSLATION) {
            r_GG0_G = dof->beta * dof->axis_G;
        } else {
            r_GG0_G = dof->screwConstant * dof->beta * dof->axis_G;
        }
        Eigen::Vector3d r_GG0_B = dof->dcm_GB.transpose() * r_GG0_G;

        if (dof->index == 0) {
            dof->r_GB_B = r_GG0_B + this->r_G0B_B;
        } else {
            dof->r_GB_B = r_GG0_B + this->jointDOFList.at(dof->index - 1)->r_GB_B;
        }
    }

    // Compute joint and general body angular velocity vectors relative to the hub frame
    for (auto& dof : this->jointDOFList) {
        Eigen::Vector3d omega_GP_G{Eigen::Vector3d::Zero()};
        if (dof->type == DOF::Type::ROTATION) {
            omega_GP_G = dof->betaDot * dof->axis_G;
        } else {
            omega_GP_G = dof->screwConstant * dof->betaDot * dof->axis_G;
        }
        Eigen::Vector3d omega_GP_B = dof->dcm_GB.transpose() * omega_GP_G;

        if (dof->index == 0) {
            dof->omega_GB_B = omega_GP_B;
        } else {
            dof->omega_GB_B = omega_GP_B + this->jointDOFList.at(dof->index - 1)->omega_GB_B;
        }
    }

    // Compute general body transformation matrix TMat and its first time derivative TMatPrime
    for (auto& dof : this->jointDOFList) {
        Eigen::Vector3d jointDOFAxis_B = dof->dcm_GB.transpose() * dof->axis_G;
        Eigen::Matrix3d omegaTilde_GB_B = eigenTilde(dof->omega_GB_B);

        Eigen::Vector3d omega_GPrevB_B = Eigen::Vector3d::Zero();
        if (dof->index != 0) {
            omega_GPrevB_B = this->jointDOFList.at(dof->index - 1)->omega_GB_B;
        }
        Eigen::Matrix3d omegaTilde_GPrevB_B = eigenTilde(omega_GPrevB_B);

        if (dof->type == DOF::Type::ROTATION) {
            this->TMat.col(dof->index).head<3>() = dof->screwConstant * jointDOFAxis_B;
            this->TMat.col(dof->index).tail<3>() = jointDOFAxis_B;
            this->TPrimeMat.col(dof->index).head<3>() = omegaTilde_GPrevB_B * dof->screwConstant * jointDOFAxis_B;
            this->TPrimeMat.col(dof->index).tail<3>() = omegaTilde_GPrevB_B * jointDOFAxis_B;
        } else {
            this->TMat.col(dof->index).head<3>() = jointDOFAxis_B;
            this->TMat.col(dof->index).tail<3>() = dof->screwConstant * jointDOFAxis_B;
            this->TPrimeMat.col(dof->index).head<3>() = omegaTilde_GPrevB_B * jointDOFAxis_B;
            this->TPrimeMat.col(dof->index).tail<3>() = omegaTilde_GPrevB_B * dof->screwConstant * jointDOFAxis_B;
        }
    }

    // Compute UMat matrix
    for (uint64_t colIdx{}; colIdx < this->numDOF; colIdx++) {
        this->UMat.col(colIdx).head<3>().setZero();
        this->UMat.col(colIdx).tail<3>().setZero();

        Eigen::VectorXd sumToN = Eigen::VectorXd::Zero(6);
        for (uint64_t idx = colIdx + 1; idx < this->numDOF; idx++) {
            sumToN += this->TMat.col(idx) * this->jointDOFList.at(idx)->beta;
        }

        Eigen::Vector3d dofRotAxis = rotMap * this->TMat.col(colIdx);
        Eigen::Matrix3d dofRotAxisTilde = eigenTilde(dofRotAxis);
        Eigen::Matrix<double, 6, 6> tildeSixMatrix = Eigen::Matrix<double, 6, 6>::Zero();
        tildeSixMatrix.topLeftCorner<3, 3>() = dofRotAxisTilde;
        tildeSixMatrix.bottomRightCorner<3, 3>() = dofRotAxisTilde;

        this->UMat.col(colIdx) = tildeSixMatrix * sumToN;
    }

    // Compute vVec vector
    this->vVec.setZero();
    Eigen::VectorXd hVecTerm1{Eigen::VectorXd::Zero(6)};
    Eigen::VectorXd hVecTerm2{Eigen::VectorXd::Zero(6)};
    for (uint64_t idx1{}; idx1 < this->numDOF; idx1++) {
        Eigen::VectorXd sumTerm1{Eigen::VectorXd::Zero(6)};
        Eigen::VectorXd sumTerm2{Eigen::VectorXd::Zero(6)};

        for (uint64_t idx2{}; idx2 < idx1; idx2++) {
            Eigen::VectorXd sumTerm1A{Eigen::VectorXd::Zero(6)};
            for (uint64_t idx3{}; idx3 < idx2; idx3++) {
                sumTerm1A += this->TMat.col(idx3) * this->jointDOFList.at(idx3)->betaDot;
            }

            Eigen::Matrix<double, 6, 6> tildeSixMatrix{Eigen::Matrix<double, 6, 6>::Zero()};
            tildeSixMatrix.topLeftCorner<3, 3>() = eigenTilde(rotMap * sumTerm1A);
            tildeSixMatrix.bottomRightCorner<3, 3>() = tildeSixMatrix.topLeftCorner<3, 3>();
            sumTerm1 += tildeSixMatrix * this->TMat.col(idx2) * this->jointDOFList.at(idx2)->betaDot;

            sumTerm2 += this->TMat.col(idx2) * this->jointDOFList.at(idx2)->betaDot;
        }

        Eigen::Matrix<double, 6, 6> tildeSixMatrix{Eigen::Matrix<double, 6, 6>::Zero()};
        tildeSixMatrix.topLeftCorner<3, 3>() = eigenTilde(rotMap * sumTerm1);
        tildeSixMatrix.bottomRightCorner<3, 3>() = tildeSixMatrix.topLeftCorner<3, 3>();
        hVecTerm1 = tildeSixMatrix * this->TMat.col(idx1) * this->jointDOFList.at(idx1)->beta;

        tildeSixMatrix = Eigen::Matrix<double, 6, 6>::Zero();
        tildeSixMatrix.topLeftCorner<3, 3>() = eigenTilde(rotMap * sumTerm2) * eigenTilde(rotMap * sumTerm2);
        tildeSixMatrix.bottomRightCorner<3, 3>() = tildeSixMatrix.topLeftCorner<3, 3>();
        hVecTerm2 = tildeSixMatrix * this->TMat.col(idx1) * this->jointDOFList.at(idx1)->beta;

        this->vVec += hVecTerm1 + hVecTerm2;
    }

    // Compute and set effProps.rEff_CB_B
    Eigen::Matrix3d dcm_GB = this->jointDOFList.at(this->numDOF - 1)->dcm_GB;
    Eigen::Vector3d r_GcG_B = dcm_GB.transpose() * this->r_GcG_G;
    this->r_GB_B = this->jointDOFList.at(this->numDOF - 1)->r_GB_B;
    Eigen::Vector3d r_GcB_B = r_GcG_B + this->r_GB_B;
    this->effProps.rEff_CB_B = r_GcB_B;

    // Compute and set effProps.IEffPntB_B
    Eigen::Matrix3d IPntGc_B = dcm_GB.transpose() * this->IPntGc_G * dcm_GB;
    Eigen::Matrix3d rTilde_GcB_B = eigenTilde(r_GcB_B);
    this->effProps.IEffPntB_B = IPntGc_B - this->mass * rTilde_GcB_B * rTilde_GcB_B;

    // Compute and set effProps.rEffPrime_CB_B
    Eigen::Matrix3d rTilde_GcG_B = eigenTilde(r_GcG_B);
    Eigen::Vector3d rPrime_GcB_B = (transMap - rTilde_GcG_B * rotMap) * this->TMat * this->betaDot + transMap * this->TPrimeMat * this->beta;
    this->effProps.rEffPrime_CB_B = rPrime_GcB_B;

    // Compute and set effProps.IEffPrimePntB_B
    Eigen::Vector3d omega_GB_B = rotMap * this->TMat * this->betaDot;
    Eigen::Matrix3d omegaTilde_GB_B = eigenTilde(omega_GB_B);
    Eigen::Matrix3d rPrimeTilde_GcB_B = eigenTilde(rPrime_GcB_B);
    this->effProps.IEffPrimePntB_B = omegaTilde_GB_B * IPntGc_B - IPntGc_B * omegaTilde_GB_B
            - this->mass * (rPrimeTilde_GcB_B * rTilde_GcB_B + rTilde_GcB_B * rPrimeTilde_GcB_B);
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
