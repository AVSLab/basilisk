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
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

GeneralSingleBodyStateEffector::GeneralSingleBodyStateEffector()
{
    this->nameOfBetaState = "generalBodyBeta" + std::to_string(effectorID);
    this->nameOfBetaDotState = "generalBodyBetaDot" + std::to_string(effectorID);
    effectorID++;
}

GeneralSingleBodyStateEffector::~GeneralSingleBodyStateEffector()
{
    this->clearDOFs();
}

uint64_t GeneralSingleBodyStateEffector::effectorID = 1;

void GeneralSingleBodyStateEffector::Reset(uint64_t currentSimNanos [[maybe_unused]])
{
    this->validateConfiguration();
}

void GeneralSingleBodyStateEffector::validateConfiguration()
{
    if (this->jointDOFList.empty()) {
        bskLogger.bskError("GeneralSingleBodyStateEffector: at least one DOF is required.");
    }
    if (!std::isfinite(this->mass) || this->mass <= 0.0) {
        bskLogger.bskError("GeneralSingleBodyStateEffector: mass must be finite and positive.");
    }
    if (!eigenIsValidInertiaMatrix(this->IPntGc_G)) {
        bskLogger.bskError("GeneralSingleBodyStateEffector: IPntGc_G must be a valid finite inertia tensor.");
    }
    if (!eigenIsRotationMatrix(this->dcm_G0B)) {
        bskLogger.bskError("GeneralSingleBodyStateEffector: dcm_G0B must be a proper rotation matrix.");
    }
    if (!this->r_GcG_G.allFinite()) {
        bskLogger.bskError("GeneralSingleBodyStateEffector: r_GcG_G must contain only finite values.");
    }
    if (!this->r_G0B_B.allFinite()) {
        bskLogger.bskError("GeneralSingleBodyStateEffector: r_G0B_B must contain only finite values.");
    }
    for (const auto& dof : this->jointDOFList) {
        if (!eigenIsUnitVector(dof->axis_G)) {
            bskLogger.bskError("GeneralSingleBodyStateEffector: axis_G must be a finite unit vector.");
        }
        if (!std::isfinite(dof->betaInit)) {
            bskLogger.bskError("GeneralSingleBodyStateEffector: betaInit must be finite.");
        }
        if (!std::isfinite(dof->betaDotInit)) {
            bskLogger.bskError("GeneralSingleBodyStateEffector: betaDotInit must be finite.");
        }
        if (!std::isfinite(dof->k) || dof->k < 0.0) {
            bskLogger.bskError("GeneralSingleBodyStateEffector: k must be finite and nonnegative.");
        }
        if (!std::isfinite(dof->c) || dof->c < 0.0) {
            bskLogger.bskError("GeneralSingleBodyStateEffector: c must be finite and nonnegative.");
        }
    }

    // Check the initial joint inertia without modifying live states or cached kinematics on reset.
    Eigen::Matrix<double, 3, 6> jointAxes_B;
    Eigen::Matrix<double, 3, 6> jointPositions_B;
    Eigen::Matrix3d dcm_BG = this->dcm_G0B.transpose();
    Eigen::Vector3d r_GB_B = this->r_G0B_B;
    for (uint32_t i = 0; i < this->numDOF; i++) {
        const auto& dof = this->jointDOFList[i];
        if (dof->type == DOF::Type::ROTATION) {
            // Match computeAttitudeProperties(), including the accepted axis normalization tolerance.
            Eigen::Vector3d prv_GG0 = dof->betaInit * dof->axis_G;
            double dcm_GG0_array[3][3];
            PRV2C(prv_GG0.data(), dcm_GG0_array);
            dcm_BG = dcm_BG * c2DArray2EigenMatrix3d(dcm_GG0_array).transpose();
        } else {
            r_GB_B += dof->betaInit * (dcm_BG * dof->axis_G);
        }
        jointAxes_B.col(i) = dcm_BG * dof->axis_G;
        jointPositions_B.col(i) = r_GB_B;
    }
    const Eigen::Vector3d r_GcB_B = r_GB_B + dcm_BG * this->r_GcG_G;
    const Eigen::Matrix3d IPntGc_B = dcm_BG * this->IPntGc_G * dcm_BG.transpose();
    DOFMapping jacobian = DOFMapping::Zero(6, this->numDOF);
    for (uint32_t i = 0; i < this->numDOF; i++) {
        if (this->jointDOFList[i]->type == DOF::Type::ROTATION) {
            jacobian.col(i).head<3>() = jointAxes_B.col(i).cross(r_GcB_B - jointPositions_B.col(i));
            jacobian.col(i).tail<3>() = jointAxes_B.col(i);
        } else {
            jacobian.col(i).head<3>() = jointAxes_B.col(i);
        }
    }
    const DOFMatrix initialMassMatrix = this->mass * jacobian.topRows<3>().transpose() * jacobian.topRows<3>() +
                                        jacobian.bottomRows<3>().transpose() * IPntGc_B * jacobian.bottomRows<3>();
    this->computeJointMassMatrixInverse(initialMassMatrix);
}

GeneralSingleBodyStateEffector::DOFMatrix GeneralSingleBodyStateEffector::computeJointMassMatrixInverse(
    const DOFMatrix& jointMassMatrix)
{
    if (!jointMassMatrix.allFinite()) {
        bskLogger.bskError("GeneralSingleBodyStateEffector: joint mass matrix must be finite.");
    }
    if ((jointMassMatrix.diagonal().array() == 0.0).any()) {
        bskLogger.bskError("GeneralSingleBodyStateEffector: joint mass matrix is singular.");
    }

    // Equilibrate translational and rotational inertia scales before comparing pivot magnitudes.
    const DOFVector scaling = jointMassMatrix.diagonal().cwiseAbs().cwiseSqrt().cwiseInverse();
    const DOFMatrix scaledMassMatrix = scaling.asDiagonal() * jointMassMatrix * scaling.asDiagonal();
    const Eigen::FullPivLU<DOFMatrix> factorization(scaledMassMatrix);
    if (!factorization.isInvertible()) {
        bskLogger.bskError("GeneralSingleBodyStateEffector: joint mass matrix is singular.");
    }
    const DOFMatrix inverse = scaling.asDiagonal() * factorization.inverse() * scaling.asDiagonal();
    if (!inverse.allFinite()) {
        bskLogger.bskError("GeneralSingleBodyStateEffector: joint mass matrix inverse must be finite.");
    }
    return inverse;
}

void GeneralSingleBodyStateEffector::linkInStates(DynParamManager& statesIn)
{
    this->inertialPositionProperty = statesIn.getPropertyReference(this->propName_inertialPosition);
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->propName_inertialVelocity);
}

void GeneralSingleBodyStateEffector::registerStates(DynParamManager& states)
{
    this->validateConfiguration();
    this->betaState = states.registerState(this->numDOF, 1, this->nameOfBetaState);
    this->betaDotState = states.registerState(this->numDOF, 1, this->nameOfBetaDotState);

    auto betaInit = this->betaState->stateView();
    auto betaDotInit = this->betaDotState->stateView();
    for (auto& dof : this->jointDOFList) {
        betaInit(dof->index, 0) = dof->betaInit;
        betaDotInit(dof->index, 0) = dof->betaDotInit;
    }
}

void GeneralSingleBodyStateEffector::UpdateState(uint64_t currentSimNanos)
{
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
            if (this->motorForceInMsg[transIdx].isLinked()) {
                ArrayMotorForceMsgPayload motorForceBuffer = this->motorForceInMsg[transIdx]();
                dof->f = motorForceBuffer.motorForce[0];
            }
            transIdx++;
        }
    }

    this->computeGeneralBodyInertialStates();
    this->writeOutputStateMessages(currentSimNanos);
}

void GeneralSingleBodyStateEffector::computeGeneralBodyInertialStates()
{
    // Inertial attitude
    Eigen::Matrix3d dcm_GB = this->jointDOFList.at(this->numDOF - 1)->dcm_GB;
    Eigen::Matrix3d dcm_GN = dcm_GB * this->dcm_BN;
    this->sigma_GN = eigenC2MRP(dcm_GN);

    // Inertial angular velocity
    Eigen::Vector3d omega_GB_B = this->TMat.bottomRows<3>() * this->betaDot;
    Eigen::Vector3d omega_GN_B = omega_GB_B + this->omega_BN_B;
    this->omega_GN_G = dcm_GB * omega_GN_B;

    // Inertial position
    Eigen::Vector3d r_GcG_B = dcm_GB.transpose() * this->r_GcG_G;
    Eigen::Vector3d r_GcB_B = r_GcG_B + this->r_GB_B;
    this->r_GcN_N = this->dcm_BN.transpose() * r_GcB_B + (Eigen::Vector3d) * this->inertialPositionProperty;

    // Inertial velocity
    Eigen::Vector3d rPrime_GcB_B =
        this->TMat.topRows<3>() * this->betaDot - r_GcG_B.cross(omega_GB_B) + this->TPrimeMat.topRows<3>() * this->beta;
    Eigen::Vector3d rDot_GcB_B = rPrime_GcB_B + this->omega_BN_B.cross(r_GcB_B);
    this->v_GcN_N = this->dcm_BN.transpose() * rDot_GcB_B + (Eigen::Vector3d) * this->inertialVelocityProperty;
}

void GeneralSingleBodyStateEffector::writeOutputStateMessages(uint64_t currentSimNanos)
{
    // Write the config log message if it is linked
    if (this->generalSingleBodyConfigLogOutMsg.isLinked()) {
        SCStatesMsgPayload configLogMsg = this->generalSingleBodyConfigLogOutMsg.zeroMsgPayload;
        eigenVector3d2CArray(this->r_GcN_N, configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_GcN_N, configLogMsg.v_BN_N);
        eigenMRPd2CArray(this->sigma_GN, configLogMsg.sigma_BN);
        eigenVector3d2CArray(this->omega_GN_G, configLogMsg.omega_BN_B);
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
                LinearTranslationRigidBodyMsgPayload translatingBodyBuffer =
                    this->translatingBodyOutMsgs[transIdx]->zeroMsgPayload;
                translatingBodyBuffer.rho = dof->beta;
                translatingBodyBuffer.rhoDot = dof->betaDot;
                this->translatingBodyOutMsgs[transIdx]->write(&translatingBodyBuffer, this->moduleID, currentSimNanos);
            }
            transIdx++;
        }
    }
}

void GeneralSingleBodyStateEffector::updateEffectorMassProps(double integTime [[maybe_unused]])
{
    // Set effProps.mEff
    this->effProps.mEff = this->mass;

    // Get current state
    this->beta = std::as_const(*this->betaState).stateView();
    this->betaDot = std::as_const(*this->betaDotState).stateView();

    // Set beta and betaDot for each DOF
    for (auto& dof : this->jointDOFList) {
        dof->beta = this->beta[dof->index];
        dof->betaDot = this->betaDot[dof->index];
    }

    this->computeAttitudeProperties();
    this->computePositionProperties();
    this->computeAngularVelocityProperties();

    // Compute general body transformation matrix TMat and its first time derivative TMatPrime
    for (auto& dof : this->jointDOFList) {
        Eigen::Vector3d jointDOFAxis_B = dof->dcm_GB.transpose() * dof->axis_G;

        Eigen::Vector3d omega_GPrevB_B = Eigen::Vector3d::Zero();
        if (dof->index != 0) {
            omega_GPrevB_B = this->jointDOFList.at(dof->index - 1)->omega_GB_B;
        }

        if (dof->type == DOF::Type::ROTATION) {
            this->TMat.col(dof->index).head<3>() = Eigen::Vector3d::Zero();
            this->TMat.col(dof->index).tail<3>() = jointDOFAxis_B;
            this->TPrimeMat.col(dof->index).head<3>() = Eigen::Vector3d::Zero();
            this->TPrimeMat.col(dof->index).tail<3>() = omega_GPrevB_B.cross(jointDOFAxis_B);
        } else {
            this->TMat.col(dof->index).head<3>() = jointDOFAxis_B;
            this->TMat.col(dof->index).tail<3>() = Eigen::Vector3d::Zero();
            this->TPrimeMat.col(dof->index).head<3>() = omega_GPrevB_B.cross(jointDOFAxis_B);
            this->TPrimeMat.col(dof->index).tail<3>() = Eigen::Vector3d::Zero();
        }
    }

    // Compute UMat matrix
    for (uint32_t colIdx{}; colIdx < this->numDOF; colIdx++) {
        this->UMat.col(colIdx).head<3>().setZero();
        this->UMat.col(colIdx).tail<3>().setZero();

        Eigen::Matrix<double, 6, 1> sumToN = Eigen::Matrix<double, 6, 1>::Zero();
        for (uint32_t idx = colIdx + 1; idx < this->numDOF; idx++) {
            sumToN += this->TMat.col(idx) * this->jointDOFList.at(idx)->beta;
        }

        Eigen::Vector3d dofRotAxis = this->TMat.col(colIdx).tail<3>();
        this->UMat.col(colIdx).head<3>() = dofRotAxis.cross(sumToN.head<3>());
        this->UMat.col(colIdx).tail<3>() = dofRotAxis.cross(sumToN.tail<3>());
    }

    // Compute vVec vector
    this->vVec.setZero();
    Eigen::Matrix<double, 6, 1> hVecTerm1;
    Eigen::Matrix<double, 6, 1> hVecTerm2;
    for (uint32_t idx1{}; idx1 < this->numDOF; idx1++) {
        Eigen::Matrix<double, 6, 1> sumTerm1{ Eigen::Matrix<double, 6, 1>::Zero() };
        Eigen::Matrix<double, 6, 1> sumTerm2{ Eigen::Matrix<double, 6, 1>::Zero() };

        for (uint32_t idx2{}; idx2 < idx1; idx2++) {
            Eigen::Matrix<double, 6, 1> sumTerm1A{ Eigen::Matrix<double, 6, 1>::Zero() };
            for (uint32_t idx3{}; idx3 < idx2; idx3++) {
                sumTerm1A += this->TMat.col(idx3) * this->jointDOFList.at(idx3)->betaDot;
            }

            sumTerm1.head<3>() +=
                sumTerm1A.tail<3>().cross(this->TMat.col(idx2).head<3>()) * this->jointDOFList.at(idx2)->betaDot;
            sumTerm1.tail<3>() +=
                sumTerm1A.tail<3>().cross(this->TMat.col(idx2).tail<3>()) * this->jointDOFList.at(idx2)->betaDot;

            sumTerm2 += this->TMat.col(idx2) * this->jointDOFList.at(idx2)->betaDot;
        }

        hVecTerm1.head<3>() =
            sumTerm1.tail<3>().cross(this->TMat.col(idx1).head<3>()) * this->jointDOFList.at(idx1)->beta;
        hVecTerm1.tail<3>() =
            sumTerm1.tail<3>().cross(this->TMat.col(idx1).tail<3>()) * this->jointDOFList.at(idx1)->beta;
        hVecTerm2.head<3>() = sumTerm2.tail<3>().cross(sumTerm2.tail<3>().cross(this->TMat.col(idx1).head<3>())) *
                              this->jointDOFList.at(idx1)->beta;
        hVecTerm2.tail<3>() = sumTerm2.tail<3>().cross(sumTerm2.tail<3>().cross(this->TMat.col(idx1).tail<3>())) *
                              this->jointDOFList.at(idx1)->beta;

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
    Eigen::Vector3d omega_GB_B = this->TMat.bottomRows<3>() * this->betaDot;
    Eigen::Vector3d rPrime_GcB_B =
        this->TMat.topRows<3>() * this->betaDot - r_GcG_B.cross(omega_GB_B) + this->TPrimeMat.topRows<3>() * this->beta;
    this->effProps.rEffPrime_CB_B = rPrime_GcB_B;

    // Compute and set effProps.IEffPrimePntB_B
    Eigen::Matrix3d omegaTilde_GB_B = eigenTilde(omega_GB_B);
    Eigen::Matrix3d rPrimeTilde_GcB_B = eigenTilde(rPrime_GcB_B);
    this->effProps.IEffPrimePntB_B = omegaTilde_GB_B * IPntGc_B - IPntGc_B * omegaTilde_GB_B -
                                     this->mass * (rPrimeTilde_GcB_B * rTilde_GcB_B + rTilde_GcB_B * rPrimeTilde_GcB_B);
}

void GeneralSingleBodyStateEffector::computeAttitudeProperties()
{
    for (auto& dof : this->jointDOFList) {
        Eigen::Vector3d prv_GG0{ Eigen::Vector3d::Zero() };
        if (dof->type == DOF::Type::ROTATION) {
            prv_GG0 = dof->beta * dof->axis_G;
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
}

void GeneralSingleBodyStateEffector::computePositionProperties()
{
    for (auto& dof : this->jointDOFList) {
        Eigen::Vector3d r_GG0_G{ Eigen::Vector3d::Zero() };
        if (dof->type == DOF::Type::TRANSLATION) {
            r_GG0_G = dof->beta * dof->axis_G;
        }
        Eigen::Vector3d r_GG0_B = dof->dcm_GB.transpose() * r_GG0_G;

        if (dof->index == 0) {
            dof->r_GB_B = r_GG0_B + this->r_G0B_B;
        } else {
            dof->r_GB_B = r_GG0_B + this->jointDOFList.at(dof->index - 1)->r_GB_B;
        }
    }
}

void GeneralSingleBodyStateEffector::computeAngularVelocityProperties()
{
    for (auto& dof : this->jointDOFList) {
        Eigen::Vector3d omega_GP_G{ Eigen::Vector3d::Zero() };
        if (dof->type == DOF::Type::ROTATION) {
            omega_GP_G = dof->betaDot * dof->axis_G;
        }
        Eigen::Vector3d omega_GP_B = dof->dcm_GB.transpose() * omega_GP_G;

        if (dof->index == 0) {
            dof->omega_GB_B = omega_GP_B;
        } else {
            dof->omega_GB_B = omega_GP_B + this->jointDOFList.at(dof->index - 1)->omega_GB_B;
        }
    }
}

void GeneralSingleBodyStateEffector::updateContributions(double integTime [[maybe_unused]],
                                                         BackSubMatrices& backSubContr,
                                                         Eigen::MRPd sigma_BN,
                                                         Eigen::Vector3d omega_BN_B,
                                                         Eigen::Vector3d g_N)
{
    // Update hub attitude and angular velocity
    this->dcm_BN = sigma_BN.toRotationMatrix().transpose();
    this->omega_BN_B = omega_BN_B;

    Eigen::Vector3d g_B = this->dcm_BN * g_N;
    Eigen::Vector3d gravityForce_B = this->mass * g_B;

    Eigen::Matrix3d dcm_GB = this->jointDOFList.at(this->numDOF - 1)->dcm_GB;
    Eigen::Vector3d r_GcG_B = dcm_GB.transpose() * this->r_GcG_G;
    Eigen::Matrix3d rTilde_GcG_B = eigenTilde(r_GcG_B);

    Eigen::Matrix3d IPntGc_B = dcm_GB.transpose() * this->IPntGc_G * dcm_GB;

    DOFMatrix MBeta;
    MBeta.resize(this->numDOF, this->numDOF);
    DOFMatrix3 ABetaStar;
    ABetaStar.resize(this->numDOF, 3);
    DOFMatrix3 BBetaStar;
    BBetaStar.resize(this->numDOF, 3);
    DOFVector CBetaStar;
    CBetaStar.resize(this->numDOF);

    // Construct MBeta matrix
    for (uint32_t i = 0; i < this->numDOF; i++) {
        auto& dofI = this->jointDOFList.at(i);
        Eigen::Vector3d dofIAxis_B = dofI->dcm_GB.transpose() * dofI->axis_G;
        if (dofI->type == DOF::Type::TRANSLATION) {
            for (uint32_t j = 0; j < this->numDOF; j++) {
                auto& dofJ = this->jointDOFList.at(j);
                Eigen::Vector3d dofJAxis_B = dofJ->dcm_GB.transpose() * dofJ->axis_G;
                if (dofJ->type == DOF::Type::TRANSLATION) {
                    MBeta(i, j) = this->mass * dofIAxis_B.dot(dofJAxis_B);
                } else {
                    MBeta(i, j) = -this->mass * dofIAxis_B.dot(r_GcG_B.cross(dofJAxis_B));
                }
                for (uint32_t k = 0; k < j; k++) {
                    auto& dofK = this->jointDOFList.at(k);
                    if (dofK->type == DOF::Type::ROTATION) {
                        Eigen::Vector3d dofKAxis_B = dofK->dcm_GB.transpose() * dofK->axis_G;
                        Eigen::Vector3d r_JJMinus1_B = Eigen::Vector3d::Zero();
                        if (j == 0) {
                            r_JJMinus1_B = this->r_GB_B;
                        } else {
                            auto& dofJMinus1 = this->jointDOFList.at(j - 1);
                            r_JJMinus1_B = dofJ->r_GB_B - dofJMinus1->r_GB_B;
                        }
                        MBeta(i, k) -= this->mass * dofIAxis_B.dot(r_JJMinus1_B.cross(dofKAxis_B));
                    }
                }
            }
        } else {
            Eigen::Vector3d r_GcGi_B = r_GcG_B + this->r_GB_B - dofI->r_GB_B;
            for (uint32_t j = 0; j < this->numDOF; j++) {
                auto& dofJ = this->jointDOFList.at(j);
                Eigen::Vector3d dofJAxis_B = dofJ->dcm_GB.transpose() * dofJ->axis_G;

                if (dofJ->type == DOF::Type::TRANSLATION) {
                    MBeta(i, j) = this->mass * dofIAxis_B.dot(r_GcGi_B.cross(dofJAxis_B));
                } else {
                    MBeta(i, j) =
                        dofIAxis_B.dot(IPntGc_B * dofJAxis_B - this->mass * r_GcGi_B.cross(r_GcG_B.cross(dofJAxis_B)));
                }
                for (uint32_t k = 0; k < j; k++) {
                    auto& dofK = this->jointDOFList.at(k);
                    if (dofK->type == DOF::Type::ROTATION) {
                        Eigen::Vector3d dofKAxis_B = dofK->dcm_GB.transpose() * dofK->axis_G;
                        Eigen::Vector3d r_JJMinus1_B = Eigen::Vector3d::Zero();
                        if (j == 0) {
                            r_JJMinus1_B = this->r_GB_B;
                        } else {
                            auto& dofJMinus1 = this->jointDOFList.at(j - 1);
                            r_JJMinus1_B = dofJ->r_GB_B - dofJMinus1->r_GB_B;
                        }
                        MBeta(i, k) -= this->mass * dofIAxis_B.dot(r_GcGi_B.cross(r_JJMinus1_B.cross(dofKAxis_B)));
                    }
                }
            }
        }
    }

    // These kinematic terms are shared by every DOF row.
    Eigen::Vector3d cBetaTerm1a = Eigen::Vector3d::Zero();
    for (uint32_t j = 0; j < this->numDOF; j++) {
        auto& dofJ = this->jointDOFList.at(j);
        Eigen::Vector3d omega_JMinus1N_B = Eigen::Vector3d::Zero();
        Eigen::Vector3d omega_JJMinus1_B = Eigen::Vector3d::Zero();
        if (j == 0) {
            omega_JMinus1N_B = this->omega_BN_B;
            omega_JJMinus1_B = dofJ->omega_GB_B;
        } else {
            auto& dofJMinus1 = this->jointDOFList.at(j - 1);
            omega_JMinus1N_B = dofJMinus1->omega_GB_B + this->omega_BN_B;
            omega_JJMinus1_B = dofJ->omega_GB_B - dofJMinus1->omega_GB_B;
        }
        cBetaTerm1a += omega_JMinus1N_B.cross(omega_JJMinus1_B);
    }
    Eigen::Vector3d omega_GnN_B = this->jointDOFList.back()->omega_GB_B + this->omega_BN_B;
    Eigen::Vector3d cBetaTerm1 =
        -this->mass * (cBetaTerm1a.cross(r_GcG_B) + omega_GnN_B.cross(omega_GnN_B.cross(r_GcG_B)));

    Eigen::Vector3d cBetaTerm2 = Eigen::Vector3d::Zero();
    for (uint32_t j = 0; j < this->numDOF; j++) {
        Eigen::Vector3d cBetaTerm2a = Eigen::Vector3d::Zero();
        for (uint32_t k = 0; k < j; k++) {
            auto& dofK = this->jointDOFList.at(k);
            Eigen::Vector3d omega_KMinus1N_B = Eigen::Vector3d::Zero();
            Eigen::Vector3d omega_KKMinus1_B = Eigen::Vector3d::Zero();
            if (k == 0) {
                omega_KMinus1N_B = this->omega_BN_B;
                omega_KKMinus1_B = dofK->omega_GB_B;
            } else {
                auto& dofKMinus1 = this->jointDOFList.at(k - 1);
                omega_KMinus1N_B = dofKMinus1->omega_GB_B + this->omega_BN_B;
                omega_KKMinus1_B = dofK->omega_GB_B - dofKMinus1->omega_GB_B;
            }
            cBetaTerm2a += omega_KMinus1N_B.cross(omega_KKMinus1_B);
        }
        auto& dofJ = this->jointDOFList.at(j);
        Eigen::Vector3d r_JJMinus1_B = Eigen::Vector3d::Zero();
        Eigen::Vector3d omega_JMinus1N_B = Eigen::Vector3d::Zero();
        if (j == 0) {
            r_JJMinus1_B = dofJ->r_GB_B;
            omega_JMinus1N_B = this->omega_BN_B;
        } else {
            auto& dofJMinus1 = this->jointDOFList.at(j - 1);
            r_JJMinus1_B = dofJ->r_GB_B - dofJMinus1->r_GB_B;
            omega_JMinus1N_B = dofJMinus1->omega_GB_B + this->omega_BN_B;
        }

        Eigen::Vector3d rJMinus1Prime_JJMinus1_B = Eigen::Vector3d::Zero();
        if (dofJ->type == DOF::Type::TRANSLATION) {
            rJMinus1Prime_JJMinus1_B = dofJ->betaDot * dofJ->dcm_GB.transpose() * dofJ->axis_G;
        }
        cBetaTerm2 += r_JJMinus1_B.cross(cBetaTerm2a) - 2 * omega_JMinus1N_B.cross(rJMinus1Prime_JJMinus1_B) -
                      omega_JMinus1N_B.cross(omega_JMinus1N_B.cross(r_JJMinus1_B));
    }

    cBetaTerm2 = this->mass * cBetaTerm2;

    const Eigen::Vector3d cBetaTerm3 = -IPntGc_B * cBetaTerm1a - omega_GnN_B.cross(IPntGc_B * omega_GnN_B);

    for (uint32_t i = 0; i < this->numDOF; i++) {
        auto& dofI = this->jointDOFList.at(i);
        Eigen::Vector3d dofAxis_B = dofI->dcm_GB.transpose() * dofI->axis_G;

        if (dofI->type == DOF::Type::TRANSLATION) {
            ABetaStar.row(i) = -this->mass * dofAxis_B.transpose();
            BBetaStar.row(i) = this->mass * dofAxis_B.cross(r_GcG_B + this->r_GB_B).transpose();

            CBetaStar(i, 0) = dofI->f + dofAxis_B.dot(gravityForce_B + cBetaTerm1 + cBetaTerm2);
        } else {
            Eigen::Vector3d r_GcGi_B = r_GcG_B + this->r_GB_B - dofI->r_GB_B;
            Eigen::Vector3d axisCrossOffset_B = dofAxis_B.cross(r_GcGi_B);
            ABetaStar.row(i) = -this->mass * axisCrossOffset_B.transpose();
            BBetaStar.row(i) = -dofAxis_B.transpose() * IPntGc_B +
                               this->mass * axisCrossOffset_B.cross(r_GcG_B + this->r_GB_B).transpose();

            Eigen::Vector3d gravityTorquePntG_B = r_GcGi_B.cross(gravityForce_B);

            CBetaStar(i, 0) =
                dofI->u + dofAxis_B.dot(gravityTorquePntG_B + cBetaTerm3 + r_GcGi_B.cross(cBetaTerm1 + cBetaTerm2));
        }
        CBetaStar(i, 0) += -dofI->k * (dofI->beta - dofI->betaRef) - dofI->c * (dofI->betaDot - dofI->betaDotRef);
    }

    // Define BSM ABeta, BBeta, and CBeta matrices
    const DOFMatrix MBetaInv = this->computeJointMassMatrixInverse(MBeta);
    this->ABeta.noalias() = MBetaInv * ABetaStar;
    this->BBeta.noalias() = MBetaInv * BBetaStar;
    this->CBeta.noalias() = MBetaInv * CBetaStar;

    // Reuse the translation and rotation mappings for all backsubstitution contributions.
    const Eigen::Vector3d r_GcB_B = r_GcG_B + this->r_GB_B;
    const Eigen::Matrix<double, 3, Eigen::Dynamic, 0, 3, 6> translationMap =
        this->TMat.topRows<3>() - rTilde_GcG_B * this->TMat.bottomRows<3>() + this->UMat.topRows<3>();
    const Eigen::Matrix<double, 3, Eigen::Dynamic, 0, 3, 6> rotationMap =
        IPntGc_B * this->TMat.bottomRows<3>() + this->mass * eigenTilde(r_GcB_B) * translationMap;
    backSubContr.matrixA.noalias() = this->mass * translationMap * this->ABeta;
    backSubContr.matrixB.noalias() = this->mass * translationMap * this->BBeta;
    backSubContr.matrixC.noalias() = rotationMap * this->ABeta;
    backSubContr.matrixD.noalias() = rotationMap * this->BBeta;

    // Define BSM vecTrans and vecRot contributions.
    const Eigen::Vector3d omega_GB_B = this->TMat.bottomRows<3>() * this->betaDot;
    const Eigen::Vector3d angularRateDerivative_B = this->TPrimeMat.bottomRows<3>() * this->betaDot;
    const Eigen::Vector3d kinematicAcceleration_B = 2 * this->TPrimeMat.topRows<3>() * this->betaDot -
                                                    r_GcG_B.cross(angularRateDerivative_B) +
                                                    omega_GB_B.cross(omega_GB_B.cross(r_GcG_B)) + this->vVec.head<3>();
    const Eigen::Vector3d rPrime_GcB_B =
        this->TMat.topRows<3>() * this->betaDot - r_GcG_B.cross(omega_GB_B) + this->TPrimeMat.topRows<3>() * this->beta;
    backSubContr.vecTrans = -this->mass * (kinematicAcceleration_B + translationMap * this->CBeta);
    backSubContr.vecRot = -IPntGc_B * angularRateDerivative_B - this->mass * r_GcB_B.cross(kinematicAcceleration_B) -
                          (omega_GB_B + this->omega_BN_B).cross(IPntGc_B * omega_GB_B) -
                          this->mass * this->omega_BN_B.cross(r_GcB_B.cross(rPrime_GcB_B)) - rotationMap * this->CBeta;
}

void GeneralSingleBodyStateEffector::computeDerivatives(double integTime [[maybe_unused]],
                                                        Eigen::Vector3d rDDot_BN_N,
                                                        Eigen::Vector3d omegaDot_BN_B,
                                                        Eigen::MRPd sigma_BN)
{
    // Update hub attitude
    this->dcm_BN = sigma_BN.toRotationMatrix().transpose();

    // Set betaState derivative
    this->betaState->derivativeView() = std::as_const(*this->betaDotState).stateView();

    // Set betaDotState derivative
    auto betaDDot = this->betaDotState->derivativeView();
    Eigen::Vector3d rDDot_BN_B = this->dcm_BN * rDDot_BN_N;
    betaDDot.noalias() = this->ABeta * rDDot_BN_B;
    betaDDot.noalias() += this->BBeta * omegaDot_BN_B;
    betaDDot += this->CBeta;
}

void GeneralSingleBodyStateEffector::updateEnergyMomContributions(double integTime [[maybe_unused]],
                                                                  Eigen::Vector3d& rotAngMomPntCContr_B,
                                                                  double& rotEnergyContr,
                                                                  Eigen::Vector3d omega_BN_B)
{
    // Update hub angular velocity
    this->omega_BN_B = omega_BN_B;

    // Rotational angular momentum contribution
    Eigen::Matrix3d dcm_GB = this->jointDOFList.at(this->numDOF - 1)->dcm_GB;
    Eigen::Vector3d r_GcG_B = dcm_GB.transpose() * this->r_GcG_G;
    Eigen::Vector3d r_GcB_B = r_GcG_B + this->r_GB_B;
    Eigen::Vector3d omega_GB_B = this->TMat.bottomRows<3>() * this->betaDot;
    Eigen::Vector3d rPrime_GcB_B =
        this->TMat.topRows<3>() * this->betaDot - r_GcG_B.cross(omega_GB_B) + this->TPrimeMat.topRows<3>() * this->beta;
    Eigen::Vector3d rDot_GcB_B = rPrime_GcB_B + this->omega_BN_B.cross(r_GcB_B);
    Eigen::Vector3d omega_GN_B = omega_GB_B + this->omega_BN_B;
    Eigen::Matrix3d IPntGc_B = dcm_GB.transpose() * this->IPntGc_G * dcm_GB;
    rotAngMomPntCContr_B = IPntGc_B * omega_GN_B + this->mass * r_GcB_B.cross(rDot_GcB_B);

    // Rotational energy contribution
    rotEnergyContr = 0.5 * omega_GN_B.dot(IPntGc_B * omega_GN_B) + 0.5 * this->mass * rDot_GcB_B.dot(rDot_GcB_B);
    for (auto& dof : this->jointDOFList) {
        double rotEnergy = 0.5 * dof->k * (dof->beta - dof->betaRef) * (dof->beta - dof->betaRef);
        rotEnergyContr += rotEnergy;
    }
}

void GeneralSingleBodyStateEffector::addRotDOF(std::shared_ptr<DOF> newDOF)
{
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

void GeneralSingleBodyStateEffector::addTransDOF(std::shared_ptr<DOF> newDOF)
{
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

void GeneralSingleBodyStateEffector::addDOF(std::shared_ptr<DOF> newDOF)
{
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

bool GeneralSingleBodyStateEffector::isDOFAdded(const std::shared_ptr<DOF>& newDOF) const
{
    return std::find(this->jointDOFList.begin(), this->jointDOFList.end(), newDOF) != this->jointDOFList.end();
}

void GeneralSingleBodyStateEffector::clearDOFs()
{
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
    for (auto* rotMsg : this->spinningBodyOutMsgs)
        delete rotMsg;
    this->spinningBodyOutMsgs.clear();
    for (auto* transMsg : this->translatingBodyOutMsgs)
        delete transMsg;
    this->translatingBodyOutMsgs.clear();

    this->TMat.resize(6, 0);
    this->TPrimeMat.resize(6, 0);
    this->UMat.resize(6, 0);
    this->ABeta.resize(0, 3);
    this->BBeta.resize(0, 3);
    this->CBeta.resize(0);
}
