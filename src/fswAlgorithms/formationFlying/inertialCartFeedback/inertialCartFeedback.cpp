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

#include "fswAlgorithms/formationFlying/inertialCartFeedback/inertialCartFeedback.h"

void InertialCartFeedback::SelfInit()
{
    CmdForceInertialMsg_C_init(&this->forceOutMsgC);
}

void InertialCartFeedback::Reset(uint64_t CurrentSimNanos)
{
    if (!this->deputyNavInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "InertialCartFeedback.deputyNavInMsg was not linked.");
    }
    if (!this->deputyRefInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "InertialCartFeedback.deputyRefInMsg was not linked.");
    }
    if (!this->deputyVehicleConfigInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "InertialCartFeedback.deputyVehicleConfigInMsg was not linked.");
    }

    const VehicleConfigMsgPayload depCfg = this->deputyVehicleConfigInMsg();
    this->deputyMass = depCfg.massSC;

    if (this->deputyMass <= 0.0) {
        bskLogger.bskLog(BSK_ERROR, "InertialCartFeedback.deputyMass must be positive.");
    }
    if (!this->setKFlag) {
        bskLogger.bskLog(BSK_ERROR, "InertialCartFeedback.K must be set before simulation start.");
    }
    if (!this->setPFlag) {
        bskLogger.bskLog(BSK_ERROR, "InertialCartFeedback.P must be set before simulation start.");
    }
}

void InertialCartFeedback::UpdateState(uint64_t CurrentSimNanos)
{
    NavTransMsgPayload depNav = this->deputyNavInMsg();
    TransRefMsgPayload depRef = this->deputyRefInMsg();
    CmdForceInertialMsgPayload forceOut = this->forceOutMsg.zeroMsgPayload;

    const Eigen::Vector3d r_BN_N = cArray2EigenVector3d(depNav.r_BN_N);
    const Eigen::Vector3d v_BN_N = cArray2EigenVector3d(depNav.v_BN_N);
    const Eigen::Vector3d r_RN_N = cArray2EigenVector3d(depRef.r_RN_N);
    const Eigen::Vector3d v_RN_N = cArray2EigenVector3d(depRef.v_RN_N);
    const Eigen::Vector3d a_RN_N = cArray2EigenVector3d(depRef.a_RN_N); // total reference acceleration for the deputy, including feed-forward terms for non-natural motion if applicable
    const Eigen::Vector3d deltaR_N = r_BN_N - r_RN_N;
    const Eigen::Vector3d deltaV_N = v_BN_N - v_RN_N;
    const double normRDeputy = r_BN_N.norm();
    const double normRDeputyRef = r_RN_N.norm();

    Eigen::Vector3d aGravDeputy_N = Eigen::Vector3d::Zero();
    if (this->mu > 0.0) {
        if (normRDeputy > 1e-6) {
            aGravDeputy_N = -this->mu * r_BN_N / (normRDeputy * normRDeputy * normRDeputy);
        } else {
            bskLogger.bskLog(BSK_ERROR, "InertialCartFeedback deputy position norm too small to evaluate gravity.");
        }
    }

    Eigen::Vector3d forceCmd_N =
            -this->deputyMass * aGravDeputy_N
            +this->deputyMass * a_RN_N
            -this->K * deltaR_N
            -this->P * deltaV_N;

    eigenVector3d2CArray(forceCmd_N, forceOut.forceRequestInertial);
    this->forceOutMsg.write(&forceOut, this->moduleID, CurrentSimNanos);
    CmdForceInertialMsg_C_write(&forceOut, &this->forceOutMsgC, this->moduleID, CurrentSimNanos);
}

void InertialCartFeedback::setMu(const double value)
{
    if (value < 0.0) {
        bskLogger.bskLog(BSK_ERROR, "inertialCartFeedback: mu must be non-negative.");
    } else {
        this->mu = value;
    }
}

void InertialCartFeedback::setK(const std::vector<double>& value)
{
    if (value.size() != 9) {
        bskLogger.bskLog(BSK_ERROR, "inertialCartFeedback: K must contain exactly 9 elements.");
        return;
    }
    this->K << value[0], value[1], value[2],
               value[3], value[4], value[5],
               value[6], value[7], value[8];

    if ((this->K - this->K.transpose()).norm() > 1e-12) {
        bskLogger.bskLog(BSK_ERROR, "inertialCartFeedback: K must be symmetric positive definite.");
        return;
    }

    const double minEigK = this->K.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff();
    if (!(minEigK > 0.0)) {
        bskLogger.bskLog(BSK_ERROR, "inertialCartFeedback: K must be symmetric positive definite.");
        return;
    }
    this->setKFlag = true;
}

void InertialCartFeedback::setP(const std::vector<double>& value)
{
    if (value.size() != 9) {
        bskLogger.bskLog(BSK_ERROR, "inertialCartFeedback: P must contain exactly 9 elements.");
        return;
    }
    this->P << value[0], value[1], value[2],
               value[3], value[4], value[5],
               value[6], value[7], value[8];

    if ((this->P - this->P.transpose()).norm() > 1e-12) {
        bskLogger.bskLog(BSK_ERROR, "inertialCartFeedback: P must be symmetric positive definite.");
        return;
    }

    const double minEigP = this->P.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff();
    if (!(minEigP > 0.0)) {
        bskLogger.bskLog(BSK_ERROR, "inertialCartFeedback: P must be symmetric positive definite.");
        return;
    }
    this->setPFlag = true;
}
