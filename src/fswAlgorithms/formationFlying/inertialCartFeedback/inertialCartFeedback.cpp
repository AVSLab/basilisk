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
    if (!this->deputyTransInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "InertialCartFeedback.deputyTransInMsg was not linked.");
    }
    if (!this->deputyTransDesiredInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "InertialCartFeedback.deputyTransDesiredInMsg was not linked.");
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
    NavTransMsgPayload depTrans = this->deputyTransInMsg();
    NavTransMsgPayload depDes = this->deputyTransDesiredInMsg();
    CmdForceInertialMsgPayload feedforward_N = this->forceOutMsg.zeroMsgPayload;
    if (this->forceFeedforwardInMsg.isLinked()) {
        feedforward_N = this->forceFeedforwardInMsg();
    }

    const Eigen::Vector3d rDeputy_N = cArray2EigenVector3d(depTrans.r_BN_N);
    const Eigen::Vector3d vDeputy_N = cArray2EigenVector3d(depTrans.v_BN_N);
    const Eigen::Vector3d rDeputyDes_N = cArray2EigenVector3d(depDes.r_BN_N);
    const Eigen::Vector3d vDeputyDes_N = cArray2EigenVector3d(depDes.v_BN_N);
    const Eigen::Vector3d deltaR_N = rDeputy_N - rDeputyDes_N;
    const Eigen::Vector3d deltaV_N = vDeputy_N - vDeputyDes_N;

    const double normRDeputy = rDeputy_N.norm();
    const double normRDeputyDes = rDeputyDes_N.norm();

    Eigen::Vector3d gravityCompAccel_N = Eigen::Vector3d::Zero();
    if (this->mu > 0.0) {
        Eigen::Vector3d aGravDeputy_N = Eigen::Vector3d::Zero();
        Eigen::Vector3d aGravDeputyStar_N = Eigen::Vector3d::Zero();
        if (normRDeputy > 1e-6) {
            aGravDeputy_N = -this->mu * rDeputy_N / (normRDeputy * normRDeputy * normRDeputy);
        } else {
            bskLogger.bskLog(BSK_ERROR, "InertialCartFeedback deputy position norm too small to evaluate gravity.");
        }
        if (normRDeputyDes > 1e-6) {
            aGravDeputyStar_N = -this->mu * rDeputyDes_N / (normRDeputyDes * normRDeputyDes * normRDeputyDes);
        } else {
            bskLogger.bskLog(BSK_ERROR, "InertialCartFeedback desired deputy position norm too small to evaluate gravity.");
        }
        gravityCompAccel_N = aGravDeputy_N - aGravDeputyStar_N;
    }

    const Eigen::Vector3d forceFF_N = cArray2EigenVector3d(feedforward_N.forceRequestInertial);
    Eigen::Vector3d forceCmd_N =
            -this->deputyMass * gravityCompAccel_N
            -this->deputyMass * this->K * deltaR_N
            -this->deputyMass * this->P * deltaV_N
            +forceFF_N;

    CmdForceInertialMsgPayload forceOutBuffer = this->forceOutMsg.zeroMsgPayload;
    eigenVector3d2CArray(forceCmd_N, forceOutBuffer.forceRequestInertial);
    this->forceOutMsg.write(&forceOutBuffer, this->moduleID, CurrentSimNanos);
    CmdForceInertialMsg_C_write(&forceOutBuffer, &this->forceOutMsgC, this->moduleID, CurrentSimNanos);
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
