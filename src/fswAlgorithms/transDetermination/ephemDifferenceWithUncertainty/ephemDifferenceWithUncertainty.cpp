/*
 ISC License

 Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

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

#include "ephemDifferenceWithUncertainty.h"

EphemDifferenceWithUncertainty::EphemDifferenceWithUncertainty() = default;

EphemDifferenceWithUncertainty::~EphemDifferenceWithUncertainty() = default;

/*! This method resets the module state to its initialized default.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void EphemDifferenceWithUncertainty::Reset(uint64_t currentSimNanos)
{
    if (!this->ephemBaseInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,
                         "EphemDifferenceWithUncertainty.ephemBaseInMsg wasn't connected.");
    }
    if (!this->ephemSecondaryInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR,
                         "EphemDifferenceWithUncertainty.ephemSecondaryInMsg wasn't connected.");
    }
}

/*! During an update, this module computes the difference between two ephemeris messages.
 @return void
 @param currentSimNanos The clock time at which the function was called (nanoseconds)
 */
void EphemDifferenceWithUncertainty::UpdateState(uint64_t currentSimNanos)
{
    EphemerisMsgPayload ephemBaseInBuffer = this->ephemBaseInMsg();
    EphemerisMsgPayload ephemSecondaryInBuffer = this->ephemSecondaryInMsg();

    // take timeTag from secondary, as timeTag from primary/base may be constant due to stand-alone message
    double timeTag = ephemSecondaryInBuffer.timeTag;

    /*! - compute relative states */
    Eigen::Vector3d r_1_N = cArray2EigenVector3d(ephemBaseInBuffer.r_BdyZero_N);
    Eigen::Vector3d v_1_N = cArray2EigenVector3d(ephemBaseInBuffer.v_BdyZero_N);
    Eigen::Vector3d r_2_N = cArray2EigenVector3d(ephemSecondaryInBuffer.r_BdyZero_N);
    Eigen::Vector3d v_2_N = cArray2EigenVector3d(ephemSecondaryInBuffer.v_BdyZero_N);

    Eigen::Vector3d r_21_N = r_2_N - r_1_N;
    Eigen::Vector3d v_21_N = v_2_N - v_1_N;

    int numStates = 6;
    Eigen::VectorXd state_21_N(numStates);
    state_21_N << r_21_N, v_21_N;

    /*! - compute relative covariance matrix */
    Eigen::MatrixXd covar_21_N(numStates, numStates);
    covar_21_N = this->covarianceBase + this->covarianceSecondary;

    /*! - output messages */
    NavTransMsgPayload navTransOutMsgBuffer;
    navTransOutMsgBuffer = this->navTransOutMsg.zeroMsgPayload;
    FilterMsgPayload filterOutMsgBuffer;
    filterOutMsgBuffer = this->filterOutMsg.zeroMsgPayload;

    navTransOutMsgBuffer.timeTag = timeTag;
    eigenVector3d2CArray(r_21_N, navTransOutMsgBuffer.r_BN_N);
    eigenVector3d2CArray(v_21_N, navTransOutMsgBuffer.v_BN_N);

    filterOutMsgBuffer.numberOfStates = numStates;
    filterOutMsgBuffer.timeTag = timeTag;
    eigenMatrixXd2CArray(state_21_N, filterOutMsgBuffer.state);
    eigenMatrixXd2CArray(covar_21_N, filterOutMsgBuffer.covar);

    this->navTransOutMsg.write(&navTransOutMsgBuffer, this->moduleID, currentSimNanos);
    this->filterOutMsg.write(&filterOutMsgBuffer, this->moduleID, currentSimNanos);
}

/*! Set the state covariance of the base celestial object (e.g. asteroid)
    @param Eigen::MatrixXd covariance
    @return void
    */
void EphemDifferenceWithUncertainty::setCovarianceBase(const Eigen::MatrixXd stateCovariance){
    this->covarianceBase.resize(stateCovariance.rows(), stateCovariance.cols());
    this->covarianceBase << stateCovariance;
}

/*! Get the state covariance of the base celestial object (e.g. asteroid)
    @return Eigen::MatrixXd covariance
    */
Eigen::MatrixXd EphemDifferenceWithUncertainty::getCovarianceBase() const {
    return this->covarianceBase;
}

/*! Set the state covariance of the secondary celestial object (e.g. spacecraft)
    @param Eigen::MatrixXd covariance
    @return void
    */
void EphemDifferenceWithUncertainty::setCovarianceSecondary(const Eigen::MatrixXd stateCovariance){
    this->covarianceSecondary.resize(stateCovariance.rows(), stateCovariance.cols());
    this->covarianceSecondary << stateCovariance;
}

/*! Get the state covariance of the secondary celestial object (e.g. spacecraft)
    @return Eigen::MatrixXd covariance
    */
Eigen::MatrixXd EphemDifferenceWithUncertainty::getCovarianceSecondary() const {
    return this->covarianceSecondary;
}
