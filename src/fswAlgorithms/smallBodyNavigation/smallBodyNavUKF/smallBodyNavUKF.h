/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef SMALLBODYNAVUKF_H
#define SMALLBODYNAVUKF_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "cMsgCInterface/EphemerisMsg_C.h"
#include "cMsgCInterface/NavTransMsg_C.h"
#include "cMsgCInterface/SmallBodyNavUKFMsg_C.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/orbitalMotion.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"


/*! @brief This module estimates relative spacecraft position, velocity with respect to the body, and the non-Keplerian acceleration perturbing the spacecraft motion, using an unscented Kalman filter (UKF)
 */
class SmallBodyNavUKF: public SysModel {
public:
    SmallBodyNavUKF();
    ~SmallBodyNavUKF();

    void SelfInit();  //!< Self initialization for C-wrapped messages
    void Reset(uint64_t CurrentSimNanos);  //!< Resets module
    void UpdateState(uint64_t CurrentSimNanos);  //!< Updates state

private:
    static constexpr int stateSize = 9;
    static constexpr int measurementSize = 3;
    static constexpr int sigmaCount = 2*stateSize + 1;
    using StateVector = Eigen::Matrix<double, stateSize, 1>;
    using StateMatrix = Eigen::Matrix<double, stateSize, stateSize>;
    using SigmaWeightVector = Eigen::Matrix<double, sigmaCount, 1>;
    using StateSigmaMatrix = Eigen::Matrix<double, stateSize, sigmaCount>;
    using MeasurementSigmaMatrix = Eigen::Matrix<double, measurementSize, sigmaCount>;
    using CrossCorrelationMatrix = Eigen::Matrix<double, stateSize, measurementSize>;

    void readMessages();  //!< Reads input messages
    void writeMessages(uint64_t CurrentSimNanos);  //!< Writes output messages
    void processUT(uint64_t CurrentSimNanos);  //!< Process unscented transform
    void measurementUT();  //!< Measurements unscented transform
    void kalmanUpdate();  //!< Computes the state and covariance update

public:
    ReadFunctor<NavTransMsgPayload> navTransInMsg;  //!< Translational nav input message
    ReadFunctor<EphemerisMsgPayload> asteroidEphemerisInMsg;  //!< Small body ephemeris input message
    Message<SmallBodyNavUKFMsgPayload> smallBodyNavUKFOutMsg;  //!< Small body nav UKF output msg - states and covariances
    SmallBodyNavUKFMsg_C smallBodyNavUKFOutMsgC = {};  //!< C-wrapped Small body nav UKF output msg - states and covariances

    BSKLogger bskLogger;  //!< -- BSK Logging

    double mu_ast;  //!< Gravitational constant of the small body
    Eigen::MatrixXd P_proc;  //!< Process noise covariance
    Eigen::MatrixXd R_meas;  //!< Measurement noise covariance
    Eigen::VectorXd x_hat_k;  //!< Current state estimate
    Eigen::MatrixXd P_k;  //!< Current state estimation covariance

    double alpha;  //!< UKF hyper-parameter
    double beta;  //!< UKF hyper-parameter
    double kappa;  //!< UKF hyper-parameter

    Eigen::Matrix3d dcm_AN;  //!< Small body dcm
    Eigen::Vector3d omega_AN_A;  //!< Small body angular velocity

private:
    NavTransMsgPayload navTransInMsgBuffer;  //!< Message buffer for input translational nav message
    EphemerisMsgPayload asteroidEphemerisInMsgBuffer;  //!< Message buffer for asteroid ephemeris

    uint64_t prevTime;  //!< Previous time, ns
    StateVector x_hat_k1_;  //!< Apriori state estimate for time k+1
    StateVector x_hat_k1;  //!< Update state estimate for time k+1
    SigmaWeightVector wm_sigma;  //!< Mean sigma weights for UT
    SigmaWeightVector wc_sigma;  //!< Covariance sigma weights for UT
    Eigen::Vector3d y_hat_k1_;  //!< Apriori measurement estimate for time k+1
    StateMatrix P_k1_;  //!< Apriori state covariance estimate for time k+1
    StateMatrix P_k1;  //!< Update state covariance estimate for time k+1
    StateSigmaMatrix X_sigma_k1_;  //!< Apriori state sigma points for time k+1
    Eigen::Matrix3d R_k1_;  //!< Apriori measurements covariance estimate for time k+1
    MeasurementSigmaMatrix Y_sigma_k1_;  //!< Apriori measurements sigma points for time k+1
    CrossCorrelationMatrix H;  //!< State-measurements cross-correlation matrix
    CrossCorrelationMatrix K;  //!< Kalman gain matrix
};


#endif
