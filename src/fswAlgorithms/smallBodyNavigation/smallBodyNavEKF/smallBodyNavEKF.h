/*
 ISC License

 Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado Boulder

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


#ifndef SMALLBODYNAVEKF_H
#define SMALLBODYNAVEKF_H

#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/msgPayloadDefC/NavTransMsgPayload.h"
#include "architecture/msgPayloadDefC/NavAttMsgPayload.h"
#include "architecture/msgPayloadDefC/EphemerisMsgPayload.h"
#include "architecture/msgPayloadDefC/RWSpeedMsgPayload.h"
#include "architecture/msgPayloadDefC/SmallBodyNavMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include <Eigen/Dense>

/*! @brief This module estimates relative spacecraft position and velocity with respect to the body,attitude and attitude rate of the body wrt. the inertial frame, and the attitude and attituderate of the spacecraft with respect to the inertial frame
 */
class SmallBodyNavEKF: public SysModel {
public:
    SmallBodyNavEKF();
    ~SmallBodyNavEKF();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);

private:
    void readMessages();
    void writeMessages();
    void predict();
    void computeAMatrix();
    void measurementUpdate();

public:
    ReadFunctor<NavTransMsgPayload> navTransInMsg;  //!< Translational nav input message
    ReadFunctor<NavAttMsgPayload> navAttInMsg;  //!< Attitude nav input message
    ReadFunctor<EphemerisMsgPayload> asteroidEphemerisInMsg;  //!< Small body ephemeris input message
    ReadFunctor<EphemerisMsgPayload> sunEphemerisInMsg;  //!< Sun ephemeris input message
    ReadFunctor<RWSpeedMsgPayload> rwSpeedInMsg;  //!< Reaction wheel speed input message

    Message<NavTransMsgPayload> navTransOutMsg;  //!< Translational nav output message
    Message<NavAttMsgPayload> navAttOutMsg;  //!< Attitude nav output message
    Message<SmallBodyNavMsgPayload> smallBodyNavOutMsg;  //!< Small body nav output msg - states and covariances
    Message<EphemerisMsgPayload> asteroidEphemerisOutMsg;  //!< Small body ephemeris output message

    BSKLogger bskLogger;              //!< -- BSK Logging

private:
    uint_64t prevTime;
    unit_64t numStates;
    Eigen::VectorXd x_hat_k;
    Eigen::VectorXd x_hat_k1_;
    Eigen::VectorXd x_hat_k1;
    Eigen::MatrixXd P_k;
    Eigen::MatrixXd P_k1_;
    Eigen::MatrixXd P_k1;
    Eigen::MatrixXd Q_k1;
    Eigen::MatrixXd R_k1;
    Eigen::MatrixXd A_k;
};


#endif
