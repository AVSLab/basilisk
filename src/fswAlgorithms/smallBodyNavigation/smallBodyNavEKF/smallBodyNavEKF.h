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
#include "architecture/utilities/orbitalMotion.h"
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
    void aprioriState();
    void aprioriCovar();
    void computeDynamicsMatrix();
    void measurementUpdate();

public:
    ReadFunctor<NavTransMsgPayload> navTransInMsg;  //!< Translational nav input message
    ReadFunctor<NavAttMsgPayload> navAttInMsg;  //!< Attitude nav input message
    ReadFunctor<EphemerisMsgPayload> asteroidEphemerisInMsg;  //!< Small body ephemeris input message
    ReadFunctor<EphemerisMsgPayload> sunEphemerisInMsg;  //!< Sun ephemeris input message
    std::vector<ReadFunctor<RWConfigLogMsgPayload>> rwInMsgs; //!< Reaction wheel speed and torque input messages
    std::vector<ReadFunctor<THROutputMsgPayload>> dvThrusterInMsgs; //!< dV thruster input msg
    std::vector<ReadFunctor<THROutputMsgPayload>> attitudeThrusterInMsgs; //!< Attitude thruster input msg

    Message<NavTransMsgPayload> navTransOutMsg;  //!< Translational nav output message
    Message<NavAttMsgPayload> navAttOutMsg;  //!< Attitude nav output message
    Message<SmallBodyNavMsgPayload> smallBodyNavOutMsg;  //!< Small body nav output msg - states and covariances
    Message<EphemerisMsgPayload> asteroidEphemerisOutMsg;  //!< Small body ephemeris output message

    BSKLogger bskLogger;              //!< -- BSK Logging

    double mu_sun;
    Eigen::MatrixXd o_hat_3_tilde; //!< Tilde matrix of the third asteroid orbit frame base vector
    Eigen::Vector3d o_hat_1; //!< First asteroid orbit frame base vector
    Eigen::MatrixXd I; //!< Identity matrix
    double C_SRP; //!< SRP scaling coefficient
    double P_0; //!< SRP at 1 AU
    double rho; //!< Surface reflectivity

    double A_sc;
    double M_sc;
    Eigen::Vector3d IHubPntC_B; // sc inertia
    Eigen::Vector3d IWheelPntC_B; // wheel inertia

    classicElements oe_ast;
    double F_dot;
    double F_ddot;
    Eigen::MatrixXd dcm_ON;
    Eigen::Vector3d r_NO_O;
    Eigen::Vector3d Omega_B;
    Eigen::Vector3d Omega_dot_B;

private:
    NavTransMsgPayload navTransInMsgBuffer;
    NavAttMsgPayload navAttInMsgBuffer;
    EphemerisMsgPayload asteroidEphemerisInMsgBuffer;
    EphemerisMsgPayload sunEphemerisInMsgBuffer;
    std::vector<RWConfigLogMsgPayload> rwConfigLogInMsgBuffer; //!< Buffer for rw speed messages
    std::vector<THROutputMsgPayload> dvThrusterInMsgBuffer; //!< Buffer for rw speed messages
    std::vector<THROutputMsgPayload> attitudeThrusterInMsgBuffer; //!< Buffer for rw speed messages

    uint_64t prevTime;
    unit_64t numStates;
    Eigen::Vector3d dVThrust_B;
    Eigen::Vector3d dVTorque_B;
    Eigen::Vector3d attitudeThrust_B;
    Eigen::Vector3d attitudeTorque_B;
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
