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
#include "architecture/msgPayloadDefC/RWConfigLogMsgPayload.h"
#include "architecture/msgPayloadDefCpp/THROutputMsgPayload.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/utilities/orbitalMotion.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"

/*! @brief This module estimates relative spacecraft position and velocity with respect to the body,attitude and attitude rate of the body wrt. the inertial frame, and the attitude and attituderate of the spacecraft with respect to the inertial frame
 */
class SmallBodyNavEKF: public SysModel {
public:
    SmallBodyNavEKF();
    ~SmallBodyNavEKF();

    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void addThrusterToFilter(Message<THROutputMsgPayload> *tmpThrusterMsg);
    void addRWToFilter(Message<RWConfigLogMsgPayload> *tmpRWMsg);

private:
    void readMessages();
    void writeMessages(uint64_t CurrentSimNanos);
    void predict(uint64_t CurrentSimNanos);
    void aprioriState(uint64_t CurrentSimNanos);
    void aprioriCovar(uint64_t CurrentSimNanos);
    void checkMRPSwitching();
    void computeDynamicsMatrix();
    void measurementUpdate();

public:
    ReadFunctor<NavTransMsgPayload> navTransInMsg;  //!< Translational nav input message
    ReadFunctor<NavAttMsgPayload> navAttInMsg;  //!< Attitude nav input message
    ReadFunctor<EphemerisMsgPayload> asteroidEphemerisInMsg;  //!< Small body ephemeris input message
    ReadFunctor<EphemerisMsgPayload> sunEphemerisInMsg;  //!< Sun ephemeris input message
    std::vector<ReadFunctor<RWConfigLogMsgPayload>> rwInMsgs;  //!< Reaction wheel speed and torque input messages
    std::vector<ReadFunctor<THROutputMsgPayload>> thrusterInMsgs;  //!< thruster input msg vector

    Message<NavTransMsgPayload> navTransOutMsg;  //!< Translational nav output message
    Message<NavAttMsgPayload> navAttOutMsg;  //!< Attitude nav output message
    Message<SmallBodyNavMsgPayload> smallBodyNavOutMsg;  //!< Small body nav output msg - states and covariances
    Message<EphemerisMsgPayload> asteroidEphemerisOutMsg;  //!< Small body ephemeris output message

    BSKLogger bskLogger;  //!< -- BSK Logging

    double C_SRP;  //!< SRP scaling coefficient
    double P_0;  //!< SRP at 1 AU
    double rho;  //!< Surface reflectivity
    double A_sc;  //!< Surface area of the spacecraft
    double M_sc;  //!< Mass of the spacecraft
    Eigen::Matrix3d IHubPntC_B;  //!< sc inertia
    Eigen::Matrix3d IWheelPntC_B;  //!< wheel inertia
    double mu_ast;  //!< Gravitational constant of the asteroid
    Eigen::MatrixXd Q;  //!< Process Noise
    Eigen::MatrixXd R;  //!< Measurement Noise
    Eigen::VectorXd x_hat_k;
    Eigen::MatrixXd P_k;


private:
    NavTransMsgPayload navTransInMsgBuffer;
    NavAttMsgPayload navAttInMsgBuffer;
    EphemerisMsgPayload asteroidEphemerisInMsgBuffer;
    EphemerisMsgPayload sunEphemerisInMsgBuffer;
    std::vector<RWConfigLogMsgPayload> rwConfigLogInMsgBuffer; //!< Buffer for rw speed messages
    std::vector<THROutputMsgPayload> thrusterInMsgBuffer; //!< Buffer for thruster force and torques

    uint64_t prevTime;
    uint64_t numStates;
    Eigen::Vector3d thrust_B;
    Eigen::Vector3d torque_B;
    Eigen::VectorXd x_hat_dot_k;
    Eigen::VectorXd x_hat_k1_;
    Eigen::VectorXd x_hat_k1;
    Eigen::MatrixXd P_dot_k;
    Eigen::MatrixXd P_k1_;
    Eigen::MatrixXd P_k1;
    Eigen::MatrixXd A_k;
    Eigen::MatrixXd L;
    Eigen::MatrixXd M;
    Eigen::MatrixXd H_k1;
    Eigen::MatrixXd I_full;


    double mu_sun;  //!< Gravitational parameter of the sun
    Eigen::Matrix3d o_hat_3_tilde;  //!< Tilde matrix of the third asteroid orbit frame base vector
    Eigen::Vector3d o_hat_1;  //!< First asteroid orbit frame base vector
    Eigen::MatrixXd I;  //!< Identity matrix
    classicElements oe_ast;  //!< Orbital elements of the asteroid
    double F_dot;
    double F_ddot;
    Eigen::Matrix3d dcm_ON;  //!< DCM from the inertial frame to the small-body's hill frame
    Eigen::Vector3d r_SO_O;  //!< Vector from the small body's origin to the inertial frame origin in small-body hill frame components
    Eigen::Vector3d Omega_B;  //!< Speed of the reaction wheels in the spacecraft body frame
    Eigen::Vector3d Omega_dot_B;  //!< Accleration of the reaction wheels in the spacecraft body frame
};


#endif
