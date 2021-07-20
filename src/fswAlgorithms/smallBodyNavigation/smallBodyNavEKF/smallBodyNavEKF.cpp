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


#include "fswAlgorithms/smallBodyNavigation/smallBodyNavEKF/smallBodyNavEKF.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <iostream>
#include <cstring>
#include <math.h>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
SmallBodyNavEKF::SmallBodyNavEKF()
{
    this->numStates = 18;
    this->mu_sun = 1.327124e20;
    this->o_hat_3_tilde.setZero();
    this->o_hat_3_tilde(0, 1) = -1;
    this->o_hat_3_tilde(1, 0) = 1;
    this->o_hat_1 << 1, 0, 0;
    this->I.setIdentity(3,3);
    this->I_full.setIdentity(this->numStates, this->numStates);
    this->C_SRP = 1.0;
    this->P_0 = 4.56e-6;
    this->rho = 0.4;
    this->x_hat_dot_k.setZero(this->numStates);
    this->x_hat_k1_.setZero(this->numStates);
    this->x_hat_k1.setZero(this->numStates);
    this->x_hat_k.setZero(this->numStates);
    this->P_dot_k.setZero(this->numStates, this->numStates);
    this->P_k1_.setZero(this->numStates, this->numStates);
    this->P_k1.setZero(this->numStates, this->numStates);
    this->A_k.setIdentity(this->numStates, this->numStates);
    this->L.setIdentity(this->numStates, this->numStates);
    this->M.setIdentity(this->numStates, this->numStates);
    this->H_k1.setIdentity(this->numStates, this->numStates);
    this->prevTime = 0.0;
    return;
}

/*! Module Destructor */
SmallBodyNavEKF::~SmallBodyNavEKF()
{
}

/*! This method is used to reset the module and checks that required input messages are connect.
    @return void
*/
void SmallBodyNavEKF::Reset(uint64_t CurrentSimNanos)
{
    /* check that required input messages are connected */
    if (!this->navTransInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "SmallBodyNavEKF.navTransInMsg was not linked.");
    }
    if (!this->navAttInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "SmallBodyNavEKF.navAttInMsg was not linked.");
    }
    if (!this->asteroidEphemerisInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "SmallBodyNavEKF.asteroidEphemerisInMsg was not linked.");
    }
    if (!this->sunEphemerisInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "SmallBodyNavEKF.sunEphemerisInMsg was not linked.");
    }

}

/*! This method is used to add a thruster to the filter.
    @return void
*/
void SmallBodyNavEKF::addThrusterToFilter(Message<THROutputMsgPayload> *tmpThrusterMsg){
    this->thrusterInMsgs.push_back(tmpThrusterMsg->addSubscriber());
    return;
}

/*! This method is used to add a rw to the filter.
    @return void
*/
void SmallBodyNavEKF::addRWToFilter(Message<RWConfigLogMsgPayload> *tmpRWMsg){
    this->rwInMsgs.push_back(tmpRWMsg->addSubscriber());
    return;
}

/*! This method is used to read the input messages.
    @return void
*/
void SmallBodyNavEKF::readMessages(){
    /* Read in the input messages */
    this->navTransInMsgBuffer = this->navTransInMsg();
    this->navAttInMsgBuffer = this->navAttInMsg();
    this->asteroidEphemerisInMsgBuffer = this->asteroidEphemerisInMsg();
    this->sunEphemerisInMsgBuffer = this->sunEphemerisInMsg();

    /* Read the RW messages */
    RWConfigLogMsgPayload rwMsg;
    this->rwConfigLogInMsgBuffer.clear();
    if (this->rwInMsgs.size() > 0){
        for (long unsigned int c = 0; c<this->rwInMsgs.size(); c++){
            rwMsg = this->rwInMsgs.at(c)();
            this->rwConfigLogInMsgBuffer.push_back(rwMsg);
        }
    } else {
        bskLogger.bskLog(BSK_WARNING, "Small Body Nav EKF has no RW messages to read.");
    }

    /* Read the thruster messages */
    THROutputMsgPayload thrusterMsg;
    this->thrusterInMsgBuffer.clear();
    if (this->thrusterInMsgs.size() > 0){
        for (long unsigned int c = 0; c<this->thrusterInMsgs.size(); c++){
            thrusterMsg = this->thrusterInMsgs.at(c)();
            this->thrusterInMsgBuffer.push_back(thrusterMsg);
        }
    } else {
        bskLogger.bskLog(BSK_WARNING, "Small Body Nav EKF has no thruster messages to read.");
    }
}

/*! This method performs the KF prediction step
    @param CurrentSimNanos
    @return void
*/
void SmallBodyNavEKF::predict(uint64_t CurrentSimNanos){
    /* Get the orbital elements of the asteroid, we assume the uncertainty on the pos. and vel. of the body are low
     * enough to consider them known apriori */
    rv2elem(mu_sun, asteroidEphemerisInMsgBuffer.r_BdyZero_N, asteroidEphemerisInMsgBuffer.v_BdyZero_N, &oe_ast);

    /* Compute F_dot and F_ddot */
    F_dot = sqrt(mu_sun/(pow(oe_ast.a*(1-pow(oe_ast.e, 2)), 3)))*pow(1+(oe_ast.e)*cos(oe_ast.f), 2);
    F_ddot = -2*(oe_ast.e)*(sqrt(mu_sun/(pow(oe_ast.a*(1-pow(oe_ast.e, 2)), 3))))*sin(oe_ast.f)*(1+oe_ast.e*cos(oe_ast.f))*(F_dot);

    /* Compute the hill frame DCM of the small body */
    double dcm_ON_array[3][3];
    hillFrame(asteroidEphemerisInMsgBuffer.r_BdyZero_N, asteroidEphemerisInMsgBuffer.v_BdyZero_N, dcm_ON_array);
    dcm_ON = cArray2EigenMatrixXd(*dcm_ON_array, 3, 3).transpose();

    /* Compute the direction of the sun from the asteroid in the small body's hill frame, assumes heliocentric frame
     * centered at the origin of the sun, not the solar system's barycenter*/
    Eigen::Vector3d r_ON_N;  // inertial to small body pos. vector
    Eigen::Vector3d r_SN_N;  // inertial to sun pos. vector
    r_ON_N = cArray2EigenVector3d(asteroidEphemerisInMsgBuffer.r_BdyZero_N);
    r_SN_N = cArray2EigenVector3d(sunEphemerisInMsgBuffer.r_BdyZero_N);
    r_SO_O = dcm_ON*(r_SN_N - r_ON_N);  // small body to sun pos vector

    /* Extract the speed and acceleration of the reaction wheels */
    /* Assumes the RW spin axis is aligned with the body-frame axes*/
    Omega_B.setZero(); // Set to zero in case no rws are used
    Omega_dot_B.setZero(); // Set to zero in case no rws are used
    for (long unsigned int c = 0; c < rwConfigLogInMsgBuffer.size(); c++){
        Omega_B(c) = rwConfigLogInMsgBuffer[c].Omega;
        Omega_dot_B(c) = rwConfigLogInMsgBuffer[c].u_current/rwConfigLogInMsgBuffer[c].Js;
    }

    /* Compute the total thrust and torque from the thrusters */
    thrust_B.setZero(); // Set to zero in case no thrusters are used
    torque_B.setZero(); // Set to zero in case no thrusters are used
    for (long unsigned int c = 0; c < thrusterInMsgBuffer.size(); c++) {
        thrust_B += cArray2EigenVector3d(thrusterInMsgBuffer[c].thrustForce_B);
        torque_B += cArray2EigenVector3d(thrusterInMsgBuffer[c].thrustTorquePntB_B);
    }

    /* Compute aprior state estimate */
    aprioriState(CurrentSimNanos);

    /* Compute apriori covariance */
    aprioriCovar(CurrentSimNanos);
}

/*! This method computes the apriori state estimate using euler integration
    @param CurrentSimNanos
    @return void
*/
void SmallBodyNavEKF::aprioriState(uint64_t CurrentSimNanos){
    /* Create temporary state vectors for readability */
    Eigen::Vector3d x_1;
    Eigen::Vector3d x_2;
    Eigen::Vector3d x_3;
    Eigen::Vector3d x_4;
    Eigen::Vector3d x_5;
    Eigen::Vector3d x_6;

    x_1 << x_hat_k.segment(0,3);
    x_2 << x_hat_k.segment(3,3);
    x_3 << x_hat_k.segment(6,3);
    x_4 << x_hat_k.segment(9,3);
    x_5 << x_hat_k.segment(12,3);
    x_6 << x_hat_k.segment(15,3);

    /* x1_dot */
    x_hat_dot_k.segment(0,3) = x_2;

    /* x2_dot */
    /* First compute dcm_OB, DCM from sc body-frame to orbit frame*/
    double sigma_BN_array[3];
    eigenVector3d2CArray(x_5, sigma_BN_array);
    double dcm_BN_meas[3][3];
    MRP2C(sigma_BN_array, dcm_BN_meas);
    Eigen::Matrix3d dcm_OB;
    dcm_OB = dcm_ON*(cArray2EigenMatrixXd(*dcm_BN_meas, 3, 3).transpose());
    /* Now compute x2_dot */
    x_hat_dot_k.segment(3,3) =
            -F_ddot*o_hat_3_tilde*x_1 - 2*F_dot*o_hat_3_tilde*x_2 -pow(F_dot,2)*o_hat_3_tilde*o_hat_3_tilde*x_1
            - mu_ast*x_1/pow(x_1.norm(), 3)
            + mu_sun*(3*(r_SO_O/r_SO_O.norm())*(r_SO_O/r_SO_O.norm()).transpose()-I)*x_1/pow(r_SO_O.norm(), 3)
            + C_SRP*P_0*(1+rho)*(A_sc/M_sc)*o_hat_1/pow(r_SO_O.norm(), 2)
            + dcm_OB*thrust_B;

    /* x3_dot */
    x_hat_dot_k.segment(6,3) = 0.25*((1-pow(x_3.norm(),2))*I + 2*eigenTilde(x_3) + 2*x_3*x_3.transpose())*x_4;

    /* x4_dot */
    x_hat_dot_k.segment(9,3) << 0, 0, 0;

    /* x5_dot */
    x_hat_dot_k.segment(12,3) = 0.25*((1-pow(x_5.norm(),2))*I + 2*eigenTilde(x_5) + 2*x_5*x_5.transpose())*x_6;

    /* x6_dot */
    x_hat_dot_k.segment(15,3) = -((IHubPntC_B+IWheelPntC_B).inverse())*(eigenTilde(x_6)*(IHubPntC_B+IWheelPntC_B)*x_6
            + IWheelPntC_B*Omega_dot_B + eigenTilde(x_6)*IWheelPntC_B*Omega_B - torque_B);

    Eigen::Vector3d term;
    term = (IHubPntC_B+IWheelPntC_B).inverse()*(eigenTilde(x_6)*(IHubPntC_B+IWheelPntC_B)*x_6 + IWheelPntC_B*Omega_dot_B);

    /* Propagate the state using euler integration */
    x_hat_k1_ = x_hat_k + x_hat_dot_k*(CurrentSimNanos-prevTime)*NANO2SEC;
}

/*! This method compute the apriori estimation error covariance through euler integration
    @param CurrentSimNanos
    @return void
*/
void SmallBodyNavEKF::aprioriCovar(uint64_t CurrentSimNanos){
    /* Compute P_dot */
    P_dot_k = A_k*P_k + P_k*(A_k.transpose()) + L*Q*L.transpose();

    /* Compute the apriori covariance using euler integration */
    P_k1_ = P_k + P_dot_k*(CurrentSimNanos-prevTime)*NANO2SEC;
}

/*! This method checks the propagated MRP states to see if they exceed a norm of 1. If they do, the appropriate
    states are transferred to the shadow set and the covariance is updated.
    @return void
 */
void SmallBodyNavEKF::checkMRPSwitching(){
    /* Create temporary values for sigma_BN, sigma_AN */
    Eigen::Vector3d sigma_AN;
    Eigen::Vector3d sigma_BN;
    sigma_AN << x_hat_k1_.segment(6,3);
    sigma_BN << x_hat_k1_.segment(12,3);

    /* Create a shadow covariance matrix */
    Eigen::MatrixXd P_k1_s;
    P_k1_s.setZero(this->numStates, this->numStates);

    /* Initialize Lambda, set it to zero, set diagonal 3x3 state blocks to identity */
    Eigen::MatrixXd Lambda;
    Lambda.setZero(this->numStates, this->numStates);
    Lambda.block(0, 0, 3, 3).setIdentity();
    Lambda.block(3, 3, 3, 3).setIdentity();
    Lambda.block(6, 6, 3, 3).setIdentity();
    Lambda.block(9, 9, 3, 3).setIdentity();
    Lambda.block(12, 12, 3, 3).setIdentity();
    Lambda.block(15, 15, 3, 3).setIdentity();

    /* Check the attitude of the small body */
    if (sigma_AN.norm() > 1.0){
        /* Switch MRPs */
        x_hat_k1_.segment(6, 3) = -sigma_AN/pow(sigma_AN.norm(), 2);
        /* Populate lambda block */
        Lambda.block(6, 6, 3, 3) = 2*sigma_AN*sigma_AN.transpose()/pow(sigma_AN.norm(), 4) - I/pow(sigma_AN.norm(), 2);
    }

    /* Check the attitude of the spacecraft */
    if (sigma_BN.norm() > 1.0){
        /* Switch MRPs */
        x_hat_k1_.segment(12, 3) = -sigma_BN/pow(sigma_BN.norm(), 2);
        /* Populate lambda block */
        Lambda.block(12, 12, 3, 3) = 2*sigma_BN*sigma_BN.transpose()/pow(sigma_BN.norm(), 4) - I/pow(sigma_BN.norm(), 2);
    }

    /* Compute the new apriori covariance */
    P_k1_s = Lambda*P_k1_*Lambda.transpose();
    P_k1_ = P_k1_s;
}


/*! This method performs the KF measurement update step
    @return void
*/
void SmallBodyNavEKF::measurementUpdate(){
    /* Compute Kalman gain */
    Eigen::MatrixXd K_k1;
    K_k1.setZero(this->numStates, this->numStates);
    K_k1 = P_k1_*H_k1.transpose() * (H_k1*P_k1_*H_k1.transpose() + M*R*M.transpose()).inverse();

    /* Grab the measurements from the input messages */
    /* Subtract the asteroid position from the spacecraft position and rotate it into the small body's hill frame*/
    Eigen::VectorXd y_k1;
    y_k1.setZero(this->numStates);
    y_k1.segment(0, 3) = dcm_ON*(cArray2EigenVector3d(navTransInMsgBuffer.r_BN_N)
            - cArray2EigenVector3d(asteroidEphemerisInMsgBuffer.r_BdyZero_N));

    /* Perform a similar operation for the relative velocity */
    y_k1.segment(3, 3) = dcm_ON*(cArray2EigenVector3d(navTransInMsgBuffer.v_BN_N)
            - cArray2EigenVector3d(asteroidEphemerisInMsgBuffer.v_BdyZero_N));

    /* Small body attitude from the ephemeris msg */
    y_k1.segment(6, 3) =  cArray2EigenVector3d(asteroidEphemerisInMsgBuffer.sigma_BN);

    /* Check if the shadow set measurement must be considered, i.e. |sigma| > 1/3 */
    if (y_k1.segment(6, 3).norm() > 1.0/3.0) {
        /* Create a temporary shadow-set MRP representation */
        Eigen::Vector3d sigma_AN_s = -y_k1.segment(6, 3)/pow(y_k1.segment(6, 3).norm(), 2);
        /* Check to see if the shadow set gives a smaller residual */
        if ((sigma_AN_s - x_hat_k1_.segment(6, 3)).norm() < (y_k1.segment(6, 3) - x_hat_k1_.segment(6, 3)).norm()){
            y_k1.segment(6, 3) = sigma_AN_s;
        }
    }

    /* Small body attitude rate from the ephemeris msg */
    y_k1.segment(9, 3) = cArray2EigenVector3d(asteroidEphemerisInMsgBuffer.omega_BN_B);

    /* Spacecraft attitude from the navAttMsg*/
    y_k1.segment(12, 3) = cArray2EigenVector3d(navAttInMsgBuffer.sigma_BN);

    /* Check if the shadow set measurement must be considered, i.e. |sigma| > 1/3 */
    if (y_k1.segment(12, 3).norm() > 1.0/3.0) {
        /* Create a temporary shadow-set MRP representation */
        Eigen::Vector3d sigma_BN_s = -y_k1.segment(12, 3)/pow(y_k1.segment(12, 3).norm(), 2);
        /* Check to see if the shadow set gives a smaller residual */
        if ((sigma_BN_s - x_hat_k1_.segment(12, 3)).norm() < (y_k1.segment(12, 3) - x_hat_k1_.segment(12, 3)).norm()){
            y_k1.segment(12, 3) = sigma_BN_s;
        }
    }

    /* Spacecraft rate from the navAttMsg*/
    y_k1.segment(15, 3) = cArray2EigenVector3d(navAttInMsgBuffer.omega_BN_B);

    /* Update the state estimate */
    x_hat_k1 = x_hat_k1_ + K_k1*(y_k1 - x_hat_k1_);

    /* Update the covariance */
    P_k1 = (I_full - K_k1*H_k1)*P_k1_*(I_full - K_k1*H_k1).transpose() + K_k1*M*R*M.transpose()*K_k1.transpose();

    /* Assign the state estimate and covariance to k for the next iteration */
    x_hat_k = x_hat_k1;
    P_k = P_k1;

    /* Update the state dynamics matrix, A, for the next iteration */
    computeDynamicsMatrix();
}

/*! This method computes the state dynamics matrix, A, for the next iteration
    @return void
*/
void SmallBodyNavEKF::computeDynamicsMatrix(){
    /* Create temporary state vectors for readability */
    Eigen::Vector3d x_1;
    Eigen::Vector3d x_2;
    Eigen::Vector3d x_3;
    Eigen::Vector3d x_4;
    Eigen::Vector3d x_5;
    Eigen::Vector3d x_6;

    x_1 << x_hat_k.segment(0,3);
    x_2 << x_hat_k.segment(3,3);
    x_3 << x_hat_k.segment(6,3);
    x_4 << x_hat_k.segment(9,3);
    x_5 << x_hat_k.segment(12,3);
    x_6 << x_hat_k.segment(15,3);

    /* First set the matrix to zero (many indices are zero) */
    A_k.setZero(this->numStates, this->numStates);

    /* x_1 partial */
    A_k.block(0, 3, 3, 3).setIdentity();

    /* x_2 partial */
    A_k.block(3, 0, 3, 3) =
            - F_ddot*o_hat_3_tilde
            - pow(F_dot, 2)*o_hat_3_tilde*o_hat_3_tilde
            - mu_ast/pow(x_1.norm(), 3)*I
            + 3*mu_ast*x_1*x_1.transpose()/pow(x_1.norm(), 5)
            + mu_sun*(3*r_SO_O*r_SO_O.transpose() - I)/pow(r_SO_O.norm(), 3);

    A_k.block(3, 3, 3, 3) = -2*F_dot*o_hat_3_tilde;

    /* x_3 partial */
    A_k.block(6, 6, 3, 3) = 0.5*(x_3*x_4.transpose() - x_4*x_3.transpose() - eigenTilde(x_4) + (x_4.transpose()*x_3)*I);
    A_k.block(6, 9, 3, 3) = 0.25*((1-pow(x_3.norm(), 2))*I + 2*eigenTilde(x_3) + 3*x_3*x_3.transpose());

    /* x_5 partial, skipping x_4 partial as it is zero */
    A_k.block(12, 12, 3, 3) = 0.5*(x_5*x_6.transpose() - x_6*x_5.transpose() - eigenTilde(x_6) + (x_6.transpose()*x_5)*I);
    A_k.block(12, 15, 3, 3) = 0.25*((1-pow(x_5.norm(), 2))*I + 2*eigenTilde(x_5) + 3*x_5*x_5.transpose());

    /* x_6 partial */
    A_k.block(15, 15, 3, 3) = -(IHubPntC_B+IWheelPntC_B).inverse() * (eigenTilde(x_6)*(IHubPntC_B+IWheelPntC_B)
            - eigenTilde((IHubPntC_B+IWheelPntC_B)*x_6) - eigenTilde(IWheelPntC_B*Omega_B));
}

/*! This is the main method that gets called every time the module is updated.
    @return void
*/
void SmallBodyNavEKF::UpdateState(uint64_t CurrentSimNanos)
{
    this->readMessages();
    this->predict(CurrentSimNanos);
    this->checkMRPSwitching();
    this->measurementUpdate();
    this->writeMessages(CurrentSimNanos);
    prevTime = CurrentSimNanos;
}

/*! This method writes the output messages
    @return void
*/
void SmallBodyNavEKF::writeMessages(uint64_t CurrentSimNanos){
    /* Create output msg buffers */
    NavTransMsgPayload navTransOutMsgBuffer;
    NavAttMsgPayload navAttOutMsgBuffer;
    SmallBodyNavMsgPayload smallBodyNavOutMsgBuffer;
    EphemerisMsgPayload asteroidEphemerisOutMsgBuffer;

    /* Zero the output message buffers before assigning values */
    navTransOutMsgBuffer = this->navTransOutMsg.zeroMsgPayload;
    navAttOutMsgBuffer = this->navAttOutMsg.zeroMsgPayload;
    smallBodyNavOutMsgBuffer = this->smallBodyNavOutMsg.zeroMsgPayload;
    asteroidEphemerisOutMsgBuffer = this->asteroidEphemerisOutMsg.zeroMsgPayload;

    /* Assign values to the nav trans output message */
    navTransOutMsgBuffer.timeTag = navTransInMsgBuffer.timeTag;
    eigenMatrixXd2CArray(cArray2EigenVector3d(asteroidEphemerisInMsgBuffer.r_BdyZero_N) + dcm_ON.transpose()*x_hat_k1.segment(0,3), navTransOutMsgBuffer.r_BN_N);
    eigenMatrixXd2CArray(cArray2EigenVector3d(asteroidEphemerisInMsgBuffer.v_BdyZero_N) + dcm_ON.transpose()*x_hat_k1.segment(3,3), navTransOutMsgBuffer.v_BN_N);
    v3Copy(navTransOutMsgBuffer.vehAccumDV, navTransInMsgBuffer.vehAccumDV);  // Not an estimated parameter, pass through

    /* Assign values to the nav att output message */
    navAttOutMsgBuffer.timeTag = navAttInMsgBuffer.timeTag;
    eigenMatrixXd2CArray(x_hat_k1.segment(12,3), navAttOutMsgBuffer.sigma_BN);
    eigenMatrixXd2CArray(x_hat_k1.segment(15,3), navAttOutMsgBuffer.omega_BN_B);
    v3Copy(navAttOutMsgBuffer.vehSunPntBdy, navAttInMsgBuffer.vehSunPntBdy); // Not an estimated parameter, pass through

    /* Assign values to the asteroid ephemeris output message */
    v3Copy(asteroidEphemerisOutMsgBuffer.r_BdyZero_N, asteroidEphemerisInMsgBuffer.r_BdyZero_N);  // Not an estimated parameter
    v3Copy(asteroidEphemerisOutMsgBuffer.v_BdyZero_N, asteroidEphemerisInMsgBuffer.v_BdyZero_N);  // Not an estimated parameter
    eigenMatrixXd2CArray(x_hat_k1.segment(6,3), asteroidEphemerisOutMsgBuffer.sigma_BN);
    eigenMatrixXd2CArray(x_hat_k1.segment(9,3), asteroidEphemerisOutMsgBuffer.omega_BN_B);
    asteroidEphemerisOutMsgBuffer.timeTag = asteroidEphemerisInMsgBuffer.timeTag;

    /* Assign values to the small body navigation output message */
    eigenMatrixXd2CArray(x_hat_k1, smallBodyNavOutMsgBuffer.state);
    eigenMatrixXd2CArray(P_k1, *smallBodyNavOutMsgBuffer.covar);

    /* Write to the output messages */
    this->navTransOutMsg.write(&navTransOutMsgBuffer, this->moduleID, CurrentSimNanos);
    this->navAttOutMsg.write(&navAttOutMsgBuffer, this->moduleID, CurrentSimNanos);
    this->smallBodyNavOutMsg.write(&smallBodyNavOutMsgBuffer, this->moduleID, CurrentSimNanos);
    this->asteroidEphemerisOutMsg.write(&asteroidEphemerisOutMsgBuffer, this->moduleID, CurrentSimNanos);
}

