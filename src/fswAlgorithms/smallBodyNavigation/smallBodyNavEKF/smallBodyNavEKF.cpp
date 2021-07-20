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
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include <iostream>
#include <cstring>
#include <math.h>

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
SmallBodyNavEKF::SmallBodyNavEKF()
{
    this->numStates = 18;
    this->mu_sun = 1.327124e20;
    this->o_hat_3_tilde << 0, -1, 0,
                            1, 0, 0,
                            0, 0, 0;
    this->o_hat_1 << 1, 0, 0;
    this->I.setIdentity(3,3);
    this->C_SRP = 1.0;
    this->P_0 = 4.56e-6;
    this->rho = 0.4;
    this->A_k.setIdentity(this->numStates, this->numStates);
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
    // check that required input messages are connected
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
            rwMsg = this->rwInMsgs.at(c);
            this->rwConfigLogInMsgBuffer.push_back(rwMsg);
        }
    } else {
        bskLogger.bskLog(BSK_WARNING, "Small Body Nav EKF has no RW messages to read.");
    }

    /* Read the dV thruster messages */
    THROutputMsgPayload dvThrusterMsg;
    this->dvThrusterInMsgBuffer.clear();
    if (this->dvThrusterInMsgs.size() > 0){
        for (long unsigned int c = 0; c<this->dvThrusterInMsgs.size(); c++){
            dvThrusterMsg = this->dvThrusterInMsgs.at(c);
            this->dvThrusterInMsgBuffer.push_back(dvThrusterMsg);
        }
    } else {
        bskLogger.bskLog(BSK_WARNING, "Small Body Nav EKF has no dV thruster messages to read.");
    }

    /* Read the attitude thruster messages */
    THROutputMsgPayload attitudeThrusterMsg;
    this->attitudeThrusterInMsgBuffer.clear();
    if (this->attitudeThrusterInMsgs.size() > 0){
        for (long unsigned int c = 0; c < this->attitudeThrusterInMsgs.size(); c++){
            attitudeThrusterMsg = this->attitudeThrusterInMsgs.at(c);
            this->attitudeThrusterInMsgBuffer.push_back(attitudeThrusterMsg);
        }
    } else {
        bskLogger.bskLog(BSK_WARNING, "Small Body Nav EKF has no attitude thruster messages to read.");
    }
}

void SmallBodyEKF::predict(){
    /* Get the orbital elements of the asteroid */
    rv2elem(this->mu_sun, *this->asteroidEphemerisInMsg.r_BdyZero_N, *this->asteroidEphemerisInMsg.v_BdyZero_N, *this->oe_ast);

    /* Compute F_dot and F_ddot */
    this->F_dot = sqrt(this->mu_sun/((this->oe.a*(1-oe.e**2))**3))*(1+oe.e*cos(oe.f))**2;
    this->F_ddot -2*oe.e*(sqrt(this->mu_sun/((this->oe.a*(1-oe.e**2))**3)))*sin(oe.f)*(1+oe.e*cos(oe.f))*this->F_dot;

    /* Compute the hill frame of the small body */
    double dcm_ON_array[3][3];
    hillFrame(*this->asteroidEphemerisInMsg.r_BdyZero_N, *this->asteroidEphemerisInMsg.v_BdyZero_N, dcm_ON_array);
    this->dcm_ON = cArray2EigenMatrixXd(*dcm_ON_array, 3, 3);

    /* Compute the direction of the sun from the asteroid in the small body's hill frame */
    Eigen::Vector3d r_ON_N;
    r_ON_N = cArray2EigenVector3d(*this->asteroidEphemerisInMsg.r_BdyZero_N)
    this->r_NO_O = -this->dcm_ON*r_ON_N;

    /* Extract the speed and acceleration of the reaction wheels */
    /* Assumes the RW spin axis is aligned with the body-frame axes*/
    for (long unsigned int c = 0; c < this->rwConfigLogInMsgBuffer.size(); c++){
        this->Omega_B(c) = this->rwConfigLogInMsgBuffer[c].Omega;
        this->Omega_dot_B(c) = this->rwConfigLogInMsgBuffer[c].u_current/this->rwConfigLogInMsgBuffer[c].Js;
    }

    /* Compute the DCM of the orbit frame with respect to the body frame */
    /*
    double dcm_BN_meas[3][3];
    MRP2C(*this->navAttInMsgBuffer.sigma_BN, dcm_BN_meas);
    Eigen::Matrix3d dcm_OB;
    dcm_OB = this->dcm_ON*(cArray2EigenMatrixXd(*dcm_BN_meas, 3, 3).transpose());
     */

    /* Compute the total thrust and torque from the dV thrusters */
    for (long unsigned int c = 0; c < this->dvThrusterInMsgBuffer.size(); c++) {
        this->dVThrust_B += cArray2EigenVector3d(*this->dvThrusterInMsgBuffer[c].thrustForce_B);
        this->dVTorque_B += cArray2EigenVector3d(*this->dvThrusterInMsgBuffer[c].thrustTorquePntB_B);
    }

    /* Compute the total thrust and torque from the attitude thrusters */
    for (long unsigned int c = 0; c < this->attitudeThrusterInMsgBuffer.size(); c++) {
        this->attitudeThrust_B += cArray2EigenVector3d(*this->attitudeThrusterInMsgBuffer[c].thrustForce_B);
        this->attitudeTorque_B += cArray2EigenVector3d(*this->attitudeThrusterInMsgBuffer[c].thrustTorquePntB_B);
    }

    /* Compute aprior state estimate */
    this->aprioriState();

    /* Compute apriori covariance */
    this->aprioriCovar();

    /* Update the state dynamics matrix, A, for the next iteration */
    this->computeDynamicsMatrix();
}

void SmallBodyEKF::aprioriState(){

}

void SmallBodyEKF::aprioriCovar(){

}

void SmallBodyEKF::computeDynamicsMatrix(){

}

void SmallBodyEKF::measurementUpdate(){

}


/*! This is the main method that gets called every time the module is updated.  Provide an appropriate description.
    @return void
*/
void SmallBodyNavEKF::UpdateState(uint64_t CurrentSimNanos)
{
    this->readMessages();
    this->predict();
    this->measurementUpdate();
    this->writeMessages();
    this->prevTime = currentSimNanos;
}


void SmallBodyNavEKF::writeMessages(){
    // always zero the output message buffers before assigning values
    navTransOutMsgBuffer = this->navTransOutMsg.zeroMsgPayload;
    navAttOutMsgBuffer = this->navAttOutMsg.zeroMsgPayload;
    smallBodyNavOutMsgBuffer = this->smallBodyNavOutMsg.zeroMsgPayload;
    asteroidEphemerisOutMsgBuffer = this->asteroidEphemerisOutMsg.zeroMsgPayload;

    // write to the output messages
    this->navTransOutMsg.write(&navTransOutMsgBuffer, this->moduleID, CurrentSimNanos);
    this->navAttOutMsg.write(&navAttOutMsgBuffer, this->moduleID, CurrentSimNanos);
    this->smallBodyNavOutMsg.write(&smallBodyNavOutMsgBuffer, this->moduleID, CurrentSimNanos);
    this->asteroidEphemerisOutMsg.write(&asteroidEphemerisOutMsgBuffer, this->moduleID, CurrentSimNanos);
}

