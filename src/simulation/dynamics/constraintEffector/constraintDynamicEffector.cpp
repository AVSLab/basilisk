/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "constraintDynamicEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include<cmath>
#include<array>
#include<iostream>

/*! This is the constructor, nothing to report here */
ConstraintDynamicEffector::ConstraintDynamicEffector()
{

}

/*! This is the destructor, nothing to report here */
ConstraintDynamicEffector::~ConstraintDynamicEffector()
{

}

/*! This method is used to reset the module.

 */
void ConstraintDynamicEffector::Reset(uint64_t CurrentSimNanos)
{
    // check if any individual gains are not specified
    bool gainset = this->k_d != 0 || this->c_d != 0 || this->k_a != 0 || this->c_a != 0;
    if (this->alpha <= 0 && !gainset) {
        bskLogger.bskLog(BSK_ERROR, "Alpha must be set to a positive nonzero value prior to initialization");
    }
    if (this->beta <= 0 && !gainset) {
        bskLogger.bskLog(BSK_ERROR, "Beta must be set to a positive nonzero value prior to initializaiton");
    }
    // if individual k's or c's are already set, don't use alpha & beta
    if (this->k_d == 0) {
        this->k_d = pow(this->alpha,2);
    }
    if (this->c_d == 0) {
        this->c_d = 2*this->beta;
    }
    if (this->k_a == 0) {
        this->k_a = pow(this->alpha,2);
    }
    if (this->c_a == 0) {
        this->c_a = 2*this->beta;
    }
}

void ConstraintDynamicEffector::setR_P2P1_B1Init(Eigen::Vector3d r_P2P1_B1Init) {
    this->r_P2P1_B1Init = r_P2P1_B1Init;
}

void ConstraintDynamicEffector::setR_P1B1_B1(Eigen::Vector3d r_P1B1_B1) {
    this->r_P1B1_B1 = r_P1B1_B1;
}

void ConstraintDynamicEffector::setR_P2B2_B2(Eigen::Vector3d r_P2B2_B2) {
    this->r_P2B2_B2 = r_P2B2_B2;
}

void ConstraintDynamicEffector::setAlpha(double alpha) {
    if (alpha > 0.0)
        this->alpha = alpha;
    else {
        bskLogger.bskLog(BSK_ERROR, "Proportional gain tuning variable alpha must be greater than 0.");
    }
}

void ConstraintDynamicEffector::setBeta(double beta) {
    if (beta > 0.0)
        this->beta = beta;
    else {
        bskLogger.bskLog(BSK_ERROR, "Derivative gain tuning parameter beta must be greater than 0.");
    }
}

void ConstraintDynamicEffector::setK_d(double k_d) {
    if (k_d > 0.0)
        this->k_d = k_d;
    else {
        bskLogger.bskLog(BSK_ERROR, "Direction constraint proportional gain k_d must be greater than 0.");
    }
}

void ConstraintDynamicEffector::setC_d(double c_d) {
    if (c_d > 0.0)
        this->c_d = c_d;
    else {
        bskLogger.bskLog(BSK_ERROR, "Direction constraint derivative gain c_d must be greater than 0.");
    }
}

void ConstraintDynamicEffector::setK_a(double k_a) {
    if (k_a > 0.0)
        this->k_a = k_a;
    else {
        bskLogger.bskLog(BSK_ERROR, "Attitude constraint proportional gain k_a must be greater than 0.");
    }
}

void ConstraintDynamicEffector::setC_a(double c_a) {
    if (c_a > 0.0)
        this->c_a = c_a;
    else {
        bskLogger.bskLog(BSK_ERROR, "Attitude constraint derivative gain c_a must be greater than 0.");
    }
}

/*! This method allows the user to set the cut-off frequency of the low pass filter which is then used to calculate the coefficients for numerical low pass filtering based on a second-order low pass filter design.

 @param wc The cut-off frequency of the low pass filter.
 @param h The constant digital time step.
 @param k The damping coefficient
*/
void ConstraintDynamicEffector::setFilter_Data(double wc, double h, double k){
    if (wc>0){
    std::array<double,3> num_coeffs = {pow(wc*h,2),2*pow(wc*h,2),pow(wc*h,2)};
    std::array<double,3> denom_coeffs = {-4+4*k*h-pow(wc*h,2),8-2*pow(wc*h,2),4+4*k*h+pow(wc*h,2)};
    this->a = denom_coeffs[1]/denom_coeffs[2];
    this->b = denom_coeffs[0]/denom_coeffs[2];
    this->c = num_coeffs[2]/denom_coeffs[2];
    this->d = num_coeffs[1]/denom_coeffs[2];
    this->e = num_coeffs[0]/denom_coeffs[2];
    }
    else{
        bskLogger.bskLog(BSK_ERROR, "Cut off frequency of low pass filter w_c must be greater than 0.");
    }
}

/*! This method allows the user to set the status of the constraint dynamic effector

*/
void ConstraintDynamicEffector::readInputMessage(){
     if(this->effectorStatusInMsg.isLinked()){
        DeviceStatusMsgPayload statusMsg;
        statusMsg = this->effectorStatusInMsg();
        this->effectorStatus = statusMsg.deviceStatus;
     }
     else{
        this->effectorStatus = 1;
     }
}

/*! This method allows the constraint effector to have access to the parent states

 @param states The states to link
 */
void ConstraintDynamicEffector::linkInStates(DynParamManager& states)
{
    if (this->scInitCounter > 1) {
        bskLogger.bskLog(BSK_ERROR, "constraintDynamicEffector: tried to attach more than 2 spacecraft");
    }

    this->hubSigma.push_back(states.getStateObject("hubSigma"));
	this->hubOmega.push_back(states.getStateObject("hubOmega"));
    this->hubPosition.push_back(states.getStateObject("hubPosition"));
    this->hubVelocity.push_back(states.getStateObject("hubVelocity"));

    this->scInitCounter++;
}

/*! This method computes the forces on torques on each spacecraft body.

 @param integTime Integration time
 @param timeStep Current integration time step used
 */
void ConstraintDynamicEffector::computeForceTorque(double integTime, double timeStep)
{
    if (this->scInitCounter == 2) { // only proceed once both spacecraft are added
        // alternate assigning the constraint force and torque
        if (this->scID == 0) { // compute all forces and torques once, assign to spacecraft 1 and store for spacecraft 2
            // - Collect states from both spacecraft
            Eigen::Vector3d r_B1N_N = this->hubPosition[0]->getState();
            Eigen::Vector3d rDot_B1N_N = this->hubVelocity[0]->getState();
            Eigen::Vector3d omega_B1N_B1 = this->hubOmega[0]->getState();
            Eigen::MRPd sigma_B1N;
            sigma_B1N = (Eigen::Vector3d)this->hubSigma[0]->getState();
            Eigen::Vector3d r_B2N_N = this->hubPosition[1]->getState();
            Eigen::Vector3d rDot_B2N_N = this->hubVelocity[1]->getState();
            Eigen::Vector3d omega_B2N_B2 = this->hubOmega[1]->getState();
            Eigen::MRPd sigma_B2N;
            sigma_B2N = (Eigen::Vector3d)this->hubSigma[1]->getState();

            // computing direction constraint psi in the N frame
            Eigen::Matrix3d dcm_B1N = (sigma_B1N.toRotationMatrix()).transpose();
            Eigen::Matrix3d dcm_B2N = (sigma_B2N.toRotationMatrix()).transpose();
            Eigen::Vector3d r_P1B1_N = dcm_B1N.transpose() * this->r_P1B1_B1;
            Eigen::Vector3d r_P2B2_N = dcm_B2N.transpose() * this->r_P2B2_B2;
            Eigen::Vector3d r_P2P1_N = r_P2B2_N + r_B2N_N - r_P1B1_N - r_B1N_N;

            // computing length constraint rate of change psiPrime in the N frame
            Eigen::Vector3d rDot_P1B1_B1 = omega_B1N_B1.cross(this->r_P1B1_B1);
            Eigen::Vector3d rDot_P2B2_B2 = omega_B2N_B2.cross(this->r_P2B2_B2);
            Eigen::Vector3d rDot_P1N_N = dcm_B1N.transpose() * rDot_P1B1_B1 + rDot_B1N_N;
            Eigen::Vector3d rDot_P2N_N = dcm_B2N.transpose() * rDot_P2B2_B2 + rDot_B2N_N;
            Eigen::Vector3d rDot_P2P1_N = rDot_P2N_N - rDot_P1N_N;
            Eigen::Vector3d omega_B1N_N = dcm_B1N.transpose() * omega_B1N_B1;

            // calculate the direction constraint violations
            this->psi_N = r_P2P1_N - dcm_B1N.transpose() * this->r_P2P1_B1Init;
            this->psiPrime_N = rDot_P2P1_N - omega_B1N_N.cross(r_P2P1_N);

            // calculate the attitude constraint violations
            this->sigma_B2B1 = eigenC2MRP(dcm_B2N * dcm_B1N.transpose()); // calculate the difference in attitude
            Eigen::Vector3d omega_B1N_B2 = dcm_B2N * dcm_B1N.transpose() * omega_B1N_B1;
            this->omega_B2B1_B2 = omega_B2N_B2 - omega_B1N_B2; // difference in angular rate

            // calculate the constraint force
            this->Fc_N = this->k_d * this->psi_N + this->c_d * this->psiPrime_N; // store the constraint force for spacecraft 2
            this->forceExternal_N = this->Fc_N;

            // calculate the torque on each spacecraft from the direction constraint
            Eigen::Vector3d Fc_B1 = dcm_B1N * this->Fc_N;
            Eigen::Vector3d L_B1_len = (this->r_P1B1_B1).cross(Fc_B1);
            Eigen::Vector3d Fc_B2 = dcm_B2N * this->Fc_N;
            Eigen::Vector3d L_B2_len = -this->r_P2B2_B2.cross(Fc_B2);

            // calculate the constraint torque imparted on each spacecraft from the attitude constraint
            Eigen::Matrix3d dcm_B1B2 = dcm_B1N * dcm_B2N.transpose();
            Eigen::Vector3d L_B2_att = -this->k_a * eigenMRPd2Vector3d(sigma_B2B1) - this->c_a * 0.25 * sigma_B2B1.Bmat() * omega_B2B1_B2;
            Eigen::Vector3d L_B1_att = - dcm_B1B2 * L_B2_att;
            this->T_B2 = L_B2_len + L_B2_att; // store the constraint torque for spacecraft 2

            // assign forces and torques for spacecraft 1
            this->forceExternal_N = this->Fc_N;
            this->torqueExternalPntB_B = L_B1_len + L_B1_att;
            this->T_B1 = this->torqueExternalPntB_B;
        }
        else if (this->scID == 1) {
            // assign forces and torques for spacecraft 2
            this->forceExternal_N = - this->Fc_N;
            this->torqueExternalPntB_B = this->T_B2;
        }
        this->scID = (1 + pow(-1,this->scID))/2; // toggle spacecraft to be assigned forces and torques
    }
}

/*! This method takes the computed constraint force and torque states and outputs them to the m
 messaging system.

 @param CurrentClock The current simulation time (used for time stamping)
 */
void ConstraintDynamicEffector::writeOutputStateMessage(uint64_t CurrentClock)
{
    ConstDynEffectorMsgPayload outputForces;
    outputForces = this->constraintElements.zeroMsgPayload;
    eigenVector3d2CArray(this->forceExternal_N,outputForces.Fc_N);
    eigenVector3d2CArray(this->T_B1,outputForces.L1_B1);
    eigenVector3d2CArray(this->T_B2,outputForces.L2_B2);
    eigenVector3d2CArray(this->psi_N,outputForces.psi_N);
    outputForces.Fc_mag_filtered = this->F_filtered_mag_t;
    outputForces.L1_mag_filtered = this->T1_filtered_mag_t;
    outputForces.L2_mag_filtered = this->T2_filtered_mag_t;
    this->constraintElements.write(&outputForces,this->moduleID,CurrentClock);
}

/*! Update state method

 @param CurrentSimNanos The current simulation time
 */
void ConstraintDynamicEffector::UpdateState(uint64_t CurrentSimNanos)
{
    this->readInputMessage();
    if(this->effectorStatus){
        this->computeFilteredForce(CurrentSimNanos);
        this->computeFilteredTorque(CurrentSimNanos);
        this->writeOutputStateMessage(CurrentSimNanos);
    }
}

/*! Filtering method to calculate filtered Constraint Force

 @param CurrentClock The current simulation time (used for time stamping)
 */
void ConstraintDynamicEffector::computeFilteredForce(uint64_t CurrentClock)
{

    double F_t[3];
    eigenVector3d2CArray(this->Fc_N,F_t);
    this->F_mag_t = std::sqrt(pow(F_t[0],2)+pow(F_t[1],2)+pow(F_t[2],2));
    this->F_filtered_mag_t = this->a*this->F_filtered_mag_tminus1 +
    this->b*this->F_filtered_mag_tminus2+this->c*this->F_mag_t+
    this->d*this->F_mag_tminus1+this->e*this->F_mag_tminus2;
    this->F_filtered_mag_tminus2 = this->F_filtered_mag_tminus1;
    this->F_filtered_mag_tminus1 = this->F_filtered_mag_t;
    this->F_mag_tminus2 = this->F_mag_tminus1;
    this->F_mag_tminus1 = this->F_mag_t;
}

/*! Filtering method to calculate filtered Constraint Torque

 @param CurrentClock The current simulation time (used for time stamping)
 */
void ConstraintDynamicEffector::computeFilteredTorque(uint64_t CurrentClock)
{
        double T_t1[3];
        eigenVector3d2CArray(this->T_B1,T_t1);
        this->T1_mag_t = std::sqrt(pow(T_t1[0],2)+pow(T_t1[1],2)+pow(T_t1[2],2));
        this->T1_filtered_mag_t = this->a*this->T1_filtered_mag_tminus1 +
        this->b*this->T1_filtered_mag_tminus2+this->c*this->T1_mag_t+
        this->d*this->T1_mag_tminus1+this->e*this->T1_mag_tminus2;
        this->T1_filtered_mag_tminus2 = this->T1_filtered_mag_tminus1;
        this->T1_filtered_mag_tminus1 = this->T1_filtered_mag_t;
        this->T1_mag_tminus2 = this->T1_mag_tminus1;
        this->T1_mag_tminus1 = this->T1_mag_t;
        double T_t2[3];
        eigenVector3d2CArray(this->T_B2,T_t2);
        this->T2_mag_t = std::sqrt(pow(T_t2[0],2)+pow(T_t2[1],2)+pow(T_t2[2],2));
        this->T2_filtered_mag_t = this->a*this->T2_filtered_mag_tminus1 + this->b*this->T2_filtered_mag_tminus2+this->c*this->T2_mag_t+this->d*this->T2_mag_tminus1+this->e*this->T2_mag_tminus2;
        this->T2_filtered_mag_tminus2 = this->T2_filtered_mag_tminus1;
        this->T2_filtered_mag_tminus1 = this->T2_filtered_mag_t;
        this->T2_mag_tminus2 = this->T2_mag_tminus1;
        this->T2_mag_tminus1 = this->T2_mag_t;
}
