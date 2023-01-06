/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "prescribedMotionStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <string>

/*! This is the constructor, setting variables to default values. */
PrescribedMotionStateEffector::PrescribedMotionStateEffector()
{
    // Zero the mass props and mass prop rate contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);

    // Initialize the prescribed variables
    this->r_FM_M.setZero();
    this->rPrime_FM_M.setZero();
    this->rPrimePrime_FM_M.setZero();
    this->omega_FM_F.setZero();
    this->omegaPrime_FM_F.setZero();
    this->sigma_FM.setIdentity();

    // Initialize the other module variables
    this->mass = 0.0;
    this->IPntFc_F.Identity();
    this->r_MB_B.setZero();
    this->r_FcF_F.setZero();
    this->omega_MB_B.setZero();
    this->omegaPrime_MB_B.setZero();
    this->sigma_MB.setIdentity();

    PrescribedMotionStateEffector::effectorID++;
}

uint64_t PrescribedMotionStateEffector::effectorID = 1;

/*! This is the destructor. */
PrescribedMotionStateEffector::~PrescribedMotionStateEffector()
{
    PrescribedMotionStateEffector::effectorID = 1;
}

/*! This method is used to reset the module.
 @return void
 @param CurrentClock [ns] Time the method is called
*/
void PrescribedMotionStateEffector::Reset(uint64_t CurrentClock)
{
    // This method is empty because the modules doesn't have any state to reset
}

/*! This method takes the computed states and outputs them to the messaging system.
 @return void
 @param CurrentClock [ns] Time the method is called
*/
void PrescribedMotionStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{
    // Write the prescribed motion output message if it is linked
    if (this->prescribedMotionOutMsg.isLinked())
    {
        PrescribedMotionMsgPayload prescribedMotionBuffer;
        prescribedMotionBuffer = this->prescribedMotionOutMsg.zeroMsgPayload;
        eigenVector3d2CArray(this->r_FM_M, prescribedMotionBuffer.r_FM_M);
        eigenVector3d2CArray(this->rPrime_FM_M, prescribedMotionBuffer.rPrime_FM_M);
        eigenVector3d2CArray(this->rPrimePrime_FM_M, prescribedMotionBuffer.rPrimePrime_FM_M);
        eigenVector3d2CArray(this->omega_FM_F, prescribedMotionBuffer.omega_FM_F);
        eigenVector3d2CArray(this->omegaPrime_FM_F, prescribedMotionBuffer.omegaPrime_FM_F);
        Eigen::Vector3d sigma_FM_loc;
        sigma_FM_loc = eigenMRPd2Vector3d(this->sigma_FM);
        eigenVector3d2CArray(sigma_FM_loc, prescribedMotionBuffer.sigma_FM);
        this->prescribedMotionOutMsg.write(&prescribedMotionBuffer, this->moduleID, CurrentClock);
    }

    // Write the config log message if it is linked
    if (this->prescribedMotionConfigLogOutMsg.isLinked())
    {
        SCStatesMsgPayload configLogMsg;
        configLogMsg = this->prescribedMotionConfigLogOutMsg.zeroMsgPayload;

        // Note that the configLogMsg B frame represents the effector body frame (frame F)
        eigenVector3d2CArray(this->r_FcN_N, configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_FcN_N, configLogMsg.v_BN_N);
        eigenVector3d2CArray(this->sigma_FN, configLogMsg.sigma_BN);
        eigenVector3d2CArray(this->omega_FN_F, configLogMsg.omega_BN_B);
        this->prescribedMotionConfigLogOutMsg.write(&configLogMsg, this->moduleID, CurrentClock);
    }
}

/*! This method allows the effector to have access to the hub states.
 @return void
 @param statesIn Pointer to give the state effector access the hub states
*/
void PrescribedMotionStateEffector::linkInStates(DynParamManager& statesIn)
{
    // Get access to the hub states needed for dynamic coupling
    this->hubSigma = statesIn.getStateObject("hubSigma");
    this->hubOmega = statesIn.getStateObject("hubOmega");
    this->inertialPositionProperty = statesIn.getPropertyReference("r_BN_N");
    this->inertialVelocityProperty = statesIn.getPropertyReference("v_BN_N");
}

/*! This method allows the state effector to register its states with the dynamic parameter manager. (unused)
 @return void
 @param states Pointer to give the state effector access the hub states
*/
void PrescribedMotionStateEffector::registerStates(DynParamManager& states)
{
    // This method is empty because this module does not register any states
}

/*! This method allows the state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft.
 @return void
 @param integTime [s] Time the method is called
*/
void PrescribedMotionStateEffector::updateEffectorMassProps(double integTime)
{
    // Give the mass of the prescribed body to the effProps mass
    this->effProps.mEff = this->mass;

    // Compute dcm_BM
    this->dcm_BM = this->sigma_MB.toRotationMatrix();

    // Compute dcm_FM
    this->dcm_FM = (this->sigma_FM.toRotationMatrix()).transpose();

    // Compute dcm_BF
    this->dcm_BF = this->dcm_BM * this->dcm_FM.transpose();

    // Compute omega_FB_B given the user inputs omega_MB_M and omega_FM_F
    this->omega_FM_B = this->dcm_BF * this->omega_FM_F;
    this->omega_FB_B = this->omega_FM_B + this->omega_MB_B;

    // Compute omegaPrime_FB_B given the user inputs
    this->omegaTilde_FB_B = eigenTilde(this->omega_FB_B);
    this->omegaPrime_FM_B = this->dcm_BF * this->omegaPrime_FM_F;
    this->omegaPrime_FB_B = this->omegaPrime_FM_B + this->omegaTilde_FB_B * this->omega_FM_B;

    // Convert the prescribed variables to the B frame
    this->r_FM_B = this->dcm_BM * this->r_FM_M;
    this->rPrime_FM_B = this->dcm_BM * this->rPrime_FM_M;
    this->rPrimePrime_FM_B = this->dcm_BM * this->rPrimePrime_FM_M;

    // Compute the effector's CoM with respect to point B
    this->r_FB_B = this->r_FM_B + this->r_MB_B;
    this->r_FcF_B = this->dcm_BF * this->r_FcF_F;
    this->r_FcB_B = this->r_FcF_B + this->r_FB_B;
    this->effProps.rEff_CB_B = this->r_FcB_B;

    // Find the effector inertia about point B
    this->rTilde_FcB_B = eigenTilde(this->r_FcB_B);
    this->IPntFc_B = this->dcm_BF * this->IPntFc_F * this->dcm_BF.transpose();
    this->effProps.IEffPntB_B = this->IPntFc_B - this->mass * this->rTilde_FcB_B * this->rTilde_FcB_B;

    // Find the B frame time derivative of r_FcB_B
    this->omegaTilde_FB_B = eigenTilde(this->omega_FB_B);
    this->rPrime_FcB_B = this->omegaTilde_FB_B * this->r_FcF_B + this->rPrime_FM_B;
    this->effProps.rEffPrime_CB_B = this->rPrime_FcB_B;

    // Find the B frame time derivative of IPntFc_B
    Eigen::Matrix3d rPrimeTilde_FcB_B = eigenTilde(this->rPrime_FcB_B);
    this->effProps.IEffPrimePntB_B = this->omegaTilde_FB_B* this->IPntFc_B - this->IPntFc_B * this->omegaTilde_FB_B
        + this->mass * (rPrimeTilde_FcB_B * this->rTilde_FcB_B.transpose() + this->rTilde_FcB_B * rPrimeTilde_FcB_B.transpose());
}

/*! This method allows the state effector to give its contributions to the matrices needed for the back-sub.
 method
 @return void
 @param integTime [s] Time the method is called
 @param backSubContr State effector contribution matrices for back-substitution
 @param sigma_BN Current B frame attitude with respect to the inertial frame
 @param omega_BN_B [rad/s] Angular velocity of the B frame with respect to the inertial frame, expressed in B frame
 components
 @param g_N [m/s^2] Gravitational acceleration in N frame components
*/
void PrescribedMotionStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{

}

/*! This method is empty because the equations of motion of the effector are prescribed.
 @return void
 @param integTime [s] Time the method is called
 @param rDDot_BN_N [m/s^2] Acceleration of the vector pointing from the inertial frame origin to the B frame origin,
 expressed in inertial frame components
 @param omegaDot_BN_B [rad/s^2] Inertial time derivative of the angular velocity of the B frame with respect to the
 inertial frame, expressed in B frame components
 @param sigma_BN Current B frame attitude with respect to the inertial frame
*/
void PrescribedMotionStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
    // This method is empty because the equations of motion of the effector are prescribed.
}

/*! This method is for calculating the contributions of the effector to the energy and momentum of the spacecraft.
 @return void
 @param integTime [s] Time the method is called
 @param rotAngMomPntCContr_B [kg m^2/s] Contribution of stateEffector to total rotational angular mom
 @param rotEnergyContr [J] Contribution of stateEffector to total rotational energy
 @param omega_BN_B [rad/s] Angular velocity of the B frame with respect to the inertial frame, expressed in B frame
 components
*/
void PrescribedMotionStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{

}

/*! This method computes the effector states relative to the inertial frame.
 @return void
*/
void PrescribedMotionStateEffector::computePrescribedMotionInertialStates()
{

}

/*! This method updates the effector state at the dynamics frequency.
 @return void
 @param CurrentSimNanos [ns] Time the method is called
*/
void PrescribedMotionStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
    // Read the input message if it is linked and written
    if (this->prescribedMotionInMsg.isLinked() && this->prescribedMotionInMsg.isWritten())
    {
        PrescribedMotionMsgPayload incomingPrescribedStates;
        incomingPrescribedStates = this->prescribedMotionInMsg();
        this->r_FM_M = cArray2EigenVector3d(incomingPrescribedStates.r_FM_M);
        this->rPrime_FM_M = cArray2EigenVector3d(incomingPrescribedStates.rPrime_FM_M);
        this->rPrimePrime_FM_M = cArray2EigenVector3d(incomingPrescribedStates.rPrimePrime_FM_M);
        this->omega_FM_F = cArray2EigenVector3d(incomingPrescribedStates.omega_FM_F);
        this->omegaPrime_FM_F = cArray2EigenVector3d(incomingPrescribedStates.omegaPrime_FM_F);
        this->sigma_FM = cArray2EigenVector3d(incomingPrescribedStates.sigma_FM);
    }

    // Call the method to write the output messages
    this->writeOutputStateMessages(CurrentSimNanos);
}
