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
#include "architecture/utilities/macroDefinitions.h"
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
    this->r_PM_M.setZero();
    this->rPrime_PM_M.setZero();
    this->rPrimePrime_PM_M.setZero();
    this->omega_PM_P.setZero();
    this->omegaPrime_PM_P.setZero();
    this->sigma_PM.setIdentity();

    // Initialize the other module variables
    this->mass = 0.0;
    this->IPntPc_P.setIdentity();
    this->r_MB_B.setZero();
    this->r_PcP_P.setZero();
    this->omega_MB_B.setZero();
    this->omegaPrime_MB_B.setZero();
    this->sigma_MB.setIdentity();
    this->currentSimTimeSec = 0.0;

    // Initialize prescribed states at epoch
    this->rEpoch_PM_M.setZero();
    this->rPrimeEpoch_PM_M.setZero();
    this->omegaEpoch_PM_P.setZero();

    // Set the sigma_PM state name
    this->nameOfsigma_PMState = "prescribedMotionsigma_PM" + std::to_string(this->effectorID);

    PrescribedMotionStateEffector::effectorID++;
}

uint64_t PrescribedMotionStateEffector::effectorID = 1;

/*! This is the destructor. */
PrescribedMotionStateEffector::~PrescribedMotionStateEffector()
{
    PrescribedMotionStateEffector::effectorID = 1;
}

/*! This method is used to reset the module.

 @param currentClock [ns] Time the method is called
*/
void PrescribedMotionStateEffector::Reset(uint64_t currentClock)
{
}

/*! This method takes the computed states and outputs them to the messaging system.

 @param currentClock [ns] Time the method is called
*/
void PrescribedMotionStateEffector::writeOutputStateMessages(uint64_t currentClock)
{

    // Write the prescribed translational motion output message if it is linked
    if (this->prescribedTranslationOutMsg.isLinked())
    {
        PrescribedTranslationMsgPayload prescribedTranslationBuffer = this->prescribedTranslationOutMsg.zeroMsgPayload;
        eigenVector3d2CArray(this->r_PM_M, prescribedTranslationBuffer.r_PM_M);
        eigenVector3d2CArray(this->rPrime_PM_M, prescribedTranslationBuffer.rPrime_PM_M);
        eigenVector3d2CArray(this->rPrimePrime_PM_M, prescribedTranslationBuffer.rPrimePrime_PM_M);
        this->prescribedTranslationOutMsg.write(&prescribedTranslationBuffer, this->moduleID, currentClock);
    }

    // Write the prescribed rotational motion output message if it is linked
    if (this->prescribedRotationOutMsg.isLinked())
    {
        PrescribedRotationMsgPayload prescribedRotationBuffer = this->prescribedRotationOutMsg.zeroMsgPayload;
        eigenVector3d2CArray(this->omega_PM_P, prescribedRotationBuffer.omega_PM_P);
        eigenVector3d2CArray(this->omegaPrime_PM_P, prescribedRotationBuffer.omegaPrime_PM_P);
        Eigen::Vector3d sigma_PM_loc = eigenMRPd2Vector3d(this->sigma_PM);
        eigenVector3d2CArray(sigma_PM_loc, prescribedRotationBuffer.sigma_PM);
        this->prescribedRotationOutMsg.write(&prescribedRotationBuffer, this->moduleID, currentClock);
    }

    // Write the config log message if it is linked
    if (this->prescribedMotionConfigLogOutMsg.isLinked())
    {
        SCStatesMsgPayload configLogMsg = this->prescribedMotionConfigLogOutMsg.zeroMsgPayload;

        // Note that the configLogMsg B frame represents the effector body frame (frame P)
        eigenVector3d2CArray(this->r_PcN_N, configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_PcN_N, configLogMsg.v_BN_N);
        eigenVector3d2CArray(this->sigma_PN, configLogMsg.sigma_BN);
        eigenVector3d2CArray(this->omega_PN_P, configLogMsg.omega_BN_B);
        this->prescribedMotionConfigLogOutMsg.write(&configLogMsg, this->moduleID, currentClock);
    }
}

/*! This method allows the effector to have access to the hub states.

 @param statesIn Pointer to give the state effector access the hub states
*/
void PrescribedMotionStateEffector::linkInStates(DynParamManager& statesIn)
{
    // Get access to the hub states needed for dynamic coupling
    this->inertialPositionProperty = statesIn.getPropertyReference(this->propName_inertialPosition);
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->propName_inertialVelocity);
}

/*! This method allows the state effector to register its states with the dynamic parameter manager.

 @param states Pointer to give the state effector access the hub states
*/
void PrescribedMotionStateEffector::registerStates(DynParamManager& states)
{
        this->sigma_PMState = states.registerState(3, 1, this->nameOfsigma_PMState);
        Eigen::Vector3d sigma_PM_loc = eigenMRPd2Vector3d(this->sigma_PM);
        Eigen::Vector3d sigma_PMInitMatrix;
        sigma_PMInitMatrix(0) = sigma_PM_loc[0];
        sigma_PMInitMatrix(1) = sigma_PM_loc[1];
        sigma_PMInitMatrix(2) = sigma_PM_loc[2];
        this->sigma_PMState->setState(sigma_PMInitMatrix);
}

/*! This method allows the state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft.

 @param integTime [s] Time the method is called
*/
void PrescribedMotionStateEffector::updateEffectorMassProps(double integTime)
{
    // Update the prescribed states
    double dt = integTime - this->currentSimTimeSec;
    this->r_PM_M = this->rEpoch_PM_M + (this->rPrimeEpoch_PM_M * dt) + (0.5 * this->rPrimePrime_PM_M * dt * dt);
    this->rPrime_PM_M = this->rPrimeEpoch_PM_M + (this->rPrimePrime_PM_M * dt);
    this->omega_PM_P = this->omegaEpoch_PM_P + (this->omegaPrime_PM_P * dt);
    this->sigma_PM = (Eigen::Vector3d)this->sigma_PMState->getState();

    // Give the mass of the prescribed body to the effProps mass
    this->effProps.mEff = this->mass;

    // Compute dcm_BM
    this->dcm_BM = this->sigma_MB.toRotationMatrix();

    // Compute dcm_PM
    this->dcm_PM = (this->sigma_PM.toRotationMatrix()).transpose();

    // Compute dcm_BP
    this->dcm_BP = this->dcm_BM * this->dcm_PM.transpose();

    // Compute omega_PB_B given the user inputs omega_MB_M and omega_PM_P
    this->omega_PM_B = this->dcm_BP * this->omega_PM_P;
    this->omega_PB_B = this->omega_PM_B + this->omega_MB_B;

    // Compute omegaPrime_PB_B given the user inputs
    this->omegaTilde_PB_B = eigenTilde(this->omega_PB_B);
    this->omegaPrime_PM_B = this->dcm_BP * this->omegaPrime_PM_P;
    this->omegaPrime_PB_B = this->omegaPrime_PM_B + this->omegaTilde_PB_B * this->omega_PM_B;

    // Convert the prescribed variables to the B frame
    this->r_PM_B = this->dcm_BM * this->r_PM_M;
    this->rPrime_PM_B = this->dcm_BM * this->rPrime_PM_M;
    this->rPrimePrime_PM_B = this->dcm_BM * this->rPrimePrime_PM_M;

    // Compute the effector's CoM with respect to point B
    this->r_PB_B = this->r_PM_B + this->r_MB_B;
    this->r_PcP_B = this->dcm_BP * this->r_PcP_P;
    this->r_PcB_B = this->r_PcP_B + this->r_PB_B;
    this->effProps.rEff_CB_B = this->r_PcB_B;

    // Find the effector inertia about point B
    this->rTilde_PcB_B = eigenTilde(this->r_PcB_B);
    this->IPntPc_B = this->dcm_BP * this->IPntPc_P * this->dcm_BP.transpose();
    this->effProps.IEffPntB_B = this->IPntPc_B - this->mass * this->rTilde_PcB_B * this->rTilde_PcB_B;

    // Find the B frame time derivative of r_PcB_B
    this->omegaTilde_PB_B = eigenTilde(this->omega_PB_B);
    this->rPrime_PcB_B = this->omegaTilde_PB_B * this->r_PcP_B + this->rPrime_PM_B;
    this->effProps.rEffPrime_CB_B = this->rPrime_PcB_B;

    // Find the B frame time derivative of IPntPc_B
    Eigen::Matrix3d rPrimeTilde_PcB_B = eigenTilde(this->rPrime_PcB_B);
    this->effProps.IEffPrimePntB_B = this->omegaTilde_PB_B * this->IPntPc_B
                                     - this->IPntPc_B * this->omegaTilde_PB_B
                                     + this->mass * (rPrimeTilde_PcB_B * this->rTilde_PcB_B.transpose()
                                     + this->rTilde_PcB_B * rPrimeTilde_PcB_B.transpose());
}

/*! This method allows the state effector to give its contributions to the matrices needed for the back-sub.
 method

 @param integTime [s] Time the method is called
 @param backSubContr State effector contribution matrices for back-substitution
 @param sigma_BN Current B frame attitude with respect to the inertial frame
 @param omega_BN_B [rad/s] Angular velocity of the B frame with respect to the inertial frame, expressed in B frame
 components
 @param g_N [m/s^2] Gravitational acceleration in N frame components
*/
void PrescribedMotionStateEffector::updateContributions(double integTime,
                                                        BackSubMatrices & backSubContr,
                                                        Eigen::Vector3d sigma_BN,
                                                        Eigen::Vector3d omega_BN_B,
                                                        Eigen::Vector3d g_N)
{
    // Compute dcm_BN
    this->sigma_BN = sigma_BN;
    this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();

    // Define omega_BN_B
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);

    // Define omegaPrimeTilde_PB_B
    Eigen::Matrix3d omegaPrimeTilde_PB_B = eigenTilde(this->omegaPrime_PB_B);

    // Compute rPrimePrime_PcB_B
    this->rPrimePrime_PcB_B = (omegaPrimeTilde_PB_B + this->omegaTilde_PB_B * this->omegaTilde_PB_B) * this->r_PcP_B + this->rPrimePrime_PM_B;

    // Translation contributions
    backSubContr.vecTrans = -this->mass * this->rPrimePrime_PcB_B;

    // Rotation contributions
    Eigen::Matrix3d IPrimePntPc_B = this->omegaTilde_PB_B * this->IPntPc_B - this->IPntPc_B * this->omegaTilde_PB_B;
    backSubContr.vecRot = -(this->mass * this->rTilde_PcB_B * this->rPrimePrime_PcB_B)
                          - (IPrimePntPc_B + this->omegaTilde_BN_B * this->IPntPc_B) * this->omega_PB_B
                          - this->IPntPc_B * this->omegaPrime_PB_B
                          - this->mass * this->omegaTilde_BN_B * rTilde_PcB_B * this->rPrime_PcB_B;
}

/*! This method is for defining the state effector's MRP state derivative

 @param integTime [s] Time the method is called
 @param rDDot_BN_N [m/s^2] Acceleration of the vector pointing from the inertial frame origin to the B frame origin,
 expressed in inertial frame components
 @param omegaDot_BN_B [rad/s^2] Inertial time derivative of the angular velocity of the B frame with respect to the
 inertial frame, expressed in B frame components
 @param sigma_BN Current B frame attitude with respect to the inertial frame
*/
void PrescribedMotionStateEffector::computeDerivatives(double integTime,
                                                       Eigen::Vector3d rDDot_BN_N,
                                                       Eigen::Vector3d omegaDot_BN_B,
                                                       Eigen::Vector3d sigma_BN)
{
    Eigen::MRPd sigma_PM_loc;
    sigma_PM_loc = (Eigen::Vector3d)this->sigma_PMState->getState();
    this->sigma_PMState->setDerivative(0.25*sigma_PM_loc.Bmat()*this->omega_PM_P);
}

/*! This method is for calculating the contributions of the effector to the energy and momentum of the spacecraft.

 @param integTime [s] Time the method is called
 @param rotAngMomPntCContr_B [kg m^2/s] Contribution of stateEffector to total rotational angular mom
 @param rotEnergyContr [J] Contribution of stateEffector to total rotational energy
 @param omega_BN_B [rad/s] Angular velocity of the B frame with respect to the inertial frame, expressed in B frame
 components
*/
void PrescribedMotionStateEffector::updateEnergyMomContributions(double integTime,
                                                                 Eigen::Vector3d & rotAngMomPntCContr_B,
                                                                 double & rotEnergyContr,
                                                                 Eigen::Vector3d omega_BN_B)
{
    // Update omega_BN_B and omega_PN_B
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    this->omega_PN_B = this->omega_PB_B + this->omega_BN_B;

    // Compute rDot_PcB_B
    this->rDot_PcB_B = this->rPrime_PcB_B + this->omegaTilde_BN_B * this->r_PcB_B;

    // Find the rotational angular momentum contribution from hub
    rotAngMomPntCContr_B = this->IPntPc_B * this->omega_PN_B + this->mass * this->rTilde_PcB_B * this->rDot_PcB_B;

    // Find the rotational energy contribution from the hub
    rotEnergyContr = 0.5 * this->omega_PN_B.dot(this->IPntPc_B * this->omega_PN_B)
                     + 0.5 * this->mass * this->rDot_PcB_B.dot(this->rDot_PcB_B);
}

/*! This method computes the effector states relative to the inertial frame.

*/
void PrescribedMotionStateEffector::computePrescribedMotionInertialStates()
{
    // Compute the effector's attitude with respect to the inertial frame
    Eigen::Matrix3d dcm_PN = (this->dcm_BP).transpose() * this->dcm_BN;
    this->sigma_PN = eigenMRPd2Vector3d(eigenC2MRP(dcm_PN));

    // Compute the effector's inertial position vector
    this->r_PcN_N = (Eigen::Vector3d)*this->inertialPositionProperty + this->dcm_BN.transpose() * this->r_PcB_B;

    // Compute the effector's inertial velocity vector
    this->v_PcN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * this->rDot_PcB_B;
}

/*! This method updates the effector state at the dynamics frequency.

 @param currentSimNanos [ns] Time the method is called
*/
void PrescribedMotionStateEffector::UpdateState(uint64_t currentSimNanos)
{
    // Store the current simulation time
    this->currentSimTimeSec = currentSimNanos * NANO2SEC;

    // Read the translational input message if it is linked and written
    if (this->prescribedTranslationInMsg.isLinked() && this->prescribedTranslationInMsg.isWritten())
    {
        PrescribedTranslationMsgPayload incomingPrescribedTransStates = this->prescribedTranslationInMsg();
        this->r_PM_M = cArray2EigenVector3d(incomingPrescribedTransStates.r_PM_M);
        this->rPrime_PM_M = cArray2EigenVector3d(incomingPrescribedTransStates.rPrime_PM_M);
        this->rPrimePrime_PM_M = cArray2EigenVector3d(incomingPrescribedTransStates.rPrimePrime_PM_M);

        // Save off the prescribed translational states at each dynamics time step
        this->rEpoch_PM_M = cArray2EigenVector3d(incomingPrescribedTransStates.r_PM_M);
        this->rPrimeEpoch_PM_M = cArray2EigenVector3d(incomingPrescribedTransStates.rPrime_PM_M);
    }

    // Read the rotational input message if it is linked and written
    if (this->prescribedRotationInMsg.isLinked() && this->prescribedRotationInMsg.isWritten())
    {
        PrescribedRotationMsgPayload incomingPrescribedRotStates = this->prescribedRotationInMsg();
        this->omega_PM_P = cArray2EigenVector3d(incomingPrescribedRotStates.omega_PM_P);
        this->omegaPrime_PM_P = cArray2EigenVector3d(incomingPrescribedRotStates.omegaPrime_PM_P);
        this->sigma_PM = cArray2EigenVector3d(incomingPrescribedRotStates.sigma_PM);

        // Save off the prescribed rotational states at each dynamics time step
        this->omegaEpoch_PM_P = cArray2EigenVector3d(incomingPrescribedRotStates.omega_PM_P);
        Eigen::Vector3d sigma_PM_loc = cArray2EigenVector3d(incomingPrescribedRotStates.sigma_PM);
        this->sigma_PMState->setState(sigma_PM_loc);
    }

    // Call the method to compute the effector's inertial states
    this->computePrescribedMotionInertialStates();

    // Call the method to write the output messages
    this->writeOutputStateMessages(currentSimNanos);
}
