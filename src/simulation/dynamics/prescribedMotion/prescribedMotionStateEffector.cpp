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
#include <vector>

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
    this->IPntFc_F.setIdentity();
    this->r_MB_B.setZero();
    this->r_FcF_F.setZero();
    this->omega_MB_B.setZero();
    this->omegaPrime_MB_B.setZero();
    this->sigma_MB.setIdentity();
    this->currentSimTimeSec = 0.0;

    // Initialize prescribed states at epoch
    this->rEpoch_FM_M.setZero();
    this->rPrimeEpoch_FM_M.setZero();
    this->omegaEpoch_FM_F.setZero();

    this->spacecraftName = "prescribedObject";

    // Set the sigma_FM state name
    this->nameOfsigma_FMState = "prescribedObjectSigma_FM" + std::to_string(this->effectorID);

    // Set the property names
    this->nameOfInertialPositionProperty = "prescribedObjectInertialPosition" + std::to_string(PrescribedMotionStateEffector::effectorID);
    this->nameOfInertialVelocityProperty = "prescribedObjectInertialVelocity" + std::to_string(PrescribedMotionStateEffector::effectorID);
    this->nameOfInertialAttitudeProperty = "prescribedObjectInertialAttitude" + std::to_string(PrescribedMotionStateEffector::effectorID);
    this->nameOfInertialAngVelocityProperty = "prescribedObjectInertialAngVelocity" + std::to_string(PrescribedMotionStateEffector::effectorID);

    this->nameOfPrescribedPositionProperty = "prescribedObjectPosition" + std::to_string(PrescribedMotionStateEffector::effectorID);
    this->nameOfPrescribedVelocityProperty = "prescribedObjectVelocity" + std::to_string(PrescribedMotionStateEffector::effectorID);
    this->nameOfPrescribedAccelerationProperty = "prescribedObjectAcceleration" + std::to_string(PrescribedMotionStateEffector::effectorID);
    this->nameOfPrescribedAttitudeProperty = "prescribedObjectAttitude" + std::to_string(PrescribedMotionStateEffector::effectorID);
    this->nameOfPrescribedAngVelocityProperty = "prescribedObjectAngVelocity" + std::to_string(PrescribedMotionStateEffector::effectorID);
    this->nameOfPrescribedAngAccelerationProperty = "prescribedObjectAngAcceleration" + std::to_string(PrescribedMotionStateEffector::effectorID);

    PrescribedMotionStateEffector::effectorID++;
}

uint64_t PrescribedMotionStateEffector::effectorID = 1;

/*! This is the destructor. */
PrescribedMotionStateEffector::~PrescribedMotionStateEffector()
{
    PrescribedMotionStateEffector::effectorID = 1;
}

/*! This method prepends the name of the spacecraft for multi-spacecraft simulations.*/
void PrescribedMotionStateEffector::prependSpacecraftNameToStates()
{
    this->nameOfsigma_FMState = this->nameOfSpacecraftAttachedTo + this->nameOfsigma_FMState;

    this->nameOfInertialPositionProperty = this->nameOfSpacecraftAttachedTo + this->nameOfInertialPositionProperty;
    this->nameOfInertialVelocityProperty = this->nameOfSpacecraftAttachedTo + this->nameOfInertialVelocityProperty;
    this->nameOfInertialAttitudeProperty = this->nameOfSpacecraftAttachedTo + this->nameOfInertialAttitudeProperty;
    this->nameOfInertialAngVelocityProperty = this->nameOfSpacecraftAttachedTo + this->nameOfInertialAngVelocityProperty;

    this->nameOfPrescribedPositionProperty = this->nameOfSpacecraftAttachedTo + this->nameOfPrescribedPositionProperty;
    this->nameOfPrescribedVelocityProperty = this->nameOfSpacecraftAttachedTo + this->nameOfPrescribedVelocityProperty;
    this->nameOfPrescribedAccelerationProperty = this->nameOfSpacecraftAttachedTo + this->nameOfPrescribedAccelerationProperty;
    this->nameOfPrescribedAttitudeProperty = this->nameOfSpacecraftAttachedTo + this->nameOfPrescribedAttitudeProperty;
    this->nameOfPrescribedAngVelocityProperty = this->nameOfSpacecraftAttachedTo + this->nameOfPrescribedAngVelocityProperty;
    this->nameOfPrescribedAngAccelerationProperty = this->nameOfSpacecraftAttachedTo + this->nameOfPrescribedAngAccelerationProperty;

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
        eigenVector3d2CArray(this->r_FM_M, prescribedTranslationBuffer.r_FM_M);
        eigenVector3d2CArray(this->rPrime_FM_M, prescribedTranslationBuffer.rPrime_FM_M);
        eigenVector3d2CArray(this->rPrimePrime_FM_M, prescribedTranslationBuffer.rPrimePrime_FM_M);
        this->prescribedTranslationOutMsg.write(&prescribedTranslationBuffer, this->moduleID, currentClock);
    }

    // Write the prescribed rotational motion output message if it is linked
    if (this->prescribedRotationOutMsg.isLinked())
    {
        PrescribedRotationMsgPayload prescribedRotationBuffer = this->prescribedRotationOutMsg.zeroMsgPayload;
        eigenVector3d2CArray(this->omega_FM_F, prescribedRotationBuffer.omega_FM_F);
        eigenVector3d2CArray(this->omegaPrime_FM_F, prescribedRotationBuffer.omegaPrime_FM_F);
        Eigen::Vector3d sigma_FM_loc = eigenMRPd2Vector3d(this->sigma_FM);
        eigenVector3d2CArray(sigma_FM_loc, prescribedRotationBuffer.sigma_FM);
        this->prescribedRotationOutMsg.write(&prescribedRotationBuffer, this->moduleID, currentClock);
    }

    // Write the config log message if it is linked
    if (this->prescribedMotionConfigLogOutMsg.isLinked())
    {
        SCStatesMsgPayload configLogMsg = this->prescribedMotionConfigLogOutMsg.zeroMsgPayload;

        // Note that the configLogMsg B frame represents the effector body frame (frame F)
        eigenMatrixXd2CArray(*this->r_FcN_N, configLogMsg.r_BN_N);
        eigenMatrixXd2CArray(*this->v_FcN_N, configLogMsg.v_BN_N);
        eigenMatrixXd2CArray(*this->sigma_FN, configLogMsg.sigma_BN);
        eigenMatrixXd2CArray(*this->omega_FN_F, configLogMsg.omega_BN_B);
        this->prescribedMotionConfigLogOutMsg.write(&configLogMsg, this->moduleID, currentClock);
    }
}

/*! This method allows the effector to have access to the hub states.

 @param statesIn Pointer to give the state effector access the hub states
*/
void PrescribedMotionStateEffector::linkInStates(DynParamManager& statesIn)
{
    this->prependSpacecraftNameToStates();

    // Get access to the hub states needed for dynamic coupling
    this->inertialPositionProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialPosition);
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialVelocity);

    // Loop through attached stateEffectors to link in their states
    std::vector<StateEffector*>::iterator stateIt;
    for(stateIt = this->stateEffectors.begin(); stateIt != this->stateEffectors.end(); stateIt++)
    {
        (*stateIt)->linkInStates(statesIn);
    }
}

/*! This method allows the state effector to register its states with the dynamic parameter manager.

 @param states Pointer to give the state effector access the hub states
*/
void PrescribedMotionStateEffector::registerStates(DynParamManager& states)
{
        this->sigma_FMState = states.registerState(3, 1, this->nameOfsigma_FMState);
        Eigen::Vector3d sigma_FM_loc = eigenMRPd2Vector3d(this->sigma_FM);
        Eigen::Vector3d sigma_FMInitMatrix;
        sigma_FMInitMatrix(0) = sigma_FM_loc[0];
        sigma_FMInitMatrix(1) = sigma_FM_loc[1];
        sigma_FMInitMatrix(2) = sigma_FM_loc[2];
        this->sigma_FMState->setState(sigma_FMInitMatrix);

        registerProperties(states);

    // Loop through attached stateEffectors to register their states
    std::vector<StateEffector*>::iterator stateIt;
    for(stateIt = this->stateEffectors.begin(); stateIt != this->stateEffectors.end(); stateIt++)
    {
        (*stateIt)->registerStates(states);
    }
}

void PrescribedMotionStateEffector::registerProperties(DynParamManager& states)
{
    Eigen::Vector3d stateInit = Eigen::Vector3d::Zero();
    this->r_FcN_N = states.createProperty(this->nameOfInertialPositionProperty, stateInit);
    this->v_FcN_N = states.createProperty(this->nameOfInertialVelocityProperty, stateInit);
    this->sigma_FN = states.createProperty(this->nameOfInertialAttitudeProperty, stateInit);
    this->omega_FN_F = states.createProperty(this->nameOfInertialAngVelocityProperty, stateInit);

    this->r_FB_B = states.createProperty(this->nameOfPrescribedPositionProperty, stateInit);
    this->rPrime_FB_B = states.createProperty(this->nameOfPrescribedVelocityProperty, stateInit);
    this->rPrimePrime_FB_B = states.createProperty(this->nameOfPrescribedAccelerationProperty, stateInit);
    this->sigma_FB = states.createProperty(this->nameOfPrescribedAttitudeProperty, stateInit);
    this->omega_FB_F = states.createProperty(this->nameOfPrescribedAngVelocityProperty, stateInit);
    this->omegaPrime_FB_F = states.createProperty(this->nameOfPrescribedAngAccelerationProperty, stateInit);

//    std::vector<StateEffector*>::iterator stateIt;
//    for(stateIt = this->stateEffectors.begin(); stateIt != this->stateEffectors.end(); stateIt++)
//    {
//        (*stateIt)->linkInProperties(states);
//    }
}

/*! This method allows the state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft.

 @param integTime [s] Time the method is called
*/
void PrescribedMotionStateEffector::updateEffectorMassProps(double integTime)
{
    // Update the prescribed states
    double dt = integTime - this->currentSimTimeSec;
    this->r_FM_M = this->rEpoch_FM_M + (this->rPrimeEpoch_FM_M * dt) + (0.5 * this->rPrimePrime_FM_M * dt * dt);
    this->rPrime_FM_M = this->rPrimeEpoch_FM_M + (this->rPrimePrime_FM_M * dt);
    this->omega_FM_F = this->omegaEpoch_FM_F + (this->omegaPrime_FM_F * dt);
    this->sigma_FM = (Eigen::Vector3d)this->sigma_FMState->getState();

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

    *this->rPrime_FB_B = this->rPrime_FM_B;
    *this->rPrimePrime_FB_B = this->rPrimePrime_FM_B;
    *this->sigma_FB = eigenMRPd2Vector3d(eigenC2MRP(dcm_BF.transpose()));
    *this->omega_FB_F = this->omega_FM_F;
    *this->omegaPrime_FB_F = this->omegaPrime_FM_F;

    // Compute the effector's CoM with respect to point B
    *this->r_FB_B = this->r_FM_B + this->r_MB_B;
    this->r_FcF_B = this->dcm_BF * this->r_FcF_F;
    this->r_FcB_B = this->r_FcF_B + *this->r_FB_B;
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
    this->effProps.IEffPrimePntB_B = this->omegaTilde_FB_B * this->IPntFc_B
                                     - this->IPntFc_B * this->omegaTilde_FB_B
                                     + this->mass * (rPrimeTilde_FcB_B * this->rTilde_FcB_B.transpose()
                                     + this->rTilde_FcB_B * rPrimeTilde_FcB_B.transpose());

    if (!this->stateEffectors.empty()) {

        // Loop through attached state effectors and call updateContributions
        std::vector<StateEffector*>::iterator it;
        for(it = this->stateEffectors.begin(); it != this->stateEffectors.end(); it++) {
            (*it)->updateEffectorMassProps(integTime);

            this->effProps.mEff += (*it)->effProps.mEff;
            this->effProps.mEffDot += (*it)->effProps.mEffDot;

            Eigen::Vector3d rEff_CP_P = (*it)->effProps.rEff_CB_B;
            Eigen::Matrix3d dcm_BP = this->dcm_BF;
            Eigen::Vector3d rEff_CP_B = dcm_BP * rEff_CP_P;
            Eigen::Vector3d r_PB_B = *this->r_FB_B;
            Eigen::Vector3d r_Eff_CB_B = rEff_CP_B + r_PB_B;
            this->effProps.rEff_CB_B += (*it)->effProps.mEff * r_Eff_CB_B;

            Eigen::Matrix rEffPPrime_CP_P = (*it)->effProps.rEffPrime_CB_B;
            Eigen::Vector3d rBPrime_PB_B = *this->rPrime_FB_B;
            Eigen::Vector3d rEffBPrime_CB_B = dcm_BP * rEffPPrime_CP_P + rBPrime_PB_B;
            this->effProps.rEffPrime_CB_B += (*it)->effProps.mEff * rEffBPrime_CB_B;

            Eigen::Matrix3d IEffPntP_P = (*it)->effProps.IEffPntB_B;
            Eigen::Matrix3d IEffPntP_B = dcm_BP * IEffPntP_P * dcm_BP.transpose();
            Eigen::Matrix3d rEffTilde_CB_B = eigenTilde(r_Eff_CB_B);
            Eigen::Matrix3d rEffTilde_CP_B = eigenTilde(rEff_CP_B);
            this->effProps.IEffPntB_B += IEffPntP_B + (*it)->effProps.mEff * (rEffTilde_CP_B * rEffTilde_CP_B
                                                                            - rEffTilde_CB_B * rEffTilde_CB_B);

            Eigen::Vector3d rEffBPrime_CP_B = dcm_BP * rEffPPrime_CP_P + this->omegaTilde_FB_B * rEff_CP_B;
            Eigen::Matrix3d rEffBPrimeTilde_CP_B = eigenTilde(rEffBPrime_CP_B);
            Eigen::Matrix3d rEffBPrimeTilde_CB_B = eigenTilde(rEffBPrime_CB_B);
            Eigen::Matrix3d IEffPPrimePntP_P = (*it)->effProps.IEffPrimePntB_B;
            Eigen::Matrix3d IEffPPrimePntP_B = dcm_BP * IEffPPrimePntP_P * dcm_BP.transpose();
            Eigen::Matrix3d IEffBPrimePntP_B = IEffPPrimePntP_B + this->omegaTilde_FB_B * IEffPntP_B
                                                                + IEffPntP_B * this->omegaTilde_FB_B.transpose();
            this->effProps.IEffPrimePntB_B += IEffBPrimePntP_B - (*it)->effProps.mEff * rEffBPrimeTilde_CP_B * rEffTilde_CP_B.transpose()
                    - (*it)->effProps.mEff * rEffTilde_CP_B * rEffBPrimeTilde_CP_B.transpose()
                    + (*it)->effProps.mEff * rEffBPrimeTilde_CB_B * rEffTilde_CB_B.transpose()
                    + (*it)->effProps.mEff * rEffTilde_CB_B * rEffBPrimeTilde_CB_B.transpose();
        }
    }
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
    // Update sigma_BN and dcm_BN
    this->sigma_BN = sigma_BN;
    this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();

    // Update omega_BN_B
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);

    if (!this->stateEffectors.empty()) {
        // Update prescribed motion properties
        this->computePrescribedMotionInertialStates();

        // Loop through attached state effectors and call updateContributions
        std::vector<StateEffector*>::iterator it;
        for(it = this->stateEffectors.begin(); it != this->stateEffectors.end(); it++) {
            (*it)->updateContributions(integTime, backSubContr, *this->sigma_FN, *this->omega_FN_F, g_N);
        }
    }

    // Define omegaPrimeTilde_FB_B
    Eigen::Matrix3d omegaPrimeTilde_FB_B = eigenTilde(this->omegaPrime_FB_B);

    // Compute rPrimePrime_FcB_B
    this->rPrimePrime_FcB_B = (omegaPrimeTilde_FB_B + this->omegaTilde_FB_B * this->omegaTilde_FB_B) * this->r_FcF_B + this->rPrimePrime_FM_B;

    // Prescribed motion trans eom rhs contributions
    backSubContr.vecTrans += -this->mass * this->rPrimePrime_FcB_B;

    // Prescribed motion rot eom rhs contributions
    Eigen::Matrix3d IPrimePntFc_B = this->omegaTilde_FB_B * this->IPntFc_B - this->IPntFc_B * this->omegaTilde_FB_B;
    backSubContr.vecRot += -(this->mass * this->rTilde_FcB_B * this->rPrimePrime_FcB_B)
                          - (IPrimePntFc_B + this->omegaTilde_BN_B * this->IPntFc_B) * this->omega_FB_B
                          - this->IPntFc_B * this->omegaPrime_FB_B
                          - this->mass * this->omegaTilde_BN_B * rTilde_FcB_B * this->rPrime_FcB_B;
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
    Eigen::MRPd sigma_FM_loc;
    sigma_FM_loc = (Eigen::Vector3d)this->sigma_FMState->getState();
    this->sigma_FMState->setDerivative(0.25*sigma_FM_loc.Bmat()*this->omega_FM_F);

    if (!this->stateEffectors.empty()) {
        // Update sigma_BN and dcm_BN
        this->sigma_BN = sigma_BN;
        this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();

        // Compute dcm_FN and sigma_FN
        Eigen::Matrix3d dcm_FN = this->dcm_BF.transpose() * this->dcm_BN;
        *this->sigma_FN = eigenMRPd2Vector3d(eigenC2MRP(dcm_FN));

        // Compute omegaDot_FN_F
        Eigen::Vector3d omegaDot_FN_B = this->omegaPrime_FM_B + this->omegaTilde_BN_B * this->omega_FM_B + omegaDot_BN_B;
        Eigen::Vector3d omegaDot_FN_F = this->dcm_BF.transpose() * omegaDot_FN_B;

        // Compute rDDot_FN_N
        Eigen::Matrix3d omegaDotTilde_BN_B = eigenTilde(omegaDot_BN_B);
        Eigen::Vector3d rDDot_FB_B = this->rPrimePrime_FM_B + 2 * this->omegaTilde_BN_B * this->rPrime_FM_B
                                     + omegaDotTilde_BN_B * (*this->r_FB_B)
                                     + this->omegaTilde_BN_B * this->omegaTilde_BN_B * (*this->r_FB_B);
        Eigen::Vector3d rDDot_FN_N = this->dcm_BN.transpose() * rDDot_FB_B + rDDot_BN_N;

        // Loop through attached state effectors for compute derivatives
        std::vector<StateEffector*>::iterator it;
        for(it = this->stateEffectors.begin(); it != this->stateEffectors.end(); it++) {
            (*it)->computeDerivatives(integTime, rDDot_FN_N, omegaDot_FN_F, *this->sigma_FN);
        }
    }
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
    // Update omega_BN_B and omega_FN_B
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    this->omega_FN_B = this->omega_FB_B + this->omega_BN_B;

    // Compute rDot_FcB_B
    this->rDot_FcB_B = this->rPrime_FcB_B + this->omegaTilde_BN_B * this->r_FcB_B;

    Eigen::Vector3d totRotAngMomPntC_B;
    totRotAngMomPntC_B.setZero();
    double totRotEnergy = 0.0;

    // Find the rotational angular momentum contribution from hub
    totRotAngMomPntC_B = this->IPntFc_B * this->omega_FN_B + this->mass * this->rTilde_FcB_B * this->rDot_FcB_B;

    // Find the rotational energy contribution from the hub
    totRotEnergy = 0.5 * this->omega_FN_B.dot(this->IPntFc_B * this->omega_FN_B)
                   + 0.5 * this->mass * this->rDot_FcB_B.dot(this->rDot_FcB_B);

    // Loop through attached state effectors
    std::vector<StateEffector*>::iterator it;
    for(it = this->stateEffectors.begin(); it != this->stateEffectors.end(); it++) {
        rotAngMomPntCContr_B.setZero();
        rotEnergyContr = 0.0;

        (*it)->updateEnergyMomContributions(integTime, rotAngMomPntCContr_B, rotEnergyContr, this->omega_FN_B);

        totRotAngMomPntC_B += rotAngMomPntCContr_B;
        totRotEnergy += rotEnergyContr;
    }

    rotAngMomPntCContr_B = totRotAngMomPntC_B;
    rotEnergyContr = totRotEnergy;

}

/*! This method computes the effector states relative to the inertial frame.

*/
void PrescribedMotionStateEffector::computePrescribedMotionInertialStates()
{
    // Compute the effector's attitude with respect to the inertial frame
    Eigen::Matrix3d dcm_FN = (this->dcm_BF).transpose() * this->dcm_BN;
    *this->sigma_FN = eigenMRPd2Vector3d(eigenC2MRP(dcm_FN));

    // Compute effector's angular velocity with respect to the inertial frame
    *this->omega_FN_F = this->dcm_BF.transpose() * this->omega_FN_B;

    // Compute the effector's inertial position vector
    *this->r_FcN_N = (Eigen::Vector3d)*this->inertialPositionProperty + this->dcm_BN.transpose() * this->r_FcB_B;

    // Compute the effector's inertial velocity vector
    *this->v_FcN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * this->rDot_FcB_B;
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
        this->r_FM_M = cArray2EigenVector3d(incomingPrescribedTransStates.r_FM_M);
        this->rPrime_FM_M = cArray2EigenVector3d(incomingPrescribedTransStates.rPrime_FM_M);
        this->rPrimePrime_FM_M = cArray2EigenVector3d(incomingPrescribedTransStates.rPrimePrime_FM_M);

        // Save off the prescribed translational states at each dynamics time step
        this->rEpoch_FM_M = cArray2EigenVector3d(incomingPrescribedTransStates.r_FM_M);
        this->rPrimeEpoch_FM_M = cArray2EigenVector3d(incomingPrescribedTransStates.rPrime_FM_M);
    }

    // Read the rotational input message if it is linked and written
    if (this->prescribedRotationInMsg.isLinked() && this->prescribedRotationInMsg.isWritten())
    {
        PrescribedRotationMsgPayload incomingPrescribedRotStates = this->prescribedRotationInMsg();
        this->omega_FM_F = cArray2EigenVector3d(incomingPrescribedRotStates.omega_FM_F);
        this->omegaPrime_FM_F = cArray2EigenVector3d(incomingPrescribedRotStates.omegaPrime_FM_F);
        this->sigma_FM = cArray2EigenVector3d(incomingPrescribedRotStates.sigma_FM);

        // Save off the prescribed rotational states at each dynamics time step
        this->omegaEpoch_FM_F = cArray2EigenVector3d(incomingPrescribedRotStates.omega_FM_F);
        Eigen::Vector3d sigma_FM_loc = cArray2EigenVector3d(incomingPrescribedRotStates.sigma_FM);
        this->sigma_FMState->setState(sigma_FM_loc);
    }

    // Call the method to compute the effector's inertial states
    this->computePrescribedMotionInertialStates();

    // Call the method to write the output messages
    this->writeOutputStateMessages(currentSimNanos);
}

/*! This method attaches a stateEffector to the prescribedMotionStateEffector */
void PrescribedMotionStateEffector::addStateEffector(StateEffector* newStateEffector)
{
    this->assignStateParamNames<StateEffector *>(newStateEffector);

    this->stateEffectors.push_back(newStateEffector);

    // Give the stateEffector the name of the prescribed object it is attached to
    newStateEffector->nameOfSpacecraftAttachedTo = this->spacecraftName;
}
