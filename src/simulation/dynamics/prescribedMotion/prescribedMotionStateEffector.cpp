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
    this->sigma_MB.setIdentity();
    this->currentSimTimeSec = 0.0;

    // Initialize prescribed states at epoch
    this->rEpoch_PM_M.setZero();
    this->rPrimeEpoch_PM_M.setZero();
    this->omegaEpoch_PM_P.setZero();

    this->spacecraftName = "prescribedObject";
    this->nameOfsigma_PMState = "prescribedObjectsigma_PM" + std::to_string(this->effectorID);

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
        eigenMatrixXd2CArray(*this->sigma_PN, configLogMsg.sigma_BN);
        eigenMatrixXd2CArray(*this->omega_PN_P, configLogMsg.omega_BN_B);
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

    // Loop through attached stateEffectors to link in their states
    std::vector<StateEffector*>::iterator stateIt;
    for(stateIt = this->stateEffectors.begin(); stateIt != this->stateEffectors.end(); stateIt++)
    {
        (*stateIt)->linkInPrescribedMotionProperties(statesIn);
        (*stateIt)->linkInStates(statesIn);
    }
}

/*! This method allows the state effector to register its states with the dynamic parameter manager.

 @param states Pointer to give the state effector access the hub states
*/
void PrescribedMotionStateEffector::registerStates(DynParamManager& states)
{
    this->sigma_PMState = states.registerState(3, 1, this->nameOfsigma_PMState);
    Eigen::Vector3d sigma_PMInitMatrix = eigenMRPd2Vector3d(this->sigma_PM);
    this->sigma_PMState->setState(sigma_PMInitMatrix);

    // Call method to register the prescribed motion properties
    registerProperties(states);

    // Loop through attached stateEffectors to register their states
    std::vector<StateEffector*>::iterator stateIt;
    for(stateIt = this->stateEffectors.begin(); stateIt != this->stateEffectors.end(); stateIt++)
    {
        (*stateIt)->registerStates(states);
    }
}

/*! This method allows the state effector to register its properties with the dynamic parameter manager.

 @param states Pointer to give the state effector access the hub states
*/
void PrescribedMotionStateEffector::registerProperties(DynParamManager& states)
{
    Eigen::Vector3d stateInit = Eigen::Vector3d::Zero();
    this->r_PN_N = states.createProperty(this->nameOfInertialPositionProperty, stateInit);
    this->v_PN_N = states.createProperty(this->nameOfInertialVelocityProperty, stateInit);
    this->sigma_PN = states.createProperty(this->nameOfInertialAttitudeProperty, stateInit);
    this->omega_PN_P = states.createProperty(this->nameOfInertialAngVelocityProperty, stateInit);

    this->r_PB_B = states.createProperty(this->nameOfPrescribedPositionProperty, stateInit);
    this->rPrime_PB_B = states.createProperty(this->nameOfPrescribedVelocityProperty, stateInit);
    this->rPrimePrime_PB_B = states.createProperty(this->nameOfPrescribedAccelerationProperty, stateInit);
    this->sigma_PB = states.createProperty(this->nameOfPrescribedAttitudeProperty, stateInit);
    this->omega_PB_P = states.createProperty(this->nameOfPrescribedAngVelocityProperty, stateInit);
    this->omegaPrime_PB_P = states.createProperty(this->nameOfPrescribedAngAccelerationProperty, stateInit);
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
    *this->sigma_PB = eigenMRPd2Vector3d(eigenC2MRP(this->dcm_BP.transpose()));

    // Compute omega_PB_B
    this->omega_PM_B = this->dcm_BP * this->omega_PM_P;
    this->omega_PB_B = this->omega_PM_B;
    *this->omega_PB_P = this->dcm_BP.transpose() * this->omega_PB_B;

    // Compute omegaPrime_PB_B given the user inputs
    this->omegaTilde_PB_B = eigenTilde(this->omega_PB_B);
    this->omegaPrime_PM_B = this->dcm_BP * this->omegaPrime_PM_P;
    this->omegaPrime_PB_B = this->omegaPrime_PM_B;
    *this->omegaPrime_PB_P = this->omegaPrime_PM_P;

    // Convert the prescribed variables to the B frame
    this->r_PM_B = this->dcm_BM * this->r_PM_M;
    this->rPrime_PM_B = this->dcm_BM * this->rPrime_PM_M;
    this->rPrimePrime_PM_B = this->dcm_BM * this->rPrimePrime_PM_M;
    *this->rPrimePrime_PB_B = this->rPrimePrime_PM_B;

    // Compute the effector's CoM with respect to point B
    *this->r_PB_B = this->r_PM_B + this->r_MB_B;
    this->r_PcP_B = this->dcm_BP * this->r_PcP_P;
    this->r_PcB_B = this->r_PcP_B + *this->r_PB_B;

    // Find the effector inertia about point B
    this->rTilde_PcB_B = eigenTilde(this->r_PcB_B);
    this->IPntPc_B = this->dcm_BP * this->IPntPc_P * this->dcm_BP.transpose();
    this->effProps.IEffPntB_B = this->IPntPc_B - this->mass * this->rTilde_PcB_B * this->rTilde_PcB_B;

    // Find the B frame time derivative of r_PcB_B
    this->rPrime_PcB_B = this->omegaTilde_PB_B * this->r_PcP_B + this->rPrime_PM_B;

    if (this->stateEffectors.empty()) {
        this->effProps.rEff_CB_B = this->r_PcB_B;
        this->effProps.rEffPrime_CB_B = this->rPrime_PcB_B;
    } else {
        this->effProps.rEff_CB_B = this->mass * this->r_PcB_B;
        this->effProps.rEffPrime_CB_B = this->mass * this->rPrime_PcB_B;
    }

    // Find the B frame time derivative of IPntPc_B
    Eigen::Matrix3d rPrimeTilde_PcB_B = eigenTilde(this->rPrime_PcB_B);
    this->effProps.IEffPrimePntB_B = this->omegaTilde_PB_B * this->IPntPc_B
                                     - this->IPntPc_B * this->omegaTilde_PB_B
                                     + this->mass * (rPrimeTilde_PcB_B * this->rTilde_PcB_B.transpose()
                                     + this->rTilde_PcB_B * rPrimeTilde_PcB_B.transpose());


    // Loop through attached state effectors and compute their contributions
    std::vector<StateEffector*>::iterator it;
    for(it = this->stateEffectors.begin(); it != this->stateEffectors.end(); it++) {
        (*it)->updateEffectorMassProps(integTime);

        this->effProps.mEff += (*it)->effProps.mEff;
        this->effProps.mEffDot += (*it)->effProps.mEffDot;

        Eigen::Vector3d r_EcP_P = (*it)->effProps.rEff_CB_B;
        Eigen::Vector3d r_EcP_B = this->dcm_BP * r_EcP_P;
        Eigen::Vector3d r_EcB_B = r_EcP_B + *this->r_PB_B;
        this->effProps.rEff_CB_B += (*it)->effProps.mEff * r_EcB_B;

        Eigen::Vector3d rPPrime_EcP_P = (*it)->effProps.rEffPrime_CB_B;
        Eigen::Vector3d rPrime_EcP_B = this->dcm_BP * rPPrime_EcP_P + this->omegaTilde_PB_B * r_EcP_B;
        *this->rPrime_PB_B = this->rPrime_PM_B;
        Eigen::Vector3d rPrime_EcB_B = rPrime_EcP_B + *this->rPrime_PB_B;
        this->effProps.rEffPrime_CB_B += (*it)->effProps.mEff * rPrime_EcB_B;

        Eigen::Matrix3d IEffPntP_P = (*it)->effProps.IEffPntB_B;
        Eigen::Matrix3d IEffPntP_B = this->dcm_BP * IEffPntP_P * this->dcm_BP.transpose();
        Eigen::Matrix3d rTilde_EcB_B = eigenTilde(r_EcB_B);
        Eigen::Matrix3d rTilde_EcP_B = eigenTilde(r_EcP_B);
        this->effProps.IEffPntB_B += IEffPntP_B + (*it)->effProps.mEff * (rTilde_EcP_B * rTilde_EcP_B
                                                                          - rTilde_EcB_B * rTilde_EcB_B);

        Eigen::Matrix3d rPrimeTilde_EcB_B = eigenTilde(rPrime_EcB_B);
        Eigen::Matrix3d rPrimeTilde_EcP_B = eigenTilde(rPrime_EcP_B);
        Eigen::Matrix3d IEffPPrimePntP_P = (*it)->effProps.IEffPrimePntB_B;
        Eigen::Matrix3d IEffPPrimePntP_B = this->dcm_BP * IEffPPrimePntP_P * this->dcm_BP.transpose();
        Eigen::Matrix3d IEffPrimePntP_B = IEffPPrimePntP_B + this->omegaTilde_PB_B * IEffPntP_B
                                           + IEffPntP_B * this->omegaTilde_PB_B.transpose();
        this->effProps.IEffPrimePntB_B += IEffPrimePntP_B - (*it)->effProps.mEff * rPrimeTilde_EcP_B * rTilde_EcP_B.transpose()
                                          - (*it)->effProps.mEff * rTilde_EcP_B * rPrimeTilde_EcP_B.transpose()
                                          + (*it)->effProps.mEff * rPrimeTilde_EcB_B * rTilde_EcB_B.transpose()
                                          + (*it)->effProps.mEff * rTilde_EcB_B * rPrimeTilde_EcB_B.transpose();
    }

    // Divide by the total mass of the prescribed component plus attached effectors to finalize mass props
    if (!this->stateEffectors.empty()) {
        this->effProps.rEff_CB_B = this->effProps.rEff_CB_B / this->effProps.mEff;
        this->effProps.rEffPrime_CB_B = this->effProps.rEffPrime_CB_B / this->effProps.mEff;
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

    // Update sigma_PN
    Eigen::Matrix3d dcm_PN = this->dcm_BP.transpose() * this->dcm_BN;
    *this->sigma_PN = eigenMRPd2Vector3d(eigenC2MRP(dcm_PN));

    // Update omega_PN_P
    *this->omega_PN_P = *this->omega_PB_P + this->dcm_BP.transpose() * this->omega_BN_B;

    Eigen::Matrix3d totMatrixA;
    Eigen::Matrix3d totMatrixB;
    Eigen::Matrix3d totMatrixC;
    Eigen::Matrix3d totMatrixD;
    Eigen::Vector3d totVecTrans;
    Eigen::Vector3d totVecRot;
    totMatrixA.setZero();
    totMatrixB.setZero();
    totMatrixC.setZero();
    totMatrixD.setZero();
    totVecTrans.setZero();
    totVecRot.setZero();

    // Loop through attached state effectors and compute their contributions
    std::vector<StateEffector*>::iterator it;
    for(it = this->stateEffectors.begin(); it != this->stateEffectors.end(); it++)
    {
        backSubContr.matrixA.setZero();
        backSubContr.matrixB.setZero();
        backSubContr.matrixC.setZero();
        backSubContr.matrixD.setZero();
        backSubContr.vecTrans.setZero();
        backSubContr.vecRot.setZero();

        (*it)->updateContributions(integTime, backSubContr, *this->sigma_PN, *this->omega_PN_P, g_N);
        (*it)->addPrescribedMotionCouplingContributions(backSubContr);

        totMatrixA += this->dcm_BP * backSubContr.matrixA * this->dcm_BP.transpose();
        totMatrixB += this->dcm_BP * backSubContr.matrixB * this->dcm_BP.transpose();
        totMatrixC += this->dcm_BP * backSubContr.matrixC * this->dcm_BP.transpose();
        totMatrixD += this->dcm_BP * backSubContr.matrixD * this->dcm_BP.transpose();
        totVecTrans += this->dcm_BP * backSubContr.vecTrans;
        totVecRot += this->dcm_BP * backSubContr.vecRot;
    }

    backSubContr.matrixA = totMatrixA;
    backSubContr.matrixB = totMatrixB;
    backSubContr.matrixC = totMatrixC;
    backSubContr.matrixD = totMatrixD;
    backSubContr.vecTrans = totVecTrans;
    backSubContr.vecRot = totVecRot;

    // Prescribed motion translation contributions
    Eigen::Matrix3d omegaPrimeTilde_PB_B = eigenTilde(this->omegaPrime_PB_B);
    this->rPrimePrime_PcB_B = (omegaPrimeTilde_PB_B + this->omegaTilde_PB_B * this->omegaTilde_PB_B) * this->r_PcP_B +
                              this->rPrimePrime_PM_B;
    backSubContr.vecTrans += -this->mass * this->rPrimePrime_PcB_B;

    // Prescribed motion rotation contributions
    Eigen::Matrix3d IPrimePntPc_B;
    IPrimePntPc_B = this->omegaTilde_PB_B * this->IPntPc_B - this->IPntPc_B * this->omegaTilde_PB_B;
    backSubContr.vecRot += -(this->mass * this->rTilde_PcB_B * this->rPrimePrime_PcB_B)
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

    // Loop through attached state effectors for compute derivatives
    if (!this->stateEffectors.empty()) {
        // Update sigma_BN and dcm_BN
        this->sigma_BN = sigma_BN;
        this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();

        // Compute dcm_FN and sigma_FN
        Eigen::Matrix3d dcm_PN = this->dcm_BP.transpose() * this->dcm_BN;
        *this->sigma_PN = eigenMRPd2Vector3d(eigenC2MRP(dcm_PN));

        // Compute omegaDot_PN_P
        Eigen::Vector3d omegaDot_PN_B = this->omegaPrime_PM_B + this->omegaTilde_BN_B * this->omega_PM_B + omegaDot_BN_B;
        Eigen::Vector3d omegaDot_PN_P = this->dcm_BP.transpose() * omegaDot_PN_B;

        // Compute rDDot_PN_N
        Eigen::Matrix3d omegaDotTilde_BN_B = eigenTilde(omegaDot_BN_B);
        Eigen::Vector3d rDDot_PB_B = this->rPrimePrime_PM_B + 2 * this->omegaTilde_BN_B * this->rPrime_PM_B
                                     + omegaDotTilde_BN_B * (*this->r_PB_B)
                                     + this->omegaTilde_BN_B * this->omegaTilde_BN_B * (*this->r_PB_B);
        Eigen::Vector3d rDDot_PN_N = this->dcm_BN.transpose() * rDDot_PB_B + rDDot_BN_N;

        std::vector<StateEffector*>::iterator it;
        for(it = this->stateEffectors.begin(); it != this->stateEffectors.end(); it++) {
            (*it)->computeDerivatives(integTime, rDDot_PN_N, omegaDot_PN_P, *this->sigma_PN);
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
    // Update omega_BN_B, omega_PN_B, and omega_PN_P
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    this->omega_PN_B = this->omega_PB_B + this->omega_BN_B;
    *this->omega_PN_P = this->dcm_BP.transpose() * this->omega_PN_B;

    Eigen::Vector3d totRotAngMomPntC_B;
    totRotAngMomPntC_B.setZero();
    double totRotEnergy = 0.0;

    // Loop through attached state effectors for contributions
    std::vector<StateEffector*>::iterator it;
    for(it = this->stateEffectors.begin(); it != this->stateEffectors.end(); it++) {
        rotAngMomPntCContr_B.setZero();
        rotEnergyContr = 0.0;

        (*it)->updateEnergyMomContributions(integTime, rotAngMomPntCContr_B, rotEnergyContr, *this->omega_PN_P);

        // Additional terms for rotational angular momentum
        Eigen::Vector3d r_EcP_B = this->dcm_BP * (*it)->effProps.rEff_CB_B;
        Eigen::Matrix3d rTilde_EcP_B = eigenTilde(r_EcP_B);
        Eigen::Vector3d rDot_PB_B = *this->rPrime_PB_B + this->omegaTilde_BN_B * *this->r_PB_B;
        Eigen::Matrix3d rTilde_PB_B = eigenTilde(*this->r_PB_B);
        Eigen::Matrix3d omegaTilde_PN_B = eigenTilde(this->omega_PN_B);
        Eigen::Vector3d rDot_EcP_B = this->dcm_BP * (*it)->effProps.rEffPrime_CB_B + omegaTilde_PN_B * r_EcP_B;
        totRotAngMomPntC_B += this->dcm_BP * rotAngMomPntCContr_B
                                + (*it)->effProps.mEff * rTilde_EcP_B * rDot_PB_B
                                + (*it)->effProps.mEff * rTilde_PB_B * rDot_EcP_B
                                + (*it)->effProps.mEff * rTilde_PB_B * rDot_PB_B;

        // Additional terms for rotational energy
        totRotEnergy += rotEnergyContr
                        + (*it)->effProps.mEff * rDot_EcP_B.dot(rDot_PB_B )
                        + 0.5 * (*it)->effProps.mEff * rDot_PB_B.dot(rDot_PB_B);
    }

    rotAngMomPntCContr_B = totRotAngMomPntC_B;
    rotEnergyContr = totRotEnergy;

    // Prescribed motion rotational angular momentum contribution
    this->rDot_PcB_B = this->rPrime_PcB_B + this->omegaTilde_BN_B * this->r_PcB_B;
    rotAngMomPntCContr_B += this->IPntPc_B * this->omega_PN_B + this->mass * this->rTilde_PcB_B * this->rDot_PcB_B;

    // Prescribed motion rotational energy contribution
    rotEnergyContr += 0.5 * this->omega_PN_B.dot(this->IPntPc_B * this->omega_PN_B)
                     + 0.5 * this->mass * this->rDot_PcB_B.dot(this->rDot_PcB_B);
}

/*! This method computes the effector states relative to the inertial frame.

*/
void PrescribedMotionStateEffector::computePrescribedMotionInertialStates()
{
    // Compute the effector's attitude with respect to the inertial frame
    Eigen::Matrix3d dcm_PN = (this->dcm_BP).transpose() * this->dcm_BN;
    *this->sigma_PN = eigenMRPd2Vector3d(eigenC2MRP(dcm_PN));

    // Compute the effector's inertial angular velocity
    *this->omega_PN_P = (this->dcm_BP).transpose() * this->omega_PN_B;

    // Compute the effector's inertial position vectors
    this->r_PcN_N = (Eigen::Vector3d)(*this->inertialPositionProperty) + this->dcm_BN.transpose() * this->r_PcB_B;
    *this->r_PN_N = (Eigen::Vector3d)(*this->inertialPositionProperty) + this->dcm_BN.transpose() * *this->r_PB_B;

    // Compute the effector's inertial velocity vectors
    this->v_PcN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * this->rDot_PcB_B;
    *this->v_PN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * (*this->rPrime_PB_B +
            this->omegaTilde_BN_B * *this->r_PB_B);
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

/*! This method attaches a stateEffector to the prescribedMotionStateEffector */
void PrescribedMotionStateEffector::addStateEffector(StateEffector* newStateEffector)
{
    this->assignStateParamNames<StateEffector *>(newStateEffector);

    this->stateEffectors.push_back(newStateEffector);

    // Give the stateEffector the name of the prescribed object it is attached to
    newStateEffector->nameOfSpacecraftAttachedTo = this->spacecraftName;
}

/*! Setter method for the effector mass.
 @param mass [kg] Effector mass
*/
void PrescribedMotionStateEffector::setMass(const double mass) { this->mass = mass; }

/*! Setter method for IPntPc_P.
 @param IPntPc_P [kg-m^2] Effector's inertia matrix about its center of mass point Pc expressed in P frame components
*/
void PrescribedMotionStateEffector::setIPntPc_P(const Eigen::Matrix3d IPntPc_P) { this->IPntPc_P = IPntPc_P; }

/*! Setter method for r_PcP_P.
 @param r_PcP_P [m] Position vector of the effector's center of mass point Pc relative to the effector's body frame
 origin point P expressed in P frame components
*/
void PrescribedMotionStateEffector::setR_PcP_P(const Eigen::Vector3d r_PcP_P) { this->r_PcP_P = r_PcP_P; }

/*! Setter method for r_PM_M.
 @param r_PM_M [m] Position vector of the effector's body frame origin point P relative to the hub-fixed mount frame
 origin point M expressed in M frame components
*/
void PrescribedMotionStateEffector::setR_PM_M(const Eigen::Vector3d r_PM_M) { this->r_PM_M = r_PM_M; }

/*! Setter method for rPrime_PM_M.
 @param rPrime_PM_M [m/s] B frame time derivative of r_PM_M expressed in M frame components
*/
void PrescribedMotionStateEffector::setRPrime_PM_M(const Eigen::Vector3d rPrime_PM_M) {
    this->rPrime_PM_M = rPrime_PM_M;
}

/*! Setter method for rPrimePrime_PM_M.
 @param rPrimePrime_PM_M [m/s^2] B frame time derivative of rPrime_PM_M expressed in M frame components
*/
void PrescribedMotionStateEffector::setRPrimePrime_PM_M(const Eigen::Vector3d rPrimePrime_PM_M) {
    this->rPrimePrime_PM_M = rPrimePrime_PM_M;
}

/*! Setter method for omega_PM_P.
 @param omega_PM_P [rad/s] Angular velocity of the effector body frame P relative to the hub-fixed mount frame M
 expressed in P frame components
*/
void PrescribedMotionStateEffector::setOmega_PM_P(const Eigen::Vector3d omega_PM_P) { this->omega_PM_P = omega_PM_P; }

/*! Setter method for omegaPrime_PM_P.
 @param omegaPrime_PM_P [rad/s^2] Angular acceleration of the effector body frame P relative to the hub-fixed mount
 frame M expressed in P frame components
*/
void PrescribedMotionStateEffector::setOmegaPrime_PM_P(const Eigen::Vector3d omegaPrime_PM_P) {
    this->omegaPrime_PM_P = omegaPrime_PM_P;
}

/*! Setter method for sigma_PM.
 @param sigma_PM MRP attitude of the effector's body frame P relative to the hub-fixed mount frame M
*/
void PrescribedMotionStateEffector::setSigma_PM(const Eigen::MRPd sigma_PM) { this->sigma_PM = sigma_PM; }

/*! Setter method for r_MB_B.
 @param r_MB_B [m] Position vector describing the hub-fixed mount frame origin point M location relative to the hub
 frame origin point B expressed in B frame components
*/
void PrescribedMotionStateEffector::setR_MB_B(const Eigen::Vector3d r_MB_B) { this->r_MB_B = r_MB_B; }

/*! Setter method for sigma_MB.
 @param sigma_MB MRP attitude of the hub-fixed frame M relative to the hub body frame B
*/
void PrescribedMotionStateEffector::setSigma_MB(const Eigen::MRPd sigma_MB) { this->sigma_MB = sigma_MB; }

/*! Getter method for the effector mass.
 @return double
*/
double PrescribedMotionStateEffector::getMass() const { return this->mass; }

/*! Getter method for IPntPc_P.
 @return const Eigen::Matrix3d
*/
const Eigen::Matrix3d PrescribedMotionStateEffector::getIPntPc_P() const { return this->IPntPc_P; }

/*! Getter method for r_PcP_P.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getR_PcP_P() const { return this->r_PcP_P; }

/*! Getter method for r_PM_M.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getR_PM_M() const { return this->r_PM_M; }

/*! Getter method for rPrime_PM_M.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getRPrime_PM_M() const { return this->rPrime_PM_M; }

/*! Getter method for rPrimePrime_PM_M.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getRPrimePrime_PM_M() const { return this->rPrimePrime_PM_M; }

/*! Getter method for omega_PM_P.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getOmega_PM_P() const { return this->omega_PM_P; }

/*! Getter method for omegaPrime_PM_P.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getOmegaPrime_PM_P() const { return this->omegaPrime_PM_P; }

/*! Getter method for sigma_PM.
 @return const Eigen::MRPd
*/
const Eigen::MRPd PrescribedMotionStateEffector::getSigma_PM() const { return this->sigma_PM; }

/*! Getter method for r_MB_B.
 @return const Eigen::Vector3d
*/
const Eigen::Vector3d PrescribedMotionStateEffector::getR_MB_B() const { return this->r_MB_B; }

/*! Getter method for sigma_MB.
 @return const Eigen::MRPd
*/
const Eigen::MRPd PrescribedMotionStateEffector::getSigma_MB() const { return this->sigma_MB; }
