/*
 ISC License

 Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "spinningBodyOneDOFStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <string>

/*! This is the constructor, setting variables to default values */
SpinningBodyOneDOFStateEffector::SpinningBodyOneDOFStateEffector()
{
    // Zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);

    // Initialize variables to working values
    this->IPntSc_S.setIdentity();
    this->dcm_S0B.setIdentity();
    this->r_SB_B.setZero();
    this->sHat_S.setZero();
    this->dcm_BS.setIdentity();
    this->rTilde_ScB_B.setZero();
    this->omegaTilde_SB_B.setZero();
    this->omegaTilde_BN_B.setZero();
    this->dcm_BS.setIdentity();
    this->dcm_BN.setIdentity();
    this->IPntSc_B.setIdentity();

    this->nameOfThetaState = "spinningBodyTheta" + std::to_string(SpinningBodyOneDOFStateEffector::effectorID);
    this->nameOfThetaDotState = "spinningBodyThetaDot" + std::to_string(SpinningBodyOneDOFStateEffector::effectorID);
    this->nameOfInertialPositionProperty = "spinningBodyInertialPosition" + std::to_string(SpinningBodyOneDOFStateEffector::effectorID);
    this->nameOfInertialVelocityProperty = "spinningBodyInertialVelocity" + std::to_string(SpinningBodyOneDOFStateEffector::effectorID);
    this->nameOfInertialAttitudeProperty = "spinningBodyInertialAttitude" + std::to_string(SpinningBodyOneDOFStateEffector::effectorID);
    this->nameOfInertialAngVelocityProperty = "spinningBodyInertialAngVelocity" + std::to_string(SpinningBodyOneDOFStateEffector::effectorID);
    SpinningBodyOneDOFStateEffector::effectorID++;
}

uint64_t SpinningBodyOneDOFStateEffector::effectorID = 1;

/*! This is the destructor, nothing to report here */
SpinningBodyOneDOFStateEffector::~SpinningBodyOneDOFStateEffector()
{
    SpinningBodyOneDOFStateEffector::effectorID = 1;
}

/*! This method is used to reset the module. */
void SpinningBodyOneDOFStateEffector::Reset(uint64_t CurrentClock)
{
    // Normalize the sHat vector
    if (this->sHat_S.norm() > 0.01) {
        this->sHat_S.normalize();
    }
    else {
        bskLogger.bskLog(BSK_ERROR, "Norm of sHat must be greater than 0. sHat may not have been set by the user.");
    }
}


/*! This method takes the computed theta states and outputs them to the messaging system. */
void SpinningBodyOneDOFStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{
    // Write out the spinning body output messages
    if (this->spinningBodyOutMsg.isLinked()) {
        HingedRigidBodyMsgPayload spinningBodyBuffer;
        spinningBodyBuffer = this->spinningBodyOutMsg.zeroMsgPayload;
        spinningBodyBuffer.theta = this->theta;
        spinningBodyBuffer.thetaDot = this->thetaDot;
        this->spinningBodyOutMsg.write(&spinningBodyBuffer, this->moduleID, CurrentClock);
    }

    // Write out the spinning body state config log message
    if (this->spinningBodyConfigLogOutMsg.isLinked()) {
        SCStatesMsgPayload configLogMsg;
        configLogMsg = this->spinningBodyConfigLogOutMsg.zeroMsgPayload;

        // Logging the S frame is the body frame B of that object
        eigenVector3d2CArray(this->r_ScN_N, configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_ScN_N, configLogMsg.v_BN_N);
        eigenMatrixXd2CArray(*this->sigma_SN, configLogMsg.sigma_BN);
        eigenMatrixXd2CArray(*this->omega_SN_S, configLogMsg.omega_BN_B);
        this->spinningBodyConfigLogOutMsg.write(&configLogMsg, this->moduleID, CurrentClock);
    }
}

/*! This method prepends the name of the spacecraft for multi-spacecraft simulations.*/
void SpinningBodyOneDOFStateEffector::prependSpacecraftNameToStates()
{
    this->nameOfThetaState = this->nameOfSpacecraftAttachedTo + this->nameOfThetaState;
    this->nameOfThetaDotState = this->nameOfSpacecraftAttachedTo + this->nameOfThetaDotState;
}

/*! This method allows the SB state effector to have access to the hub states and gravity*/
void SpinningBodyOneDOFStateEffector::linkInStates(DynParamManager& statesIn)
{
    // Get access to the hub's states needed for dynamic coupling
    this->hubOmega = statesIn.getStateObject("hubOmega");

    // Get access to properties needed for dynamic coupling (Hub or prescribed)
    this->inertialPositionProperty = statesIn.getPropertyReference(this->propName_inertialPosition);
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->propName_inertialVelocity);
}

/*! This method is used to link prescribed motion properties */
void SpinningBodyOneDOFStateEffector::linkInPrescribedMotionProperties(DynParamManager& properties)
{
    this->prescribedPositionProperty = properties.getPropertyReference(this->propName_prescribedPosition);
    this->prescribedVelocityProperty = properties.getPropertyReference(this->propName_prescribedVelocity);
    this->prescribedAccelerationProperty = properties.getPropertyReference(this->propName_prescribedAcceleration);
    this->prescribedAttitudeProperty = properties.getPropertyReference(this->propName_prescribedAttitude);
    this->prescribedAngVelocityProperty = properties.getPropertyReference(this->propName_prescribedAngVelocity);
    this->prescribedAngAccelerationProperty = properties.getPropertyReference(this->propName_prescribedAngAcceleration);
}

/*! This method allows the SB state effector to register its states: theta and thetaDot with the dynamic parameter manager */
void SpinningBodyOneDOFStateEffector::registerStates(DynParamManager& states)
{
    // Register the theta state
    this->thetaState = states.registerState(1, 1, this->nameOfThetaState);
    Eigen::MatrixXd thetaInitMatrix(1,1);
    thetaInitMatrix(0,0) = this->thetaInit;
    this->thetaState->setState(thetaInitMatrix);

    // Register the thetaDot state
    this->thetaDotState = states.registerState(1, 1, this->nameOfThetaDotState);
    Eigen::MatrixXd thetaDotInitMatrix(1,1);
    thetaDotInitMatrix(0,0) = this->thetaDotInit;
    this->thetaDotState->setState(thetaDotInitMatrix);

    registerProperties(states);
}

/*! This method attaches a dynamicEffector
 @param newDynamicEffector the dynamic effector to be attached to the SB
 @param segment defaults to the only segment for 1DOF (base segment 1) */
void SpinningBodyOneDOFStateEffector::addDynamicEffector(DynamicEffector *newDynamicEffector, int segment)
{
    if (segment != 1) {
        bskLogger.bskLog(BSK_ERROR, "Specifying attachment to a non-existent spinning bodies linkage.");
    }

    this->assignStateParamNames<DynamicEffector *>(newDynamicEffector);

    this->dynEffectors.push_back(newDynamicEffector);
}

/*! This method registers the SB inertial properties with the dynamic parameter manager and links
 them into dependent dynamic effectors  */
void SpinningBodyOneDOFStateEffector::registerProperties(DynParamManager& states)
{
    Eigen::Vector3d stateInit = Eigen::Vector3d::Zero();
    this->r_SN_N = states.createProperty(this->nameOfInertialPositionProperty, stateInit);
    this->v_SN_N = states.createProperty(this->nameOfInertialVelocityProperty, stateInit);
    this->sigma_SN = states.createProperty(this->nameOfInertialAttitudeProperty, stateInit);
    this->omega_SN_S = states.createProperty(this->nameOfInertialAngVelocityProperty, stateInit);

    std::vector<DynamicEffector*>::iterator dynIt;
    for(dynIt = this->dynEffectors.begin(); dynIt != this->dynEffectors.end(); dynIt++)
    {
        (*dynIt)->linkInProperties(states);
    }
}

/*! This method allows the SB state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft */
void SpinningBodyOneDOFStateEffector::updateEffectorMassProps(double integTime)
{
    // Give the mass of the spinning body to the effProps mass
    this->effProps.mEff = this->mass;

    // Lock the axis if the flag is set to 1
    if (this->lockFlag == 1)
    {
        Eigen::MatrixXd zeroMatrix = Eigen::MatrixXd::Constant(1, 1, 0.0);
        this->thetaDotState->setState(zeroMatrix);
    }

    // Grab current states
    this->theta = this->thetaState->getState()(0, 0);
    this->thetaDot = this->thetaDotState->getState()(0, 0);

    // Compute the DCM from S frame to B frame and write sHat in B frame
    double dcm_S0S[3][3];
    double prv_S0S_array[3];
    Eigen::Vector3d prv_S0S = -this->theta * this->sHat_S;
    eigenVector3d2CArray(prv_S0S, prv_S0S_array);
    PRV2C(prv_S0S_array, dcm_S0S);
    this->dcm_BS = this->dcm_S0B.transpose() * c2DArray2EigenMatrix3d(dcm_S0S);
    this->sHat_B = this->dcm_BS * this->sHat_S;

    // Compute the effector's CoM with respect to point B
    this->r_ScS_B = this->dcm_BS * this->r_ScS_S;
    this->r_ScB_B = this->r_ScS_B + this->r_SB_B;
    this->effProps.rEff_CB_B = this->r_ScB_B;

    // Find the inertia of the hinged rigid body about point B
    this->rTilde_ScB_B = eigenTilde(this->r_ScB_B);
    this->IPntSc_B = this->dcm_BS * this->IPntSc_S * this->dcm_BS.transpose();
    this->effProps.IEffPntB_B = this->IPntSc_B - this->mass * this->rTilde_ScB_B * this->rTilde_ScB_B;

    // Define omega_SB_B and its cross product operator
    this->omega_SB_B = this->thetaDot * this->sHat_B;
    this->omegaTilde_SB_B = eigenTilde(this->omega_SB_B);

    // Find rPrime_ScB_B
    this->rPrime_ScS_B = this->omegaTilde_SB_B * this->r_ScS_B;
    this->rPrime_ScB_B = this->rPrime_ScS_B;
    this->effProps.rEffPrime_CB_B = this->rPrime_ScB_B;

    // Find body time derivative of IPntSc_B
    Eigen::Matrix3d rPrimeTilde_ScB_B = eigenTilde(this->rPrime_ScB_B);
    this->effProps.IEffPrimePntB_B = this->omegaTilde_SB_B* this->IPntSc_B - this->IPntSc_B *this->omegaTilde_SB_B
        - this->mass * (rPrimeTilde_ScB_B * this->rTilde_ScB_B + this->rTilde_ScB_B * rPrimeTilde_ScB_B);
}

/*! This method allows the SB state effector to give its contributions to the matrices needed for the back-sub
 method */
void SpinningBodyOneDOFStateEffector::updateContributions(double integTime,
                                                          BackSubMatrices & backSubContr,
                                                          Eigen::Vector3d sigma_BN,
                                                          Eigen::Vector3d omega_BN_B,
                                                          Eigen::Vector3d g_N)
{
    // Define omega_SN_B
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    this->omega_SN_B = this->omega_SB_B + this->omega_BN_B;
    Eigen::Matrix3d omegaTilde_SN_B = eigenTilde(this->omega_SN_B);

    // Define IPntS_B for compactness
    Eigen::Matrix3d rTilde_ScS_B = eigenTilde(this->r_ScS_B);
    Eigen::Matrix3d IPntS_B = this->IPntSc_B - this->mass * rTilde_ScS_B * rTilde_ScS_B;

    // Find the DCM from N to B frames
    this->sigma_BN = sigma_BN;
    this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();

    // Map gravity to body frame
    const Eigen::Vector3d& gLocal_N = g_N;
    Eigen::Vector3d g_B = this->dcm_BN * gLocal_N;

    // Define auxiliary variable mTheta
    this->mTheta = this->sHat_B.transpose() * IPntS_B * this->sHat_B;

    // Loop through to collect forces and torques from any connected dynamic effectors
    Eigen::Vector3d attBodyForce_S = Eigen::Vector3d::Zero();  // sum of external forces in S frame components
    Eigen::Vector3d attBodyTorquePntS_S = Eigen::Vector3d::Zero();  // sum of external torque about point S in S frame components
    std::vector<DynamicEffector*>::iterator dynIt;
    for(dynIt = this->dynEffectors.begin(); dynIt != this->dynEffectors.end(); dynIt++)
    {
        // Compute the force and torque contributions from the dynamicEffectors in parent frame
        (*dynIt)->computeForceTorque(integTime, double(0.0));
        attBodyForce_S += (*dynIt)->forceExternal_B; // here parent frame is S, not B
        attBodyTorquePntS_S += (*dynIt)->torqueExternalPntB_B; // here parent frame is S, not B
    }

    // Lock the axis if the flag is set to 1
    if (this->lockFlag == 1)
    {
        // Define aTheta, bTheta and cTheta
        this->aTheta.setZero();
        this->bTheta.setZero();
        this->cTheta = 0.0;
    }
    else {
        // Define aTheta
        this->aTheta = this->mass * rTilde_ScS_B * this->sHat_B / this->mTheta;

        // Define bTheta
        Eigen::Matrix3d rTilde_SB_B = eigenTilde(this->r_SB_B);
        this->bTheta = -(IPntS_B - this->mass * rTilde_SB_B * rTilde_ScS_B) * this->sHat_B / this->mTheta;

        // Define cTheta
        this->rDot_SB_B = this->omegaTilde_BN_B * this->r_SB_B;
        Eigen::Vector3d gravityTorquePntS_B = rTilde_ScS_B * this->mass * g_B;
        this->cTheta = (this->u - this->k * (this->theta - this->thetaRef) - this->c * (this->thetaDot - this->thetaDotRef)
                + this->sHat_B.dot(gravityTorquePntS_B - omegaTilde_SN_B * IPntS_B * this->omega_SN_B
                - IPntS_B * this->omegaTilde_BN_B * this->omega_SB_B + this->dcm_BS * attBodyTorquePntS_S
                - this->mass * rTilde_ScS_B * this->omegaTilde_BN_B * this->rDot_SB_B)) / this->mTheta;
    }

    // For documentation on contributions see Vaz Carneiro, Allard, Schaub spinning body paper
    // Translation contributions
    backSubContr.matrixA = -this->mass * rTilde_ScS_B * this->sHat_B * this->aTheta.transpose();
    backSubContr.matrixB = -this->mass * rTilde_ScS_B * this->sHat_B * this->bTheta.transpose();
    backSubContr.vecTrans = -this->mass * this->omegaTilde_SB_B * this->rPrime_ScS_B
            + this->mass * rTilde_ScS_B * this->sHat_B * this->cTheta + this->dcm_BS * attBodyForce_S;

    // Rotation contributions
    backSubContr.matrixC = (this->IPntSc_B - this->mass * this->rTilde_ScB_B * rTilde_ScS_B)
                           * this->sHat_B * this->aTheta.transpose();
    backSubContr.matrixD = (this->IPntSc_B - this->mass * this->rTilde_ScB_B * rTilde_ScS_B)
                           * this->sHat_B * this->bTheta.transpose();
    backSubContr.vecRot = -omegaTilde_SN_B * this->IPntSc_B * this->omega_SB_B
            - this->mass * this->omegaTilde_BN_B * this->rTilde_ScB_B * this->rPrime_ScB_B
            - this->mass * this->rTilde_ScB_B * this->omegaTilde_SB_B * this->rPrime_ScS_B
            - (this->IPntSc_B - this->mass * this->rTilde_ScB_B * rTilde_ScS_B) * this->sHat_B * this->cTheta
            + this->dcm_BS * attBodyTorquePntS_S + eigenTilde(this->r_SB_B) * (this->dcm_BS * attBodyForce_S);
}

void SpinningBodyOneDOFStateEffector::addPrescribedMotionCouplingContributions(BackSubMatrices & backSubContr) {

    // Access prescribed motion properties
    Eigen::Vector3d r_PB_B = (Eigen::Vector3d)*this->prescribedPositionProperty;
    Eigen::Vector3d rPrime_PB_B = (Eigen::Vector3d)*this->prescribedVelocityProperty;
    Eigen::Vector3d rPrimePrime_PB_B = (Eigen::Vector3d)*this->prescribedAccelerationProperty;
    Eigen::MRPd sigma_PB;
    sigma_PB = (Eigen::Vector3d)*this->prescribedAttitudeProperty;
    Eigen::Vector3d omega_PB_P = (Eigen::Vector3d)*this->prescribedAngVelocityProperty;
    Eigen::Vector3d omegaPrime_PB_P = (Eigen::Vector3d)*this->prescribedAngAccelerationProperty;
    Eigen::Matrix3d dcm_PB = sigma_PB.toRotationMatrix().transpose();

    // Collect hub states
    Eigen::Vector3d omega_BN_B = this->hubOmega->getState();
    Eigen::Vector3d omega_BN_P = dcm_PB * omega_BN_B;

    // Prescribed motion translation coupling contributions
    Eigen::Vector3d sHat_P = this->sHat_B;
    Eigen::Vector3d r_PB_P = dcm_PB * r_PB_B;
    Eigen::Matrix3d rTilde_PB_P = eigenTilde(r_PB_P);
    Eigen::Vector3d r_ScS_P = this->r_ScS_B;
    Eigen::Matrix3d rTilde_ScS_P = eigenTilde(r_ScS_P);
    backSubContr.matrixB += this->mass * rTilde_ScS_P * sHat_P * this->aTheta.transpose() * rTilde_PB_P;

    Eigen::Matrix3d omegaTilde_PB_P = eigenTilde(omega_PB_P);
    Eigen::Vector3d rPPrime_ScP_P = this->rPrime_ScB_B;
    Eigen::Matrix3d omegaPrimeTilde_PB_P = eigenTilde(omegaPrime_PB_P);
    Eigen::Vector3d r_ScP_P = this->r_ScB_B;
    Eigen::Vector3d rPrimePrime_PB_P = dcm_PB * rPrimePrime_PB_B;
    Eigen::Matrix3d omegaTilde_BN_P = eigenTilde(omega_BN_P);
    Eigen::Vector3d rPrime_PB_P = dcm_PB * rPrime_PB_B;
    Eigen::Vector3d term1 = - 2.0 * this->mass * omegaTilde_PB_P * rPPrime_ScP_P
                            - this->mass * omegaPrimeTilde_PB_P * r_ScP_P
                            - this->mass * omegaTilde_PB_P * omegaTilde_PB_P * r_ScP_P
                            - this->mass * rPrimePrime_PB_P;
    double term2 = this->aTheta.transpose() * (rPrimePrime_PB_P + 2.0 * omegaTilde_BN_P * rPrime_PB_P + omegaTilde_BN_P * omegaTilde_BN_P * r_PB_P);
    double term3 = this->bTheta.transpose() * (omegaPrime_PB_P + omegaTilde_BN_P * omega_PB_P);
    backSubContr.vecTrans += term1 + this->mass * (term2 + term3) * rTilde_ScS_P * sHat_P;

    // Prescribed motion rotation coupling contributions
    backSubContr.matrixC += - this->mass * rTilde_PB_P * rTilde_ScS_P * sHat_P * this->aTheta.transpose();

    Eigen::Matrix3d IPntSc_P = this->IPntSc_B;
    Eigen::Matrix3d rTilde_ScP_P = eigenTilde(r_ScP_P);
    backSubContr.matrixD += - this->mass * rTilde_PB_P * rTilde_ScS_P * sHat_P * this->bTheta.transpose()
                            - (IPntSc_P - this->mass * rTilde_ScP_P * rTilde_ScS_P
                               - this->mass * rTilde_PB_P * rTilde_ScS_P) * sHat_P * this->aTheta.transpose() * rTilde_PB_P;

    Eigen::Vector3d r_ScB_P = r_ScP_P + r_PB_P;
    Eigen::Matrix3d rTilde_ScB_P = eigenTilde(r_ScB_P);
    Eigen::Vector3d omega_SP_P = this->omega_SB_B;
    Eigen::Matrix3d omegaTilde_SP_P = eigenTilde(omega_SP_P);
    Eigen::Matrix3d omegaTilde_SN_B = eigenTilde(this->omega_SN_B);
    Eigen::Matrix3d omegaTilde_SN_P = omegaTilde_SN_B;
    Eigen::Vector3d rPPrime_ScS_P = this->rPrime_ScS_B;
    Eigen::Vector3d omega_PN_P = omega_PB_P + omega_BN_P;
    Eigen::Matrix3d omegaTilde_PN_P = eigenTilde(omega_PN_P);
    backSubContr.vecRot += - IPntSc_P * (omegaTilde_PB_P * omega_SP_P + omegaPrime_PB_P)
                           - omegaTilde_SN_P * IPntSc_P * omega_PB_P
                           - this->mass * rTilde_PB_P * omegaTilde_SP_P * rPPrime_ScS_P
                           - this->mass * rTilde_ScB_P * (2.0 * omegaTilde_PB_P * rPPrime_ScP_P
                                                          + omegaPrimeTilde_PB_P * r_ScP_P
                                                          + omegaTilde_PB_P * omegaTilde_PB_P * r_ScP_P
                                                          + rPrimePrime_PB_P)
                           + this->mass * omegaTilde_PB_P * rTilde_ScP_P * rPPrime_ScP_P
                           - this->mass * omegaTilde_PN_P * rTilde_PB_P * rPPrime_ScP_P
                           + this->mass * omegaTilde_PB_P * rTilde_PB_P * rPPrime_ScP_P
                           - this->mass * omegaTilde_PN_P * rTilde_ScB_P * (omegaTilde_PB_P * r_ScP_P + rPrime_PB_P)
                           + this->mass * omegaTilde_PB_P * rTilde_ScB_P * (omegaTilde_PB_P * r_ScP_P + rPrime_PB_P)
                           - this->mass * this->cTheta * rTilde_PB_P * rTilde_ScS_P * sHat_P
                           - ((IPntSc_P - this->mass * rTilde_ScP_P * rTilde_ScS_P
                               - this->mass * rTilde_PB_P * rTilde_ScS_P) * sHat_P * this->aTheta.transpose())
                             * (rPrimePrime_PB_P + 2.0 * omegaTilde_BN_P * rPrime_PB_P + omegaTilde_BN_P * omegaTilde_BN_P * r_PB_P)
                           - ((IPntSc_P - this->mass * rTilde_ScP_P * rTilde_ScS_P
                               - this->mass * rTilde_PB_P * rTilde_ScS_P) * sHat_P * this->bTheta.transpose())
                             * (omegaPrime_PB_P + omegaTilde_BN_P * omega_PB_P);
}

/*! This method is used to find the derivatives for the SB stateEffector: thetaDDot and the kinematic derivative */
void SpinningBodyOneDOFStateEffector::computeDerivatives(double integTime,
                                                         Eigen::Vector3d rDDot_BN_N,
                                                         Eigen::Vector3d omegaDot_BN_B,
                                                         Eigen::Vector3d sigma_BN)
{
    // Update dcm_BN
    this->sigma_BN = sigma_BN;
    this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();

    // Grab omegaDot_BN_B
    Eigen::Vector3d omegaDotLocal_BN_B;
    omegaDotLocal_BN_B = omegaDot_BN_B;

    // Find rDDotLoc_BN_B
    const Eigen::Vector3d& rDDotLocal_BN_N = rDDot_BN_N;
    Eigen::Vector3d rDDotLocal_BN_B;
    rDDotLocal_BN_B = this->dcm_BN * rDDotLocal_BN_N;

    // Compute Derivatives
    this->thetaState->setDerivative(this->thetaDotState->getState());
    Eigen::MatrixXd thetaDDot(1, 1);
    thetaDDot(0, 0) = this->aTheta.dot(rDDotLocal_BN_B)
            + this->bTheta.dot(omegaDotLocal_BN_B) + this->cTheta;
    this->thetaDotState->setDerivative(thetaDDot);
}

/*! This method is for calculating the contributions of the SB state effector to the energy and momentum of the spacecraft */
void SpinningBodyOneDOFStateEffector::updateEnergyMomContributions(double integTime,
                                                                   Eigen::Vector3d & rotAngMomPntCContr_B,
                                                                   double & rotEnergyContr,
                                                                   Eigen::Vector3d omega_BN_B)
{
    // Update omega_BN_B and omega_SN_B
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    this->omega_SN_B = this->omega_SB_B + this->omega_BN_B;

    // Compute rDot_ScB_B
    this->rDot_ScB_B = this->rPrime_ScB_B + this->omegaTilde_BN_B * this->r_ScB_B;

    // Find rotational angular momentum contribution
    rotAngMomPntCContr_B = this->IPntSc_B * this->omega_SN_B + this->mass * this->rTilde_ScB_B * this->rDot_ScB_B;

    // Find rotational energy contribution
    rotEnergyContr = 1.0 / 2.0 * this->omega_SN_B.dot(this->IPntSc_B * this->omega_SN_B)
                     + 1.0 / 2.0 * this->mass * this->rDot_ScB_B.dot(this->rDot_ScB_B)
                     + 1.0 / 2.0 * this->k * (this->theta - this->thetaRef) * (this->theta - this->thetaRef);
}

/*! This method computes the spinning body states relative to the inertial frame */
void SpinningBodyOneDOFStateEffector::computeSpinningBodyInertialStates()
{
    // inertial attitude
    Eigen::Matrix3d dcm_SN;
    dcm_SN = (this->dcm_BS).transpose() * this->dcm_BN;
    *this->sigma_SN = eigenMRPd2Vector3d(eigenC2MRP(dcm_SN));
    *this->omega_SN_S = (this->dcm_BS).transpose() * this->omega_SN_B;

    // inertial position vector
    this->r_ScN_N = (Eigen::Vector3d)*this->inertialPositionProperty + this->dcm_BN.transpose() * this->r_ScB_B;
    *this->r_SN_N = (Eigen::Vector3d)(*this->inertialPositionProperty) + this->dcm_BN.transpose() * this->r_SB_B;

    // inertial velocity vector
    this->v_ScN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * this->rDot_ScB_B;
    *this->v_SN_N = (Eigen::Vector3d)*this->inertialVelocityProperty + this->dcm_BN.transpose() * this->rDot_SB_B;
}

/*! This method is used so that the simulation will ask SB to update messages */
void SpinningBodyOneDOFStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Read the incoming command array
    if (this->motorTorqueInMsg.isLinked() && this->motorTorqueInMsg.isWritten()) {
        ArrayMotorTorqueMsgPayload incomingCmdBuffer;
        incomingCmdBuffer = this->motorTorqueInMsg();
        this->u = incomingCmdBuffer.motorTorque[0];
    }

    //! - Read the incoming lock command array
    if (this->motorLockInMsg.isLinked() && this->motorLockInMsg.isWritten()) {
        ArrayEffectorLockMsgPayload incomingLockBuffer;
        incomingLockBuffer = this->motorLockInMsg();
        this->lockFlag = incomingLockBuffer.effectorLockFlag[0];
    }

    //! - Read the incoming angle command array
    if (this->spinningBodyRefInMsg.isLinked() && this->spinningBodyRefInMsg.isWritten()) {
        HingedRigidBodyMsgPayload incomingRefBuffer;
        incomingRefBuffer = this->spinningBodyRefInMsg();
        this->thetaRef = incomingRefBuffer.theta;
        this->thetaDotRef = incomingRefBuffer.thetaDot;
    }

    /* Compute spinning body inertial states */
    this->computeSpinningBodyInertialStates();

    this->writeOutputStateMessages(CurrentSimNanos);
}
