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

#include "spinningBodyTwoDOFStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include <string>

/*! This is the constructor, setting variables to default values */
SpinningBodyTwoDOFStateEffector::SpinningBodyTwoDOFStateEffector()
{
    // Zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);

    // Initialize variables to working values
    this->IS1PntSc1_S1.setIdentity();
    this->IS2PntSc2_S2.setIdentity();
    this->dcm_S10B.setIdentity();
    this->dcm_S20S1.setIdentity();
    this->ATheta.setZero();
    this->BTheta.setZero();
    this->rTilde_Sc1B_B.setZero();
    this->rTilde_Sc2B_B.setZero();
    this->omegaTilde_S1B_B.setZero();
    this->omegaTilde_S2B_B.setZero();
    this->omegaTilde_BN_B.setZero();
    this->dcm_BS1.setIdentity();
    this->dcm_BS2.setIdentity();
    this->dcm_BN.setIdentity();
    this->IS1PntSc1_B.setIdentity();
    this->IS2PntSc2_B.setIdentity();
    this->IPrimeS1PntSc1_B.setZero();
    this->IPrimeS2PntSc2_B.setZero();

    this->nameOfTheta1State = "spinningBodyTheta1" + std::to_string(SpinningBodyTwoDOFStateEffector::effectorID);
    this->nameOfTheta1DotState = "spinningBodyTheta1Dot" + std::to_string(SpinningBodyTwoDOFStateEffector::effectorID);
    this->nameOfTheta2State = "spinningBodyTheta2" + std::to_string(SpinningBodyTwoDOFStateEffector::effectorID);
    this->nameOfTheta2DotState = "spinningBodyTheta2Dot" + std::to_string(SpinningBodyTwoDOFStateEffector::effectorID);
    SpinningBodyTwoDOFStateEffector::effectorID++;
}

uint64_t SpinningBodyTwoDOFStateEffector::effectorID = 1;

/*! This is the destructor, nothing to report here */
SpinningBodyTwoDOFStateEffector::~SpinningBodyTwoDOFStateEffector()
{
    SpinningBodyTwoDOFStateEffector::effectorID --;    /* reset the panel ID*/
}

/*! This method is used to reset the module. */
void SpinningBodyTwoDOFStateEffector::Reset(uint64_t CurrentClock)
{
    // Normalize both sHat vectors
    if (this->s1Hat_S1.norm() > 0.01) {
        this->s1Hat_S1.normalize();
    }
    else {
        bskLogger.bskLog(BSK_ERROR, "Norm of s1Hat must be greater than 0. s1Hat may not have been set by the user.");
    }

    if (this->s2Hat_S2.norm() > 0.01) {
        this->s2Hat_S2.normalize();
    }
    else {
        bskLogger.bskLog(BSK_ERROR, "Norm of s2Hat must be greater than 0. s1Hat may not have been set by the user.");
    }
}


/*! This method takes the computed theta states and outputs them to the messaging system. */
void SpinningBodyTwoDOFStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{
    // Write out the spinning body output messages
    HingedRigidBodyMsgPayload spinningBodyBuffer;
    if (this->spinningBodyOutMsgs[0]->isLinked()) {
        spinningBodyBuffer = this->spinningBodyOutMsgs[0]->zeroMsgPayload;
        spinningBodyBuffer.theta = this->theta1;
        spinningBodyBuffer.thetaDot = this->theta1Dot;
        this->spinningBodyOutMsgs[0]->write(&spinningBodyBuffer, this->moduleID, CurrentClock);
    }
    if (this->spinningBodyOutMsgs[1]->isLinked()) {
        spinningBodyBuffer = this->spinningBodyOutMsgs[1]->zeroMsgPayload;
        spinningBodyBuffer.theta = this->theta2;
        spinningBodyBuffer.thetaDot = this->theta2Dot;
        this->spinningBodyOutMsgs[1]->write(&spinningBodyBuffer, this->moduleID, CurrentClock);
    }

    // Write out the spinning body state config log message
    if (this->spinningBodyConfigLogOutMsgs[0]->isLinked()) {
        SCStatesMsgPayload configLogMsg;
        configLogMsg = this->spinningBodyConfigLogOutMsgs[0]->zeroMsgPayload;

        // Logging the S frame is the body frame B of that object
        eigenVector3d2CArray(this->r_Sc1N_N, configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_Sc1N_N, configLogMsg.v_BN_N);
        eigenVector3d2CArray(this->sigma_S1N, configLogMsg.sigma_BN);
        eigenVector3d2CArray(this->omega_S1N_S1, configLogMsg.omega_BN_B);
        this->spinningBodyConfigLogOutMsgs[0]->write(&configLogMsg, this->moduleID, CurrentClock);
    }

    if (this->spinningBodyConfigLogOutMsgs[1]->isLinked()) {
        SCStatesMsgPayload configLogMsg;
        configLogMsg = this->spinningBodyConfigLogOutMsgs[1]->zeroMsgPayload;

        // Logging the S frame is the body frame B of that object
        eigenVector3d2CArray(this->r_Sc2N_N, configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_Sc2N_N, configLogMsg.v_BN_N);
        eigenVector3d2CArray(this->sigma_S2N, configLogMsg.sigma_BN);
        eigenVector3d2CArray(this->omega_S2N_S2, configLogMsg.omega_BN_B);
        this->spinningBodyConfigLogOutMsgs[1]->write(&configLogMsg, this->moduleID, CurrentClock);
    }
}

/*! This method prepends the name of the spacecraft for multi-spacecraft simulations.*/
void SpinningBodyTwoDOFStateEffector::prependSpacecraftNameToStates()
{
    this->nameOfTheta1State = this->nameOfSpacecraftAttachedTo + this->nameOfTheta1State;
    this->nameOfTheta1DotState = this->nameOfSpacecraftAttachedTo + this->nameOfTheta1DotState;
    this->nameOfTheta2State = this->nameOfSpacecraftAttachedTo + this->nameOfTheta2State;
    this->nameOfTheta2DotState = this->nameOfSpacecraftAttachedTo + this->nameOfTheta2DotState;
}

/*! This method allows the SB state effector to have access to the hub states and gravity*/
void SpinningBodyTwoDOFStateEffector::linkInStates(DynParamManager& statesIn)
{
    this->inertialPositionProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialPosition);
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialVelocity);
}

/*! This method allows the SB state effector to register its states: theta and thetaDot with the dynamic parameter manager */
void SpinningBodyTwoDOFStateEffector::registerStates(DynParamManager& states)
{
    // Register the theta states
    this->theta1State = states.registerState(1, 1, this->nameOfTheta1State);
    this->theta2State = states.registerState(1, 1, this->nameOfTheta2State);
    Eigen::MatrixXd thetaInitMatrix(2,1);
    thetaInitMatrix(0,0) = this->theta1Init;
    thetaInitMatrix(1,0) = this->theta2Init;
    this->theta1State->setState(thetaInitMatrix.row(0));
    this->theta2State->setState(thetaInitMatrix.row(1));

    // Register the thetaDot states
    this->theta1DotState = states.registerState(1, 1, this->nameOfTheta1DotState);
    this->theta2DotState = states.registerState(1, 1, this->nameOfTheta2DotState);
    Eigen::MatrixXd thetaDotInitMatrix(2,1);
    thetaDotInitMatrix(0,0) = this->theta1DotInit;
    thetaDotInitMatrix(1,0) = this->theta2DotInit;
    this->theta1DotState->setState(thetaDotInitMatrix.row(0));
    this->theta2DotState->setState(thetaDotInitMatrix.row(1));
}

/*! This method allows the SB state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft */
void SpinningBodyTwoDOFStateEffector::updateEffectorMassProps(double integTime)
{
    // Give the mass of the spinning body to the effProps mass
    this->mass = this->mass1 + this->mass2;
    this->effProps.mEff = this->mass;

    // Lock the axis if the flag is set to 1
    Eigen::MatrixXd zeroMatrix = Eigen::MatrixXd::Constant(1, 1, 0.0);
    if (this->lockFlag1 == 1)
    {
        this->theta1DotState->setState(zeroMatrix);
    }
    if (this->lockFlag2 == 1)
    {
        this->theta2DotState->setState(zeroMatrix);
    }

    // Grab current states
    this->theta1 = this->theta1State->getState()(0, 0);
    this->theta1Dot = this->theta1DotState->getState()(0, 0);
    this->theta2 = this->theta2State->getState()(0, 0);
    this->theta2Dot = this->theta2DotState->getState()(0, 0);

    // Compute the DCM from both S frames to B frame
    double dcm_S0S[3][3];
    double prv_S0S_array[3];
    Eigen::Vector3d prv_S0S;
    prv_S0S  = - this->theta1 * this->s1Hat_S1;
    eigenVector3d2CArray(prv_S0S, prv_S0S_array);
    PRV2C(prv_S0S_array, dcm_S0S);
    this->dcm_BS1 = this->dcm_S10B.transpose() * c2DArray2EigenMatrix3d(dcm_S0S);
    prv_S0S = - this->theta2 * this->s2Hat_S2;
    eigenVector3d2CArray(prv_S0S, prv_S0S_array);
    PRV2C(prv_S0S_array, dcm_S0S);
    this->dcm_BS2 = this->dcm_BS1 * this->dcm_S20S1.transpose() * c2DArray2EigenMatrix3d(dcm_S0S);

    // Write the spinning axis in B frame
    this->s1Hat_B = this->dcm_BS1 * this->s1Hat_S1;
    this->s2Hat_B = this->dcm_BS2 * this->s2Hat_S2;

    // Compute the effector's CoM with respect to point B
    this->r_Sc1S1_B = this->dcm_BS1 * this->r_Sc1S1_S1;
    this->r_Sc1B_B = this->r_Sc1S1_B + this->r_S1B_B;
    this->r_Sc2S2_B = this->dcm_BS2 * this->r_Sc2S2_S2;
    this->r_S2S1_B = this->dcm_BS1 * this->r_S2S1_S1;
    this->r_Sc2S1_B = this->r_Sc2S2_B + this->r_S2S1_B;
    this->r_Sc2B_B = this->r_Sc2S1_B + this->r_S1B_B;
    this->r_ScB_B = (this->mass1 * this->r_Sc1B_B + this->mass2 * this->r_Sc2B_B) / this->mass;
    this->effProps.rEff_CB_B = this->r_ScB_B;

    // Find the inertia of the hinged rigid bodies about point B
    this->rTilde_Sc1B_B = eigenTilde(this->r_Sc1B_B);
    this->IS1PntSc1_B = this->dcm_BS1 * this->IS1PntSc1_S1 * this->dcm_BS1.transpose();
    this->rTilde_Sc2B_B = eigenTilde(this->r_Sc2B_B);
    this->IS2PntSc2_B = this->dcm_BS2 * this->IS2PntSc2_S2 * this->dcm_BS2.transpose();
    this->effProps.IEffPntB_B = this->IS1PntSc1_B + this->IS2PntSc2_B - this->mass1 * this->rTilde_Sc1B_B * this->rTilde_Sc1B_B - this->mass2 * this->rTilde_Sc2B_B * this->rTilde_Sc2B_B;

    // Define omega_S1B_B, omega_S2S1_B, omega_S2B_B, and their cross product operator
    this->omega_S1B_B = this->theta1Dot * this->s1Hat_B;
    this->omega_S2S1_B = this->theta2Dot * this->s2Hat_B;
    this->omega_S2B_B = this->omega_S2S1_B + this->omega_S1B_B;
    this->omegaTilde_S1B_B = eigenTilde(this->omega_S1B_B);
    this->omegaTilde_S2B_B = eigenTilde(this->omega_S2B_B);

    // Find rPrime_Sc1B_B and rPrime_Sc2B_B
    this->rPrime_Sc1S1_B = this->omegaTilde_S1B_B * this->r_Sc1S1_B;
    this->rPrime_Sc1B_B = this->rPrime_Sc1S1_B;
    this->rPrime_Sc2S2_B = this->omegaTilde_S2B_B * this->r_Sc2S2_B;
    this->rPrime_S2S1_B = this->omegaTilde_S1B_B * this->r_S2S1_B;
    this->rPrime_Sc2S1_B = this->rPrime_Sc2S2_B + this->rPrime_S2S1_B;
    this->rPrime_Sc2B_B = this->rPrime_Sc2S1_B;
    this->rPrime_ScB_B = (this->mass1 * this->rPrime_Sc1B_B + this->mass2 * this->rPrime_Sc2B_B) / this->mass;
    this->effProps.rEffPrime_CB_B = this->rPrime_ScB_B;

    // Find the body-frame time derivative of the inertias of each spinner
    this->IPrimeS1PntSc1_B = this->omegaTilde_S1B_B * this->IS1PntSc1_B - this->IS1PntSc1_B * this->omegaTilde_S1B_B;
    this->IPrimeS2PntSc2_B = this->omegaTilde_S2B_B * this->IS2PntSc2_B - this->IS2PntSc2_B * this->omegaTilde_S2B_B;

    // Find body time derivative of IPntSc_B
    Eigen::Matrix3d rPrimeTilde_Sc1B_B = eigenTilde(this->rPrime_Sc1B_B);
    Eigen::Matrix3d rPrimeTilde_Sc2B_B = eigenTilde(this->rPrime_Sc2B_B);
    this->effProps.IEffPrimePntB_B = this->IPrimeS1PntSc1_B + this->IPrimeS2PntSc2_B
        - this->mass1 * (rPrimeTilde_Sc1B_B * this->rTilde_Sc1B_B + this->rTilde_Sc1B_B * rPrimeTilde_Sc1B_B)
        - this->mass2 * (rPrimeTilde_Sc2B_B * this->rTilde_Sc2B_B + this->rTilde_Sc2B_B * rPrimeTilde_Sc2B_B);
}

/*! This method allows the SB state effector to give its contributions to the matrices needed for the back-sub
 method */
void SpinningBodyTwoDOFStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    // Find the DCM from N to B frames
    this->sigma_BN = sigma_BN;
    this->dcm_BN = (this->sigma_BN.toRotationMatrix()).transpose();

    // Map gravity to body frame
    Eigen::Vector3d gLocal_N;
    Eigen::Vector3d g_B;
    gLocal_N = g_N;
    g_B = this->dcm_BN * gLocal_N;

    // Update omega_BN_B
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    this->omega_S1N_B = this->omega_S1B_B + this->omega_BN_B;
    this->omega_S2N_B = this->omega_S2B_B + this->omega_BN_B;

    // Define auxiliary position vectors
    Eigen::Vector3d r_ScS1_B = (this->mass1 * this->r_Sc1S1_B + this->mass2 * this->r_Sc2S1_B) / this->mass;
    Eigen::Vector3d r_S2B_B = this->r_S2S1_B + this->r_S1B_B;

    // Define auxiliary rtilde matrices
    Eigen::Matrix3d rTilde_Sc1S1_B = eigenTilde(this->r_Sc1S1_B);
    Eigen::Matrix3d rTilde_S1B_B = eigenTilde(this->r_S1B_B);
    Eigen::Matrix3d rTilde_Sc2S1_B = eigenTilde(r_Sc2S1_B);
    Eigen::Matrix3d rTilde_Sc2S2_B = eigenTilde(this->r_Sc2S2_B);
    Eigen::Matrix3d rTilde_S2S1_B = eigenTilde(this->r_S2S1_B);
    Eigen::Matrix3d rTilde_S2B_B = eigenTilde(r_S2B_B);
    Eigen::Matrix3d rTilde_ScS1_B = eigenTilde(r_ScS1_B);
    Eigen::Matrix3d rPrimeTilde_Sc1S1_B = eigenTilde(this->rPrime_Sc1S1_B);
    Eigen::Matrix3d rPrimeTilde_Sc2S1_B = eigenTilde(this->rPrime_Sc2S1_B);
    Eigen::Matrix3d rPrimeTilde_Sc2S2_B = eigenTilde(this->rPrime_Sc2S2_B);

    // Define auxiliary omegaTilde matrices
    Eigen::Matrix3d omegaTilde_S2S1_B = eigenTilde(this->omega_S2S1_B);
    Eigen::Matrix3d omegaTilde_S1N_B = eigenTilde(this->omega_S1N_B);

    // Define auxiliary inertia matrices
    Eigen::Matrix3d IS2PntS2_B = this->IS2PntSc2_B - this->mass2 * rTilde_Sc2S2_B * rTilde_Sc2S2_B;
    Eigen::Matrix3d ISPntS1_B = this->IS1PntSc1_B - this->mass1 * rTilde_Sc1S1_B * rTilde_Sc1S1_B + this->IS2PntSc2_B - this->mass2 * rTilde_Sc2S1_B * rTilde_Sc2S1_B;
    Eigen::Matrix3d IPrimeSPntS1_B = this->IPrimeS1PntSc1_B - this->mass1 * (rPrimeTilde_Sc1S1_B * rTilde_Sc1S1_B + rTilde_Sc1S1_B * rPrimeTilde_Sc1S1_B)
                                   + this->IPrimeS2PntSc2_B - this->mass2 * (rPrimeTilde_Sc2S1_B * rTilde_Sc2S1_B + rTilde_Sc2S1_B * rPrimeTilde_Sc2S1_B);
    Eigen::Matrix3d IPrimeS2PntS2_B = this->IPrimeS2PntSc2_B - this->mass2 * (rPrimeTilde_Sc2S2_B * rTilde_Sc2S2_B + rTilde_Sc2S2_B * rPrimeTilde_Sc2S2_B);

    // Define and populate the mass matrix for thetaDDot
    Eigen::Matrix2d MTheta;
    MTheta << this->s1Hat_B.transpose() * ISPntS1_B * this->s1Hat_B, this->s1Hat_B.transpose() * (this->IS2PntSc2_B - this->mass2 * rTilde_Sc2S1_B * rTilde_Sc2S2_B) * this->s2Hat_B,
              this->s2Hat_B.transpose() * (IS2PntS2_B - this->mass2 * rTilde_Sc2S2_B * rTilde_S2S1_B) * this->s1Hat_B, this->s2Hat_B.transpose() * IS2PntS2_B * this->s2Hat_B;

    // Define AThetaStar matrix
    Eigen::Matrix<double, 2, 3> AThetaStar;
    AThetaStar.row(0) = - this->mass * this->s1Hat_B.transpose() * rTilde_ScS1_B;
    AThetaStar.row(1) = - this->mass2 * this->s2Hat_B.transpose() * rTilde_Sc2S2_B;

    // Define BThetaStar matrix
    Eigen::Matrix<double, 2, 3> BThetaStar;
    BThetaStar.row(0) = - this->s1Hat_B.transpose() * (ISPntS1_B - this->mass * rTilde_ScS1_B * rTilde_S1B_B);
    BThetaStar.row(1) = - this->s2Hat_B.transpose() * (IS2PntS2_B - this->mass2 * rTilde_Sc2S2_B * rTilde_S2B_B);

    // Define CThetaStar vector
    Eigen::Vector3d rDot_S1B_B = this->omegaTilde_BN_B * this->r_S1B_B;
    Eigen::Vector3d rDot_S2S1_B = this->rPrime_S2S1_B + this->omegaTilde_BN_B * this->r_S2S1_B;
    Eigen::Vector3d gravityTorquePntS1_B = rTilde_ScS1_B * this->mass * g_B;
    Eigen::Vector3d gravityTorquePntS2_B = rTilde_Sc2S2_B * this->mass2 * g_B;
    Eigen::Vector2d CThetaStar;
    CThetaStar(0,0) = this->u1 - this->k1 * (this->theta1 - this->theta1Ref)
        - this->c1 * (this->theta1Dot - this->theta1DotRef) + this->s1Hat_B.transpose() * gravityTorquePntS1_B
        - this->s1Hat_B.transpose() * ((IPrimeSPntS1_B + this->omegaTilde_BN_B * ISPntS1_B) * this->omega_BN_B
        + (this->IPrimeS1PntSc1_B + this->omegaTilde_BN_B * this->IS1PntSc1_B) * this->omega_S1B_B
        + (this->IPrimeS2PntSc2_B + this->omegaTilde_BN_B * this->IS2PntSc2_B) * this->omega_S2B_B
        + (this->IS2PntSc2_B - this->mass2 * rTilde_Sc2S1_B * rTilde_Sc2S2_B) * this->omegaTilde_S1B_B * this->omega_S2S1_B
        + this->mass1 * (rTilde_Sc1S1_B * this->omegaTilde_S1B_B + this->omegaTilde_BN_B * rTilde_Sc1S1_B) * this->rPrime_Sc1S1_B
        + this->mass2 * (rTilde_Sc2S1_B * this->omegaTilde_S1B_B + this->omegaTilde_BN_B * rTilde_Sc2S1_B) * this->rPrime_Sc2S1_B
        + this->mass2 * rTilde_Sc2S1_B * omegaTilde_S2S1_B * this->rPrime_Sc2S2_B
        + this->mass * rTilde_ScS1_B * this->omegaTilde_BN_B * rDot_S1B_B);
    CThetaStar(1, 0) = this->u2 - this->k2 * (this->theta2 - this->theta2Ref)
        - this->c2 * (this->theta2Dot - this->theta2DotRef) + this->s2Hat_B.transpose() * gravityTorquePntS2_B
        - this->s2Hat_B.transpose() * ((IPrimeS2PntS2_B + this->omegaTilde_BN_B * IS2PntS2_B) * this->omega_S2N_B
        + IS2PntS2_B * this->omegaTilde_S1B_B * this->omega_S2S1_B + this->mass2 * rTilde_Sc2S2_B * omegaTilde_S1N_B * this->rPrime_S2S1_B
        + this->mass2 * rTilde_Sc2S2_B * this->omegaTilde_BN_B * (rDot_S2S1_B + rDot_S1B_B));

    // Check if any of the axis are locked and change dynamics accordingly
    if (this->lockFlag1 == 1 || this->lockFlag2 == 1)
    {
        // Remove cross-coupling
        MTheta(0, 1) = 0.0;
        MTheta(1, 0) = 0.0;

        if (this->lockFlag1 == 1)
        {
            AThetaStar.row(0).setZero();
            BThetaStar.row(0).setZero();
            CThetaStar.row(0).setZero();

        }
        if (this->lockFlag2 == 1)
        {
            AThetaStar.row(1).setZero();
            BThetaStar.row(1).setZero();
            CThetaStar.row(1).setZero();
        }
    }

    // Define the ATheta, BTheta and CTheta matrices
    this->ATheta = MTheta.inverse() * AThetaStar;
    this->BTheta = MTheta.inverse() * BThetaStar;
    this->CTheta = MTheta.inverse() * CThetaStar;

    // For documentation on contributions see Vaz Carneiro, Allard, Schaub spinning body paper
    // Translation contributions
    backSubContr.matrixA = - this->mass * rTilde_ScS1_B * this->s1Hat_B * this->ATheta.row(0) - this->mass2 * rTilde_Sc2S2_B * this->s2Hat_B * this->ATheta.row(1);
    backSubContr.matrixB = - this->mass * rTilde_ScS1_B * this->s1Hat_B * this->BTheta.row(0) - this->mass2 * rTilde_Sc2S2_B * this->s2Hat_B * this->BTheta.row(1);
    backSubContr.vecTrans = - this->mass * this->omegaTilde_S1B_B * this->rPrime_ScB_B - this->mass2 * (omegaTilde_S2S1_B * this->rPrime_Sc2S2_B - rTilde_Sc2S2_B * this->omegaTilde_S1B_B * this->omega_S2S1_B)
        + this->mass * rTilde_ScS1_B * this->s1Hat_B * this->CTheta.row(0) + this->mass2 * rTilde_Sc2S2_B * this->s2Hat_B * this->CTheta.row(1);

    // Rotation contributions
    backSubContr.matrixC = (this->IS1PntSc1_B + this->IS2PntSc2_B - this->mass1 * this->rTilde_Sc1B_B * rTilde_Sc1S1_B - this->mass2 * this->rTilde_Sc2B_B * rTilde_Sc2S1_B) * this->s1Hat_B * this->ATheta.row(0)
        + (this->IS2PntSc2_B - this->mass2 * this->rTilde_Sc2B_B * rTilde_Sc2S2_B) * this->s2Hat_B * this->ATheta.row(1);
    backSubContr.matrixD = (this->IS1PntSc1_B + this->IS2PntSc2_B - this->mass1 * this->rTilde_Sc1B_B * rTilde_Sc1S1_B - this->mass2 * this->rTilde_Sc2B_B * rTilde_Sc2S1_B) * this->s1Hat_B * this->BTheta.row(0)
        + (this->IS2PntSc2_B - this->mass2 * this->rTilde_Sc2B_B * rTilde_Sc2S2_B) * this->s2Hat_B * this->BTheta.row(1);
    backSubContr.vecRot = - (this->IPrimeS1PntSc1_B + this->omegaTilde_BN_B * this->IS1PntSc1_B) * this->omega_S1B_B - (this->IPrimeS2PntSc2_B + this->omegaTilde_BN_B * this->IS2PntSc2_B) * this->omega_S2B_B
        - (this->IS2PntSc2_B - this->mass2 * this->rTilde_Sc2B_B * rTilde_Sc2S2_B) * this->omegaTilde_S1B_B * this->omega_S2S1_B
        - this->mass1 * (this->rTilde_Sc1B_B * this->omegaTilde_S1B_B + this->omegaTilde_BN_B * this->rTilde_Sc1B_B) * this->rPrime_Sc1B_B
        - this->mass2 * (this->rTilde_Sc2B_B * this->omegaTilde_S1B_B + this->omegaTilde_BN_B * this->rTilde_Sc2B_B) * this->rPrime_Sc2B_B
        - this->mass2 * this->rTilde_Sc2B_B * omegaTilde_S2S1_B * this->rPrime_Sc2S2_B
        - (this->IS1PntSc1_B + this->IS2PntSc2_B - this->mass1 * this->rTilde_Sc1B_B * rTilde_Sc1S1_B - this->mass2 * this->rTilde_Sc2B_B * rTilde_Sc2S1_B) * this->s1Hat_B * this->CTheta.row(0)
        - (this->IS2PntSc2_B - this->mass2 * this->rTilde_Sc2B_B * rTilde_Sc2S2_B) * this->s2Hat_B * this->CTheta.row(1);
}

/*! This method is used to find the derivatives for the SB stateEffector: thetaDDot and the kinematic derivative */
void SpinningBodyTwoDOFStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
    // Grab omegaDot_BN_B
    Eigen::Vector3d omegaDotLocal_BN_B;
    omegaDotLocal_BN_B = omegaDot_BN_B;

    // Find rDDotLoc_BN_B
    const Eigen::Vector3d& rDDotLocal_BN_N = rDDot_BN_N;
    Eigen::Vector3d rDDotLocal_BN_B = this->dcm_BN * rDDotLocal_BN_N;

    // Compute theta and thetaDot derivatives
    Eigen::Vector2d thetaDDot;
    thetaDDot = this->ATheta * rDDotLocal_BN_B + this->BTheta * omegaDotLocal_BN_B + this->CTheta;
    this->theta1State->setDerivative(this->theta1DotState->getState());
    this->theta1DotState->setDerivative(thetaDDot.row(0));
    this->theta2State->setDerivative(this->theta2DotState->getState());
    this->theta2DotState->setDerivative(thetaDDot.row(1));
}

/*! This method is for calculating the contributions of the SB state effector to the energy and momentum of the spacecraft */
void SpinningBodyTwoDOFStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    // Update omega_BN_B and omega_SN_B
    this->omega_BN_B = omega_BN_B;
    this->omegaTilde_BN_B = eigenTilde(this->omega_BN_B);
    this->omega_S1N_B = this->omega_S1B_B + this->omega_BN_B;
    this->omega_S2N_B = this->omega_S2B_B + this->omega_BN_B;

    // Compute rDot_ScB_B
    this->rDot_Sc1B_B = this->rPrime_Sc1B_B + this->omegaTilde_BN_B * this->r_Sc1B_B;
    this->rDot_Sc2B_B = this->rPrime_Sc2B_B + this->omegaTilde_BN_B * this->r_Sc2B_B;

    // Find rotational angular momentum contribution from hub
    rotAngMomPntCContr_B = this->IS1PntSc1_B * this->omega_S1N_B + this->mass1 * this->rTilde_Sc1B_B * this->rDot_Sc1B_B
                         + this->IS2PntSc2_B * this->omega_S2N_B + this->mass2 * this->rTilde_Sc2B_B * this->rDot_Sc2B_B;

    // Find rotational energy contribution from the hub
    rotEnergyContr = 1.0 / 2.0 * this->omega_S1N_B.dot(this->IS1PntSc1_B * this->omega_S1N_B) + 1.0 / 2.0 * this->mass1 * this->rDot_Sc1B_B.dot(this->rDot_Sc1B_B) + 1.0 / 2.0 * this->k1 * (this->theta1 - this->theta1Ref) * (this->theta1 - this->theta1Ref)
                   + 1.0 / 2.0 * this->omega_S2N_B.dot(this->IS2PntSc2_B * this->omega_S2N_B) + 1.0 / 2.0 * this->mass2 * this->rDot_Sc2B_B.dot(this->rDot_Sc2B_B) + 1.0 / 2.0 * this->k2 * (this->theta2 - this->theta2Ref) * (this->theta2 - this->theta2Ref);
}

/*! This method computes the spinning body states relative to the inertial frame */
void SpinningBodyTwoDOFStateEffector::computeSpinningBodyInertialStates()
{
    // Compute the inertial attitude
    Eigen::Matrix3d dcm_S1N;
    Eigen::Matrix3d dcm_S2N;
    dcm_S1N = (this->dcm_BS1).transpose() * this->dcm_BN;
    dcm_S2N = (this->dcm_BS2).transpose() * this->dcm_BN;
    this->sigma_S1N = eigenMRPd2Vector3d(eigenC2MRP(dcm_S1N));
    this->sigma_S2N = eigenMRPd2Vector3d(eigenC2MRP(dcm_S2N));

    // Convert the angular velocity to the corresponding frame
    this->omega_S1N_S1 = dcm_BS1.transpose() * this->omega_S1N_B;
    this->omega_S2N_S2 = dcm_BS2.transpose() * this->omega_S2N_B;

    // Compute the inertial position vector
    this->r_Sc1N_N = (Eigen::Vector3d)(*this->inertialPositionProperty) + this->dcm_BN.transpose() * this->r_Sc1B_B;
    this->r_Sc2N_N = (Eigen::Vector3d)(*this->inertialPositionProperty) + this->dcm_BN.transpose() * this->r_Sc2B_B;

    // Compute the inertial velocity vector
    this->v_Sc1N_N = (Eigen::Vector3d)(*this->inertialVelocityProperty) + this->dcm_BN.transpose() * this->rDot_Sc1B_B;
    this->v_Sc2N_N = (Eigen::Vector3d)(*this->inertialVelocityProperty) + this->dcm_BN.transpose() * this->rDot_Sc2B_B;
}

/*! This method is used so that the simulation will ask SB to update messages */
void SpinningBodyTwoDOFStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Read the incoming command array
    if (this->motorTorqueInMsg.isLinked() && this->motorTorqueInMsg.isWritten()) {
        ArrayMotorTorqueMsgPayload incomingCmdBuffer;
        incomingCmdBuffer = this->motorTorqueInMsg();
        this->u1 = incomingCmdBuffer.motorTorque[0];
        this->u2 = incomingCmdBuffer.motorTorque[1];
    }

    //! - Zero the command buffer and read the incoming command array
    if (this->motorLockInMsg.isLinked() && this->motorLockInMsg.isWritten()) {
        ArrayEffectorLockMsgPayload incomingLockBuffer;
        incomingLockBuffer = this->motorLockInMsg();
        this->lockFlag1 = incomingLockBuffer.effectorLockFlag[0];
        this->lockFlag2 = incomingLockBuffer.effectorLockFlag[1];
    }

    //! - Read the incoming angle command array
    if (this->spinningBodyRefInMsgs[0].isLinked() && this->spinningBodyRefInMsgs[0].isWritten()) {
        HingedRigidBodyMsgPayload incomingRefBuffer;
        incomingRefBuffer = this->spinningBodyRefInMsgs[0]();
        this->theta1Ref = incomingRefBuffer.theta;
        this->theta1DotRef = incomingRefBuffer.thetaDot;
    }
    if (this->spinningBodyRefInMsgs[1].isLinked() && this->spinningBodyRefInMsgs[1].isWritten()) {
        HingedRigidBodyMsgPayload incomingRefBuffer;
        incomingRefBuffer = this->spinningBodyRefInMsgs[1]();
        this->theta2Ref = incomingRefBuffer.theta;
        this->theta2DotRef = incomingRefBuffer.thetaDot;
    }

    /* Compute spinning body inertial states */
    this->computeSpinningBodyInertialStates();

    /* Write output messages*/
    this->writeOutputStateMessages(CurrentSimNanos);
}
