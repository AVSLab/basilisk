/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "hingedRigidBodyStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <string>

/*! This is the constructor, setting variables to default values */
HingedRigidBodyStateEffector::HingedRigidBodyStateEffector()
{
    // - zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rEffPrime_CB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);

    // - Initialize variables to working values
    this->mass = 0.0;
    this->d = 1.0;
    this->k = 1.0;
    this->c = 0.0;
    this->u = 0.0;
    this->thetaRef = 0.0;
    this->thetaDotRef = 0.0;
    this->thetaInit = 0.00;
    this->thetaDotInit = 0.0;
    this->IPntS_S.Identity();
    this->r_HB_B.setZero();
    this->dcm_HB.Identity();
    this->nameOfThetaState = "hingedRigidBodyTheta" + std::to_string(this->effectorID);
    this->nameOfThetaDotState = "hingedRigidBodyThetaDot" + std::to_string(this->effectorID);
    this->effectorID++;

    return;
}

uint64_t HingedRigidBodyStateEffector::effectorID = 1;

/*! This is the destructor, nothing to report here */
HingedRigidBodyStateEffector::~HingedRigidBodyStateEffector()
{
    this->effectorID = 1;    /* reset the panel ID*/
    return;
}


/*! This method takes the computed theta states and outputs them to the m
 messaging system.

 @param CurrentClock The current simulation time (used for time stamping)
 */
void HingedRigidBodyStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{

    if (this->hingedRigidBodyOutMsg.isLinked()) {
        this->HRBoutputStates = this->hingedRigidBodyOutMsg.zeroMsgPayload;
        this->HRBoutputStates.theta = this->theta;
        this->HRBoutputStates.thetaDot = this->thetaDot;
        this->hingedRigidBodyOutMsg.write(&this->HRBoutputStates, this->moduleID, CurrentClock);
    }

    // write out the panel state config log message
    if (this->hingedRigidBodyConfigLogOutMsg.isLinked()) {
        SCStatesMsgPayload configLogMsg;
        configLogMsg = this->hingedRigidBodyConfigLogOutMsg.zeroMsgPayload;
        // Note, logging the hinge frame S is the body frame B of that object
        eigenVector3d2CArray(this->r_SN_N, configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_SN_N, configLogMsg.v_BN_N);
        eigenVector3d2CArray(this->sigma_SN, configLogMsg.sigma_BN);
        eigenVector3d2CArray(this->omega_SN_S, configLogMsg.omega_BN_B);
        this->hingedRigidBodyConfigLogOutMsg.write(&configLogMsg, this->moduleID, CurrentClock);
    }

}

void HingedRigidBodyStateEffector::prependSpacecraftNameToStates()
{
    this->nameOfThetaState = this->nameOfSpacecraftAttachedTo + this->nameOfThetaState;
    this->nameOfThetaDotState = this->nameOfSpacecraftAttachedTo + this->nameOfThetaDotState;

    return;
}

/*! This method allows the HRB state effector to have access to the hub states and gravity*/
void HingedRigidBodyStateEffector::linkInStates(DynParamManager& statesIn)
{
    // - Get access to the hubs sigma, omegaBN_B and velocity needed for dynamic coupling and gravity
    std::string tmpMsgName;
    tmpMsgName = this->nameOfSpacecraftAttachedTo + "centerOfMassSC";
    this->c_B = statesIn.getPropertyReference(tmpMsgName);
    tmpMsgName = this->nameOfSpacecraftAttachedTo + "centerOfMassPrimeSC";
    this->cPrime_B = statesIn.getPropertyReference(tmpMsgName);

    this->inertialPositionProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialPosition);
    this->inertialVelocityProperty = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialVelocity);

    return;
}

/*! This method allows the HRB state effector to register its states: theta and thetaDot with the dyn param manager */
void HingedRigidBodyStateEffector::registerStates(DynParamManager& states)
{
    // - Register the states associated with hinged rigid bodies - theta and thetaDot
    this->thetaState = states.registerState(1, 1, this->nameOfThetaState);
    Eigen::MatrixXd thetaInitMatrix(1,1);
    thetaInitMatrix(0,0) = this->thetaInit;
    this->thetaState->setState(thetaInitMatrix);
    this->thetaDotState = states.registerState(1, 1, this->nameOfThetaDotState);
    Eigen::MatrixXd thetaDotInitMatrix(1,1);
    thetaDotInitMatrix(0,0) = this->thetaDotInit;
    this->thetaDotState->setState(thetaDotInitMatrix);

    return;
}

/*! This method allows the HRB state effector to provide its contributions to the mass props and mass prop rates of the
 spacecraft */
void HingedRigidBodyStateEffector::updateEffectorMassProps(double integTime)
{
    // - Convert intial variables to mother craft frame relative information
    this->r_HP_P = this->r_BP_P + this->dcm_BP.transpose()*r_HB_B;
    this->dcm_HP = this->dcm_HB*this->dcm_BP;

    // - Give the mass of the hinged rigid body to the effProps mass
    this->effProps.mEff = this->mass;

    // - Find hinged rigid bodies' position with respect to point B
    // - First need to grab current states
    this->theta = this->thetaState->getState()(0, 0);
    this->thetaDot = this->thetaDotState->getState()(0, 0);
    // - Next find the sHat unit vectors
    this->dcm_SH = eigenM2(this->theta);
    this->dcm_SP = this->dcm_SH*this->dcm_HP;
    this->sHat1_P = this->dcm_SP.row(0);
    this->sHat2_P = this->dcm_SP.row(1);
    this->sHat3_P = this->dcm_SP.row(2);
    this->r_SP_P = this->r_HP_P - this->d*this->sHat1_P;
    this->effProps.rEff_CB_B = this->r_SP_P;

    // - Find the inertia of the hinged rigid body about point B
    // - Define rTilde_SB_B
    this->rTilde_SP_P = eigenTilde(this->r_SP_P);
    this->effProps.IEffPntB_B = this->dcm_SP.transpose()*this->IPntS_S*this->dcm_SP
                                                           + this->mass*this->rTilde_SP_P*this->rTilde_SP_P.transpose();

    // - Find rPrime_SB_B
    this->rPrime_SP_P = this->d*this->thetaDot*this->sHat3_P;
    this->effProps.rEffPrime_CB_B = this->rPrime_SP_P;

    // - Next find the body time derivative of the inertia about point B
    // - Define tilde matrix of rPrime_SB_B
    this->rPrimeTilde_SP_P = eigenTilde(this->rPrime_SP_P);
    // - Find body time derivative of IPntS_B
    this->ISPrimePntS_P = this->thetaDot*(this->IPntS_S(2,2)
              - this->IPntS_S(0,0))*(this->sHat1_P*this->sHat3_P.transpose() + this->sHat3_P*this->sHat1_P.transpose());
    // - Find body time derivative of IPntB_B
    this->effProps.IEffPrimePntB_B = this->ISPrimePntS_P
                     - this->mass*(this->rPrimeTilde_SP_P*this->rTilde_SP_P + this->rTilde_SP_P*this->rPrimeTilde_SP_P);

    return;
}

/*! This method allows the HRB state effector to give its contributions to the matrices needed for the back-sub
 method */
void HingedRigidBodyStateEffector::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{
    // - Find dcm_BN
    Eigen::MRPd sigmaLocal_PN;
    this->sigma_BN = sigma_BN;
    sigmaLocal_PN = this->sigma_BN;
    Eigen::Matrix3d dcm_PN;
    Eigen::Matrix3d dcm_NP;
    dcm_NP = sigmaLocal_PN.toRotationMatrix();
    dcm_PN = dcm_NP.transpose();

    // - Map gravity to body frame
    Eigen::Vector3d gLocal_N;
    Eigen::Vector3d g_P;
    gLocal_N = g_N;
    g_P = dcm_PN*gLocal_N;

    // - Define omega_BN_S
    this->omega_BN_B = omega_BN_B;
    this->omegaLoc_PN_P = this->omega_BN_B;
    this->omega_PN_S = this->dcm_SP*this->omegaLoc_PN_P;
    // - Define omegaTildeLoc_BN_B
    this->omegaTildeLoc_PN_P = eigenTilde(this->omegaLoc_PN_P);

    // - Define aTheta
    this->aTheta = -this->mass*this->d/(this->IPntS_S(1,1) + this->mass*this->d*this->d)*this->sHat3_P;

    // - Define bTheta
    this->rTilde_HP_P = eigenTilde(this->r_HP_P);
    this->bTheta = -1.0/(this->IPntS_S(1,1) + this->mass*this->d*this->d)*((this->IPntS_S(1,1)
                      + this->mass*this->d*this->d)*this->sHat2_P + this->mass*this->d*this->rTilde_HP_P*this->sHat3_P);

    // - Define cTheta
    Eigen::Vector3d gravityTorquePntH_P;
    gravityTorquePntH_P = -this->d*this->sHat1_P.cross(this->mass*g_P);
    this->cTheta = 1.0/(this->IPntS_S(1,1) + this->mass*this->d*this->d)*(this->u -this->k*(this->theta-this->thetaRef) - this->c*(this->thetaDot-this->thetaDotRef)
                    + this->sHat2_P.dot(gravityTorquePntH_P) + (this->IPntS_S(2,2) - this->IPntS_S(0,0)
                     + this->mass*this->d*this->d)*this->omega_PN_S(2)*this->omega_PN_S(0) - this->mass*this->d*
                              this->sHat3_P.transpose()*this->omegaTildeLoc_PN_P*this->omegaTildeLoc_PN_P*this->r_HP_P);

    // - Start defining them good old contributions - start with translation
    // - For documentation on contributions see Allard, Diaz, Schaub flex/slosh paper
    backSubContr.matrixA = this->mass*this->d*this->sHat3_P*this->aTheta.transpose();
    backSubContr.matrixB = this->mass*this->d*this->sHat3_P*this->bTheta.transpose();
    backSubContr.vecTrans = -(this->mass*this->d*this->thetaDot*this->thetaDot*this->sHat1_P
                                                                       + this->mass*this->d*this->cTheta*this->sHat3_P);

    // - Define rotational matrice contributions
    backSubContr.matrixC = (this->IPntS_S(1,1)*this->sHat2_P + this->mass*this->d*this->rTilde_SP_P*this->sHat3_P)
                                                                                              *this->aTheta.transpose();
    backSubContr.matrixD = (this->IPntS_S(1,1)*this->sHat2_P + this->mass*this->d*this->rTilde_SP_P*this->sHat3_P)
                                                                                              *this->bTheta.transpose();
    Eigen::Matrix3d intermediateMatrix;
    backSubContr.vecRot = -((this->thetaDot*this->omegaTildeLoc_PN_P + this->cTheta*intermediateMatrix.Identity())
                    *(this->IPntS_S(1,1)*this->sHat2_P + this->mass*this->d*this->rTilde_SP_P*this->sHat3_P)
                                    + this->mass*this->d*this->thetaDot*this->thetaDot*this->rTilde_SP_P*this->sHat1_P);

    return;
}

/*! This method is used to find the derivatives for the HRB stateEffector: thetaDDot and the kinematic derivative */
void HingedRigidBodyStateEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
    // - Grab necessarry values from manager (these have been previously computed in hubEffector)
    Eigen::Vector3d rDDotLoc_PN_N = rDDot_BN_N;
    Eigen::MRPd sigmaLocal_PN;
    Eigen::Vector3d omegaDotLoc_PN_P;
    this->sigma_BN = sigma_BN;  // store current hub attitude
    sigmaLocal_PN = this->sigma_BN;

    // - Find rDDotLoc_BN_B
    Eigen::Matrix3d dcm_PN;
    Eigen::Vector3d rDDotLoc_PN_P;
    dcm_PN = (sigmaLocal_PN.toRotationMatrix()).transpose();
    rDDotLoc_PN_P = dcm_PN*rDDotLoc_PN_N;

    // - Compute Derivatives
    // - First is trivial
    this->thetaState->setDerivative(thetaDotState->getState());
    // - Second, a little more involved
    Eigen::MatrixXd thetaDDot(1,1);
    thetaDDot(0,0) = this->aTheta.dot(rDDotLoc_PN_P) + this->bTheta.dot(omegaDot_BN_B) + this->cTheta;
    this->thetaDotState->setDerivative(thetaDDot);

    return;
}

/*! This method is for calculating the contributions of the HRB state effector to the energy and momentum of the s/c */
void HingedRigidBodyStateEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                                                double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    // - Get the current omega state
    Eigen::Vector3d omegaLocal_PN_P;
    omegaLocal_PN_P = omega_BN_B;

    // - Find rotational angular momentum contribution from hub
    Eigen::Vector3d omega_SP_P;
    Eigen::Vector3d omega_SN_P;
    Eigen::Matrix3d IPntS_P;
    Eigen::Vector3d rDot_SP_P;
    omega_SP_P = this->thetaDot*this->sHat2_P;
    omega_SN_P = omega_SP_P + omegaLocal_PN_P;
    IPntS_P = this->dcm_SP.transpose()*this->IPntS_S*this->dcm_SP;
    rDot_SP_P = this->rPrime_SP_P + omegaLocal_PN_P.cross(this->r_SP_P);
    rotAngMomPntCContr_B = IPntS_P*omega_SN_P + this->mass*this->r_SP_P.cross(rDot_SP_P);

    // - Find rotational energy contribution from the hub
    rotEnergyContr = 1.0/2.0*omega_SN_P.dot(IPntS_P*omega_SN_P) + 1.0/2.0*this->mass*rDot_SP_P.dot(rDot_SP_P)
                                                                              + 1.0/2.0*this->k*(this->theta-this->thetaRef)*(this->theta-this->thetaRef);

    return;
}
/*! This method is used so that the simulation will ask HRB to update messages.

 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void HingedRigidBodyStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Zero the command buffer and read the incoming command array
    if (this->motorTorqueInMsg.isLinked() && this->motorTorqueInMsg.isWritten()) {
        ArrayMotorTorqueMsgPayload incomingCmdBuffer;
        incomingCmdBuffer = this->motorTorqueInMsg();
        this->u = incomingCmdBuffer.motorTorque[0];
    }
    if (this->hingedRigidBodyRefMsg.isLinked() && this->hingedRigidBodyRefMsg.isWritten()) {
        HingedRigidBodyMsgPayload incomingRefBuffer;
        incomingRefBuffer = this->hingedRigidBodyRefMsg();
        this->thetaRef = incomingRefBuffer.theta;
        this->thetaDotRef = incomingRefBuffer.thetaDot;
    }

    /* compute panel inertial states */
    this->computePanelInertialStates();

    this->writeOutputStateMessages(CurrentSimNanos);

    return;
}

void HingedRigidBodyStateEffector::calcForceTorqueOnBody(double integTime, Eigen::Vector3d omega_BN_B)
{

    // - Get the current omega state
    Eigen::Vector3d omegaLocal_PN_P = omega_BN_B;
    Eigen::Matrix3d omegaLocalTilde_PN_P;
    omegaLocalTilde_PN_P = eigenTilde(omegaLoc_PN_P);

    // - Get thetaDDot from last integrator call
    double thetaDDotLocal;
    thetaDDotLocal = thetaDotState->getStateDeriv()(0, 0);

    // - Calculate force that the HRB is applying to the spacecraft
    this->forceOnBody_B = -(this->mass*this->d*this->sHat3_P*thetaDDotLocal + this->mass*this->d*this->thetaDot
                            *this->thetaDot*this->sHat1_P + 2.0*omegaLocalTilde_PN_P*this->mass*this->d*this->thetaDot
                            *this->sHat3_P);

    // - Calculate torque that the HRB is applying about point B
    this->torqueOnBodyPntB_B = -((this->IPntS_S(1,1)*this->sHat2_P + this->mass*this->d*this->rTilde_SP_P*this->sHat3_P)
                                 *thetaDDotLocal + (this->ISPrimePntS_P - this->mass*(this->rPrimeTilde_SP_P*rTilde_SP_P
                                                                                      + this->rTilde_SP_P*this->rPrimeTilde_SP_P))*omegaLocal_PN_P + this->thetaDot
                                 *omegaLocalTilde_PN_P*(this->IPntS_S(1,1)*this->sHat2_P + this->mass*this->d
                                 *this->rTilde_SP_P*this->sHat3_P) + this->mass*this->d*this->thetaDot*this->thetaDot
                                 *this->rTilde_SP_P*this->sHat1_P);

    // - Define values needed to get the torque about point C
    Eigen::Vector3d cLocal_B = *this->c_B;
    Eigen::Vector3d cPrimeLocal_B = *cPrime_B;
    Eigen::Vector3d r_SC_B = this->r_SP_P - cLocal_B;
    Eigen::Vector3d rPrime_SC_B = this->rPrime_SP_P - cPrimeLocal_B;
    Eigen::Matrix3d rTilde_SC_B = eigenTilde(r_SC_B);
    Eigen::Matrix3d rPrimeTilde_SC_B = eigenTilde(rPrime_SC_B);

    // - Calculate the torque about point C
    this->torqueOnBodyPntC_B = -((this->IPntS_S(1,1)*this->sHat2_P + this->mass*this->d*rTilde_SC_B*this->sHat3_P)
                                 *thetaDDotLocal + (this->ISPrimePntS_P - this->mass*(rPrimeTilde_SC_B*rTilde_SC_B
                                 + rTilde_SC_B*rPrimeTilde_SC_B))*omegaLocal_PN_P + this->thetaDot
                                 *omegaLocalTilde_PN_P*(this->IPntS_S(1,1)*this->sHat2_P + this->mass*this->d
                                 *rTilde_SC_B*this->sHat3_P) + this->mass*this->d*this->thetaDot*this->thetaDot
                                 *rTilde_SC_B*this->sHat1_P);

    return;
}

/*! This method computes the panel states relative to the inertial frame

 */
void HingedRigidBodyStateEffector::computePanelInertialStates()
{
    // inertial attitude
    Eigen::MRPd sigmaBN;
    sigmaBN = this->sigma_BN;
    Eigen::Matrix3d dcm_NP = sigmaBN.toRotationMatrix();  // assumes P and B are idential
    Eigen::Matrix3d dcm_SN;
    dcm_SN = this->dcm_SP*dcm_NP.transpose();
    this->sigma_SN = eigenMRPd2Vector3d(eigenC2MRP(dcm_SN));


    // inertial angular velocity
    Eigen::Vector3d omega_BN_B;
    omega_BN_B = this->omega_BN_B;
    this->omega_SN_S = this->dcm_SP * ( omega_BN_B + this->thetaDot*this->sHat2_P);

    // inertial position vector
    this->r_SN_N = (dcm_NP * this->r_SP_P) + (Eigen::Vector3d)(*this->inertialPositionProperty);

    // inertial velocity vector
    this->v_SN_N = (Eigen::Vector3d)(*this->inertialVelocityProperty)
                  + this->d * this->thetaDot * this->sHat3_P - this->d * (omega_BN_B.cross(this->sHat1_P))
                  + omega_BN_B.cross(this->r_HP_P);

    return;
}
