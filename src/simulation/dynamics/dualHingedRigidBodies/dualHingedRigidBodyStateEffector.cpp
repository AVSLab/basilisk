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


#include "dualHingedRigidBodyStateEffector.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <string>

DualHingedRigidBodyStateEffector::DualHingedRigidBodyStateEffector()
{
    // - zero the mass props and mass prop rates contributions
    this->effProps.mEff = 0.0;
    this->effProps.mEffDot = 0.0;
    this->effProps.rEff_CB_B.setZero();
    this->effProps.IEffPntB_B.setZero();
    this->effProps.rEffPrime_CB_B.setZero();
    this->effProps.IEffPrimePntB_B.setZero();
    this->matrixFDHRB.resize(2, 3);
    this->matrixGDHRB.resize(2, 3);

    // - Initialize the variables to working values
    this->mass1 = 0.0;
    this->d1 = 1.0;
    this->k1 = 1.0;
    this->c1 = 0.0;
    this->l1 = 0.0;
    this->u1 = 0.0;
    this->mass2 = 0.0;
    this->d2 = 1.0;
    this->k2 = 1.0;
    this->c2 = 0.0;
    this->u2 = 0.0;
    this->theta1Init = 0.0;
    this->theta1DotInit = 0.0;
    this->theta2Init = 0.0;
    this->theta2DotInit = 0.0;
    this->IPntS1_S1.setIdentity();
    this->IPntS2_S2.setIdentity();
    this->r_H1B_B.setZero();
    this->dcm_H1B.setIdentity();
    this->thetaH2S1 = 0.0;
    this->nameOfTheta1State = "DualHingedRigidBodyStateEffectorTheta1" + std::to_string(this->effectorID);
    this->nameOfTheta1DotState = "DualHingedRigidBodyStateEffectorTheta1Dot" + std::to_string(this->effectorID);
    this->nameOfTheta2State = "DualHingedRigidBodyStateEffectorTheta2" + std::to_string(this->effectorID);
    this->nameOfTheta2DotState = "DualHingedRigidBodyStateEffectorTheta2Dot" + std::to_string(this->effectorID);
    this->nameOfInertialPositionProperty1 = "DualHingedRigidBodyStateEffectorInertialPosition1" + std::to_string(this->effectorID);
    this->nameOfInertialVelocityProperty1 = "DualHingedRigidBodyStateEffectorInertialVelocity1" + std::to_string(this->effectorID);
    this->nameOfInertialAttitudeProperty1 = "DualHingedRigidBodyStateEffectorInertialAttitude1" + std::to_string(this->effectorID);
    this->nameOfInertialAngVelocityProperty1 = "DualHingedRigidBodyStateEffectorInertialAngVelocity1" + std::to_string(this->effectorID);
    this->nameOfInertialPositionProperty2 = "DualHingedRigidBodyStateEffectorInertialPosition2" + std::to_string(this->effectorID);
    this->nameOfInertialVelocityProperty2 = "DualHingedRigidBodyStateEffectorInertialVelocity2" + std::to_string(this->effectorID);
    this->nameOfInertialAttitudeProperty2 = "DualHingedRigidBodyStateEffectorInertialAttitude2" + std::to_string(this->effectorID);
    this->nameOfInertialAngVelocityProperty2 = "DualHingedRigidBodyStateEffectorInertialAngVelocity2" + std::to_string(this->effectorID);
    this->effectorID++;

    Message<HingedRigidBodyMsgPayload> *panelMsg;
    Message<SCStatesMsgPayload> *scMsg;
    for (int c = 0; c < 2; c++) {
        panelMsg = new Message<HingedRigidBodyMsgPayload>;
        this->dualHingedRigidBodyOutMsgs.push_back(panelMsg);
        scMsg = new Message<SCStatesMsgPayload>;
        this->dualHingedRigidBodyConfigLogOutMsgs.push_back(scMsg);
    }

    this->ModelTag = "";

    return;
}

uint64_t DualHingedRigidBodyStateEffector::effectorID = 1;

DualHingedRigidBodyStateEffector::~DualHingedRigidBodyStateEffector()
{
    for (size_t c = 0; c < 2; c++) {
        delete this->dualHingedRigidBodyOutMsgs.at(c);
        delete this->dualHingedRigidBodyConfigLogOutMsgs.at(c);
    }

    return;
}


/*! This method is used to reset the module.
 *
 * @param[in] CurrentSimNanos [ns] Current simulation time.
 */
void DualHingedRigidBodyStateEffector::Reset(uint64_t CurrentSimNanos [[maybe_unused]])
{

    return;
}

void DualHingedRigidBodyStateEffector::prependSpacecraftNameToStates()
{
    this->nameOfTheta1State = this->nameOfSpacecraftAttachedTo + this->nameOfTheta1State;
    this->nameOfTheta1DotState = this->nameOfSpacecraftAttachedTo + this->nameOfTheta1DotState;
    this->nameOfTheta2State = this->nameOfSpacecraftAttachedTo + this->nameOfTheta2State;
    this->nameOfTheta2DotState = this->nameOfSpacecraftAttachedTo + this->nameOfTheta2DotState;

    return;
}


/*! @brief Link the required dynamics states.
 *
 * @param[in] states Dynamic parameter manager containing the required states.
 */
void DualHingedRigidBodyStateEffector::linkInStates(DynParamManager& states)
{
    // - Get access to the hubs sigma, omegaBN_B and velocity needed for dynamic coupling
    this->g_N = states.getPropertyReference(this->nameOfSpacecraftAttachedTo + "g_N");

    this->inertialPositionProperty = states.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialPosition);
    this->inertialVelocityProperty = states.getPropertyReference(this->nameOfSpacecraftAttachedTo + this->propName_inertialVelocity);
    this->v_BN_NState = states.getStateObject(this->nameOfSpacecraftAttachedTo + this->stateNameOfVelocity);
    this->hubSigmaState = states.getStateObject(this->nameOfSpacecraftAttachedTo + this->stateNameOfSigma);

    return;
}

/*! @brief Register the effector dynamics states.
 *
 * @param[in,out] statesIn Dynamic parameter manager used to register states or properties.
 */
void DualHingedRigidBodyStateEffector::registerStates(DynParamManager& statesIn)
{
    // - Register the states associated with hinged rigid bodies - theta and thetaDot
    this->theta1State = statesIn.registerState(1, 1, this->nameOfTheta1State);
    this->theta1DotState = statesIn.registerState(1, 1, this->nameOfTheta1DotState);
    this->theta2State = statesIn.registerState(1, 1, this->nameOfTheta2State);
    this->theta2DotState = statesIn.registerState(1, 1, this->nameOfTheta2DotState);

    // - Add this code to allow for non-zero initial conditions, as well hingedRigidBody
    Eigen::MatrixXd theta1InitMatrix(1,1);
    theta1InitMatrix(0,0) = this->theta1Init;
    this->theta1State->setState(theta1InitMatrix);
    Eigen::MatrixXd theta1DotInitMatrix(1,1);
    theta1DotInitMatrix(0,0) = this->theta1DotInit;
    this->theta1DotState->setState(theta1DotInitMatrix);
    Eigen::MatrixXd theta2InitMatrix(1,1);
    theta2InitMatrix(0,0) = this->theta2Init;
    this->theta2State->setState(theta2InitMatrix);
    Eigen::MatrixXd theta2DotInitMatrix(1,1);
    theta2DotInitMatrix(0,0) = this->theta2DotInit;
    this->theta2DotState->setState(theta2DotInitMatrix);

    registerProperties(statesIn);
}

/*! This method attaches a dynamicEffector to one of the two panels
 @param newDynamicEffector the dynamic effector to be attached
 @param segment the panel to attach to, either 1 or 2 */
void DualHingedRigidBodyStateEffector::addDynamicEffector(DynamicEffector *newDynamicEffector, int segment)
{
    if (segment != 1 && segment != 2) {
        bskLogger.bskError("DualHingedRigidBodyStateEffector: segment must be either 1 or 2.");
    }

    this->assignStateParamNames<DynamicEffector *>(newDynamicEffector, segment);

    this->dynEffectors.push_back(newDynamicEffector);
    this->dynEffectorSegments.push_back(segment);
}

/*! This method registers the panel inertial properties with the dynamic parameter manager and links
 them into dependent dynamic effectors
 *
 * @param[in,out] states Dynamic parameter manager used to register states or properties.
 */
void DualHingedRigidBodyStateEffector::registerProperties(DynParamManager& states)
{
    Eigen::Vector3d stateInit = Eigen::Vector3d::Zero();

    this->r_SN_N.resize(2, Eigen::Vector3d::Zero());
    this->v_SN_N.resize(2, Eigen::Vector3d::Zero());
    this->r_HN_N.resize(2, nullptr);
    this->v_HN_N.resize(2, nullptr);
    this->sigma_SN.resize(2);
    this->omega_SN_S.resize(2);

    this->r_HN_N[0] = states.createProperty(this->nameOfInertialPositionProperty1, stateInit);
    this->r_HN_N[1] = states.createProperty(this->nameOfInertialPositionProperty2, stateInit);
    this->v_HN_N[0] = states.createProperty(this->nameOfInertialVelocityProperty1, stateInit);
    this->v_HN_N[1] = states.createProperty(this->nameOfInertialVelocityProperty2, stateInit);
    this->sigma_SN[0] = states.createProperty(this->nameOfInertialAttitudeProperty1, stateInit);
    this->sigma_SN[1] = states.createProperty(this->nameOfInertialAttitudeProperty2, stateInit);
    this->omega_SN_S[0] = states.createProperty(this->nameOfInertialAngVelocityProperty1, stateInit);
    this->omega_SN_S[1] = states.createProperty(this->nameOfInertialAngVelocityProperty2, stateInit);

    std::vector<DynamicEffector*>::iterator dynIt;
    for(dynIt = this->dynEffectors.begin(); dynIt != this->dynEffectors.end(); dynIt++)
    {
        (*dynIt)->linkInProperties(states);
    }
}

/*! @brief Update the effector mass properties.
 *
 * @param[in] integTime [s] Current integration time.
 */
void DualHingedRigidBodyStateEffector::updateEffectorMassProps(double integTime [[maybe_unused]])
{
    // - Convert initial variables to mother craft frame relative information
    this->r_H1P_P = this->r_BP_P + this->dcm_BP.transpose()*this->r_H1B_B;
    this->dcm_H1P = this->dcm_H1B*this->dcm_BP;

    // - Give the mass of the hinged rigid body to the effProps mass
    this->effProps.mEff = this->mass1 + this->mass2;

    // - find hinged rigid bodies' position with respect to point B
    // - First need to grab current states
    this->theta1 = this->theta1State->getStateReference()(0, 0);
    this->theta1Dot = this->theta1DotState->getStateReference()(0, 0);
    this->theta2 = this->theta2State->getStateReference()(0, 0);
    this->theta2Dot = this->theta2DotState->getStateReference()(0, 0);
    // - Next find the sHat unit vectors
    Eigen::Matrix3d dcmS1H1;
    dcmS1H1 = eigenM2(this->theta1);
    this->dcm_S1P = dcmS1H1*this->dcm_H1P;
    Eigen::Matrix3d dcmH2S1;
    dcmH2S1 = eigenM2(this->thetaH2S1);
    Eigen::Matrix3d dcmH2P;
    dcmH2P = dcmH2S1*this->dcm_S1P;
    Eigen::Matrix3d dcmS2H2;
    dcmS2H2 = eigenM2(this->theta2);
    this->dcm_S2P = dcmS2H2 * dcmH2P;
    this->sHat11_P = this->dcm_S1P.row(0);
    this->sHat12_P = this->dcm_S1P.row(1);
    this->sHat13_P = this->dcm_S1P.row(2);
    this->sHat21_P = this->dcm_S2P.row(0);
    this->sHat22_P = this->dcm_S2P.row(1);
    this->sHat23_P = this->dcm_S2P.row(2);
    this->r_H2P_P = this->r_H1P_P - this->l1 * this->sHat11_P;
    this->r_S1P_P = this->r_H1P_P - this->d1*this->sHat11_P;
    this->r_S2P_P = this->r_H1P_P - this->l1*this->sHat11_P - this->d2*this->sHat21_P;
    this->effProps.rEff_CB_B = 1.0/this->effProps.mEff*(this->mass1*this->r_S1P_P + this->mass2*this->r_S2P_P);

    // - Find the inertia of the hinged rigid body about point B
    // - Define rTildeSB_B
    this->rTildeS1P_P = eigenTilde(this->r_S1P_P);
    this->rTildeS2P_P = eigenTilde(this->r_S2P_P);
    this->effProps.IEffPntB_B = this->dcm_S1P.transpose()*this->IPntS1_S1*this->dcm_S1P + this->mass1*this->rTildeS1P_P*this->rTildeS1P_P.transpose() + this->dcm_S2P.transpose()*this->IPntS2_S2*this->dcm_S2P + this->mass2*this->rTildeS2P_P*this->rTildeS2P_P.transpose();

    // First, find the rPrimeSB_B
    this->rPrimeS1P_P = this->d1*this->theta1Dot*this->sHat13_P;
    this->rPrimeS2P_P = this->l1*this->theta1Dot*this->sHat13_P + this->d2*(this->theta1Dot + this->theta2Dot)*this->sHat23_P;
    this->effProps.rEffPrime_CB_B = 1.0/this->effProps.mEff*(this->mass1*this->rPrimeS1P_P + this->mass2*this->rPrimeS2P_P);

    // - Next find the body time derivative of the inertia about point B
    // - Define tilde matrix of rPrimeSB_B
    this->rPrimeTildeS1P_P = eigenTilde(this->rPrimeS1P_P);
    this->rPrimeTildeS2P_P = eigenTilde(this->rPrimeS2P_P);
    // - Find body time derivative of IPntS_B
    this->IS1PrimePntS1_P = this->theta1Dot*(this->IPntS1_S1(2,2) - this->IPntS1_S1(0,0))*(this->sHat11_P*this->sHat13_P.transpose() + this->sHat13_P*this->sHat11_P.transpose());
    this->IS2PrimePntS2_P = (this->theta1Dot+this->theta2Dot)*(this->IPntS2_S2(2,2) - this->IPntS2_S2(0,0))*(this->sHat21_P*this->sHat23_P.transpose() + this->sHat23_P*this->sHat21_P.transpose());
    // - Find body time derivative of IPntB_B
    this->effProps.IEffPrimePntB_B = this->IS1PrimePntS1_P - this->mass1*(this->rPrimeTildeS1P_P*this->rTildeS1P_P + this->rTildeS1P_P*this->rPrimeTildeS1P_P) + this->IS2PrimePntS2_P - this->mass2*(this->rPrimeTildeS2P_P*this->rTildeS2P_P + this->rTildeS2P_P*this->rPrimeTildeS2P_P);

    return;
}

/*! @brief Update the effector back-substitution contributions.
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in,out] backSubContr Back-substitution contributions.
 * @param[in] sigma_BN Hub attitude relative to the inertial frame.
 * @param[in] omega_BN_B [rad/s] Hub angular velocity expressed in body-frame components.
 * @param[in] g_N [m/s^2] Gravitational acceleration expressed in inertial-frame components.
 */
void DualHingedRigidBodyStateEffector::updateContributions(double integTime [[maybe_unused]], BackSubMatrices & backSubContr, Eigen::MRPd sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N [[maybe_unused]])
{
    Eigen::MRPd sigmaPNLocal;
    Eigen::Matrix3d dcmPN;                        /* direction cosine matrix from N to B */
    Eigen::Matrix3d dcmNP;                        /* direction cosine matrix from B to N */
    Eigen::Vector3d gLocal_N;                          /* gravitational acceleration in N frame */
    Eigen::Vector3d g_P;                          /* gravitational acceleration in B frame */
    gLocal_N = *this->g_N;

    // - Find dcmBN
    this->sigma_BN = sigma_BN;
    sigmaPNLocal = this->sigma_BN;
    dcmNP = sigmaPNLocal.toRotationMatrix();
    dcmPN = dcmNP.transpose();
    // - Map gravity to body frame
    g_P = dcmPN*gLocal_N;

    if (!this->dynEffectors.empty()) {
        this->omega_BN_B = omega_BN_B;
        this->computePanelInertialStates();
    }

    // Loop through to collect forces and torques from any connected dynamic effectors
    Eigen::Vector3d attBodyForce_S1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d attBodyTorquePntH1_S1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d attBodyForce_S2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d attBodyTorquePntH2_S2 = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < this->dynEffectors.size(); i++) {
        DynamicEffector* dynEffector = this->dynEffectors[i];
        // - Compute the force and torque contributions from the dynamicEffectors
        dynEffector->computeForceTorque(integTime, double(0.0));
        if (this->dynEffectorSegments[i] == 1) {
            // a child's "_B" loads are already in panel 1's S1 frame, so only "_N" is rotated
            attBodyForce_S1 += dynEffector->forceExternal_B + this->dcm_S1P * dcmPN * dynEffector->forceExternal_N;
            attBodyTorquePntH1_S1 += dynEffector->torqueExternalPntB_B;
        } else if (this->dynEffectorSegments[i] == 2) {
            // a child's "_B" loads are already in panel 2's S2 frame, so only "_N" is rotated
            attBodyForce_S2 += dynEffector->forceExternal_B + this->dcm_S2P * dcmPN * dynEffector->forceExternal_N;
            attBodyTorquePntH2_S2 += dynEffector->torqueExternalPntB_B;
        }
    }

    // - Define gravity terms
    Eigen::Vector3d gravTorquePan1PntH1_P = -this->d1*this->sHat11_P.cross(this->mass1*g_P);
    Eigen::Vector3d gravForcePan2_P = this->mass2*g_P;
    Eigen::Vector3d gravTorquePan2PntH2_P = -this->d2*this->sHat21_P.cross(this->mass2*g_P);

    // - Sum of forces and torques
    Eigen::Vector3d externalForcePan1_P = this->dcm_S1P.transpose() * attBodyForce_S1;
    Eigen::Vector3d externalForcePan2_P = this->dcm_S2P.transpose() * attBodyForce_S2;
    Eigen::Vector3d externalTorquePan1PntH1_P = this->dcm_S1P.transpose() * attBodyTorquePntH1_S1;
    Eigen::Vector3d externalTorquePan2PntH2_P = this->dcm_S2P.transpose() * attBodyTorquePntH2_S2;

    // - Define omegaBN_S
    this->omega_BN_B = omega_BN_B;
    this->omega_PNLoc_P = this->omega_BN_B;
    this->omega_PN_S1 = this->dcm_S1P*this->omega_PNLoc_P;
    this->omega_PN_S2 = this->dcm_S2P*this->omega_PNLoc_P;

    // - Define matrices needed for back substitution
    this->matrixADHRB(0,0) = this->IPntS1_S1(1,1) + this->mass1*this->d1*this->d1 + this->mass2*this->l1*this->l1 + this->mass2*this->l1*this->d2*this->sHat13_P.transpose()*(this->sHat23_P);
    this->matrixADHRB(0,1) = this->mass2*this->l1*this->d2*this->sHat13_P.transpose()*(this->sHat23_P);
    this->matrixADHRB(1,0) = IPntS2_S2(1,1) + this->mass2*this->d2*this->d2 + this->mass2*this->l1*this->d2*this->sHat23_P.transpose()*this->sHat13_P;
    this->matrixADHRB(1,1) = this->IPntS2_S2(1,1) + this->mass2*this->d2*this->d2;
    this->matrixEDHRB = this->matrixADHRB.inverse();
    this->matrixFDHRB.row(0) = -(this->mass2*this->l1 + this->mass1*this->d1)*this->sHat13_P.transpose();
    this->matrixFDHRB.row(1) = -this->mass2*this->d2*this->sHat23_P.transpose();

    this->matrixGDHRB.row(0) = -(this->IPntS1_S1(1,1)*this->sHat12_P.transpose()
                                 - this->mass1*this->d1*this->sHat13_P.cross(this->r_S1P_P).transpose()
                                 - this->mass2*this->l1*this->sHat13_P.cross(this->r_S2P_P).transpose());
    this->matrixGDHRB.row(1) = -(this->IPntS2_S2(1,1)*this->sHat22_P.transpose()
                                 - this->mass2*this->d2*this->sHat23_P.cross(this->r_S2P_P).transpose());

    this->vectorVDHRB(0) =  -(this->IPntS1_S1(0,0) - this->IPntS1_S1(2,2))*this->omega_PN_S1(2)*this->omega_PN_S1(0)
                            + this->u1 - this->u2 - this->k1*this->theta1 - this->c1*this->theta1Dot + this->k2*this->theta2 + this->c2*this->theta2Dot
                            + this->sHat12_P.dot(gravTorquePan1PntH1_P + externalTorquePan1PntH1_P)
                            + this->l1*this->sHat13_P.dot(gravForcePan2_P + externalForcePan2_P) -
                            this->mass1*this->d1*this->sHat13_P.dot(2*this->omega_PNLoc_P.cross(this->rPrimeS1P_P)
                            + this->omega_PNLoc_P.cross(this->omega_PNLoc_P.cross(this->r_S1P_P)))
                            - this->mass2*this->l1*this->sHat13_P.dot(2*this->omega_PNLoc_P.cross(this->rPrimeS2P_P)
                            + this->omega_PNLoc_P.cross(this->omega_PNLoc_P.cross(this->r_S2P_P))
                            + this->l1*this->theta1Dot*this->theta1Dot*this->sHat11_P
                            + this->d2*(this->theta1Dot + this->theta2Dot)*(this->theta1Dot + this->theta2Dot)*this->sHat21_P);

    this->vectorVDHRB(1) =  -(this->IPntS2_S2(0,0) - this->IPntS2_S2(2,2))*this->omega_PN_S2(2)*this->omega_PN_S2(0)
                            + this->u2 - this->k2*this->theta2 - this->c2*this->theta2Dot
                            + this->sHat22_P.dot(gravTorquePan2PntH2_P + externalTorquePan2PntH2_P)
                            - this->mass2*this->d2*this->sHat23_P.dot(2*this->omega_PNLoc_P.cross(this->rPrimeS2P_P)
                            + this->omega_PNLoc_P.cross(this->omega_PNLoc_P.cross(this->r_S2P_P)) + this->l1*this->theta1Dot*this->theta1Dot*this->sHat11_P);

    // - Start defining them good old contributions - start with translation
    // - For documentation on contributions see Allard, Diaz, Schaub flex/slosh paper
    backSubContr.matrixA = (this->mass1*this->d1*this->sHat13_P + this->mass2*this->l1*this->sHat13_P + this->mass2*this->d2*this->sHat23_P)*matrixEDHRB.row(0)*this->matrixFDHRB + this->mass2*this->d2*this->sHat23_P*this->matrixEDHRB.row(1)*this->matrixFDHRB;
    backSubContr.matrixB = (this->mass1*this->d1*this->sHat13_P + this->mass2*this->l1*this->sHat13_P + this->mass2*this->d2*this->sHat23_P)*this->matrixEDHRB.row(0)*(matrixGDHRB) + this->mass2*this->d2*this->sHat23_P*this->matrixEDHRB.row(1)*(matrixGDHRB);
    backSubContr.vecTrans = -(this->mass1*this->d1*this->theta1Dot*this->theta1Dot*this->sHat11_P + this->mass2*(this->l1*this->theta1Dot*this->theta1Dot*this->sHat11_P + this->d2*(this->theta1Dot+this->theta2Dot)*(this->theta1Dot+this->theta2Dot)*this->sHat21_P)
                    + (this->mass1*this->d1*this->sHat13_P + this->mass2*this->l1*this->sHat13_P + this->mass2*this->d2*this->sHat23_P)*this->matrixEDHRB.row(0)*this->vectorVDHRB + this->mass2*this->d2*this->sHat23_P*this->matrixEDHRB.row(1)*this->vectorVDHRB)
                    + externalForcePan1_P + externalForcePan2_P;

    // - Define rotational matrice contributions (Eq 96 in paper)

    Eigen::Vector3d rotFactor0_P = this->IPntS1_S1(1,1)*this->sHat12_P
                                   + this->mass1*this->d1*this->r_S1P_P.cross(this->sHat13_P)
                                   + this->IPntS2_S2(1,1)*this->sHat22_P
                                   + this->mass2*this->l1*this->r_S2P_P.cross(this->sHat13_P)
                                   + this->mass2*this->d2*this->r_S2P_P.cross(this->sHat23_P);
    Eigen::Vector3d rotFactor1_P = this->IPntS2_S2(1,1)*this->sHat22_P
                                   + this->mass2*this->d2*this->r_S2P_P.cross(this->sHat23_P);
    backSubContr.matrixC = rotFactor0_P*this->matrixEDHRB.row(0)*this->matrixFDHRB
                    + rotFactor1_P*this->matrixEDHRB.row(1)*this->matrixFDHRB;

    backSubContr.matrixD = rotFactor0_P*this->matrixEDHRB.row(0)*this->matrixGDHRB
                    + rotFactor1_P*this->matrixEDHRB.row(1)*this->matrixGDHRB;

    backSubContr.vecRot = -(this->theta1Dot*this->IPntS1_S1(1,1)*this->omega_PNLoc_P.cross(this->sHat12_P)
                    + this->mass1*this->omega_PNLoc_P.cross(this->r_S1P_P.cross(this->rPrimeS1P_P))
                    + this->mass1*this->d1*this->theta1Dot*this->theta1Dot*this->r_S1P_P.cross(this->sHat11_P)
                    + (this->theta1Dot+this->theta2Dot)*this->IPntS2_S2(1,1)*this->omega_PNLoc_P.cross(this->sHat22_P)
                    + this->mass2*this->omega_PNLoc_P.cross(this->r_S2P_P.cross(this->rPrimeS2P_P))
                    + this->mass2*this->r_S2P_P.cross(this->l1*this->theta1Dot*this->theta1Dot*this->sHat11_P
                    + this->d2*(this->theta1Dot+this->theta2Dot)*(this->theta1Dot+this->theta2Dot)*this->sHat21_P)
                    + rotFactor0_P*this->matrixEDHRB.row(0)*this->vectorVDHRB
                    + rotFactor1_P*this->matrixEDHRB.row(1)*this->vectorVDHRB)
                    + this->dcm_S1P.transpose()*attBodyTorquePntH1_S1
                    + this->dcm_S2P.transpose()*attBodyTorquePntH2_S2
                    + this->r_H1P_P.cross(externalForcePan1_P)
                    + this->r_H2P_P.cross(externalForcePan2_P);

    return;
}

/*! @brief Compute the effector state derivatives.
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in] rDDot_BN_N [m/s^2] Hub translational acceleration expressed in inertial-frame components.
 * @param[in] omegaDot_BN_B [rad/s^2] Hub angular acceleration expressed in body-frame components.
 * @param[in] sigma_BN Hub attitude relative to the inertial frame.
 */
void DualHingedRigidBodyStateEffector::computeDerivatives(double integTime [[maybe_unused]], Eigen::Vector3d rDDot_BN_N [[maybe_unused]], Eigen::Vector3d omegaDot_BN_B, Eigen::MRPd sigma_BN)
{
    // - Define necessary variables
    Eigen::MRPd sigmaBNLocal;
    Eigen::Matrix3d dcmBN;                        /* direction cosine matrix from N to B */
    Eigen::Matrix3d dcmNB;                        /* direction cosine matrix from B to N */
    Eigen::MatrixXd theta1DDot(1,1);              /* thetaDDot variable to send to state manager */
    Eigen::MatrixXd theta2DDot(1,1);              /* thetaDDot variable to send to state manager */
    Eigen::Vector3d rDDotBNLoc_N;                 /* second time derivative of rBN in N frame */
    Eigen::Vector3d rDDotBNLoc_B;                 /* second time derivative of rBN in B frame */
    Eigen::Vector3d omegaDotBNLoc_B;              /* time derivative of omegaBN in B frame */

    // Grab necessary values from manager (these have been previously computed in hubEffector)
    rDDotBNLoc_N = this->v_BN_NState->getStateDerivReference();
    this->sigma_BN = sigma_BN;
    sigmaBNLocal = this->sigma_BN;
    omegaDotBNLoc_B = omegaDot_BN_B;
    dcmNB = sigmaBNLocal.toRotationMatrix();
    dcmBN = dcmNB.transpose();
    rDDotBNLoc_B = dcmBN*rDDotBNLoc_N;

    // - Compute Derivatives
    // - First is trivial
    this->theta1State->setDerivative(theta1DotState->getStateReference());
    // - Second, a little more involved - see Allard, Diaz, Schaub flex/slosh paper
    theta1DDot(0,0) = this->matrixEDHRB.row(0).dot(this->matrixFDHRB*rDDotBNLoc_B) + this->matrixEDHRB.row(0)*this->matrixGDHRB*omegaDotBNLoc_B + this->matrixEDHRB.row(0)*this->vectorVDHRB;
    this->theta1DotState->setDerivative(theta1DDot);
    this->theta2State->setDerivative(theta2DotState->getStateReference());
    theta2DDot(0,0) = this->matrixEDHRB.row(1)*(this->matrixFDHRB*rDDotBNLoc_B) + this->matrixEDHRB.row(1).dot(this->matrixGDHRB*omegaDotBNLoc_B) + this->matrixEDHRB.row(1)*this->vectorVDHRB;
    this->theta2DotState->setDerivative(theta2DDot);

    return;
}
/*! This method is for calculating the contributions of the DHRB state effector to the energy and momentum of the s/c
 *
 * @param[in] integTime [s] Current integration time.
 * @param[in,out] rotAngMomPntCContr_B [kg*m^2/s] Rotational angular momentum contribution.
 * @param[in,out] rotEnergyContr [J] Rotational energy contribution.
 * @param[in] omega_BN_B [rad/s] Hub angular velocity expressed in body-frame components.
 */
void DualHingedRigidBodyStateEffector::updateEnergyMomContributions(double integTime [[maybe_unused]], Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    // - Get the current omega state
    Eigen::Vector3d omegaLocal_PN_P;
    this->omega_BN_B = omega_BN_B;
    omegaLocal_PN_P = this->omega_BN_B;

    // - Find rotational angular momentum contribution from hub
    Eigen::Vector3d omega_S1P_P;
    Eigen::Vector3d omega_S2P_P;
    Eigen::Vector3d omega_S1N_P;
    Eigen::Vector3d omega_S2N_P;
    Eigen::Matrix3d IPntS1_P;
    Eigen::Matrix3d IPntS2_P;
    Eigen::Vector3d rDot_S1P_P;
    Eigen::Vector3d rDot_S2P_P;
    omega_S1P_P = this->theta1Dot*this->sHat12_P;
    omega_S2P_P = (this->theta1Dot + this->theta2Dot)*this->sHat22_P;
    omega_S1N_P = omega_S1P_P + omegaLocal_PN_P;
    omega_S2N_P = omega_S2P_P + omegaLocal_PN_P;
    IPntS1_P = this->dcm_S1P.transpose()*this->IPntS1_S1*this->dcm_S1P;
    IPntS2_P = this->dcm_S2P.transpose()*this->IPntS2_S2*this->dcm_S2P;
    rDot_S1P_P = this->rPrimeS1P_P + omegaLocal_PN_P.cross(this->r_S1P_P);
    rDot_S2P_P = this->rPrimeS2P_P + omegaLocal_PN_P.cross(this->r_S2P_P);
    rotAngMomPntCContr_B = IPntS1_P*omega_S1N_P + this->mass1*this->r_S1P_P.cross(rDot_S1P_P)
                            + IPntS2_P*omega_S2N_P + this->mass2*this->r_S2P_P.cross(rDot_S2P_P);

    // - Find rotational energy contribution from the hub
    double rotEnergyContrS1;
    double rotEnergyContrS2;
    rotEnergyContrS1 =  0.5*omega_S1N_P.dot(IPntS1_P*omega_S1N_P)
                        + 0.5*this->mass1*rDot_S1P_P.dot(rDot_S1P_P)
                        + 0.5*this->k1*this->theta1*this->theta1;
    rotEnergyContrS2 =  0.5*omega_S2N_P.dot(IPntS2_P*omega_S2N_P)
                        + 0.5*this->mass2*rDot_S2P_P.dot(rDot_S2P_P)
                        + 0.5*this->k2*this->theta2*this->theta2;
    rotEnergyContr = rotEnergyContrS1 + rotEnergyContrS2;

    return;
}

/*! This method takes the computed theta states and outputs them to the m
 messaging system.

 @param CurrentClock The current simulation time (used for time stamping)
 */
void DualHingedRigidBodyStateEffector::writeOutputStateMessages(uint64_t CurrentClock)
{
    this->computePanelInertialStates();

    HingedRigidBodyMsgPayload panelOutputStates;  //!< instance of messaging system message struct

    // panel 1 states
    panelOutputStates.theta = this->theta1;
    panelOutputStates.thetaDot = this->theta1Dot;
    this->dualHingedRigidBodyOutMsgs[0]->write(&panelOutputStates, this->moduleID, CurrentClock);
    // panel 2 states
    panelOutputStates.theta = this->theta2;
    panelOutputStates.thetaDot = this->theta2Dot;
    this->dualHingedRigidBodyOutMsgs[1]->write(&panelOutputStates, this->moduleID, CurrentClock);


    // write out the panel state config log message
    SCStatesMsgPayload configLogMsg;
    // Note, logging the hinge frame S is the body frame B of that object
    for (size_t i = 0; i < 2; i++) {
        configLogMsg = this->dualHingedRigidBodyConfigLogOutMsgs[i]->zeroMsgPayload;
        eigenVector3d2CArray(this->r_SN_N[i], configLogMsg.r_BN_N);
        eigenVector3d2CArray(this->v_SN_N[i], configLogMsg.v_BN_N);
        eigenMatrixXd2CArray(*this->sigma_SN[i], configLogMsg.sigma_BN);
        eigenMatrixXd2CArray(*this->omega_SN_S[i], configLogMsg.omega_BN_B);
        this->dualHingedRigidBodyConfigLogOutMsgs[i]->write(&configLogMsg, this->moduleID, CurrentClock);
    }
}


/*! This method is used so that the simulation will ask DHRB to update messages.

 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void DualHingedRigidBodyStateEffector::UpdateState(uint64_t CurrentSimNanos)
{
    //! - Zero the command buffer and read the incoming command array
    if (this->motorTorqueInMsg.isLinked()) {
        ArrayMotorTorqueMsgPayload incomingCmdBuffer;
        incomingCmdBuffer = this->motorTorqueInMsg();
        this->u1 = incomingCmdBuffer.motorTorque[0];
        this->u2 = incomingCmdBuffer.motorTorque[1];
    }

    this->writeOutputStateMessages(CurrentSimNanos);

    return;
}

/*! This method computes the panel states relative to the inertial frame

 */
void DualHingedRigidBodyStateEffector::computePanelInertialStates()
{
    // - read live: the cached copy lags half a step at write time, unless a prescribed body set it
    if (this->prescribedAttitudeProperty == nullptr) {
        this->sigma_BN = Eigen::MRPd(this->hubSigmaState->getStateReference().data());
    }
    Eigen::MRPd sigmaPN = this->sigma_BN;
    Eigen::Matrix3d dcm_NP = sigmaPN.toRotationMatrix();
    const Eigen::MRPd sigma_S1N = eigenC2MRP(this->dcm_S1P*dcm_NP.transpose());
    const Eigen::MRPd sigma_S2N = eigenC2MRP(this->dcm_S2P*dcm_NP.transpose());
    *this->sigma_SN[0] = sigma_S1N.coeffs();
    *this->sigma_SN[1] = sigma_S2N.coeffs();

    // inertial angular velocities
    Eigen::Vector3d omega_PN_P;
    omega_PN_P = this->omega_BN_B;
    *this->omega_SN_S[0] = this->dcm_S1P * (omega_PN_P + this->theta1Dot*this->sHat12_P);
    *this->omega_SN_S[1] = this->dcm_S2P * (omega_PN_P + this->theta1Dot*this->sHat12_P + this->theta2Dot*this->sHat22_P);

    // inertial position vectors
    Eigen::Vector3d r_PN_N;
    r_PN_N = (Eigen::Vector3d)(*this->inertialPositionProperty);
    this->r_SN_N[0] = Eigen::Vector3d(dcm_NP * this->r_S1P_P) + r_PN_N;
    this->r_SN_N[1] = Eigen::Vector3d(dcm_NP * this->r_S2P_P) + r_PN_N;

    *this->r_HN_N[0] = Eigen::Vector3d(dcm_NP * this->r_H1P_P) + r_PN_N;
    *this->r_HN_N[1] = Eigen::Vector3d(dcm_NP * this->r_H2P_P) + r_PN_N;

    // inertial velocity vectors
    Eigen::Vector3d v_PN_N = (Eigen::Vector3d)(*this->inertialVelocityProperty);
    Eigen::Vector3d omega_S1N_P = this->theta1Dot * this->sHat12_P + omega_PN_P;
    Eigen::Vector3d omega_S2N_P = this->theta2Dot * this->sHat22_P + omega_S1N_P;
    Eigen::Vector3d rDot_H1P_P = omega_PN_P.cross(this->r_H1P_P);
    Eigen::Vector3d rDot_H2P_P = rDot_H1P_P + omega_S1N_P.cross( -this->l1 * this->sHat11_P);

    this->v_SN_N[0] = v_PN_N + Eigen::Vector3d(dcm_NP * (rDot_H1P_P
                    + omega_S1N_P.cross( -this->d1 * this->sHat11_P)));
    this->v_SN_N[1] = v_PN_N + Eigen::Vector3d(dcm_NP * (rDot_H2P_P
                    + omega_S2N_P.cross( -this->d2 * this->sHat21_P)));

    *this->v_HN_N[0] = v_PN_N + Eigen::Vector3d(dcm_NP * rDot_H1P_P);
    *this->v_HN_N[1] = v_PN_N + Eigen::Vector3d(dcm_NP * rDot_H2P_P);

    return;
}
