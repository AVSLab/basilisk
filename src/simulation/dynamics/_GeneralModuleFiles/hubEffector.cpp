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

#include "hubEffector.h"
#include "architecture/utilities/avsEigenSupport.h"

/*! This is the constructor, setting variables to default values */
HubEffector::HubEffector()
{
    // - zero certain variables
    this->MRPSwitchCount = 0;
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.setZero();
    this->effProps.IEffPntB_B.setZero();
    this->effProps.rEffPrime_CB_B.setZero();
    this->effProps.IEffPrimePntB_B.setZero();
    this->r_CN_NInit.setZero();
    this->v_CN_NInit.setZero();
    this->sigma_BNInit.setZero();
    this->omega_BN_BInit.setZero();

    // - define default names for the hub states
    this->nameOfHubPosition = "hubPosition";
    this->nameOfHubVelocity = "hubVelocity";
    this->nameOfHubSigma = "hubSigma";
    this->nameOfHubOmega = "hubOmega";
    this->nameOfHubGravVelocity = "hubGravVelocity";
    this->nameOfBcGravVelocity = "BcGravVelocity";

    // - define a default mass of 1kg
    this->mHub = 1.0;
    this->IHubPntBc_B.setIdentity();
    this->r_BcB_B.fill(0.0);

    return;
}

/*! This is the destructor, nothing to report here */
HubEffector::~HubEffector()
{
    return;
}

/*! This method allows the hub access to gravity and also gets access to the properties in the dyn Manager because uses
 these values in the computeDerivatives method call */
void HubEffector::linkInStates(DynParamManager& statesIn)
{
    this->g_N = statesIn.getPropertyReference(this->nameOfSpacecraftAttachedTo + "g_N");
    return;
}

void HubEffector::prependSpacecraftNameToStates()
{
    this->nameOfHubPosition = this->nameOfSpacecraftAttachedTo + this->nameOfHubPosition;
    this->nameOfHubVelocity = this->nameOfSpacecraftAttachedTo + this->nameOfHubVelocity;
    this->nameOfHubSigma = this->nameOfSpacecraftAttachedTo + this->nameOfHubSigma;
    this->nameOfHubOmega = this->nameOfSpacecraftAttachedTo + this->nameOfHubOmega;
    this->nameOfHubGravVelocity = this->nameOfSpacecraftAttachedTo + this->nameOfHubGravVelocity;
    this->nameOfBcGravVelocity = this->nameOfSpacecraftAttachedTo + this->nameOfBcGravVelocity;

    return;
}

/*! This method allows the hub to register its states: r_BN_N, v_BN_N, sigma_BN and omega_BN_B */
void HubEffector::registerStates(DynParamManager& states)
{
    // - Register the hub states and set with initial values
    this->posState = states.registerState(3, 1, this->nameOfHubPosition);
    this->velocityState = states.registerState(3, 1, this->nameOfHubVelocity);
    this->sigmaState = states.registerState(3, 1, this->nameOfHubSigma);
    this->omegaState = states.registerState(3, 1, this->nameOfHubOmega);
    this->gravVelocityState = states.registerState(3, 1, this->nameOfHubGravVelocity);
    this->gravVelocityBcState = states.registerState(3, 1, this->nameOfBcGravVelocity);
    /* - r_BN_N and v_BN_N of the hub is first set to r_CN_N and v_CN_N and then is corrected in spacecraft
     initializeDynamics to incorporate the fact that point B and point C are not necessarily coincident */
    this->posState->setState(this->r_CN_NInit);
    this->velocityState->setState(this->v_CN_NInit);
    this->sigmaState->setState(this->sigma_BNInit);
    this->omegaState->setState(this->omega_BN_BInit);
    this->gravVelocityState->setState(this->v_CN_NInit);
    this->gravVelocityBcState->setState(this->v_CN_NInit);

    return;
}

/*! This method allows the hub to give its mass properties to the spacecraft */
void HubEffector::updateEffectorMassProps(double integTime)
{
    // - Give the mass to mass props
    this->effProps.mEff = this->mHub;

    // - Provide information about multi-spacecraft origin if needed
    this->r_BcP_P = this->r_BP_P + this->dcm_BP.transpose()*(this->r_BcB_B);
    this->IHubPntBc_P = this->dcm_BP.transpose()*this->IHubPntBc_B*this->dcm_BP;

    // - Give inertia of hub about point B to mass props
    this->effProps.IEffPntB_B = this->IHubPntBc_P
                                           + this->mHub*eigenTilde(this->r_BcP_P)*eigenTilde(this->r_BcP_P).transpose();

    // - Give position of center of mass of hub with respect to point B to mass props
    this->effProps.rEff_CB_B = this->r_BcP_P;

    // - Zero body derivatives for position and inertia;
    this->effProps.rEffPrime_CB_B.setZero();
    this->effProps.IEffPrimePntB_B.setZero();

    return;
}

/*! This method is for computing the derivatives of the hub: rDDot_BN_N and omegaDot_BN_B, along with the kinematic
 derivatives */
void HubEffector::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
    // - Get variables from state manager
    Eigen::Vector3d rDotLocal_BN_N;
    Eigen::MRPd sigmaLocal_BN;
    Eigen::Vector3d omegaLocal_BN_B;
    Eigen::Vector3d cLocal_B;
    Eigen::Vector3d cPrimeLocal_B;
    Eigen::Vector3d gLocal_N;
    rDotLocal_BN_N = velocityState->getState();
    sigmaLocal_BN = (Eigen::Vector3d )sigmaState->getState();
    omegaLocal_BN_B = omegaState->getState();
    gLocal_N = *this->g_N;

    // - Set kinematic derivative
    sigmaState->setDerivative(1.0/4.0*sigmaLocal_BN.Bmat()*omegaLocal_BN_B);

    // - Define dcm's
    Eigen::Matrix3d dcm_NB;
    Eigen::Matrix3d dcm_BN;
    dcm_NB = sigmaLocal_BN.toRotationMatrix();
    dcm_BN = dcm_NB.transpose();

    // - Solve for omegaDot_BN_B
    Eigen::Vector3d omegaDotLocal_BN_B;
    Eigen::Matrix3d intermediateMatrix;
    Eigen::Vector3d intermediateVector;
    intermediateVector = this->hubBackSubMatrices.vecRot - this->hubBackSubMatrices.matrixC*this->hubBackSubMatrices.matrixA.inverse()*this->hubBackSubMatrices.vecTrans;
    intermediateMatrix = hubBackSubMatrices.matrixD - hubBackSubMatrices.matrixC*hubBackSubMatrices.matrixA.inverse()*hubBackSubMatrices.matrixB;
    omegaDotLocal_BN_B = intermediateMatrix.inverse()*intermediateVector;
    omegaState->setDerivative(omegaDotLocal_BN_B);

    // - Solve for rDDot_BN_N
    velocityState->setDerivative(dcm_NB*hubBackSubMatrices.matrixA.inverse()*(hubBackSubMatrices.vecTrans - hubBackSubMatrices.matrixB*omegaDotLocal_BN_B));

    // - Set gravity velocity derivatives
    gravVelocityState->setDerivative(gLocal_N);
    gravVelocityBcState->setDerivative(gLocal_N);

    // - Set kinematic derivative
    posState->setDerivative(rDotLocal_BN_N);

    return;
}

/*! This method is for computing the energy and momentum contributions from the hub */
void HubEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                               double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    // - Get variables needed for energy momentum calcs
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = omegaState->getState();

    //  - Find rotational angular momentum contribution from hub
    Eigen::Vector3d rDot_BcB_B;
    rDot_BcB_B = omegaLocal_BN_B.cross(r_BcP_P);
    rotAngMomPntCContr_B = IHubPntBc_P*omegaLocal_BN_B + mHub*r_BcP_P.cross(rDot_BcB_B);

    // - Find rotational energy contribution from the hub
    rotEnergyContr = 1.0/2.0*omegaLocal_BN_B.dot(IHubPntBc_P*omegaLocal_BN_B) + 1.0/2.0*mHub*rDot_BcB_B.dot(rDot_BcB_B);
    
    return;
}

/*! This method is for switching the MRPs */
void HubEffector::modifyStates(double integTime)
{
    // Lets switch those MRPs!!
    Eigen::Vector3d sigmaBNLoc;
    sigmaBNLoc = (Eigen::Vector3d) this->sigmaState->getState();
    if (sigmaBNLoc.norm() > 1) {
        sigmaBNLoc = -sigmaBNLoc/(sigmaBNLoc.dot(sigmaBNLoc));
        this->sigmaState->setState(sigmaBNLoc);
        this->MRPSwitchCount++;
    }
    return;
}

/*! This method is used to set the gravitational velocity state equal to the base velocity state */
void HubEffector::matchGravitytoVelocityState(Eigen::Vector3d v_CN_N)
{
    this->gravVelocityState->setState(this->velocityState->getState());
    this->gravVelocityBcState->setState(v_CN_N);
}