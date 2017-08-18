/*
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "utilities/avsEigenSupport.h"

/*! This is the constructor, setting variables to default values */
HubEffector::HubEffector()
{
    // - zero certain variables
    this->effProps.mEff = 0.0;
    this->effProps.rEff_CB_B.setZero();
    this->effProps.IEffPntB_B.setZero();
    this->effProps.rEffPrime_CB_B.setZero();
    this->effProps.IEffPrimePntB_B.setZero();
    this->sumForceExternal_N.setZero();
    this->sumForceExternal_B.setZero();
    this->sumTorquePntB_B.setZero();
    this->r_CN_NInit.setZero();
    this->v_CN_NInit.setZero();
    this->sigma_BNInit.setZero();
    this->omega_BN_BInit.setZero();

    // - define default names for the hub states
    this->nameOfHubPosition = "hubPosition";
    this->nameOfHubVelocity = "hubVelocity";
    this->nameOfHubSigma = "hubSigma";
    this->nameOfHubOmega = "hubOmega";

    // - define a default mass of 1kg
    this->mHub = 1.0;
    this->IHubPntBc_B.setIdentity();
    this->r_BcB_B.fill(0.0);

    // - Default simulation to useTranslation and useRotation as true for now
    this->useTranslation = true;
    this->useRotation = true;

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
    // - Get reference to mass props
	this->m_SC = statesIn.getPropertyReference("m_SC");
	this->mDot_SC = statesIn.getPropertyReference("mDot_SC");
    this->c_B = statesIn.getPropertyReference("centerOfMassSC");
    this->ISCPntB_B = statesIn.getPropertyReference("inertiaSC");
    this->cPrime_B = statesIn.getPropertyReference("centerOfMassPrimeSC");
    this->ISCPntBPrime_B = statesIn.getPropertyReference("inertiaPrimeSC");
    this->g_N = statesIn.getPropertyReference("g_N");

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
    /* - r_BN_N and v_BN_N of the hub is first set to r_CN_N and v_CN_N and then is corrected in spacecraftPlus
     initializeDynamics to incorporate the fact that point B and point C are not necessarily coincident */
    this->posState->setState(this->r_CN_NInit);
    this->velocityState->setState(this->v_CN_NInit);
    this->sigmaState->setState(this->sigma_BNInit);
    this->omegaState->setState(this->omega_BN_BInit);

    return;
}

/*! This method allows the hub to give its mass properties to the spacecraft */
void HubEffector::updateEffectorMassProps(double integTime)
{
    // - Give the mass to mass props
    this->effProps.mEff = this->mHub;

    // - Give inertia of hub about point B to mass props
    this->effProps.IEffPntB_B = this->IHubPntBc_B
                                           + this->mHub*eigenTilde(this->r_BcB_B)*eigenTilde(this->r_BcB_B).transpose();

    // - Give position of center of mass of hub with respect to point B to mass props
    this->effProps.rEff_CB_B = this->r_BcB_B;

    // - Zero body derivatives for position and inertia;
    this->effProps.rEffPrime_CB_B.setZero();
    this->effProps.IEffPrimePntB_B.setZero();

    return;
}

/*! This method is for computing the derivatives of the hub: rDDot_BN_N and omegaDot_BN_B, along with the kinematic
 derivatives */
void HubEffector::computeDerivatives(double integTime)
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
    cLocal_B = *this->c_B;
    cPrimeLocal_B = *cPrime_B;
    gLocal_N = *this->g_N;

    // -  Make additional contributions to the matrices from the hub
    Eigen::Matrix3d intermediateMatrix;
    Eigen::Vector3d intermediateVector;
    this->matrixA += (*this->m_SC)(0,0)*intermediateMatrix.Identity();
    intermediateMatrix = eigenTilde((*this->c_B));  // make c_B skew symmetric matrix
    this->matrixB += -(*this->m_SC)(0,0)*intermediateMatrix;
    this->matrixC += (*this->m_SC)(0,0)*intermediateMatrix;
    this->matrixD += *ISCPntB_B;
	this->vecTrans += -2.0*(*this->m_SC)(0, 0)*omegaLocal_BN_B.cross(cPrimeLocal_B)
		- (*this->m_SC)(0, 0)*omegaLocal_BN_B.cross(omegaLocal_BN_B.cross(cLocal_B));
											- 2.0*(*mDot_SC)(0,0)*(cPrimeLocal_B+omegaLocal_BN_B.cross(cLocal_B));
    intermediateVector = *ISCPntB_B*omegaLocal_BN_B;
    this->vecRot += -omegaLocal_BN_B.cross(intermediateVector) - *ISCPntBPrime_B*omegaLocal_BN_B;

    // - Need to find force of gravity on the spacecraft
    Eigen::Vector3d gravityForce_N;
    gravityForce_N = (*this->m_SC)(0,0)*gLocal_N;

    if (this->useRotation == true) {
        // - Set kinematic derivative
        sigmaState->setDerivative(1.0/4.0*sigmaLocal_BN.Bmat()*omegaLocal_BN_B);

        if (this->useTranslation == true) {
            // - Define dcm's
            Eigen::Matrix3d dcm_NB;
            Eigen::Matrix3d dcm_BN;
            dcm_NB = sigmaLocal_BN.toRotationMatrix();
            dcm_BN = dcm_NB.transpose();

            // - Map external force_N to the body frame
            Eigen::Vector3d sumForceExternalMappedToB;
            sumForceExternalMappedToB = dcm_BN*this->sumForceExternal_N;

            // - Edit both v_trans and v_rot with gravity and external force and torque
            Eigen::Vector3d gravityForce_B;
            gravityForce_B = dcm_BN*gravityForce_N;
            this->vecTrans += gravityForce_B + sumForceExternalMappedToB + this->sumForceExternal_B;
            this->vecRot += cLocal_B.cross(gravityForce_B) + this->sumTorquePntB_B;

            // - Solve for omegaDot_BN_B
            Eigen::Vector3d omegaDot_BN_B;
            intermediateVector = this->vecRot - this->matrixC*this->matrixA.inverse()*this->vecTrans;
            intermediateMatrix = matrixD - matrixC*matrixA.inverse()*matrixB;
            omegaDot_BN_B = intermediateMatrix.inverse()*intermediateVector;
            omegaState->setDerivative(omegaDot_BN_B);

            // - Solve for rDDot_BN_N
            velocityState->setDerivative(dcm_NB*matrixA.inverse()*(vecTrans - matrixB*omegaDot_BN_B));
        } else {
            // - Only add in sumTorques to vecRot;
            vecRot += this->sumTorquePntB_B;

            // - Solve for omegaDot_BN_B
            omegaState->setDerivative(this->matrixD.inverse()*vecRot);
        }
    }

    if (this->useTranslation==true) {
        // - Set kinematic derivative
        posState->setDerivative(rDotLocal_BN_N);

        if (this->useRotation==false) {
            // - If it is just translating, only compute translational terms
            // - Add in gravity
            vecTrans += gravityForce_N + this->sumForceExternal_N;

            // - Find rDDot_BN_N
            velocityState->setDerivative(matrixA.inverse()*(vecTrans));
        }
	}

    return;
}

/*! This method is for computing the energy and momentum contributions from the hub */
void HubEffector::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                               double & rotEnergyContr)
{
    // - Get variables needed for energy momentum calcs
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = omegaState->getState();

    //  - Find rotational angular momentum contribution from hub
    Eigen::Vector3d rDot_BcB_B;
    rDot_BcB_B = omegaLocal_BN_B.cross(r_BcB_B);
    rotAngMomPntCContr_B = IHubPntBc_B*omegaLocal_BN_B + mHub*r_BcB_B.cross(rDot_BcB_B);

    // - Find rotational energy contribution from the hub
    rotEnergyContr = 1.0/2.0*omegaLocal_BN_B.dot(IHubPntBc_B*omegaLocal_BN_B) + 1.0/2.0*mHub*rDot_BcB_B.dot(rDot_BcB_B);
    
    return;
}
