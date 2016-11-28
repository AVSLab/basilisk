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

HubEffector::HubEffector()
{
    //! - zero mass contributions
    this->effProps.mEff = 0.0;
    this->effProps.rCB_B.fill(0.0);
    this->effProps.IEffPntB_B.fill(0.0);
    this->effProps.rPrimeCB_B.fill(0.0);
    this->effProps.IEffPrimePntB_B.fill(0.0);
    this->sumForceExternal_N.fill(0.0);
    this->sumForceExternal_B.fill(0.0);
    this->sumTorquePntB_B.fill(0.0);

    //! - define default names for the hub states
    this->nameOfHubPosition = "hubPosition";
    this->nameOfHubVelocity = "hubVelocity";
    this->nameOfHubSigma = "hubSigma";
    this->nameOfHubOmega = "hubOmega";

    // define a default mass of 1kg.
    this->mHub = 1.0;                       /*!< [kg]   default mass value */
    this->IHubPntBc_B.setIdentity();
    this->rBcB_B.fill(0.0);

    //! - Default simulation to useTranslation and useRotation as true for now
    this->useTranslation = true;
    this->useRotation = true;
    return;
}


HubEffector::~HubEffector()
{
    return;
}

void HubEffector::linkInStates(DynParamManager& statesIn)
{
    //! - Get reference to mass props
    this->m_SC = statesIn.getPropertyReference("m_SC");
    this->c_B = statesIn.getPropertyReference("centerOfMassSC");
    this->ISCPntB_B = statesIn.getPropertyReference("inertiaSC");
    this->cPrime_B = statesIn.getPropertyReference("centerOfMassPrimeSC");
    this->ISCPntBPrime_B = statesIn.getPropertyReference("inertiaPrimeSC");
    this->g_N = statesIn.getPropertyReference("g_N");

    return;
}

void HubEffector::registerStates(DynParamManager& states)
{
    //! - Register the hub states
    this->posState = states.registerState(3, 1, this->nameOfHubPosition);
    this->velocityState = states.registerState(3, 1, this->nameOfHubVelocity);
    this->sigmaState = states.registerState(3, 1, this->nameOfHubSigma);
    this->omegaState = states.registerState(3, 1, this->nameOfHubOmega);

    return;
}

void HubEffector::updateEffectorMassProps(double integTime)
{
    //! Give the mass to mass props
    effProps.mEff = this->mHub;

    //! Give inertia of hub about point B to mass props
    Eigen::Matrix3d intermediateMatrix;
    intermediateMatrix << 0 , -this->rBcB_B(2), this->rBcB_B(1), this->rBcB_B(2), 0, -this->rBcB_B(0), -this->rBcB_B(1), this->rBcB_B(0), 0;
    effProps.IEffPntB_B = this->IHubPntBc_B + this->mHub*intermediateMatrix*intermediateMatrix.transpose();

    //! Give position of center of mass of hub with respect to point B to mass props
    effProps.rCB_B = this->rBcB_B;
}

void HubEffector::computeDerivatives(double integTime)
{
    //! - Define variables needed for derivative calculations
    Eigen::Vector3d omegaBNDot_B;
    Eigen::Vector3d rBNDDotLocal_B;
    Eigen::Vector3d rBNDDotLocal_N;
    Eigen::Matrix3d intermediateMatrix;
    Eigen::Vector3d intermediateVector;
    Eigen::Vector3d omegaBNLocal;
    Eigen::Vector3d cPrimeLocal_B;
    Eigen::Vector3d cLocal_B;
    Eigen::Vector3d gLocal_N;
    Eigen::Vector3d rBNDotLocal_N;
    Eigen::Matrix3d Bmat;
    Eigen::MRPd sigmaBNLocal;
    Eigen::Vector3d sigmaBNDotLocal;
    Eigen::Vector3d gravityForce_N;
    Eigen::Vector3d gravityForce_B;
    sigmaBNLocal = (Eigen::Vector3d )sigmaState->getState();
    omegaBNLocal = omegaState->getState();
    rBNDotLocal_N = velocityState->getState();
    cPrimeLocal_B = *cPrime_B;
    cLocal_B = *this->c_B;
    gLocal_N = *this->g_N;

    ////! Make additional contributions to the matrices from the hub
    intermediateMatrix = intermediateMatrix.Identity();
    this->matrixA += (*this->m_SC)(0,0)*intermediateMatrix;
    //! make c_B skew symmetric matrix
    intermediateMatrix <<  0 , -(*c_B)(2,0),
    (*c_B)(1,0), (*c_B)(2,0), 0, -(*c_B)(0,0), -(*c_B)(1,0), (*c_B)(0,0), 0;
    this->matrixB += -(*this->m_SC)(0,0)*intermediateMatrix;
    this->matrixC += (*this->m_SC)(0,0)*intermediateMatrix;
    this->matrixD += *ISCPntB_B;
    this->vecTrans += -2.0*(*this->m_SC)(0,0)*omegaBNLocal.cross(cPrimeLocal_B) - (*this->m_SC)(0,0)*omegaBNLocal.cross(omegaBNLocal.cross(cLocal_B));
    intermediateVector = *ISCPntB_B*omegaBNLocal;
    this->vecRot += -omegaBNLocal.cross(intermediateVector) - *ISCPntBPrime_B*omegaBNLocal;

    //! - Need to find force of gravity on the spacecraft
    gravityForce_N = (*this->m_SC)(0,0)*gLocal_N;

    if (this->useRotation == true) {
        //! Set kinematic derivative
        Bmat = sigmaBNLocal.Bmat();
        sigmaBNDotLocal = 1.0/4.0*Bmat*omegaBNLocal;
        sigmaState->setDerivative(sigmaBNDotLocal);

        if (this->useTranslation == true) {
            //! - Define values only needed for rotation and translation together
            Eigen::Matrix3d dcmNB;
            Eigen::Matrix3d dcmBN;
            Eigen::Vector3d sumForceExternalMappedToB;

            //! - Define dcm's
            dcmNB = sigmaBNLocal.toRotationMatrix();
            dcmBN = dcmNB.transpose();

            //! - Map external force_N to the body frame
            sumForceExternalMappedToB = dcmBN*this->sumForceExternal_N;

            //! - Edit both v_trans and v_rot with gravity and external force and torque
            gravityForce_B = dcmBN*gravityForce_N;
            this->vecTrans += gravityForce_B + sumForceExternalMappedToB + this->sumForceExternal_B;
            this->vecRot += cLocal_B.cross(gravityForce_B) + this->sumTorquePntB_B;

            //! - Complete the Back-Substitution Method
            intermediateVector = this->vecRot - this->matrixC*this->matrixA.inverse()*this->vecTrans;
            intermediateMatrix = matrixD - matrixC*matrixA.inverse()*matrixB;
            omegaBNDot_B = intermediateMatrix.inverse()*intermediateVector;
            omegaState->setDerivative(omegaBNDot_B);
            rBNDDotLocal_B = matrixA.inverse()*(vecTrans - matrixB*omegaBNDot_B);
            //! - Map rBNDDotLocal_B to rBNDotLocal_N
            dcmNB = sigmaBNLocal.toRotationMatrix();
            rBNDDotLocal_N = dcmNB*rBNDDotLocal_B;
            velocityState->setDerivative(rBNDDotLocal_N);
        } else {
            //! - Edit only v_rot with gravity
            vecRot += cLocal_B.cross(gravityForce_B) + this->sumTorquePntB_B;

            //! - Only compute rotational terms
            omegaBNDot_B = matrixD.inverse()*vecRot;
            omegaState->setDerivative(omegaBNDot_B);
        }
    }

    if (this->useTranslation==true) {
        //! - Set kinematic derivative
        posState->setDerivative(rBNDotLocal_N);
        if (this->useRotation==false) {
            //! - If it is just translating, only compute translational terms
            //! - need to add in gravity
            vecTrans += gravityForce_N + this->sumForceExternal_N;
            
            rBNDDotLocal_N = matrixA.inverse()*(vecTrans);
            velocityState->setDerivative(rBNDDotLocal_N);
        }
	}
}
