/*
 Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder
 
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

using namespace Eigen;
using namespace std;

HubEffector::HubEffector()
{
    effProps.IEffPntB_B.fill(0.0);
    effProps.IEffPrimePntB_B.fill(0.0);
    effProps.rCB_B.fill(0.0);
    effProps.rPrimeCB_B.fill(0.0);
    effProps.mEff = 0.0;

    return;
}


HubEffector::~HubEffector()
{
    return;
}

void HubEffector::linkInStates(DynParamManager& statesIn)
{
    this->m_SC = statesIn.getPropertyReference("m_SC");
    this->c_B = statesIn.getPropertyReference("centerOfMassSC");
    this->ISCPntB_B = statesIn.getPropertyReference("inertiaSC");
    this->cPrime_B = statesIn.getPropertyReference("centerOfMassPrimeSC");
    this->ISCPntBPrime_B = statesIn.getPropertyReference("inertiaPrimeSC");
}

void HubEffector::registerStates(DynParamManager& states)
{
    this->posState = states.registerState(3, 1, "hubPosition");
    this->velocityState = states.registerState(3, 1, "hubVelocity");
    this->sigmaState = states.registerState(3, 1, "hubSigma");
    this->omegaState = states.registerState(3, 1, "hubOmega");
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
    Eigen::Vector3d omegaBNDot_B;
    Eigen::Vector3d rBNDDotLocal_B;
    Eigen::Vector3d rBNDDotLocal_N;
    Eigen::Matrix3d intermediateMatrix;
    Eigen::Vector3d intermediateVector;
    Eigen::Vector3d omegaBNLocal;
    Eigen::Vector3d cPrimeLocal_B;
    Eigen::Vector3d cLocal_B;
    Eigen::Vector3d rBNDotLocal_N;
    Eigen::Matrix3d Bmat;
    Eigen::Matrix3d dcmNB;
    MRPd sigmaBNLocal;
    Eigen::Vector3d sigmaBNDotLocal;
    Eigen::Matrix3d BN;
    sigmaBNLocal = (Eigen::Vector3d )sigmaState->getState();
    omegaBNLocal = omegaState->getState();
    rBNDotLocal_N = velocityState->getState();
    cPrimeLocal_B = *cPrime_B;
    cLocal_B = *c_B;

    //! - Need to scale [A] [B] and vecTrans by m_SC
    matrixA = matrixA/(*this->m_SC)(0,0);
    matrixB = matrixB/(*this->m_SC)(0,0);
    vecTrans = vecTrans/(*this->m_SC)(0,0);

    ////! Need to make contributions to the matrices from the hub
    intermediateMatrix = intermediateMatrix.Identity();
    matrixA += intermediateMatrix;
    //! make c_B skew symmetric matrix
    intermediateMatrix <<  0 , -(*c_B)(2,0),
    (*c_B)(1,0), (*c_B)(2,0), 0, -(*c_B)(0,0), -(*c_B)(1,0), (*c_B)(0,0), 0;
    matrixB -= intermediateMatrix;
    matrixC += this->m_SC->value()*intermediateMatrix;
    matrixD += *ISCPntB_B;
    vecTrans += -2*omegaBNLocal.cross(cPrimeLocal_B) -omegaBNLocal.cross(omegaBNLocal.cross(cLocal_B));
    intermediateVector = *ISCPntB_B*omegaBNLocal;
    vecRot += -omegaBNLocal.cross(intermediateVector) - *ISCPntBPrime_B*omegaBNLocal;

    if (this->useRotation==true) {
        //! Set kinematic derivative
        Bmat = sigmaBNLocal.Bmat();
        sigmaBNDotLocal = 1.0/4.0*Bmat*omegaBNLocal;
        sigmaState->setDerivative(sigmaBNDotLocal);

        if (this->useTranslation==true) {
            intermediateVector = vecRot - matrixC*matrixA.inverse()*vecTrans;
            intermediateMatrix = matrixD - matrixC*matrixA.inverse()*matrixB;
            omegaBNDot_B = intermediateMatrix.inverse()*intermediateVector;
            omegaState->setDerivative(omegaBNDot_B);
            rBNDDotLocal_B = matrixA.inverse()*(vecTrans - matrixB*omegaBNDot_B);
            //! - Map rBNDDotLocal_B to rBNDotLocal_N
            dcmNB = sigmaBNLocal.toRotationMatrix();
            rBNDDotLocal_N = dcmNB*rBNDDotLocal_B;
            velocityState->setDerivative(rBNDDotLocal_N);
        }
        omegaBNDot_B = matrixD.inverse()*vecRot;
        omegaState->setDerivative(omegaBNDot_B);
    }

    if (this->useTranslation==true) {
        //! - Set kinematic derivative
        posState->setDerivative(rBNDotLocal_N);
        if (this->useRotation==false) {
            rBNDDotLocal_N = matrixA.inverse()*(vecTrans);
            velocityState->setDerivative(rBNDDotLocal_N);
        }
	}
}
