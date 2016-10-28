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
    //# posRef.setState([[1.0], [0.0], [0.0]])
    //# omegaRef.setState([[0.001], [0.0], [0.0]])
    //# sigmaRef.setState([[0.0], [0.0], [0.0]])
    //# velRef.setState([[0.01], [0.0], [0.0]])
    this->posState->state << 1.0, 0.0, 0.0;
    this->omegaState->state << 0.001, -0.002, 0.003;
    this->sigmaState->state.setZero();
    this->velocityState->state << 55.24, 0.0, 0.0;
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
    //! - Need to add hub to m_SC, c_B and ISCPntB_B
    *this->m_SC += this->mHub;
    *this->c_B += this->mHub.value()*this->rBcB_B;
    *ISCPntB_B += this->IHubPntB_B;

    //! - Need to scale [A] [B] and vecTrans by m_SC
    matrixASCP = matrixASCP/this->m_SC->value();
    matrixASCP = matrixBSCP/this->m_SC->value();
    vecTransSCP = vecTransSCP/this->m_SC->value();

    ////! Need to make contributions to the matrices from the hub
    intermediateMatrix = intermediateMatrix.Identity();
    matrixASCP += intermediateMatrix;
    //! make c_B skew symmetric matrix
    intermediateMatrix <<  0 , -(*c_B)(2,0),
    (*c_B)(1,0), (*c_B)(2,0), 0, -(*c_B)(0,0), -(*c_B)(1,0), (*c_B)(0,0), 0;
    matrixBSCP -= intermediateMatrix;
    matrixCSCP += this->m_SC->value()*intermediateMatrix;
    matrixDSCP += *ISCPntB_B;
    vecTransSCP += -2*omegaBNLocal.cross(cPrimeLocal_B) -omegaBNLocal.cross(omegaBNLocal.cross(cLocal_B));
    intermediateVector = *ISCPntB_B*omegaBNLocal;
    vecRotSCP += -omegaBNLocal.cross(intermediateVector) - *ISCPntBPrime_B*omegaBNLocal;

    if (this->useRotation==true) {
        //! Set kinematic derivative
        Bmat = sigmaBNLocal.Bmat();
        sigmaBNDotLocal = 1.0/4.0*Bmat*omegaBNLocal;
        sigmaState->setDerivative(sigmaBNDotLocal);

        if (this->useTranslation==true) {
            intermediateVector = vecRotSCP - matrixCSCP*matrixASCP.inverse()*vecTransSCP;
            intermediateMatrix = matrixDSCP - matrixCSCP*matrixASCP.inverse()*matrixBSCP;
            omegaBNDot_B = intermediateMatrix.inverse()*intermediateVector;
            omegaState->setDerivative(omegaBNDot_B);
            rBNDDotLocal_B = matrixASCP.inverse()*(vecTransSCP - matrixBSCP*omegaBNDot_B);
            //! - Map rBNDDotLocal_B to rBNDotLocal_N
            dcmNB = sigmaBNLocal.toRotationMatrix();
            rBNDDotLocal_N = dcmNB*rBNDDotLocal_B;
            velocityState->setDerivative(rBNDDotLocal_N);
        }
        omegaBNDot_B = matrixDSCP.inverse()*vecRotSCP;
        omegaState->setDerivative(omegaBNDot_B);
    }

    if (this->useTranslation==true) {
        //! - Set kinematic derivative
        posState->setDerivative(rBNDotLocal_N);
        if (this->useRotation==false) {
            rBNDDotLocal_N = matrixASCP.inverse()*(vecTransSCP);
            velocityState->setDerivative(rBNDDotLocal_N);
        }
	}
}

void updateEffectorMassProps(double integTime){}
void updateEffectorMassPropRates(double integTime){}
