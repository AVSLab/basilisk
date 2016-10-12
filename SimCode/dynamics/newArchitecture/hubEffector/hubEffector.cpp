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

HubEffector::HubEffector()
{
    return;
}


HubEffector::~HubEffector()
{
    return;
}

void HubEffector::dynamicsSelfInit()
{

}

void HubEffector::dynamicsCrossInit()
{

}

void HubEffector::linkInStates(DynParamManager& statesIn)
{
    this->posState = statesIn.getStateObject("hubPosition");
    this->velocityState = statesIn.getStateObject("hubVelocity");
    this->sigmaState = statesIn.getStateObject("hubSigma");
    this->omegaState = statesIn.getStateObject("hubOmega");
    this->m_SC = statesIn.getPropertyReference("m_SC");
    this->c_B = statesIn.getPropertyReference("centerOfMassSC");
    this->ISCPntB_B = statesIn.getPropertyReference("inertiaSC");
    this->cPrime_B = statesIn.getPropertyReference("centerOfMassPrimeSC");
    this->ISCPntBPrime_B = statesIn.getPropertyReference("inertiaPrimeSC");
}

void HubEffector::registerStates(DynParamManager& states)
{
    states.registerState(3, 1, "hubPosition");
    states.registerState(3, 1, "hubVelocity");
    states.registerState(3, 1, "hubSigma");
    states.registerState(3, 1, "hubOmega");
}

//void updateContributions(double integTime)
//{
//}

void HubEffector::computeDerivatives(double integTime, Eigen::Matrix3d matrixA, Eigen::Matrix3d matrixB, Eigen::Matrix3d matrixC, Eigen::Matrix3d matrixD, Eigen::Vector3d vecTrans, Eigen::Vector3d vecRot)
{
    Eigen::Vector3d omegaBNDot_B;
    Eigen::Vector3d rBNDDot_B;
    Eigen::Matrix3d intermediateMatrix;
    Eigen::Vector3d intermediateVector;
    Eigen::MatrixXd omegaLocal;
    Eigen::Vector3d cPrimeLocal_B;
    Eigen::Vector3d cLocal_B;
    omegaLocal(3,1);
    omegaLocal = omegaState->getState();
    cPrimeLocal_B << (*cPrime_B)(0,0), (*cPrime_B)(1,0), (*cPrime_B)(2,0);
    cLocal_B << (*c_B)(0,0), (*c_B)(1,0), (*c_B)(2,0);
    //! - Need to add hub to m_SC, c_B and ISCPntB_B
    *this->m_SC += this->mHub;
    *this->c_B += this->mHub*this->rBcB_B;
    *ISCPntB_B += this->IHubPntB_B;

    //! - Need to scale [A] [B] and vecTrans by m_SC
    matrixA = matrixA/this->m_SC->value();
    matrixA = matrixB/this->m_SC->value();
    vecTrans = vecTrans/this->m_SC->value();

    //! Need to make contributions to the matrices from the hub
    intermediateMatrix = intermediateMatrix.Identity();
    matrixA += intermediateMatrix;
    //! make c_B skew symmetric matrix
    intermediateMatrix <<  0 , -(*c_B)(2,0),
    (*c_B)(1,0), (*c_B)(2,0), 0, -(*c_B)(0,0), -(*c_B)(1,0), (*c_B)(0,0), 0;
    matrixB -= intermediateMatrix;
    matrixC += this->m_SC->value()*intermediateMatrix;
    matrixD += *ISCPntB_B;
    vecTrans += -2*omegaLocal.cross3(cPrimeLocal_B) -omegaLocal.cross3(omegaLocal.cross3(cLocal_B));
    intermediateVector = *ISCPntB_B*omegaLocal;
    vecRot += -omegaLocal.cross3(intermediateVector) - *ISCPntBPrime_B*omegaLocal;

    if (this->useRotation) {
        if (useTranslation) {
            intermediateVector = vecRot - matrixC*matrixA.inverse()*vecTrans;
            intermediateMatrix = matrixD - matrixC*matrixA.inverse()*matrixB;
            omegaBNDot_B = intermediateMatrix.inverse()*intermediateVector;
            rBNDDot_B = matrixA.inverse()*(vecTrans - matrixB*omegaBNDot_B);
        }
        omegaBNDot_B = matrixD.inverse()*vecRot;
    }
}

void updateEffectorMassProps(double integTime){}
void updateEffectorMassPropRates(double integTime){}
