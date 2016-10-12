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

void HubEffector::linkInStates(DynParamManager& DynParamManager)
{
    this->posState = DynParamManager.getStateObject("hubPosition");
    this->velocityState = DynParamManager.getStateObject("hubVelocity");
    this->sigmaState = DynParamManager.getStateObject("hubSigma");
    this->omegaState = DynParamManager.getStateObject("hubOmega");
    
}

void HubEffector::registerStates(DynParamManager& DynParamManager)
{
    DynParamManager.registerState(3, 1, "hubPosition");
    DynParamManager.registerState(3, 1, "hubVelocity");
    DynParamManager.registerState(3, 1, "hubSigma");
    DynParamManager.registerState(3, 1, "hubOmega");
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
