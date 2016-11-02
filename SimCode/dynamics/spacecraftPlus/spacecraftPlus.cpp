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


#include "spacecraftPlus.h"
#include "utilities/simMacros.h"
#include "../_GeneralModuleFiles/rk4SVIntegrator.h"

SpacecraftPlus::SpacecraftPlus()
{
	currTimeStep = 0.0;
	timePrevious = 0.0;
	integrator = new rk4SVIntegrator(this);
    sysTimePropertyName = "systemTime";
    simTimePrevious = 0;
    return;
}


SpacecraftPlus::~SpacecraftPlus()
{
    return;
}

void SpacecraftPlus::computeEnergyMomentum()
{
    
}

void SpacecraftPlus::linkInStates(DynParamManager& statesIn)
{
    hubSigma = statesIn.getStateObject("hubSigma");
}

void SpacecraftPlus::equationsOfMotion(double t)
{
    std::vector<StateEffector*>::iterator it;
    std::vector<DynamicEffector*>::iterator dynIt;

    uint64_t CurrentSimNanos;
    CurrentSimNanos = simTimePrevious + (t-timePrevious)/NANO2SEC;
    (*sysTime) << CurrentSimNanos, t;

    //! - Zero all Matrices and vectors
    this->matrixAContr.setZero();
    this->matrixBContr.setZero();
    this->matrixCContr.setZero();
    this->matrixDContr.setZero();
    this->vecTransContr.setZero();
    this->vecRotContr.setZero();
    hub.matrixA.setZero();
    hub.matrixB.setZero();
    hub.matrixC.setZero();
    hub.matrixD.setZero();
    hub.vecTrans.setZero();
    hub.vecRot.setZero();
    (*this->m_SC).setZero();
    (*this->c_B).setZero();
    (*this->ISCPntB_B).setZero();
    (*this->cPrime_B).setZero();
    (*this->ISCPntBPrime_B).setZero();


    //! - This is where gravity will be called
    gravField.computeGravityField();

    //! Add in hubs mass to the spaceCraft mass props
    hub.updateEffectorMassProps(t);
    (*this->m_SC)(0,0) += hub.effProps.mEff;
    (*this->ISCPntB_B) += hub.effProps.IEffPntB_B;
    (*this->c_B) += hub.effProps.mEff*hub.effProps.rCB_B;

    //! - Loop through state effectors
    for(it = states.begin(); it != states.end(); it++)
    {
        //! Add in effectors mass props into mass props of spacecraft
        (*it)->updateEffectorMassProps(t);
        (*this->m_SC)(0,0) += (*it)->effProps.mEff;
        (*this->ISCPntB_B) += (*it)->effProps.IEffPntB_B;
        (*this->c_B) += (*it)->effProps.mEff*(*it)->effProps.rCB_B;

        //! Add in effectors mass prop rates into mass prop rates of spacecraft
        (*it)->updateEffectorMassPropRates(t);
        (*this->ISCPntBPrime_B) += (*it)->effProps.IEffPrimePntB_B;
        (*this->cPrime_B) += (*it)->effProps.rPrimeCB_B;

        //! Add contributions to matrices
        (*it)->updateContributions(t, matrixAContr, matrixBContr, matrixCContr, matrixDContr, vecTransContr, vecRotContr);
        hub.matrixA += matrixAContr;
        hub.matrixB += matrixBContr;
        hub.matrixC += matrixCContr;
        hub.matrixD += matrixDContr;
        hub.vecTrans += vecTransContr;
        hub.vecRot += vecRotContr;
    }

    //! Divide c_B and cPrime_B by the total mass of the spaceCraft
    (*this->c_B) = (*this->c_B)/(*this->m_SC)(0,0);
    (*this->cPrime_B) = (*this->cPrime_B)/(*this->m_SC)(0,0);

    //! - Loop through dynEffectors
    for(dynIt = dynEffectors.begin(); dynIt != dynEffectors.end(); dynIt++)
    {
        //! Empty for now
    }

    //! - Compute the derivatives of the hub states before looping through stateEffectors
    hub.computeDerivatives(t);

    //! - Loop through state effectors for compute derivatives
    for(it = states.begin(); it != states.end(); it++)
    {
        //! These matrices should be NULL, because the stateEffectors don't need to know about these Matrices
        (*it)->computeDerivatives(t);
    }

}
void SpacecraftPlus::integrateState(double t)
{
	double currTimeStep = t - timePrevious;
	integrator->integrate(t, currTimeStep);

    //! Lets switch those MRPs!!
    Eigen::Vector3d sigmaBNLoc;
    sigmaBNLoc = hubSigma->getState();
    if (sigmaBNLoc.norm() > 1) {
        sigmaBNLoc = -sigmaBNLoc/(sigmaBNLoc.dot(sigmaBNLoc));
        hubSigma->setState(sigmaBNLoc);
    }

	timePrevious = t;
}

void SpacecraftPlus::initializeDynamics()
{
    Eigen::MatrixXd m_SC(1,1);
    Eigen::MatrixXd c_B(3,1);
    Eigen::MatrixXd ISCPntB_B(3,3);
    Eigen::MatrixXd cPrime_B(3,1);
    Eigen::MatrixXd ISCPntBPrime_B(3,3);
    Eigen::MatrixXd systemTime(2,1);
    systemTime.setZero();
    this->m_SC = dynManager.createProperty("m_SC", m_SC);
    this->c_B = dynManager.createProperty("centerOfMassSC", c_B);
    this->ISCPntB_B = dynManager.createProperty("inertiaSC", ISCPntB_B);
    this->ISCPntBPrime_B = dynManager.createProperty("inertiaPrimeSC", ISCPntBPrime_B);
    this->cPrime_B = dynManager.createProperty("centerOfMassPrimeSC", cPrime_B);
    this->sysTime = dynManager.createProperty(sysTimePropertyName, systemTime);
    std::vector<StateEffector*>::iterator it;
    std::vector<DynamicEffector*>::iterator dynIt;
    
    gravField.registerProperties(dynManager);
    hub.registerStates(dynManager);

    for(it = states.begin(); it != states.end(); it++)
    {
        (*it)->registerStates(dynManager);
    }

    this->linkInStates(dynManager);
    gravField.linkInStates(dynManager);
    hub.linkInStates(dynManager);

    for(it = states.begin(); it != states.end(); it++)
    {
        (*it)->linkInStates(dynManager);
    }
    
    for(dynIt = dynEffectors.begin(); dynIt != dynEffectors.end(); dynIt++)
    {
        (*dynIt)->linkInStates(dynManager);
    }

    hub.useTranslation = true;
    hub.useRotation = true;
}

void SpacecraftPlus::SelfInit()
{
    gravField.SelfInit();
}
void SpacecraftPlus::CrossInit()
{
    gravField.CrossInit();
	initializeDynamics();
}
void SpacecraftPlus::UpdateState(uint64_t CurrentSimNanos)
{
	double newTime = CurrentSimNanos*NANO2SEC;
    gravField.UpdateState(CurrentSimNanos);
	integrateState(newTime);
    simTimePrevious = CurrentSimNanos;
}
