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
#include "../utilities/avsEigenSupport.h"
#include <iostream>


SpacecraftPlus::SpacecraftPlus()
{
	currTimeStep = 0.0;
	timePrevious = 0.0;
	integrator = new rk4SVIntegrator(this);
    sysTimePropertyName = "systemTime";
    simTimePrevious = 0;
	scStateOutMsgName = "inertial_state_output";
	numOutMsgBuffers = 2;
    dcm_BS.setIdentity();
    struct2BdyPropertyName = "dcm_BS";
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
	this->hubR_N = statesIn.getStateObject("hubPosition");
	this->hubV_N = statesIn.getStateObject("hubVelocity");
    this->hubSigma = statesIn.getStateObject("hubSigma");   /* Need sigmaBN for MRP switching */
	this->hubOmega_BN_B = statesIn.getStateObject("hubOmega");
}

void SpacecraftPlus::equationsOfMotion(double t)
{
    std::vector<StateEffector*>::iterator it;
    std::vector<DynamicEffector*>::iterator dynIt;

    uint64_t CurrentSimNanos;
    CurrentSimNanos = simTimePrevious + (t-timePrevious)/NANO2SEC;
    (*sysTime) << CurrentSimNanos, t;

    //! - Zero all Matrices and vectors
    this->hub.matrixA.setZero();
    this->hub.matrixB.setZero();
    this->hub.matrixC.setZero();
    this->hub.matrixD.setZero();
    this->hub.vecTrans.setZero();
    this->hub.vecRot.setZero();
    this->hub.sumForceExternal_B.setZero();
    this->hub.sumForceExternal_N.setZero();
    this->hub.sumTorquePntB_B.setZero();
    (*this->m_SC).setZero();
    (*this->c_B).setZero();
    (*this->ISCPntB_B).setZero();
    (*this->cPrime_B).setZero();
    (*this->ISCPntBPrime_B).setZero();


    //! - This is where gravity is computed
    this->gravField.computeGravityField();

    //! Add in hubs mass to the spaceCraft mass props
    this->hub.updateEffectorMassProps(t);
    (*this->m_SC)(0,0) += hub.effProps.mEff;
    (*this->ISCPntB_B) += hub.effProps.IEffPntB_B;
    (*this->c_B) += hub.effProps.mEff*hub.effProps.rCB_B;

    //! - Loop through state effectors
    for(it = this->states.begin(); it != states.end(); it++)
    {
        //! - Set the matrices to zero
        this->matrixAContr.setZero();
        this->matrixBContr.setZero();
        this->matrixCContr.setZero();
        this->matrixDContr.setZero();
        this->vecTransContr.setZero();
        this->vecRotContr.setZero();

        //! Add in effectors mass props into mass props of spacecraft
        (*it)->updateEffectorMassProps(t);
        (*this->m_SC)(0,0) += (*it)->effProps.mEff;
        (*this->ISCPntB_B) += (*it)->effProps.IEffPntB_B;
        (*this->c_B) += (*it)->effProps.mEff*(*it)->effProps.rCB_B;
        (*this->ISCPntBPrime_B) += (*it)->effProps.IEffPrimePntB_B;
        (*this->cPrime_B) += (*it)->effProps.mEff*(*it)->effProps.rPrimeCB_B;

        //! Add contributions to matrices
        (*it)->updateContributions(t, matrixAContr, matrixBContr, matrixCContr, matrixDContr, vecTransContr, vecRotContr);
        this->hub.matrixA += matrixAContr;
        this->hub.matrixB += matrixBContr;
        this->hub.matrixC += matrixCContr;
        this->hub.matrixD += matrixDContr;
        this->hub.vecTrans += vecTransContr;
        this->hub.vecRot += vecRotContr;
    }

    //! Divide c_B and cPrime_B by the total mass of the spaceCraft
    (*this->c_B) = (*this->c_B)/(*this->m_SC)(0,0);
    (*this->cPrime_B) = (*this->cPrime_B)/(*this->m_SC)(0,0);

    //! - Loop through dynEffectors
    for(dynIt = dynEffectors.begin(); dynIt != dynEffectors.end(); dynIt++)
    {
        //! - Compute the force and torque contributions from the dynamicEffectors
        (*dynIt)->computeBodyForceTorque(t);
        this->hub.sumForceExternal_N += (*dynIt)->forceExternal_N;
        this->hub.sumForceExternal_B += (*dynIt)->forceExternal_B;
        this->hub.sumTorquePntB_B += (*dynIt)->torqueExternalPntB_B;
    }

    //! - Compute the derivatives of the hub states before looping through stateEffectors
    this->hub.computeDerivatives(t);

    //! - Loop through state effectors for compute derivatives
    for(it = states.begin(); it != states.end(); it++)
    {
        (*it)->computeDerivatives(t);
    }

}
void SpacecraftPlus::integrateState(double t)
{
	double currTimeStep = t - timePrevious;
	this->integrator->integrate(t, currTimeStep);
	this->timePrevious = t;

    //! Lets switch those MRPs!!
    Eigen::Vector3d sigmaBNLoc;
    sigmaBNLoc = this->hubSigma->getState();
    if (sigmaBNLoc.norm() > 1) {
        sigmaBNLoc = -sigmaBNLoc/(sigmaBNLoc.dot(sigmaBNLoc));
        this->hubSigma->setState(sigmaBNLoc);
    }

    //! - Compute Energy and Momentum
    this->computeEnergyMomentum();

}

void SpacecraftPlus::initializeDynamics()
{
    //! SpaceCraftPlus initiates all of the spaceCraft mass properties
    Eigen::MatrixXd initM_SC(1,1);
    Eigen::MatrixXd initC_B(3,1);
    Eigen::MatrixXd initISCPntB_B(3,3);
    Eigen::MatrixXd initCPrime_B(3,1);
    Eigen::MatrixXd initISCPntBPrime_B(3,3);
    Eigen::MatrixXd systemTime(2,1);
    systemTime.setZero();
    //! - Create the properties
    this->m_SC = dynManager.createProperty("m_SC", initM_SC);
    this->c_B = dynManager.createProperty("centerOfMassSC", initC_B);
    this->ISCPntB_B = dynManager.createProperty("inertiaSC", initISCPntB_B);
    this->ISCPntBPrime_B = dynManager.createProperty("inertiaPrimeSC", initISCPntBPrime_B);
    this->cPrime_B = dynManager.createProperty("centerOfMassPrimeSC", initCPrime_B);
    this->property_dcm_BS = dynManager.createProperty(this->struct2BdyPropertyName, this->dcm_BS);
    this->sysTime = dynManager.createProperty(sysTimePropertyName, systemTime);

    //! - Register the gravity properties with the dynManager, 'erbody wants g_N!
    this->gravField.registerProperties(dynManager);

    //! - Register the hub states
    this->hub.registerStates(dynManager);

    //! - Loop through stateEffectors to register their states
    std::vector<StateEffector*>::iterator it;
    for(it = this->states.begin(); it != this->states.end(); it++)
    {
        (*it)->registerStates(dynManager);
    }

    //! - Link in states for the spaceCraftPlus to switch some MRPs
    this->linkInStates(dynManager);

    //! - Link in states for gravity and the hub
    this->gravField.linkInStates(dynManager);
    this->hub.linkInStates(dynManager);

    //! - Loop through the dynamicEffectros to link in the states needed
    std::vector<DynamicEffector*>::iterator dynIt;
    for(it = this->states.begin(); it != this->states.end(); it++)
    {
        (*it)->linkInStates(dynManager);
    }

    //! - Loop though the stateEffectors to link in the states needed
    for(dynIt = this->dynEffectors.begin(); dynIt != this->dynEffectors.end(); dynIt++)
    {
        (*dynIt)->linkInStates(this->dynManager);
    }
}

void SpacecraftPlus::writeOutputMessages(uint64_t clockTime)
{
	SCPlusOutputStateData stateOut;

    eigenMatrixXd2CArray(hubR_N->getState(), stateOut.r_BN_N);
    eigenMatrixXd2CArray(hubV_N->getState(), stateOut.v_BN_N);
    eigenMatrixXd2CArray(hubSigma->getState(), stateOut.sigma_BN);
    eigenMatrixXd2CArray(hubOmega_BN_B->getState(), stateOut.omega_BN_B);
    eigenMatrix3d2CArray(this->dcm_BS, (double *)stateOut.dcm_BS);

	SystemMessaging::GetInstance()->WriteMessage(scStateOutMsgID, clockTime, sizeof(SCPlusOutputStateData),
		reinterpret_cast<uint8_t*> (&stateOut), moduleID);
}

void SpacecraftPlus::SelfInit()
{
	scStateOutMsgID = SystemMessaging::GetInstance()->CreateNewMessage(scStateOutMsgName,
		sizeof(SCPlusOutputStateData), numOutMsgBuffers, "SCPlusOutputStateData", moduleID);
    this->gravField.SelfInit();
}
void SpacecraftPlus::CrossInit()
{
    this->gravField.CrossInit();
	this->initializeDynamics();
}
void SpacecraftPlus::UpdateState(uint64_t CurrentSimNanos)
{
	double newTime = CurrentSimNanos*NANO2SEC;
    this->gravField.UpdateState(CurrentSimNanos);
	this->integrateState(newTime);
	writeOutputMessages(CurrentSimNanos);
    this->simTimePrevious = CurrentSimNanos;
}
