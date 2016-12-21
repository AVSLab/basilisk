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


#include "spacecraftPlus.h"
#include "utilities/simMacros.h"
#include "../_GeneralModuleFiles/svIntegratorRK4.h"
#include "utilities/avsEigenSupport.h"
#include "utilities/avsEigenMRP.h"
#include <iostream>


SpacecraftPlus::SpacecraftPlus()
{
	this->currTimeStep = 0.0;
	this->timePrevious = 0.0;
	this->integrator = new svIntegratorRK4(this);
    this->sysTimePropertyName = "systemTime";
    this->simTimePrevious = 0;
	this->scStateOutMsgName = "inertial_state_output";
    this->scStateOutMsgId = -1;
    this->centralBodyInMsgName = "central_body_spice";
    this->centralBodyInMsgId = -1;
	this->numOutMsgBuffers = 2;
    this->dcm_BS.setIdentity();
    this->struct2BdyPropertyName = "dcm_BS";
    this->dvAccum_B.fill(0.0);
    return;
}


SpacecraftPlus::~SpacecraftPlus()
{
    return;
}

void SpacecraftPlus::SelfInit()
{
    this->scStateOutMsgId = SystemMessaging::GetInstance()->CreateNewMessage(this->scStateOutMsgName,
                                                                             sizeof(SCPlusOutputStateData), this->numOutMsgBuffers, "SCPlusOutputStateData", this->moduleID);
    this->gravField.SelfInit();
    return;
}

void SpacecraftPlus::CrossInit()
{
    this->gravField.CrossInit();
    this->initializeDynamics();
    return;
}

void SpacecraftPlus::UpdateState(uint64_t CurrentSimNanos)
{
    double newTime = CurrentSimNanos*NANO2SEC;
    this->gravField.UpdateState(CurrentSimNanos);
    this->integrateState(newTime);
    this->writeOutputMessages(CurrentSimNanos);
    this->simTimePrevious = CurrentSimNanos;
    return;
}

void SpacecraftPlus::linkInStates(DynParamManager& statesIn)
{
	this->hubR_N = statesIn.getStateObject("hubPosition");
	this->hubV_N = statesIn.getStateObject("hubVelocity");
    this->hubSigma = statesIn.getStateObject("hubSigma");   /* Need sigmaBN for MRP switching */
	this->hubOmega_BN_B = statesIn.getStateObject("hubOmega");
    this->inertialPositionProperty = statesIn.getPropertyReference("r_BN_N");
    this->inertialVelocityProperty = statesIn.getPropertyReference("v_BN_N");
    return;
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
    
    //! - Loop though the dynamicEffectors to link in the states needed
    for(dynIt = this->dynEffectors.begin(); dynIt != this->dynEffectors.end(); dynIt++)
    {
        (*dynIt)->linkInStates(this->dynManager);
    }
    return;
}

void SpacecraftPlus::equationsOfMotion(double t)
{
    std::vector<StateEffector*>::iterator it;
    std::vector<DynamicEffector*>::iterator dynIt;

    uint64_t CurrentSimNanos;
    CurrentSimNanos = this->simTimePrevious + (t-this->timePrevious)/NANO2SEC;
    (*this->sysTime) << CurrentSimNanos, t;

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

    //! Add in hubs mass to the spaceCraft mass props
    this->hub.updateEffectorMassProps(t);
    (*this->m_SC)(0,0) += this->hub.effProps.mEff;
    (*this->ISCPntB_B) += this->hub.effProps.IEffPntB_B;
    (*this->c_B) += this->hub.effProps.mEff*this->hub.effProps.rCB_B;

    //! - Loop through state effectors to get mass props
    for(it = this->states.begin(); it != this->states.end(); it++)
    {
        //! Add in effectors mass props into mass props of spacecraft
        (*it)->updateEffectorMassProps(t);
        (*this->m_SC)(0,0) += (*it)->effProps.mEff;
        (*this->ISCPntB_B) += (*it)->effProps.IEffPntB_B;
        (*this->c_B) += (*it)->effProps.mEff*(*it)->effProps.rCB_B;
        (*this->ISCPntBPrime_B) += (*it)->effProps.IEffPrimePntB_B;
        (*this->cPrime_B) += (*it)->effProps.mEff*(*it)->effProps.rPrimeCB_B;
    }

    //! Divide c_B and cPrime_B by the total mass of the spaceCraft
    (*this->c_B) = (*this->c_B)/(*this->m_SC)(0,0);
    (*this->cPrime_B) = (*this->cPrime_B)/(*this->m_SC)(0,0);

    //! - This is where gravity is computed
    this->gravField.computeGravityField();

    //! - Loop through state effectors to get contributions
    for(it = this->states.begin(); it != this->states.end(); it++)
    {
        //! - Set the matrices to zero
        this->matrixAContr.setZero();
        this->matrixBContr.setZero();
        this->matrixCContr.setZero();
        this->matrixDContr.setZero();
        this->vecTransContr.setZero();
        this->vecRotContr.setZero();

        //! Add contributions to matrices
        (*it)->updateContributions(t, this->matrixAContr, this->matrixBContr, this->matrixCContr, this->matrixDContr, this->vecTransContr, this->vecRotContr);
        this->hub.matrixA += this->matrixAContr;
        this->hub.matrixB += this->matrixBContr;
        this->hub.matrixC += this->matrixCContr;
        this->hub.matrixD += this->matrixDContr;
        this->hub.vecTrans += this->vecTransContr;
        this->hub.vecRot += this->vecRotContr;
    }

    //! - Loop through dynEffectors
    for(dynIt = this->dynEffectors.begin(); dynIt != this->dynEffectors.end(); dynIt++)
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
    return;
}

void SpacecraftPlus::integrateState(double t)
{
    Eigen::MRPd sigma_BN;
    Eigen::Matrix3d dcm_BN;
    Eigen::Vector3d g_N;
    Eigen::Vector3d initV_N;
    Eigen::Vector3d newV_N;
    Eigen::Vector3d dV_N;
	double localTimeStep = t - timePrevious;
    initV_N = this->hubV_N->getState();
	this->integrator->integrate(t, localTimeStep);
	this->timePrevious = t;
    

    //! Lets switch those MRPs!!
    Eigen::Vector3d sigmaBNLoc;
    sigmaBNLoc = (Eigen::Vector3d) this->hubSigma->getState();
    if (sigmaBNLoc.norm() > 1) {
        sigmaBNLoc = -sigmaBNLoc/(sigmaBNLoc.dot(sigmaBNLoc));
        this->hubSigma->setState(sigmaBNLoc);
    }
    
    // = (Eigen::Vector3d) this->hubSigma->getState();
    sigma_BN = sigmaBNLoc;
    dcm_BN = sigma_BN.toRotationMatrix();
    dcm_BN.transposeInPlace();
    g_N = *(this->hub.g_N);
    newV_N = this->hubV_N->getState();
    dV_N = newV_N - initV_N;
    dV_N -= g_N*localTimeStep;
    this->dvAccum_B += dcm_BN*dV_N;
    

    //! - Compute Energy and Momentum
    this->computeEnergyMomentum(t);
    return;
}

void SpacecraftPlus::computeEnergyMomentum(double t)
{
    // - Define variables needed for calculations
    Eigen::Vector3d rBNLocal_N;
    Eigen::Vector3d rDotBNLocal_N;
    Eigen::Vector3d rBNLocal_B;
    Eigen::Vector3d rDotBNLocal_B;
    Eigen::Vector3d cLocal_B;
    Eigen::Vector3d cPrimeLocal_B;
    Eigen::Vector3d cDotLocal_B;
    Eigen::MRPd sigmaLocal_BN;
    Eigen::Vector3d omegaLocal_BN_B;
    Eigen::Matrix3d dcmNBLocal;
    Eigen::Matrix3d dcmBNLocal;
    Eigen::Vector3d totOrbAngMomPntN_B;
    Eigen::Vector3d totRotAngMomPntC_B;

    double mSCLocal = 0.0;

    // - Grab values from state Manager
    rBNLocal_N = hubR_N->getState();
    rDotBNLocal_N = hubV_N->getState();
    sigmaLocal_BN = (Eigen::Vector3d ) hubSigma->getState();
    omegaLocal_BN_B = hubOmega_BN_B->getState();

    // - Find DCM's
    dcmNBLocal = sigmaLocal_BN.toRotationMatrix();
    dcmBNLocal = dcmNBLocal.transpose();

    // - Convert from inertial frame to body frame
    rBNLocal_B = dcmBNLocal*rBNLocal_N;
    rDotBNLocal_B = dcmBNLocal*rDotBNLocal_N;

    // - zero necessarry variables
    this->totOrbKinEnergy = 0.0;
    this->totRotEnergy = 0.0;
    this->totOrbAngMomPntN_N.setZero();
    this->totRotAngMomPntC_N.setZero();
    totOrbAngMomPntN_B.setZero();
    totRotAngMomPntC_B.setZero();
    this->rotAngMomPntCContr_B.setZero();
    this->rotEnergyContr = 0.0;
    cLocal_B.setZero();
    cPrimeLocal_B.setZero();
    cDotLocal_B.setZero();

    // - Get the hubs contribution
    this->hub.updateEnergyMomContributions(t, this->rotAngMomPntCContr_B, this->rotEnergyContr);
    mSCLocal += this->hub.effProps.mEff;
    cLocal_B += this->hub.effProps.mEff*this->hub.effProps.rCB_B;
    totRotAngMomPntC_B += this->rotAngMomPntCContr_B;
    this->totRotEnergy += this->rotEnergyContr;

    // - Loop over stateEffectors to get their contributions to energy and momentum
    std::vector<StateEffector*>::iterator it;
    for(it = this->states.begin(); it != this->states.end(); it++)
    {
        // - Set the matrices to zero
        this->rotAngMomPntCContr_B.setZero();
        this->rotEnergyContr = 0.0;

        // - Add in effectors mass props into mass props of spacecraft
        (*it)->updateEffectorMassProps(t);
        mSCLocal += (*it)->effProps.mEff;
        cLocal_B += (*it)->effProps.mEff*(*it)->effProps.rCB_B;
        cPrimeLocal_B += (*it)->effProps.mEff*(*it)->effProps.rPrimeCB_B;

        // - Call energy and momentum calulations for stateEffectors
        (*it)->updateEnergyMomContributions(t, this->rotAngMomPntCContr_B, this->rotEnergyContr);
        totRotAngMomPntC_B += this->rotAngMomPntCContr_B;
        this->totRotEnergy += this->rotEnergyContr;
    }

    // - correct c_B and cPrime_B by 1/mSC
    cLocal_B = cLocal_B/mSCLocal;
    cPrimeLocal_B = cPrimeLocal_B/mSCLocal;

    // - Find cDot_B
    cDotLocal_B = cPrimeLocal_B + omegaLocal_BN_B.cross(cLocal_B);

    // - Find orbital kinetic energy for the spacecraft
    this->totOrbKinEnergy += 1.0/2.0*mSCLocal*(rDotBNLocal_B.dot(rDotBNLocal_B) + 2.0*rDotBNLocal_B.dot(cDotLocal_B) + cDotLocal_B.dot(cDotLocal_B));

    // - Find total rotational energy
    this->totRotEnergy += -1.0/2.0*mSCLocal*cDotLocal_B.dot(cDotLocal_B);

    // - Find orbital angular momentum for the spacecraft
    Eigen::Vector3d rCN_N;
    Eigen::Vector3d rDotCN_N;
    rCN_N = rBNLocal_N + dcmNBLocal*cLocal_B;
    rDotCN_N = rDotBNLocal_N + dcmNBLocal*cDotLocal_B;
    this->totOrbAngMomPntN_N = mSCLocal*(rCN_N.cross(rDotCN_N));

    // - Find rotational angular momentum for the spacecraft
    totRotAngMomPntC_B += -mSCLocal*cLocal_B.cross(cDotLocal_B);
    this->totRotAngMomPntC_N = dcmNBLocal*totRotAngMomPntC_B;
    return;
}

void SpacecraftPlus::writeOutputMessages(uint64_t clockTime)
{
	SCPlusOutputStateData stateOut;

    eigenMatrixXd2CArray(*this->inertialPositionProperty, stateOut.r_BN_N);
    eigenMatrixXd2CArray(*this->inertialVelocityProperty, stateOut.v_BN_N);
    eigenMatrixXd2CArray(this->hubSigma->getState(), stateOut.sigma_BN);
    eigenMatrixXd2CArray(this->hubOmega_BN_B->getState(), stateOut.omega_BN_B);
    eigenMatrix3d2CArray(this->dcm_BS, (double *)stateOut.dcm_BS);
    eigenMatrixXd2CArray(this->dvAccum_B, stateOut.TotalAccumDVBdy);

	SystemMessaging::GetInstance()->WriteMessage(this->scStateOutMsgId, clockTime, sizeof(SCPlusOutputStateData),
		reinterpret_cast<uint8_t*> (&stateOut), this->moduleID);
    return;
}
