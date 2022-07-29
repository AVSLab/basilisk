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

#include "fuelTank.h"
#include <iostream>

/*Able to be accesses from python, used to set up fuel tank model*/
FuelTankModelConstantVolume_t FuelTankModelConstantVolume;
FuelTankModelConstantDensity_t FuelTankModelConstantDensity;
FuelTankModelEmptying_t FuelTankModelEmptying;
FuelTankModelUniformBurn_t FuelTankModelUniformBurn;
FuelTankModelCentrifugalBurn_t FuelTankModelCentrifugalBurn;

FuelTankModel* FuelTankModels[TANK_MODEL_LAST_MODEL - TANK_MODEL_FIRST_MODEL] = {
	&FuelTankModelConstantVolume,
	&FuelTankModelConstantDensity,
	&FuelTankModelEmptying,
	&FuelTankModelUniformBurn,
	&FuelTankModelCentrifugalBurn,
};

/*! This is the constructor, setting variables to default values */
FuelTank::FuelTank()
{
    this->updateOnly = true;
	// - zero the contributions for mass props and mass rates'
	this->effProps.mEff = 0.0;
	this->effProps.IEffPntB_B.setZero();
	this->effProps.rEff_CB_B.setZero();
	this->effProps.rEffPrime_CB_B.setZero();
	this->effProps.IEffPrimePntB_B.setZero();
	this->dcm_TB = dcm_TB.Identity();
	this->r_TB_B.setZero();

	// - Initialize the variables to working values
	this->nameOfMassState = "fuelTankMass" + std::to_string(this->effectorID);
    this->effectorID++;
    
    return;
}

uint64_t FuelTank::effectorID = 1;

/*! This is the destructor, nothing to report here */
FuelTank::~FuelTank()
{
    this->effectorID = 1;    /* reset the panel ID*/
    return;
}


/*! set fuel tank model
 @return void
 @param model fuel tank model type
 */
void FuelTank::setTankModel(FuelTankModelTypes model){
	fuelTankModel = FuelTankModels[model];

    return;
}

/*! This is a method to attach a fuel slosh particle to the tank */
void FuelTank::pushFuelSloshParticle(FuelSlosh *particle)
{
    // - Add a fuel slosh particle to the vector of fuel slosh particles
	this->fuelSloshParticles.push_back(particle);

    return;
}

/*! Method for fuel tank to access the states that it needs, needs omega */
void FuelTank::linkInStates(DynParamManager& statesIn)
{
    // - Grab access to the hubs omega_BN_N
	this->omegaState = statesIn.getStateObject("hubOmega");

    return;
}

/*! Method for fuel tank to register states. The fuel tank has one state associated with it: mass, and it also has the
 responsibility to call register states for the fuel slosh particles */
void FuelTank::registerStates(DynParamManager& statesIn)
{
    // - Register the mass state associated with the tank
    Eigen::MatrixXd massMatrix(1,1);
	this->massState = statesIn.registerState(1, 1, this->nameOfMassState);
    massMatrix(0,0) = this->fuelTankModel->propMassInit;
    this->massState->setState(massMatrix);

    return;
}

/*! This method gives the fuel tank the ability to add its contributions the mass of the vehicle. */
void FuelTank::updateEffectorMassProps(double integTime)
{
    // - Add contributions of the mass of the tank
	double massLocal = this->massState->getState()(0, 0);
	this->fuelTankModel->computeTankProps(massLocal);
	this->r_TcB_B = r_TB_B +this->dcm_TB.transpose()*this->fuelTankModel->r_TcT_T;
	this->effProps.mEff = massLocal;
	this->ITankPntT_B = this->dcm_TB.transpose()*fuelTankModel->ITankPntT_T*this->dcm_TB;
	this->effProps.IEffPntB_B = ITankPntT_B+massLocal * (r_TcB_B.dot(r_TcB_B)*Eigen::Matrix3d::Identity()
                                                                                         - r_TcB_B * r_TcB_B.transpose());
	this->effProps.rEff_CB_B = this->r_TcB_B;

    // - This does not incorportate mEffDot into cPrime for high fidelity mass depletion
	this->effProps.rEffPrime_CB_B = Eigen::Vector3d::Zero();

    //! - Mass depletion (call thrusters attached to this tank to get their mDot, and contributions)
    this->fuelConsumption = 0.0;
    std::vector<DynamicEffector*>::iterator dynIt;
    for (dynIt = this->dynEffectors.begin(); dynIt != this->dynEffectors.end(); dynIt++)
    {
        (*dynIt)->computeStateContribution(integTime);
        this->fuelConsumption += (*dynIt)->stateDerivContribution(0);
    }
    std::vector<StateEffector*>::iterator stateIt;
    for (stateIt = this->stateEffectors.begin(); stateIt != this->stateEffectors.end(); stateIt++)
    {
        (*stateIt)->updateEffectorMassProps(integTime);
        this->fuelConsumption += (*stateIt)->stateDerivContribution(0);
    }

    std::vector<FuelSlosh*>::iterator fuelSloshInt;
    // - Mass depletion (finding total mass in tank)
    double totalMass = massLocal;
    for (fuelSloshInt = this->fuelSloshParticles.begin(); fuelSloshInt < this->fuelSloshParticles.end(); fuelSloshInt++) {
        // - Retrieve current mass value of fuelSlosh particle
        (*fuelSloshInt)->retrieveMassValue(integTime);
        // - Add fuelSlosh mass to total mass of tank
        totalMass += (*fuelSloshInt)->fuelMass;
    }
    // - Set mass depletion rate of fuelSloshParticles
    for (fuelSloshInt = this->fuelSloshParticles.begin(); fuelSloshInt < this->fuelSloshParticles.end(); fuelSloshInt++) {
        // - Find fuelSlosh particle mass to fuel tank mass ratio
        (*fuelSloshInt)->massToTotalTankMassRatio = (*fuelSloshInt)->fuelMass/totalMass;
        // - Scale total fuelConsumption by mass ratio to find fuelSloshParticle mass depletion rate
        (*fuelSloshInt)->fuelMassDot = (*fuelSloshInt)->massToTotalTankMassRatio*(-this->fuelConsumption);
    }

    // - Set fuel consumption rate of fuelTank (not negative because the negative sign is in the computeDerivatives call
    this->tankFuelConsumption = massLocal/totalMass*(this->fuelConsumption);

    this->effProps.mEffDot = -this->fuelConsumption;

    return;
}

/*! This method allows the fuel tank to add its contributions to the matrices for the back-sub method. */
void FuelTank::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{

	Eigen::Vector3d r_TB_BLocal, rPrime_TB_BLocal, rPPrime_TB_BLocal;
	Eigen::Vector3d omega_BN_BLocal;


    // - Zero some matrices
    backSubContr.matrixA = backSubContr.matrixB = backSubContr.matrixC = backSubContr.matrixD = Eigen::Matrix3d::Zero();
    backSubContr.vecTrans = backSubContr.vecRot = Eigen::Vector3d::Zero();

    // Calculate the fuel consumption properties for the tank
	tankFuelConsumption = fuelConsumption*massState->getState()(0, 0) / effProps.mEff;
	fuelTankModel->computeTankPropDerivs(massState->getState()(0, 0), -tankFuelConsumption);
	r_TB_BLocal = fuelTankModel->r_TcT_T;
	rPrime_TB_BLocal = fuelTankModel->rPrime_TcT_T;
	rPPrime_TB_BLocal = fuelTankModel->rPPrime_TcT_T;
	omega_BN_BLocal = omegaState->getState();
	if (!this->updateOnly) {
		backSubContr.vecRot = -massState->getState()(0, 0) * r_TB_BLocal.cross(rPPrime_TB_BLocal)
			- massState->getState()(0, 0)*omega_BN_BLocal.cross(r_TB_BLocal.cross(rPrime_TB_BLocal))
			- massState->getStateDeriv()(0, 0)*r_TB_BLocal.cross(rPrime_TB_BLocal);
		backSubContr.vecRot -= fuelTankModel->IPrimeTankPntT_T * omega_BN_BLocal;
	}

    return;
}

/*! This method allows the fuel tank to compute its derivative */
void FuelTank::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{
	// - Call compute derivatives
	Eigen::MatrixXd conv(1, 1);
	conv(0, 0) = -this->tankFuelConsumption;
	this->massState->setDerivative(conv);
    return;
}

/*! This method allows the fuel tank to contribute to the energy and momentum calculations */
void FuelTank::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                            double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{
    // - Get variables needed for energy momentum calcs
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = omegaState->getState();
    Eigen::Vector3d rDot_TcB_B;

    // - Find rotational angular momentum contribution from hub
    double massLocal = this->massState->getState()(0, 0);
    rDot_TcB_B = omegaLocal_BN_B.cross(r_TcB_B);
    rotAngMomPntCContr_B += ITankPntT_B*omegaLocal_BN_B + massLocal*r_TcB_B.cross(rDot_TcB_B);

    // - Find rotational energy contribution from the hub
    rotEnergyContr += 1.0/2.0*omegaLocal_BN_B.dot(ITankPntT_B*omegaLocal_BN_B) + 1.0/2.0*massLocal*
                                                                             rDot_TcB_B.dot(rDot_TcB_B);

	 return;
}

/*! This method takes the computed fuel tank mass properties and outputs them to the messaging system.
 @return void
 @param CurrentClock The current simulation time (used for time stamping)
 */
void FuelTank::WriteOutputMessages(uint64_t CurrentClock)
{
    this->fuelTankMassPropMsg = this->fuelTankOutMsg.zeroMsgPayload;
    this->fuelTankMassPropMsg.fuelMass = this->effProps.mEff;
    this->fuelTankMassPropMsg.fuelMassDot = this->effProps.mEffDot;
    this->fuelTankMassPropMsg.maxFuelMass = this->fuelTankModel->maxFuelMass;
    this->fuelTankOutMsg.write(&this->fuelTankMassPropMsg, this->moduleID, CurrentClock);
}

/*! This method allows the fuel tank to write out its messages to the messaging system
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void FuelTank::UpdateState(uint64_t CurrentSimNanos)
{
    // Writing the fuel tank mass property message out to the messaging system
    WriteOutputMessages(CurrentSimNanos);

    return;
}
