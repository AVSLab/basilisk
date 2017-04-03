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

#include "fuelSloshParticle.h"
#include "utilities/avsEigenSupport.h"

/*! This is the constructor, setting variables to default values */
FuelSloshParticle::FuelSloshParticle()
{
	// - zero the contributions for mass props and mass rates
	this->effProps.mEff = 0.0;
	this->effProps.IEffPntB_B.setZero();
	this->effProps.rEff_CB_B.setZero();
	this->effProps.rEffPrime_CB_B.setZero();
	this->effProps.IEffPrimePntB_B.setZero();

	// - Initialize the variables to working values
	this->massFSP = 0.0;
	this->r_PB_B.setZero();
	this->pHat_B.setIdentity();
	this->k = 1.0;
	this->c = 0.0;
	this->nameOfRhoState = "fuelSloshParticleRho";
	this->nameOfRhoDotState = "fuelSloshParticleRhoDot";
	this->nameOfMassState = "fuelSloshParticleMass";

	return;
}

/*! This is the destructor, nothing to report here */
FuelSloshParticle::~FuelSloshParticle()
{
	return;
}

/*! Method for fuel slosh particle to access the states that it needs. It needs gravity and the hub states */
void FuelSloshParticle::linkInStates(DynParamManager& statesIn)
{
    // - Grab access to the hub states
	this->omegaState = statesIn.getStateObject("hubOmega");
	this->sigmaState = statesIn.getStateObject("hubSigma");
	this->velocityState = statesIn.getStateObject("hubVelocity");

    // - Grab access to gravity
    this->g_N = statesIn.getPropertyReference("g_N");

    return;
}

/*! This is the method for the fuel slosh particle to register its states: rho and rhoDot */
void FuelSloshParticle::registerStates(DynParamManager& states)
{
    // - Register rho and rhoDot
	this->rhoState = states.registerState(1, 1, nameOfRhoState);
	this->rhoDotState = states.registerState(1, 1, nameOfRhoDotState);

	// - Register m
	this->massState = states.registerState(1, 1, nameOfMassState);

	return;
}

/*! This is the method for the FSP to add its contributions to the mass props and mass prop rates of the vehicle */
void FuelSloshParticle::updateEffectorMassProps(double integTime)
{
	// - Grab rho from state manager and define r_PcB_B
	this->rho = this->rhoState->getState()(0,0);
	this->r_PcB_B = this->rho * this->pHat_B + this->r_PB_B;
	this->massFSP = this->massState->getState()(0, 0);

	// - Update the effectors mass
	this->effProps.mEff = this->massFSP;
	// - Update the position of CoM
	this->effProps.rEff_CB_B = this->r_PcB_B;
	// - Update the inertia about B
	this->rTilde_PcB_B = eigenTilde(this->r_PcB_B);
	this->effProps.IEffPntB_B = this->massFSP * this->rTilde_PcB_B * this->rTilde_PcB_B.transpose();

	// - Grab rhoDot from the stateManager and define rPrime_PcB_B
	this->rhoDot = this->rhoDotState->getState()(0, 0);
	this->rPrime_PcB_B = this->rhoDot * this->pHat_B;
	this->effProps.rEffPrime_CB_B = this->rPrime_PcB_B;

	// - Update the body time derivative of inertia about B
	this->rPrimeTilde_PcB_B = eigenTilde(this->rPrime_PcB_B);
	this->effProps.IEffPrimePntB_B = -this->massFSP*(this->rPrimeTilde_PcB_B*this->rTilde_PcB_B
                                                                          + this->rTilde_PcB_B*this->rPrimeTilde_PcB_B);

    return;
}

/*! This method is for the FSP to add its contributions to the back-sub method */
void FuelSloshParticle::updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr,
                                            Eigen::Matrix3d & matrixBcontr, Eigen::Matrix3d & matrixCcontr,
                                            Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr,
                                            Eigen::Vector3d & vecRotcontr)
{
    // - Find dcm_BN
    Eigen::MRPd sigmaLocal_BN;
    Eigen::Matrix3d dcm_BN;
    Eigen::Matrix3d dcm_NB;
    sigmaLocal_BN = (Eigen::Vector3d ) this->sigmaState->getState();
    dcm_NB = sigmaLocal_BN.toRotationMatrix();
    dcm_BN = dcm_NB.transpose();

    // - Map gravity to body frame
    Eigen::Vector3d gLocal_N;
    Eigen::Vector3d g_B;
    gLocal_N = *this->g_N;
    g_B = dcm_BN*gLocal_N;

	// - Define aRho
    this->aRho = -this->pHat_B;

    // - Define bRho
    this->bRho = -this->rTilde_PcB_B*this->pHat_B;

    // - Define cRho
    Eigen::Vector3d omega_BN_B_local = this->omegaState->getState();
    Eigen::Matrix3d omegaTilde_BN_B_local;
    omegaTilde_BN_B_local = eigenTilde(omega_BN_B_local);
	cRho = 1.0/(this->massFSP)*(this->pHat_B.dot(this->massFSP * g_B) - this->k*this->rho - this->c*this->rhoDot
		         - 2 * this->massFSP*this->pHat_B.dot(omegaTilde_BN_B_local * this->rPrime_PcB_B)
		                   - this->massFSP*this->pHat_B.dot(omegaTilde_BN_B_local*omegaTilde_BN_B_local*this->r_PcB_B));
	
	// - Compute matrix/vector contributions
	matrixAcontr = this->massFSP*this->pHat_B*this->aRho.transpose();
    matrixBcontr = this->massFSP*this->pHat_B*this->bRho.transpose();
    matrixCcontr = this->massFSP*this->rTilde_PcB_B*this->pHat_B*this->aRho.transpose();
	matrixDcontr = this->massFSP*this->rTilde_PcB_B*this->pHat_B*this->bRho.transpose();
	vecTranscontr = -this->massFSP*this->cRho*this->pHat_B;
	vecRotcontr = -this->massFSP*omegaTilde_BN_B_local * this->rTilde_PcB_B *this->rPrime_PcB_B -
                                                             this->massFSP*this->cRho*this->rTilde_PcB_B * this->pHat_B;
    return;
}

/*! This method is used to define the derivatives of the FSP. One is the trivial kinematic derivative and the other is 
 derived using the back-sub method */
void FuelSloshParticle::computeDerivatives(double integTime)
{
	
	// - Find DCM
	Eigen::MRPd sigmaLocal_BN;
	Eigen::Matrix3d dcm_BN;
	sigmaLocal_BN = (Eigen::Vector3d) this->sigmaState->getState();
	dcm_BN = (sigmaLocal_BN.toRotationMatrix()).transpose();
	
	// - Set the derivative of rho to rhoDot
	this->rhoState->setDerivative(this->rhoDotState->getState());

	// - Compute rhoDDot
	Eigen::MatrixXd conv(1,1);
    Eigen::Vector3d omegaDot_BN_B_local = this->omegaState->getStateDeriv();
    Eigen::Vector3d rDDot_BN_N_local = this->velocityState->getStateDeriv();
	Eigen::Vector3d rDDot_BN_B_local = dcm_BN*rDDot_BN_N_local;
    conv(0, 0) = this->aRho.dot(rDDot_BN_B_local) + this->bRho.dot(omegaDot_BN_B_local) + this->cRho;
	this->rhoDotState->setDerivative(conv);


    return;
}

/*! This method is for the FSP to add its contributions to energy and momentum */
void FuelSloshParticle::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr)
{
    //  - Get variables needed for energy momentum calcs
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = omegaState->getState();
    Eigen::Vector3d rDotPcB_B;

    // - Fuel slosh particles already have updated mass props due to fuel tank call

    // - Find rotational angular momentum contribution from hub
    rDotPcB_B = this->rPrime_PcB_B + omegaLocal_BN_B.cross(this->r_PcB_B);
    rotAngMomPntCContr_B = this->massFSP*this->r_PcB_B.cross(rDotPcB_B);

    // - Find rotational energy contribution from the hub
    rotEnergyContr = 1.0/2.0*this->massFSP*rDotPcB_B.dot(rDotPcB_B) + 1.0/2.0*this->k*this->rho*this->rho;

    return;
}

