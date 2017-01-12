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

FuelSloshParticle::FuelSloshParticle()
{
	//! - zero the contributions for mass props and mass rates
	this->effProps.mEff = 0.0;
	this->effProps.IEffPntB_B.setZero();
	this->effProps.rEff_CB_B.setZero();
	this->effProps.rEffPrime_CB_B.setZero();
	this->effProps.IEffPrimePntB_B.setZero();

	//! - Initialize the variables to working values
	this->massFSP = 0.0;
	this->r_PB_B.setZero();
	this->pHat_B.setIdentity();
	this->k = 1.0;
	this->c = 0.0;
	this->nameOfRhoState = "fuelSloshParticleRho";
	this->nameOfRhoDotState = "fuelSloshParticleRhoDot";

	return;
}


FuelSloshParticle::~FuelSloshParticle()
{
	return;
}

void FuelSloshParticle::linkInStates(DynParamManager& statesIn)
{
	this->omegaState = statesIn.getStateObject("hubOmega");
	this->sigmaState = statesIn.getStateObject("hubSigma");
	this->velocityState = statesIn.getStateObject("hubVelocity");
    this->g_N = statesIn.getPropertyReference("g_N");

    return;
}

void FuelSloshParticle::registerStates(DynParamManager& states)
{
	this->rhoState = states.registerState(1, 1, nameOfRhoState);
	this->rhoDotState = states.registerState(1, 1, nameOfRhoDotState);

    return;
}

void FuelSloshParticle::updateEffectorMassProps(double integTime) {
	//Cached values used in this function
	this->rho = this->rhoState->getState()(0,0);
	this->rPcB_B = this->rho * this->pHat_B + this->r_PB_B;

	//Update the effectors mass
	this->effProps.mEff = this->massFSP;
	//Update the position of CoM
	this->effProps.rEff_CB_B = this->rPcB_B;
	//Update the inertia about B
	this->rTildePcB_B = eigenTilde(this->rPcB_B);
	this->effProps.IEffPntB_B = this->massFSP * this->rTildePcB_B * this->rTildePcB_B.transpose();

	//Cached values used in this function
	this->rhoDot = this->rhoDotState->getState()(0, 0);
	this->rPrimePcB_B = this->rhoDot * this->pHat_B;

	//Update derivative of CoM
	this->effProps.rEffPrime_CB_B = this->rPrimePcB_B;
	//Update the body time derivative of inertia about B
	this->rPrimeTildePcB_B = eigenTilde(this->rPrimePcB_B);
	this->effProps.IEffPrimePntB_B = -this->massFSP*(this->rPrimeTildePcB_B*this->rTildePcB_B + this->rTildePcB_B*this->rPrimeTildePcB_B);

    return;
}

void FuelSloshParticle::updateContributions(double integTime, Eigen::Matrix3d & matrixAcontr, Eigen::Matrix3d & matrixBcontr,
	Eigen::Matrix3d & matrixCcontr, Eigen::Matrix3d & matrixDcontr, Eigen::Vector3d & vecTranscontr,
	Eigen::Vector3d & vecRotcontr) {
    Eigen::MRPd sigmaBNLocal;
    Eigen::Matrix3d dcm_BN;                       /* direction cosine matrix from N to B */
    Eigen::Matrix3d dcm_NB;                       /* direction cosine matrix from B to N */
	Eigen::Vector3d omega_BN_B_local = this->omegaState->getState();
	Eigen::Matrix3d omegaTilde_BN_B_local;
    Eigen::Vector3d gLocal_N;                          /* gravitational acceleration in N frame */
    Eigen::Vector3d g_B;                          /* gravitational acceleration in B frame */
    gLocal_N = *this->g_N;

    //! - Find dcm_BN
    sigmaBNLocal = (Eigen::Vector3d ) this->sigmaState->getState();
    dcm_NB = sigmaBNLocal.toRotationMatrix();
    dcm_BN = dcm_NB.transpose();
    //! - Map gravity to body frame
    g_B = dcm_BN*gLocal_N;
	omegaTilde_BN_B_local = eigenTilde(omega_BN_B_local);

	// - Define aRho
    this->aRho = -this->pHat_B;

    // - Define bRho
    this->bRho = -this->rTildePcB_B*this->pHat_B;

    // - Define cRho
	cRho = 1.0/(this->massFSP)*(this->pHat_B.dot(this->massFSP * g_B) - this->k*this->rho - this->c*this->rhoDot
		- 2 * this->massFSP*this->pHat_B.dot(omegaTilde_BN_B_local * this->rPrimePcB_B)
		- this->massFSP*this->pHat_B.dot(omegaTilde_BN_B_local*omegaTilde_BN_B_local*this->rPcB_B));

	//Compute matrix/vector contributions
	matrixAcontr = this->massFSP*this->pHat_B*this->aRho.transpose();
    matrixBcontr = this->massFSP*this->pHat_B*this->bRho.transpose();
    matrixCcontr = this->massFSP*this->rTildePcB_B*this->pHat_B*this->aRho.transpose();
	matrixDcontr = this->massFSP*this->rTildePcB_B*this->pHat_B*this->bRho.transpose();

	vecTranscontr = -this->massFSP*this->cRho*this->pHat_B;
	vecRotcontr = -this->massFSP*omegaTilde_BN_B_local * this->rTildePcB_B *this->rPrimePcB_B -
		this->massFSP*this->cRho*this->rTildePcB_B * this->pHat_B;

    return;
}

void FuelSloshParticle::computeDerivatives(double integTime)
{
	Eigen::Vector3d omegaDot_BN_B_local = this->omegaState->getStateDeriv();
	Eigen::Vector3d rDDot_BN_N_local = this->velocityState->getStateDeriv();
	
	//Convert rDDot_BN_N to rDDot_BN_B
	Eigen::MRPd sigmaBNLocal;
	Eigen::Matrix3d dcm_BN;                        /* direction cosine matrix from N to B */
	Eigen::Matrix3d dcm_NB;                        /* direction cosine matrix from B to N */
	sigmaBNLocal = (Eigen::Vector3d) this->sigmaState->getState();
	dcm_NB = sigmaBNLocal.toRotationMatrix();
	dcm_BN = dcm_NB.transpose();
	Eigen::Vector3d rDDot_BN_B_local = dcm_BN*rDDot_BN_N_local;
	
	//Set the derivative of rho to rhoDot
	this->rhoState->setDerivative(this->rhoDotState->getState());

	//Compute rhoDDot
	Eigen::MatrixXd conv(1,1);
    conv(0, 0) = this->aRho.dot(rDDot_BN_B_local) + this->bRho.dot(omegaDot_BN_B_local) + this->cRho;
	this->rhoDotState->setDerivative(conv);

    return;
}

void FuelSloshParticle::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B, double & rotEnergyContr)
{
    // Get variables needed for energy momentum calcs
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = omegaState->getState();
    Eigen::Vector3d rDotPcB_B;

    // Fuel slosh particles already have updated mass props due to fuel tank call

    // Find rotational angular momentum contribution from hub
    rDotPcB_B = this->rPrimePcB_B + omegaLocal_BN_B.cross(this->rPcB_B);
    rotAngMomPntCContr_B = this->massFSP*this->rPcB_B.cross(rDotPcB_B);

    // Find rotational energy contribution from the hub
    rotEnergyContr = 1.0/2.0*this->massFSP*rDotPcB_B.dot(rDotPcB_B) + 1.0/2.0*this->k*this->rho*this->rho;
    return;
}

