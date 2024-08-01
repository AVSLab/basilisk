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

#include "sphericalPendulum.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <math.h>
#include <iostream>

/*! This is the constructor, setting variables to default values */
SphericalPendulum::SphericalPendulum()
{
	// - zero the contributions for mass props and mass rates
	this->effProps.mEff = 0.0;
	this->effProps.IEffPntB_B.setZero();
	this->effProps.rEff_CB_B.setZero();
	this->effProps.rEffPrime_CB_B.setZero();
	this->effProps.IEffPrimePntB_B.setZero();

	// - Initialize the variables to working values
	this->massFSP = 0.0;
	this->d.setZero();
	this->D.setZero();

    this->pHat_01 << 1,0,0;
    this->pHat_02 << 0,1,0;
    this->pHat_03 << 0,0,1;
    this->phiInit=0.0;
    this->l_B.setZero();
    this->lPrime_B.setZero();
    this->lPrime_P0.setZero();
    this->phiDotInit=0.0;
    this->thetaInit=0.0;
    this->thetaDotInit=0.0;
    this->massInit = 1.0;
	this->nameOfPhiState = "sphericalPendulumPhi" + std::to_string(this->effectorID);
	this->nameOfPhiDotState = "sphericalPendulumPhiDot" + std::to_string(this->effectorID);
	this->nameOfThetaState = "sphericalPendulumTheta" + std::to_string(this->effectorID);
	this->nameOfThetaDotState = "sphericalPendulumThetaDot" + std::to_string(this->effectorID);
	this->nameOfMassState = "sphericalPendulumMass" + std::to_string(this->effectorID);
    this->effectorID++;

	return;
}

uint64_t SphericalPendulum::effectorID = 1;

/*! This is the destructor, nothing to report here */
SphericalPendulum::~SphericalPendulum()
{
    this->effectorID = 1;    /* reset the panel ID*/
	return;
}

/*! Method for spherical pendulum to access the states that it needs. It needs gravity and the hub states */
void SphericalPendulum::linkInStates(DynParamManager& statesIn)
{
    // - Grab access to gravity
    this->g_N = statesIn.getPropertyReference(this->propName_vehicleGravity);

    return;
}

/*! This is the method for the spherical pendulum to register its states: l and lDot */
void SphericalPendulum::registerStates(DynParamManager& states)
{
	    // - Register phi, theta, phiDot and thetaDot
	this->phiState = states.registerState(1, 1, nameOfPhiState);
    Eigen::MatrixXd phiInitMatrix(1,1);
    phiInitMatrix(0,0) = this->phiInit;
    this->phiState->setState(phiInitMatrix);

    this->thetaState = states.registerState(1, 1, nameOfThetaState);
    Eigen::MatrixXd thetaInitMatrix(1,1);
    thetaInitMatrix(0,0) = this->thetaInit;
    this->thetaState->setState(thetaInitMatrix);

	this->phiDotState = states.registerState(1, 1, nameOfPhiDotState);
    Eigen::MatrixXd phiDotInitMatrix(1,1);
    phiDotInitMatrix(0,0) = this->phiDotInit;
    this->phiDotState->setState(phiDotInitMatrix);

	this->thetaDotState = states.registerState(1, 1, nameOfThetaDotState);
    Eigen::MatrixXd thetaDotInitMatrix(1,1);
    thetaDotInitMatrix(0,0) = this->thetaDotInit;
    this->thetaDotState->setState(thetaDotInitMatrix);

	// - Register m
	this->massState = states.registerState(1, 1, nameOfMassState);
    Eigen::MatrixXd massInitMatrix(1,1);
    massInitMatrix(0,0) = this->massInit;
    this->massState->setState(massInitMatrix);

	return;
}

/*! This is the method for the FSP to add its contributions to the mass props and mass prop rates of the vehicle */
void SphericalPendulum::updateEffectorMassProps(double integTime)
{

    // - Grab phi and theta from state manager and define r_PcB_B
	this->phi = this->phiState->getState()(0,0);
	this->theta = this->thetaState->getState()(0,0);

	// mantain phi and theta between 0 and 2pi
	if (this->phi>2*M_PI) {
		this->phi=this->phi-2*M_PI;
		Eigen::MatrixXd phiInitMatrix(1,1);
    	phiInitMatrix(0,0) = this->phi;
    	this->phiState->setState(phiInitMatrix);
	}
	if (this->phi<-2*M_PI) {
		this->phi=this->phi+2*M_PI;
		Eigen::MatrixXd phiInitMatrix(1,1);
    	phiInitMatrix(0,0) = this->phi;
    	this->phiState->setState(phiInitMatrix);
	}

	if (this->theta>2*M_PI) {
		this->theta=this->theta-2*M_PI;
		Eigen::MatrixXd thetaInitMatrix(1,1);
    	thetaInitMatrix(0,0) = this->theta;
    	this->thetaState->setState(thetaInitMatrix);
	}
	if (this->theta<-2*M_PI) {
		this->theta=this->theta+2*M_PI;
		Eigen::MatrixXd thetaInitMatrix(1,1);
    	thetaInitMatrix(0,0) = this->theta;
    	this->thetaState->setState(thetaInitMatrix);
	}
	// define l in P0 frame
	Eigen::Vector3d l_P0;
	l_P0 << this->pendulumRadius*cos(this->phi)*cos(this->theta), this->pendulumRadius*sin(this->phi)*cos(this->theta),
	 -this->pendulumRadius*sin(this->theta);

	// define the rotation matrix from P0 to B frame
	dcm_B_P0 << pHat_01(0,0), pHat_02(0,0), pHat_03(0,0),
				pHat_01(1,0), pHat_02(1,0), pHat_03(1,0),
				pHat_01(2,0), pHat_02(2,0), pHat_03(2,0);
	this->l_B=dcm_B_P0*l_P0;

	this->r_PcB_B = this->d + this->l_B;
	this->massFSP = this->massState->getState()(0, 0);

	// - Update the effectors mass
	this->effProps.mEff = this->massFSP;
	// - Update the position of CoM
	this->effProps.rEff_CB_B = this->r_PcB_B;
	// - Update the inertia about B
	this->rTilde_PcB_B = eigenTilde(this->r_PcB_B);
	this->effProps.IEffPntB_B = this->massFSP * this->rTilde_PcB_B * this->rTilde_PcB_B.transpose();

	// - Grab phiDot and thetaDot from the stateManager and define rPrime_PcB_B
	this->phiDot=this->phiDotState->getState()(0,0);
	this->thetaDot=this->thetaDotState->getState()(0,0);

	// define the derivative of l in P0 frame
	this->lPrime_P0 << this->pendulumRadius*(-this->phiDot*sin(this->phi)*cos(this->theta)-this->thetaDot*cos(this->phi)*sin(this->theta)),
    		this->pendulumRadius*(this->phiDot*cos(this->phi)*cos(this->theta)-this->thetaDot*sin(this->phi)*sin(this->theta)),
    		-this->pendulumRadius*(this->thetaDot*cos(this->theta));
    // rotate l derivative in B frame
    this->lPrime_B = dcm_B_P0*this->lPrime_P0;

	this->rPrime_PcB_B = this->lPrime_B;
	this->effProps.rEffPrime_CB_B = this->rPrime_PcB_B;

	// - Update the body time derivative of inertia about B
	this->rPrimeTilde_PcB_B = eigenTilde(this->rPrime_PcB_B);
	this->effProps.IEffPrimePntB_B = -this->massFSP*(this->rPrimeTilde_PcB_B*this->rTilde_PcB_B
                                                                          + this->rTilde_PcB_B*this->rPrimeTilde_PcB_B);

    return;
}

/*! This is method is used to pass mass properties information to the fuelTank */
void SphericalPendulum::retrieveMassValue(double integTime)
{
    // Save mass value into the fuelSlosh class variable
    this->fuelMass = this->massFSP;

    return;
}

/*! This method is for the FSP to add its contributions to the back-sub method */
void SphericalPendulum::updateContributions(double integTime, BackSubMatrices & backSubContr, Eigen::Vector3d sigma_BN, Eigen::Vector3d omega_BN_B, Eigen::Vector3d g_N)
{

    // - Find dcm_BN
    Eigen::MRPd sigmaLocal_BN;
    Eigen::Matrix3d dcm_BN;
    Eigen::Matrix3d dcm_NB;
    sigmaLocal_BN = (Eigen::Vector3d ) sigma_BN;
    dcm_NB = sigmaLocal_BN.toRotationMatrix();
    dcm_BN = dcm_NB.transpose();

    Eigen::Matrix3d dTilde;
    Eigen::Matrix3d lTilde;
    dTilde = eigenTilde(this->d);
	lTilde = eigenTilde(this->l_B);

	// - Define aPhi.transpose()
    this->aPhi = -(this->pHat_03.transpose()*lTilde)/(this->pendulumRadius*this->pendulumRadius*cos(this->theta)*cos(this->theta));
    // - Define bPhi.transpose()
    this->bPhi = this->pHat_03.transpose()*lTilde*(lTilde+dTilde)/(this->pendulumRadius*this->pendulumRadius*cos(this->theta)*cos(this->theta));


      // - Map gravity to body frame
    Eigen::Vector3d gLocal_N;
    Eigen::Vector3d g_B;
    gLocal_N = *this->g_N;
    g_B = dcm_BN*gLocal_N;
    Eigen::Vector3d L_T;
    L_T=this->l_B.cross(this->massFSP*g_B)-dcm_B_P0*this->D*this->lPrime_P0;

    // - Define cPhi
    Eigen::Vector3d omega_BN_B_local = omega_BN_B;
    Eigen::Matrix3d omegaTilde_BN_B_local;
    omegaTilde_BN_B_local = eigenTilde(omega_BN_B_local);
	this->cPhi = 1.0/(this->massFSP*this->pendulumRadius*this->pendulumRadius*cos(this->theta)*cos(this->theta))*
			(-this->massFSP*this->pHat_03.transpose().dot(lTilde*omegaTilde_BN_B_local*omegaTilde_BN_B_local*this->d)
			+this->pHat_03.transpose()*(L_T) +
			2*this->massFSP*this->pendulumRadius*this->pendulumRadius*this->phiDot*this->thetaDot*cos(this->theta)*sin(this->theta)
			-this->massFSP*this->pHat_03.transpose().dot(lTilde*(2*omegaTilde_BN_B_local*this->lPrime_B+omegaTilde_BN_B_local*omegaTilde_BN_B_local*this->l_B)));

	// Define pHat_02_Prime axes of rotation of theta in P0 frame
	Eigen::Vector3d pHat_02_Prime_P0;
	pHat_02_Prime_P0 << -sin(this->phi),cos(this->phi),0;
	// Rotate pHat_02_Prime axes in B frame components
	Eigen::Vector3d pHat_02_Prime;
	pHat_02_Prime = dcm_B_P0*pHat_02_Prime_P0;

	// Define aTheta.transpose()
	this->aTheta = -(pHat_02_Prime.transpose()*lTilde)/(this->pendulumRadius*this->pendulumRadius);
	// Define bTheta.transpose()
	this->bTheta = pHat_02_Prime.transpose()*lTilde*(lTilde+dTilde)/(this->pendulumRadius*this->pendulumRadius);
	// Define cTheta
	this->cTheta =  1.0/(this->massFSP*this->pendulumRadius*this->pendulumRadius)*
			(-this->massFSP*pHat_02_Prime.transpose().dot(lTilde*omegaTilde_BN_B_local*omegaTilde_BN_B_local*this->d)
			+pHat_02_Prime.transpose()*(L_T)
			-this->massFSP*this->pendulumRadius*this->pendulumRadius*this->phiDot*this->phiDot*cos(this->theta)*sin(this->theta)
			-this->massFSP*pHat_02_Prime.transpose().dot(lTilde*(2*omegaTilde_BN_B_local*this->lPrime_B+omegaTilde_BN_B_local*omegaTilde_BN_B_local*this->l_B)));

	// - Compute matrix/vector contributions
	backSubContr.matrixA = -this->massFSP*this->pendulumRadius*((sin(this->phi)*cos(this->theta)*this->pHat_01
	 - cos(this->phi)*cos(this->theta)*this->pHat_02)*this->aPhi.transpose()+(cos(this->phi)*sin(this->theta)*this->pHat_01
	 +sin(this->phi)*sin(this->theta)*this->pHat_02+cos(this->theta)*this->pHat_03)*this->aTheta.transpose());

    backSubContr.matrixB = -this->massFSP*this->pendulumRadius*((sin(this->phi)*cos(this->theta)*this->pHat_01
	 - cos(this->phi)*cos(this->theta)*this->pHat_02)*this->bPhi.transpose() + (cos(this->phi)*sin(this->theta)*this->pHat_01
	 +sin(this->phi)*sin(this->theta)*this->pHat_02+cos(this->theta)*this->pHat_03)*this->bTheta.transpose());

    backSubContr.matrixC = -this->massFSP*this->pendulumRadius*this->rTilde_PcB_B*((sin(this->phi)*cos(this->theta)*this->pHat_01
	 - cos(this->phi)*cos(this->theta)*this->pHat_02)*this->aPhi.transpose()+(cos(this->phi)*sin(this->theta)*this->pHat_01
	 +sin(this->phi)*sin(this->theta)*this->pHat_02+cos(this->theta)*this->pHat_03)*this->aTheta.transpose());

	backSubContr.matrixD = -this->massFSP*this->pendulumRadius*this->rTilde_PcB_B*((sin(this->phi)*cos(this->theta)*this->pHat_01
	 - cos(this->phi)*cos(this->theta)*this->pHat_02)*this->bPhi.transpose()+(cos(this->phi)*sin(this->theta)*this->pHat_01
	 +sin(this->phi)*sin(this->theta)*this->pHat_02+cos(this->theta)*this->pHat_03)*this->bTheta.transpose());

	backSubContr.vecTrans = -this->massFSP*this->pendulumRadius*((-cos(this->phi)*cos(this->theta)*this->pHat_01
	-sin(this->phi)*cos(this->theta)*this->pHat_02)*this->phiDot*this->phiDot
	+(-cos(this->phi)*cos(this->theta)*this->pHat_01-sin(this->phi)*cos(this->theta)*this->pHat_02+sin(this->theta)*this->pHat_03)*this->thetaDot*this->thetaDot
	+(2*sin(this->phi)*sin(this->theta)*this->pHat_01-2*cos(this->phi)*sin(this->theta)*this->pHat_02)*this->phiDot*this->thetaDot
	-(sin(this->phi)*cos(this->theta)*this->pHat_01-cos(this->phi)*cos(this->theta)*this->pHat_02)*this->cPhi
	-(cos(this->phi)*sin(this->theta)*this->pHat_01+sin(this->phi)*sin(this->theta)*this->pHat_02+cos(theta)*this->pHat_03)*this->cTheta);

	backSubContr.vecRot = -this->massFSP*(omegaTilde_BN_B_local*this->rTilde_PcB_B*this->rPrime_PcB_B
		+this->pendulumRadius*rTilde_PcB_B*(
	(-cos(this->phi)*cos(this->theta)*this->pHat_01-sin(this->phi)*cos(this->theta)*this->pHat_02)*this->phiDot*this->phiDot
	+(-cos(this->phi)*cos(this->theta)*this->pHat_01-sin(this->phi)*cos(this->theta)*this->pHat_02+sin(this->theta)*this->pHat_03)*this->thetaDot*this->thetaDot
	+(2*sin(this->phi)*sin(this->theta)*this->pHat_01-2*cos(this->phi)*sin(this->theta)*this->pHat_02)*this->phiDot*this->thetaDot
	-(sin(this->phi)*cos(this->theta)*this->pHat_01-cos(this->phi)*cos(this->theta)*this->pHat_02)*this->cPhi
	-(cos(this->phi)*sin(this->theta)*this->pHat_01+sin(this->phi)*sin(this->theta)*this->pHat_02+cos(theta)*this->pHat_03)*this->cTheta
			)
		);

    return;
}

/*! This method is used to define the derivatives of the FSP. One is the trivial kinematic derivative and the other is
 derived using the back-sub method */
void SphericalPendulum::computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN)
{

	// - Find DCM
	Eigen::MRPd sigmaLocal_BN;
	Eigen::Matrix3d dcm_BN;
	sigmaLocal_BN = (Eigen::Vector3d) sigma_BN;
	dcm_BN = (sigmaLocal_BN.toRotationMatrix()).transpose();

	// - Set the derivative of l to lDot
	this->phiState->setDerivative(this->phiDotState->getState());
	this->thetaState->setDerivative(this->thetaDotState->getState());

	// - Compute lDDot
	Eigen::MatrixXd phi_conv(1,1);
	Eigen::MatrixXd theta_conv(1,1);
    Eigen::Vector3d omegaDot_BN_B_local = omegaDot_BN_B;
    Eigen::Vector3d rDDot_BN_N_local = rDDot_BN_N;
	Eigen::Vector3d rDDot_BN_B_local = dcm_BN*rDDot_BN_N_local;
    phi_conv(0, 0) = this->aPhi.dot(rDDot_BN_B_local) + this->bPhi.dot(omegaDot_BN_B_local) + this->cPhi;
	this->phiDotState->setDerivative(phi_conv);
	theta_conv(0, 0) = this->aTheta.dot(rDDot_BN_B_local) + this->bTheta.dot(omegaDot_BN_B_local) + this->cTheta;
	this->thetaDotState->setDerivative(theta_conv);

    // - Set the massDot already computed from fuelTank to the stateDerivative of mass
    Eigen::MatrixXd conv(1,1);
    conv(0,0) = this->fuelMassDot;
    this->massState->setDerivative(conv);

    return;
}

/*! This method is for the FSP to add its contributions to energy and momentum */
void SphericalPendulum::updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                                     double & rotEnergyContr, Eigen::Vector3d omega_BN_B)
{

    //  - Get variables needed for energy momentum calcs
    Eigen::Vector3d omegaLocal_BN_B;
    omegaLocal_BN_B = omega_BN_B;
    Eigen::Vector3d rDotPcB_B;

    // - Find rotational angular momentum contribution from hub
    rDotPcB_B = this->rPrime_PcB_B + omegaLocal_BN_B.cross(this->r_PcB_B);
    rotAngMomPntCContr_B = this->massFSP*this->r_PcB_B.cross(rDotPcB_B);

    // - Find rotational energy contribution from the hub
    rotEnergyContr = 1.0/2.0*this->massFSP*rDotPcB_B.dot(rDotPcB_B);


    return;

}

void SphericalPendulum::modifyStates(double integTime){
	// when theta>45° change reference system in order to avoid singularities on aPhi, bPhi, cPhi
	if (fabs(cos(this->theta))<sqrt(2)/2){
		// define the rotation matrix from P0 to P0new
    	Eigen::Matrix3d dcm_P0_P0new;
    	dcm_P0_P0new << cos(this->phi)*cos(this->theta), -sin(this->phi), cos(this->phi)*sin(this->theta),
    					sin(this->phi)*cos(this->theta), cos(this->phi), sin(this->phi)*sin(this->theta),
    					-sin(this->theta), 0, cos(this->theta);
    	// define the P0new vectors in P0new components
    	this->pHat_01 << 1,0,0;
    	this->pHat_02 << 0,1,0;
    	this->pHat_03 << 0,0,1;

    	// rotate these vectors before in P0 frame and then in B frame
    	this->pHat_01=dcm_B_P0*dcm_P0_P0new*this->pHat_01;
    	this->pHat_02=dcm_B_P0*dcm_P0_P0new*this->pHat_02;
    	this->pHat_03=dcm_B_P0*dcm_P0_P0new*this->pHat_03;

    	// define the new rotation matrix from P0 to B
		dcm_B_P0 << pHat_01(0,0), pHat_02(0,0), pHat_03(0,0),
					pHat_01(1,0), pHat_02(1,0), pHat_03(1,0),
					pHat_01(2,0), pHat_02(2,0), pHat_03(2,0);		// compute lPrime in P0new components
		Eigen::Vector3d lPrime_P0new;
       	lPrime_P0new=dcm_B_P0.transpose()*this->lPrime_B;
       	// define the new values for phiDot and thetaDot inverting the lPrime definition
       	this->thetaDot=-lPrime_P0new(2,0)/this->pendulumRadius;
       	this->phiDot=lPrime_P0new(1,0)/this->pendulumRadius;
		// set the new values of theta and phi
    	this->theta=0;
    	this->phi=0;

    	// set the new state of phi,theta, phiDot, thetaDot
		Eigen::MatrixXd phiMatrix(1,1);
    	phiMatrix(0,0) = this->phi;
    	this->phiState->setState(phiMatrix);

		Eigen::MatrixXd thetaMatrix(1,1);
   		thetaMatrix(0,0) = this->theta;
    	this->thetaState->setState(thetaMatrix);

		Eigen::MatrixXd phiDotMatrix(1,1);
    	phiDotMatrix(0,0) = this->phiDot;
    	this->phiDotState->setState(phiDotMatrix);

        Eigen::MatrixXd thetaDotMatrix(1,1);
    	thetaDotMatrix(0,0) = this->thetaDot;
    	this->thetaDotState->setState(thetaDotMatrix);
    }

    	return;
    }
