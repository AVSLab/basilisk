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

#include <iostream>
#include "aeroDynamicEffector.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/astroConstants.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/macroDefinitions.h"

AeroDynamicEffector::AeroDynamicEffector()
{
	this->coreParams.projectedArea = 0.0;
	this->coreParams.dragCoeff = 0.0;
    this->coreParams.comOffset.setZero();
	this->forceExternal_B.fill(0.0);
	this->torqueExternalPntB_B.fill(0.0);
	this->u_B.fill(0.0);
	this->u_hat_B.fill(0.0);
	this->omega_PN_N.fill(0.0);

    return;
}

/*! The destructor.*/
AeroDynamicEffector::~AeroDynamicEffector()
{
	return;
}


/*! This method is used to reset the module.
 @return void
 */
void AeroDynamicEffector::Reset(uint64_t CurrentSimNanos)
{
    // check if input message has not been included
    if (!this->atmoDensInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "aeroDynamicEffector.atmoDensInMsg was not linked.");
    }

}

/*! The AeroEffector does not write output messages to the rest of the sim.
@return void
 */
void AeroDynamicEffector::WriteOutputMessages(uint64_t CurrentClock)
{
	return;
}


/*! This method is used to read the incoming density message and update the internal density/
atmospheric data.
 @return void
 */
bool AeroDynamicEffector::ReadInputs()
{
	bool dataGood;
    this->atmoInData = this->atmoDensInMsg();
	this->planetState = this->spiceInMsg();
	
	dataGood = this->atmoDensInMsg.isWritten() && this->spiceInMsg.isWritten();

	return(dataGood);
}

/*!
    This method is used to link the aeroEffector to the hub attitude and velocity,
    which are required for calculating aero forces and torques.
    @return void
    @param states simulation states
 */
void AeroDynamicEffector::linkInStates(DynParamManager& states){
    this->hubSigma = states.getStateObject("hubSigma");
	this->hubPosition = states.getStateObject("hubPosition");
	this->hubVelocity = states.getStateObject("hubVelocity");
}

/*! This method updates the internal drag direction based on the spacecraft velocity vector.
*/
void AeroDynamicEffector::updateAeroDir(){
    /* compute DCM [BN] */
    Eigen::MRPd sigmaBN;
    sigmaBN = (Eigen::Vector3d)this->hubSigma->getState();
    Eigen::Matrix3d dcm_BN = sigmaBN.toRotationMatrix().transpose();
	
	/* compute planet-relative sc velocity */
	Eigen::Vector3d r_N;
	Eigen::Vector3d v_N;
	Eigen::Vector3d u_N;
	
	r_N = this->hubPosition->getState(); // [m] sc radial position
	v_N = this->hubVelocity->getState(); // [m/s] sc velocity
	u_N = v_N - this->omega_PN_N.cross(r_N);
	this->u_B = dcm_BN*u_N;
	this->u_hat_B = this->u_B / this->u_B.norm();
	
	return;
}

/*! This method implements a simple "cannnonball" (attitude-independent) aero model.
*/
void AeroDynamicEffector::cannonballAero(){
  	//! Begin method steps
  	//! - Zero out the structure force/torque for the aero set
  	this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();
    
  	this->forceExternal_B  = 0.5 * this->coreParams.dragCoeff * pow(this->u_B.norm(), 2.0) * this->coreParams.projectedArea * this->atmoInData.neutralDensity * (-1.0)*this->u_hat_B;
  	this->torqueExternalPntB_B = this->coreParams.comOffset.cross(forceExternal_B);

  	return;
}

/*! This method computes the body forces and torques for the aeroEffector in a simulation loop,
selecting the model type based on the settable attribute "modelType."
*/
void AeroDynamicEffector::computeForceTorque(double integTime, double timeStep){
	updateAeroDir();
	cannonballAero();
	
  	return;
}

/*! This method is called to update the local atmospheric conditions at each timestep.
Naturally, this means that conditions are held piecewise-constant over an integration step.
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void AeroDynamicEffector::UpdateState(uint64_t CurrentSimNanos)
{
	ReadInputs();
	convertEphemData();
	return;
}

/*!
    convert ephemeris data to get planetary rotation vector wrt inertial frame, in inertial components
 */
void AeroDynamicEffector::convertEphemData()
{
    Eigen::Matrix3d dcm_PN;
    Eigen::Vector3d sigma_PN;
    Eigen::Matrix3d dcm_PN_dot;
    Eigen::Matrix3d omega_tilde_PN_P_eigen;
    double omega_tilde_PN_P[3][3];
    double omega_tilde_PN_P_array[9];
	Eigen::Vector3d omega_PN_P;

	/* Compute sigma_BN */
	dcm_PN = cArray2EigenMatrix3d(*this->planetState.J20002Pfix);
	sigma_PN = eigenMRPd2Vector3d(eigenC2MRP(dcm_PN));

	/* Compute omega_BN_B */
	dcm_PN_dot = cArray2EigenMatrix3d(*this->planetState.J20002Pfix_dot);
	omega_tilde_PN_P_eigen = -dcm_PN_dot*dcm_PN.transpose();
	eigenMatrix3d2CArray(omega_tilde_PN_P_eigen, omega_tilde_PN_P_array);
	m33Copy(RECAST3X3 omega_tilde_PN_P_array, omega_tilde_PN_P);
	omega_PN_P[0] = omega_tilde_PN_P[2][1];
	omega_PN_P[1] = omega_tilde_PN_P[0][2];
	omega_PN_P[2] = omega_tilde_PN_P[1][0];
	this->omega_PN_N = dcm_PN*omega_PN_P;
}