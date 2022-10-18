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

AeroDynamicEffector::AeroDynamicEffector()
{
	this->coreParams.projectedArea = 0.0;
	this->coreParams.dragCoeff = 0.0;
    this->coreParams.comOffset.setZero();
	this->forceExternal_B.fill(0.0);
	this->torqueExternalPntB_B.fill(0.0);
	this->v_B.fill(0.0);
	this->v_hat_B.fill(0.0);

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
    bool planetRead = false;
    this->atmoInData = this->atmoDensInMsg();
    dataGood = this->atmoDensInMsg.isWritten();
    if (this->planetPosInMsg.isLinked())
    {
        this->planetState = this->planetPosInMsg();
        planetRead = this->planetPosInMsg.isWritten();
        this->dcm_PN_dot = Eigen::Map<Eigen::Matrix3d>(&(this->planetState.J20002Pfix_dot[0][0]), 3, 3);
    }
    std::cout << "planetRead:" << planetRead << std::endl;
    std::cout << "dcm_PN_dot" << dcm_PN_dot << std::endl;

    return(dataGood && planetRead);
}

/*!
    This method is used to link the aeroEffector to the hub attitude and velocity,
    which are required for calculating aero forces and torques.
    @return void
    @param states simulation states
 */
void AeroDynamicEffector::linkInStates(DynParamManager& states){
    this->hubSigma = states.getStateObject("hubSigma");
	this->hubVelocity = states.getStateObject("hubVelocity");
}

/*! This method updates the internal drag direction based on the spacecraft velocity vector.
*/
void AeroDynamicEffector::updateAeroDir(){
    /* compute DCN [BN] */
    Eigen::MRPd sigmaBN;
    sigmaBN = (Eigen::Vector3d)this->hubSigma->getState();
    Eigen::Matrix3d dcm_BN = sigmaBN.toRotationMatrix().transpose();
    
	this->v_B = dcm_BN*this->hubVelocity->getState(); // [m/s] sc velocity
	this->v_hat_B = this->v_B / this->v_B.norm();
	
	return;
}

/*! This method implements a simple "cannnonball" (attitude-independent) aero model.
*/
void AeroDynamicEffector::cannonballAero(){
  	//! Begin method steps
  	//! - Zero out the structure force/torque for the aero set
  	this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();
    
  	this->forceExternal_B  = 0.5 * this->coreParams.dragCoeff * pow(this->v_B.norm(), 2.0) * this->coreParams.projectedArea * this->atmoInData.neutralDensity * (-1.0)*this->v_hat_B;
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
	return;
}
