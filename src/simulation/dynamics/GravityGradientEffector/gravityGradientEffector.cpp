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
#include "gravityGradientEffector.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"

GravityGradientEffector::GravityGradientEffector()
{
	this->forceExternal_B.fill(0.0);
	this->torqueExternalPntB_B.fill(0.0);
	this->gravityGradientOutMsgId = -1;
	return;
}

/*! The destructor.*/
GravityGradientEffector::~GravityGradientEffector()
{
	return;
}

/*! Create the outgoing gravity gradient torque message.
 @return void
 */
void GravityGradientEffector::SelfInit()
{
  return;
}

/*! This method is used to connect to incoming message.  For this module there are none.
 @return void
 */
void GravityGradientEffector::CrossInit()
{
    return;
}

/*! Write the gravity gradient torque output message.
@return void
 */
void GravityGradientEffector::WriteOutputMessages(uint64_t CurrentClock)
{
	return;
}

/*! This method is used to link the dragEffector to the hub attitude and velocity,
which are required for calculating drag forces and torques.
 @return void
 @param currentTime The current simulation time converted to a double
 */

void GravityGradientEffector::linkInStates(DynParamManager& states){
    this->hubSigma = states.getStateObject("hubSigma");
	this->ISCPntB_B = states.getStateObject("inertiaSC");
    this->c_B = states.getStateObject("centerOfMassSC");
}

/*! This method updates the internal drag direction based on the spacecraft velocity vector.
*/
//void GravityGradientEffector::updateDragDir(){
//    Eigen::MRPd sigmaBN;
//    sigmaBN = (Eigen::Vector3d)this->hubSigma->getState();
//    Eigen::Matrix3d dcm_BN = sigmaBN.toRotationMatrix().transpose();
//
//	this->v_B = dcm_BN*this->hubVelocity->getState(); // [m/s] sc velocity
//	this->v_hat_B = this->v_B / this->v_B.norm();
//
//	return;
//}


/*! This method computes the body forces and torques for the dragEffector in a simulation loop,
selecting the model type based on the settable attribute "modelType."
*/
void GravityGradientEffector::computeForceTorque(double integTime){
	//! - Zero out the structure force/torque for the drag set
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();


  	return;
}

/*! This method is called to update the local atmospheric conditions at each timestep.
Naturally, this means that conditions are held piecewise-constant over an integration step.
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void GravityGradientEffector::UpdateState(uint64_t CurrentSimNanos)
{


	this->WriteOutputMessages(CurrentSimNanos);
	return;
}
