/*
 ISC License

 Copyright (c) 2016-2018, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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
#include "facetDragDynamicEffector.h"
#include "architecture/messaging/system_messaging.h"
#include "utilities/linearAlgebra.h"
#include "utilities/astroConstants.h"
#include "utilities/avsEigenSupport.h"
#include "../simulation/utilities/avsEigenMRP.h"

FacetDragDynamicEffector::FacetDragDynamicEffector()
{
	this->coreParams.projectedArea = 0.0;
	this->coreParams.velocityMag = 0.0;
	this->coreParams.dragCoeff = 0.0;

	this->atmoDensInMsgName = "atmo_dens_0_data";
	this->forceExternal_B.fill(0.0);
	this->torqueExternalPntB_B.fill(0.0);
	this->forceExternal_N.fill(0.0);
	this->locInertialVel.fill(0.0);
	this->dragDirection.fill(0.0);
	this->DensInMsgId = -1;

	this->numFacets = 0;
	return;
}

/*! The destructor.*/
FacetDragDynamicEffector::~FacetDragDynamicEffector()
{
	return;
}

/*! This method currently does very little.
 @return void
 */
void FacetDragDynamicEffector::SelfInit()
{
  return;
}

/*! This method is used to connect the input density message to the drag effector.
 It sets the message ID based on what it finds for the input string.
 @return void
 */
void FacetDragDynamicEffector::CrossInit()
{

	//! Begin method steps
	//! - Find the message ID associated with the atmoDensInMsgName string.
	this->DensInMsgId = SystemMessaging::GetInstance()->subscribeToMessage(this->atmoDensInMsgName,
																	 sizeof(AtmoPropsSimMsg), moduleID);
}


void FacetDragDynamicEffector::Reset()
{
    return;
}
/*! This method is used to set the input density message produced by some atmospheric model.
@return void
*/
void FacetDragDynamicEffector::setDensityMessage(std::string newDensMessage)
{
	this->atmoDensInMsgName = newDensMessage;
	return;
}

/*! The DragEffector does not write output messages to the rest of the sim.
@return void
 */
void FacetDragDynamicEffector::WriteOutputMessages(uint64_t CurrentClock)
{
	return;
}


/*! This method is used to read the incoming density message and update the internal density/
atmospheric data.
 @return void
 */
bool FacetDragDynamicEffector::ReadInputs()
{
	bool dataGood;
	//! - Zero the command buffer and read the incoming command array
	SingleMessageHeader LocalHeader;
	memset(&densityBuffer, 0x0, sizeof(AtmoPropsSimMsg));
	memset(&LocalHeader, 0x0, sizeof(LocalHeader));
	dataGood = SystemMessaging::GetInstance()->ReadMessage(this->DensInMsgId, &LocalHeader,
														  sizeof(AtmoPropsSimMsg),
														   reinterpret_cast<uint8_t*> (&densityBuffer), moduleID);
	this->atmoInData = densityBuffer;
	return(dataGood);

}

void FacetDragDynamicEffector::addFacet(double area, double dragCoeff, Eigen::Vector3d B_normal_hat, Eigen::Vector3d B_location){
	this->scGeometry.facetAreas.push_back(area);
	this->scGeometry.facetCoeffs.push_back(dragCoeff);
	this->scGeometry.B_facetNormals.push_back(B_normal_hat);
	this->scGeometry.B_facetLocations.push_back(B_location);
	this->numFacets = this->numFacets + 1;
}

/*! This method is used to link the dragEffector to the hub attitude and velocity,
which are required for calculating drag forces and torques.
 @return void
 @param currentTime The current simulation time converted to a double
 */

void FacetDragDynamicEffector::linkInStates(DynParamManager& states){
	this->hubSigma = states.getStateObject("hubSigma");
	this->hubVelocity = states.getStateObject("hubVelocity");
}

/*! This method updates the internal drag direction based on the spacecraft velocity vector.
*/
void FacetDragDynamicEffector::updateDragDir(){
	this->locInertialVel = this->hubVelocity->getState();
  	//std::cout<<"Velocity direction:"<<this->locInertialVel<<std::endl;
	this->dragDirection = -(this->locInertialVel / this->locInertialVel.norm());
	this->coreParams.velocityMag = this->locInertialVel.norm();
	//std::cout<<"Drag direction: "<<this->dragDirection<<", Velocity direction:"<<this->locInertialVel / this->locInertialVel.norm()<<std::endl;
	return;
}

/*! This method implements a simple "cannnonball" (attitude-independent) drag model.
*/
// void FacetDragDynamicEffector::cannonballDrag(){
//   	Eigen::Vector3d SingleDragForce;
//   	Eigen::Vector3d SingleDragTorque;
//   	//! Begin method steps
//   	//! - Zero out the structure force/torque for the drag set
//   	this->forceExternal_B.setZero();
//   	this->forceExternal_N.setZero();
//   	this->torqueExternalPntB_B.setZero();
//   	SingleDragForce = 0.5 * this->coreParams.dragCoeff * pow(this->coreParams.velocityMag, 2.0) * this->coreParams.projectedArea * this->atmoInData.neutralDensity * this->dragDirection;
//   	this->forceExternal_N = SingleDragForce;
//   	SingleDragTorque = this->coreParams.comOffset.cross(SingleDragForce);
//   	this->torqueExternalPntB_B = SingleDragTorque;

//   	return;
// }

/*! This method WILL implement a more complex flat-plate aerodynamics model with attitude
dependence and lift forces.
*/
void FacetDragDynamicEffector::plateDrag(){
	Eigen::Vector3d FacetDragForce;
	Eigen::Vector3d FacetDragTorque;
	Eigen::Vector3d TotalDragForce;
	Eigen::Vector3d TotalDragTorque;
	double projectedArea = 0.0;

	Eigen::MRPd sigmaLocal_BN;
	Eigen::Matrix3d dcm_BN;

	sigmaLocal_BN = (Eigen::Vector3d )this->hubSigma->getState();
	dcm_BN = sigmaLocal_BN.toRotationMatrix();
	//! Begin method steps
	//! - Zero out the structure force/torque for the drag set
	TotalDragForce.setZero();
	TotalDragTorque.setZero();
	this->forceExternal_B.setZero();
	this->forceExternal_N.setZero();


	this->torqueExternalPntB_B.setZero();
	for(int ind = 0; ind < this->numFacets; ind++){
		projectedArea = this->scGeometry.facetAreas[ind] * (this->scGeometry.B_facetNormals[ind].dot(dcm_BN * this->dragDirection));
		if(projectedArea < 0.0){
			FacetDragForce = 0.5 * pow(this->coreParams.velocityMag, 2.0) * this->scGeometry.facetCoeffs[ind] * (-projectedArea) * this->atmoInData.neutralDensity * this->dragDirection;
			FacetDragTorque = FacetDragForce.cross(this->scGeometry.B_facetLocations[ind]);
			TotalDragForce = TotalDragForce + FacetDragForce;
			TotalDragTorque = TotalDragTorque + FacetDragTorque;
		}
	}
	this->forceExternal_N = TotalDragForce;
	this->torqueExternalPntB_B = TotalDragTorque;

  return;
}


/*! This method computes the body forces and torques for the dragEffector in a simulation loop,
selecting the model type based on the settable attribute "modelType."
*/
void FacetDragDynamicEffector::computeForceTorque(double integTime){
	updateDragDir();
	plateDrag();
  return;
}

/*! This method is called to update the local atmospheric conditions at each timestep.
Naturally, this means that conditions are held piecewise-constant over an integration step.
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void FacetDragDynamicEffector::UpdateState(uint64_t CurrentSimNanos)
{
	ReadInputs();
	return;
}
