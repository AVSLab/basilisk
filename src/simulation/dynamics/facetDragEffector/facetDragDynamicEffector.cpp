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

#include "facetDragDynamicEffector.h"
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"

FacetDragDynamicEffector::FacetDragDynamicEffector()
{
    this->forceExternal_B.fill(0.0);
    this->torqueExternalPntB_B.fill(0.0);
    this->v_B.fill(0.0);
    this->v_hat_B.fill(0.0);
    this->atmoInData = {};  // Initialize atmosphere data to zero
    this->windInData = {};  // Initialize wind data to zero
	this->numFacets = 0;
	return;
}

/*! The destructor.*/
FacetDragDynamicEffector::~FacetDragDynamicEffector()
{
	return;
}



void FacetDragDynamicEffector::Reset(uint64_t CurrentSimNanos)
{
	// check if input message has not been included
	if (!this->atmoDensInMsg.isLinked()) {
		bskLogger.bskLog(BSK_ERROR, "facetDragDynamicEffector.atmoDensInMsg was not linked.");
	}

    return;
}

/*! The DragEffector does not write output messages to the rest of the sim.

 */
void FacetDragDynamicEffector::WriteOutputMessages(uint64_t CurrentClock)
{
	return;
}


/*! This method is used to read the incoming density message and wind velocity message (if linked)
and update the internal density/atmospheric data.

 */
bool FacetDragDynamicEffector::ReadInputs()
{
    bool dataGood;
    dataGood = this->atmoDensInMsg.isWritten();
    if (dataGood) {
        this->atmoInData = this->atmoDensInMsg();
    }
    if (this->windVelInMsg.isLinked() && this->windVelInMsg.isWritten()) {
        this->windInData = this->windVelInMsg();
    } else {
        this->windInData = {};
    }
    return(dataGood);
}

/*!
    add a facet
    @param area
    @param dragCoeff
    @param B_normal_hat
    @param B_location
 */
void FacetDragDynamicEffector::addFacet(double area, double dragCoeff, Eigen::Vector3d B_normal_hat, Eigen::Vector3d B_location){
	this->scGeometry.facetAreas.push_back(area);
	this->scGeometry.facetCoeffs.push_back(dragCoeff);
	this->scGeometry.facetNormals_B.push_back(B_normal_hat);
	this->scGeometry.facetLocations_B.push_back(B_location);
	this->numFacets = this->numFacets + 1;
}

/*! This method is used to link the dragEffector to the hub attitude and velocity,
which are required for calculating drag forces and torques.

 @param states dynamic parameter states
 */

void FacetDragDynamicEffector::linkInStates(DynParamManager& states){
	this->hubSigma = states.getStateObject(this->stateNameOfSigma);
	this->hubVelocity = states.getStateObject(this->stateNameOfVelocity);
}

/*! This method updates the internal drag direction based on the spacecraft velocity vector.
 * It accounts for wind velocity if the wind message is linked.
*/
void FacetDragDynamicEffector::updateDragDir(){
    Eigen::MRPd sigmaBN;
    sigmaBN = (Eigen::Vector3d)this->hubSigma->getState();
    Eigen::Matrix3d dcm_BN = sigmaBN.toRotationMatrix().transpose();

    Eigen::Vector3d v_BN_N = this->hubVelocity->getState();

    if (this->windVelInMsg.isLinked()) {
        Eigen::Map<Eigen::Vector3d> v_air_N(this->windInData.v_air_N);
        Eigen::Vector3d v_B_air_N = v_BN_N - v_air_N;
        this->v_B = dcm_BN * v_B_air_N;
    } else {
        this->v_B = dcm_BN * v_BN_N;
    }

    double vNorm = this->v_B.norm();
    if (vNorm > 1e-12) {  // [m/s]
        this->v_hat_B = this->v_B / vNorm;
    } else {
        this->v_hat_B.setZero();
    }

    return;
}

/*! This method WILL implement a more complex flat-plate aerodynamics model with attitude
dependence and lift forces.
*/
void FacetDragDynamicEffector::plateDrag(){
	Eigen::Vector3d facetDragForce, facetDragTorque;
	Eigen::Vector3d totalDragForce, totalDragTorque;

	//! - Zero out the structure force/torque for the drag set
    double projectedArea = 0.0;
    double projectionTerm = 0.0;
	totalDragForce.setZero();
	totalDragTorque.setZero();
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();

	double vMag = this->v_B.norm();
	if (vMag <= 1e-12) {  // [m/s]
		return;
	}

	for(size_t i = 0; i < this->numFacets; i++){
	    projectionTerm = this->scGeometry.facetNormals_B[i].dot(this->v_hat_B);
		projectedArea = this->scGeometry.facetAreas[i] * projectionTerm;
		if(projectedArea > 0.0){
			facetDragForce = 0.5 * pow(vMag, 2.0) * this->scGeometry.facetCoeffs[i] * projectedArea * this->getDensity() * (-1.0)*this->v_hat_B;
			facetDragTorque = (-1)*facetDragForce.cross(this->scGeometry.facetLocations_B[i]);
			totalDragForce = totalDragForce + facetDragForce;
			totalDragTorque = totalDragTorque + facetDragTorque;
		}
	}
	this->forceExternal_B = totalDragForce;
	this->torqueExternalPntB_B = totalDragTorque;

  return;
}


/*! This method computes the body forces and torques for the dragEffector in a simulation loop,
selecting the model type based on the settable attribute "modelType."
*/
void FacetDragDynamicEffector::computeForceTorque(double integTime, double timeStep){
	updateDragDir();
	plateDrag();
  return;
}

/*! This method is called to update the local atmospheric conditions at each timestep.
Naturally, this means that conditions are held piecewise-constant over an integration step.

 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void FacetDragDynamicEffector::UpdateState(uint64_t CurrentSimNanos)
{
	ReadInputs();
	return;
}

/** This method obtains the density from the input data
 */
double FacetDragDynamicEffector::getDensity()
{
	return this->atmoInData.neutralDensity;
}
