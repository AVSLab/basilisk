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

#include "dragDynamicEffector.h"

DragDynamicEffector::DragDynamicEffector()
{
	this->coreParams.projectedArea = 0.0;
	this->coreParams.dragCoeff = 0.0;
    this->coreParams.comOffset.setZero();
	this->modelType = "cannonball";
	this->forceExternal_B.fill(0.0);
	this->torqueExternalPntB_B.fill(0.0);
	this->v_B.fill(0.0);
	this->v_hat_B.fill(0.0);
    this->atmoInData = {};  // Initialize atmosphere data to zero
    this->windInData = {};  // Initialize wind data to zero

    return;
}

/*! The destructor.*/
DragDynamicEffector::~DragDynamicEffector()
{
	return;
}


/*! This method is used to reset the module.

 */
void DragDynamicEffector::Reset(uint64_t CurrentSimNanos)
{
    // check if input message has not been included
    if (!this->atmoDensInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "dragDynamicEffector.atmoDensInMsg was not linked.");
    }
}

/*! The DragEffector does not write output messages to the rest of the sim.

 */
void DragDynamicEffector::WriteOutputMessages(uint64_t CurrentClock)
{
	return;
}


/*! This method is used to read the incoming density message and wind velocity message (if linked)
and update the internal density/atmospheric data.

 */
bool DragDynamicEffector::ReadInputs()
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
    This method is used to link the dragEffector to the hub attitude and velocity,
    which are required for calculating drag forces and torques.

    @param states simulation states
 */
void DragDynamicEffector::linkInStates(DynParamManager& states){
    this->hubSigma = states.getStateObject(this->stateNameOfSigma);
    this->hubVelocity = states.getStateObject(this->stateNameOfVelocity);

	if (this->densityCorrectionStateName.empty())
		this->densityCorrection = nullptr;
	else
		this->densityCorrection = states.getStateObject(this->densityCorrectionStateName);
}

/*! This method updates the internal drag direction based on the spacecraft velocity vector.
 * It accounts for wind velocity if the wind message is linked.
*/
void DragDynamicEffector::updateDragDir(){
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

/** This method obtains the density from the input data and applies a correction based on the
 * density correction state (if it was configured)
 */
double DragDynamicEffector::getDensity()
{
	double density = this->atmoInData.neutralDensity;
	if (this->densityCorrection)
	{
		density *= 1 + this->densityCorrection->getState()(0,0);
	}
	return density;
}

/*! This method implements a simple "cannonball" (attitude-independent) drag model.
*/
void DragDynamicEffector::cannonballDrag(){
  	//! Begin method steps
  	//! - Zero out the structure force/torque for the drag set
  	this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();

	double vMag = this->v_B.norm();
	if (vMag <= 1e-12) {  // [m/s]
		return;
	}

  	this->forceExternal_B = 0.5 * this->coreParams.dragCoeff * pow(vMag, 2.0) * this->coreParams.projectedArea * this->getDensity() * (-1.0)*this->v_hat_B;
  	this->torqueExternalPntB_B = this->coreParams.comOffset.cross(forceExternal_B);

  	return;
}

/*! This method computes the body forces and torques for the dragEffector in a simulation loop,
selecting the model type based on the settable attribute "modelType."
*/
void DragDynamicEffector::computeForceTorque(double integTime, double timeStep){
	updateDragDir();
	if(this->modelType == "cannonball"){
		cannonballDrag();
  	}
  	return;
}

/*! This method is called to update the local atmospheric conditions at each timestep.
Naturally, this means that conditions are held piecewise-constant over an integration step.

 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void DragDynamicEffector::UpdateState(uint64_t CurrentSimNanos)
{
	ReadInputs();
	return;
}
