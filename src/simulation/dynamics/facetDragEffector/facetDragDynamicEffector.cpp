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
	this->numFacets = 0;
	this->useAtmosphereRelativeVelocity = false;
	this->planetOmega_N << 0.0, 0.0, OMEGA_EARTH;
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
	if (this->useAtmosphereRelativeVelocity && this->hubPosition == nullptr) {
    	bskLogger.bskLog(BSK_ERROR,
                     "facetDragDynamicEffector requires hub position state when useAtmosphereRelativeVelocity is enabled.");
	}

    return;
}

/*! The DragEffector does not write output messages to the rest of the sim.

 */
void FacetDragDynamicEffector::WriteOutputMessages(uint64_t CurrentClock)
{
	return;
}


/*! This method is used to read the incoming density message and update the internal density/
atmospheric data.

 */
bool FacetDragDynamicEffector::ReadInputs()
{
    bool dataGood;
    this->atmoInData = this->atmoDensInMsg();
    dataGood = this->atmoDensInMsg.isWritten();
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

	if (this->useAtmosphereRelativeVelocity) {
		this->hubPosition = states.getStateObject(this->stateNameOfPosition);
	}

	if (this->densityCorrectionStateName.empty())
		this->densityCorrection = nullptr;
	else
		this->densityCorrection = states.getStateObject(this->densityCorrectionStateName);
}

/*! This method updates the internal drag direction based on the spacecraft velocity vector.
*/
void FacetDragDynamicEffector::updateDragDir(){
    Eigen::MRPd sigmaBN;
    sigmaBN = (Eigen::Vector3d)this->hubSigma->getState();
    Eigen::Matrix3d dcm_BN = sigmaBN.toRotationMatrix().transpose();

	Eigen::Vector3d vRel_N = this->hubVelocity->getState();  // Initializing vRel_N = v_N

	if (this->useAtmosphereRelativeVelocity && this->hubPosition != nullptr) {
        Eigen::Vector3d r_N = this->hubPosition->getState();
        Eigen::Vector3d vAtmo_N = this->planetOmega_N.cross(r_N);
        vRel_N -= vAtmo_N;
    }

    this->v_B = dcm_BN * vRel_N;  // [m/s] sc velocity

    double vNorm = this->v_B.norm();
    if (vNorm > 1e-12) {
        this->v_hat_B = this->v_B / vNorm;
    } else {
        this->v_hat_B.setZero();
    }

    return;
}

/** This method obtains the density from the input data and applies a correction based on the
 * density correction state (if it was configured)
 */
double FacetDragDynamicEffector::getDensity()
{
	double density = this->atmoInData.neutralDensity;
	if (this->densityCorrection)
	{
		density *= 1 + this->densityCorrection->getState()(0,0);
	}
	return density;
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

	for(size_t i = 0; i < this->numFacets; i++){
	    projectionTerm = this->scGeometry.facetNormals_B[i].dot(this->v_hat_B);
		projectedArea = this->scGeometry.facetAreas[i] * projectionTerm;
		if(projectedArea > 0.0){
			facetDragForce = 0.5 * pow(this->v_B.norm(), 2.0) * this->scGeometry.facetCoeffs[i] * projectedArea * this->getDensity() * (-1.0)*this->v_hat_B;
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

/*!
 * @brief Enables or disables the use of atmosphere-relative velocity for drag computation.
 * When enabled, the drag force is computed using v_rel = v_sc - (omega_planet x r_sc).
 * Requires hub position state to be available.
 * @param useRelVel  true to use atmosphere-relative velocity, false to use inertial velocity
 */
void FacetDragDynamicEffector::setUseAtmosphereRelativeVelocity(bool useRelVel)
{
    this->useAtmosphereRelativeVelocity = useRelVel;
}

/*!
 * @brief Returns whether atmosphere-relative velocity is used in drag computation.
 * @return true if atmosphere-relative velocity is enabled
 */
bool FacetDragDynamicEffector::getUseAtmosphereRelativeVelocity() const
{
    return this->useAtmosphereRelativeVelocity;
}

/*!
 * @brief Sets the planetary rotation vector expressed in the inertial frame.
 * Used to compute the atmosphere velocity when useAtmosphereRelativeVelocity is enabled.
 * For Earth, the default is taken from OMEGA_EARTH in astroConstants.h: [0, 0, 7.2921159e-5] rad/s.
 * @param omega  Planetary rotation vector [rad/s] in inertial frame N
 */
void FacetDragDynamicEffector::setPlanetOmega_N(const Eigen::Vector3d& omega)
{
    this->planetOmega_N = omega;
}

/*!
 * @brief Returns the planetary rotation vector expressed in the inertial frame.
 * @return omega_planet [rad/s] in inertial frame N
 */
Eigen::Vector3d FacetDragDynamicEffector::getPlanetOmega_N() const
{
    return this->planetOmega_N;
}

/*!
 * @brief Sets the name of the scalar state used as a multiplicative density correction.
 * When set, the effective density becomes rho_eff = rho_in * (1 + delta), where delta is
 * read from the state registered under this name in the dynamic state manager.
 * Leave empty (default) to use the raw atmospheric density from atmoDensInMsg.
 * @param name  State manager key for the density correction scalar state
 */
void FacetDragDynamicEffector::setDensityCorrectionStateName(const std::string& name)
{
	this->densityCorrectionStateName = name;
}

/*!
 * @brief Returns the name of the scalar state used as a multiplicative density correction.
 * @return State manager key for the density correction state, or empty string if not set
 */
std::string FacetDragDynamicEffector::getDensityCorrectionStateName() const
{
	return this->densityCorrectionStateName;
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
