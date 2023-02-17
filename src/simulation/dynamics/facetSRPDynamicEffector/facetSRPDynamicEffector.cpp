/*
 ISC License

 Copyright (c) 2023, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "facetSRPDynamicEffector.h"
#include <cmath>

/*! The constructor initializes the member variables to zero. */
FacetSRPDynamicEffector::FacetSRPDynamicEffector()
{
    this->forceExternal_B.fill(0.0);
    this->torqueExternalPntB_B.fill(0.0);
    this->numFacets = 0;
}

/*! The destructor. */
FacetSRPDynamicEffector::~FacetSRPDynamicEffector()
{
}

/*! The reset member function. This method checks to ensure the input message is linked.
 @return void
 @param currentSimNanos [ns]  Time the method is called
*/
void FacetSRPDynamicEffector::Reset(uint64_t currentSimNanos)
{
    if (!this->sunInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "FacetSRPDynamicEffector.sunInMsg was not linked.");
    }
}

/*! The SRP dynamic effector does not write any output messages.
 @return void
 @param currentClock [ns] Time the method is called
*/
void FacetSRPDynamicEffector::writeOutputMessages(uint64_t currentClock)
{
}

/*! This member function populates the spacecraft geometry structure with user-input facet information.
 @return void
 @param area  [m^2] Facet area
 @param specCoeff  Facet spectral reflection optical coefficient
 @param diffCoeff  Facet diffuse reflection optical coefficient
 @param normal_B  Facet normal expressed in B frame components
 @param locationPntB_B  [m] Facet location wrt point B in B frame components
*/
void FacetSRPDynamicEffector::addFacet(double area,
                                       double specCoeff,
                                       double diffCoeff,
                                       Eigen::Vector3d normal_B,
                                       Eigen::Vector3d locationPntB_B)
{
    this->scGeometry.facetAreas.push_back(area);
    this->scGeometry.facetSpecCoeffs.push_back(specCoeff);
    this->scGeometry.facetDiffCoeffs.push_back(diffCoeff);
    this->scGeometry.facetNormals_B.push_back(normal_B);
    this->scGeometry.facetLocationsPntB_B.push_back(locationPntB_B);
    this->numFacets = this->numFacets + 1;
}

/*! This method is used to link the faceted SRP effector to the hub attitude and position,
which are required for calculating SRP forces and torques.
 @return void
 @param states  Dynamic parameter states
*/
void FacetSRPDynamicEffector::linkInStates(DynParamManager& states)
{
    this->hubSigma = states.getStateObject("hubSigma");
    this->hubPosition = states.getStateObject("hubPosition");
}

/*! This method computes the body forces and torques for the SRP effector.
 @return void
 @param integTime  [s] Time the method is called
 @param timeStep  [s] Simulation time step
*/
void FacetSRPDynamicEffector::computeForceTorque(double integTime, double timeStep)
{
}

/*! This is the UpdateState() method
 @return void
 @param currentSimNanos [ns] Time the method is called
*/
void FacetSRPDynamicEffector::UpdateState(uint64_t currentSimNanos)
{
}
