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


#ifndef FACET_SRP_DYNAMIC_EFFECTOR_H
#define FACET_SRP_DYNAMIC_EFFECTOR_H

#include <Eigen/Dense>
#include <vector>
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"

/*! @brief Spacecraft Geometry Data */
typedef struct {
    std::vector<double> facetAreas;                                   //!< [m^2] Vector of facet areas
    std::vector<double> facetSpecCoeffs;                              //!< Vector of facet spectral reflection optical coefficients
    std::vector<double> facetDiffCoeffs;                              //!< Vector of facet diffuse reflection optical coefficients
    std::vector<Eigen::Vector3d> facetNormals_B;                      //!< Vector of facet normals expressed in B frame components
    std::vector<Eigen::Vector3d> facetLocationsPntB_B;                //!< [m] Vector of facet locations wrt point B in B frame components
}FacetedSRPSpacecraftGeometryData;

/*! @brief Faceted Solar Radiation Pressure Dynamic Effector */
class FacetSRPDynamicEffector: public SysModel, public DynamicEffector
{
public:                                                             
    FacetSRPDynamicEffector();                                      //!< The module constructor
    ~FacetSRPDynamicEffector();                                     //!< The module destructor
    void linkInStates(DynParamManager& states) override;            //!< Method for giving the effector access to the hub states
    void computeForceTorque(double integTime, double timeStep) override;  //!< Method for computing the SRP force and torque about point B
    void Reset(uint64_t currentSimNanos) override;                  //!< Reset method
    void UpdateState(uint64_t currentSimNanos) override;            //!< Method for updating the effector states
    void writeOutputMessages(uint64_t currentClock);                //!< Method for writing the output messages
    void addFacet(double area, double specCoeff, double diffCoeff, Eigen::Vector3d normal_B, Eigen::Vector3d locationPntB_B);  //!< Method for adding facets to the spacecraft geometry structure

    ReadFunctor<SpicePlanetStateMsgPayload> sunInMsg;               //!< Sun spice ephemeris input message
    uint64_t numFacets;                                             //!< Total number of spacecraft facets
    Eigen::Vector3d r_SN_N;                                         //!< [m] Position vector of the Sun with respect to the inertial frame
    StateData *hubPosition;                                         //!< [m] Hub inertial position vector
    StateData *hubSigma;                                            //!< MRP attitude of the hub B frame with respect to the inertial frame

private:
    FacetedSRPSpacecraftGeometryData scGeometry;                    //!< Structure to hold the spacecraft facet data
};

#endif 
