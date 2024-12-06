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
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"

/*! @brief Spacecraft Geometry Data */
typedef struct {
    std::vector<double> facetAreaList;                                //!< [m^2] Vector of facet areas
    std::vector<Eigen::Matrix3d> facetDcm_F0BList;                    //!< Vector of facet frame F initial attitude DCMs relative to the B frame
    std::vector<Eigen::Vector3d> facetNHat_FList;                     //!< Vector of facet normals expressed in facet F frame components
    std::vector<Eigen::Vector3d> facetRotHat_FList;                   //!< [m] Vector of facet rotation axes expressed in facet F frame components
    std::vector<Eigen::Vector3d> facetR_CopB_BList;                   //!< [m] Vector of facet COP locations wrt point B expressed in B frame components
    std::vector<double> facetDiffuseCoeffList;                        //!< Vector of facet diffuse reflection optical coefficients
    std::vector<double> facetSpecularCoeffList;                       //!< Vector of facet spectral reflection optical coefficients
}FacetedSRPSpacecraftGeometryData;

/*! @brief Faceted Solar Radiation Pressure Dynamic Effector */
class FacetSRPDynamicEffector: public SysModel, public DynamicEffector {
public:
    FacetSRPDynamicEffector() = default;                                                 //!< Constructor
    ~FacetSRPDynamicEffector() = default;                                                //!< Destructor
    void linkInStates(DynParamManager& states) override;                                 //!< Method for giving the effector access to the hub states
    void computeForceTorque(double callTime, double timeStep) override;                  //!< Method for computing the total SRP force and torque about point B
    void Reset(uint64_t currentSimNanos) override;                                       //!< Reset method
    void setNumFacets(const uint64_t numFacets);                                         //!< Setter method for the total number of spacecraft facets
    void setNumArticulatedFacets(const uint64_t numArticulatedFacets);                   //!< Setter method for the number of articulated facets
    uint64_t getNumFacets() const;                                                       //!< Getter method for the total number of spacecraft facets
    uint64_t getNumArticulatedFacets() const;                                            //!< Getter method for the number of articulated facets
    void addFacet(double area,
                  Eigen::Matrix3d dcm_F0B,
                  Eigen::Vector3d nHat_F,
                  Eigen::Vector3d rotHat_F,
                  Eigen::Vector3d r_CopB_B,
                  double diffuseCoeff,
                  double specularCoeff);                                                 //!< Method for adding facets to the spacecraft geometry structure
    void addArticulatedFacet(Message<HingedRigidBodyMsgPayload> *tmpMsg);                //!< Method for adding articulated facets to the spacecraft geometry structure
    void ReadMessages();                                                                 //!< Method to read input messages

    ReadFunctor<SpicePlanetStateMsgPayload> sunInMsg;                                    //!< Sun spice ephemeris input message

    uint64_t numFacets = 0;                                                              //!< Total number of spacecraft facets
    uint64_t numArticulatedFacets = 0;                                                   //!< Number of articulated facets
private:
    std::vector<ReadFunctor<HingedRigidBodyMsgPayload>> articulatedFacetDataInMsgs;      //!< Articulated facet angle data input message
    std::vector<double> facetArticulationAngleList;                                      //!< [rad] Vector of facet rotation angles
    std::vector<Eigen::Vector3d> facetNHat_BList;                                        //!< Vector of facet normals expressed in B frame components
    FacetedSRPSpacecraftGeometryData scGeometry;                                         //!< Spacecraft facet data structure
    Eigen::Vector3d r_SN_N;                                                              //!< [m] Sun inertial position vector
    StateData *hubPosition = nullptr;                                                    //!< [m] Hub inertial position vector
    StateData *hubSigma = nullptr;                                                       //!< Hub MRP inertial attitude
    bool facetAngleMsgRead = false;                                                      //!< Boolean variable signaling that the facet articulation messages are read
};

#endif
