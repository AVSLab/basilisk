/*
 ISC License

 Copyright (c) 2025, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef COMPUTE_SC_SUNLIT_FACET_AREA_H
#define COMPUTE_SC_SUNLIT_FACET_AREA_H

#include <Eigen/Dense>
#include <vector>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/SCSunlitFacetAreaMsgPayload.h"
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
}FacetedSpacecraftGeometryData;

/*! @brief Module for Computing the Spacecraft Sunlit Facet Area */
class ComputeSCSunlitFacetArea: public SysModel {
public:
    ComputeSCSunlitFacetArea() = default;                                                 //!< Constructor
    ~ComputeSCSunlitFacetArea() = default;                                                //!< Destructor
    void Reset(uint64_t currentSimNanos) override;
    void UpdateState(uint64_t currentSimNanos) override;
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

    ReadFunctor<SCStatesMsgPayload> scStatesInMsg;                                       //!< Spacecraft state input message
    ReadFunctor<SpicePlanetStateMsgPayload> sunInMsg;                                    //!< Sun spice ephemeris input message
    Message<SCSunlitFacetAreaMsgPayload> scSunlitFacetAreaOutMsg;                        //!< Spacecraft sunlit facet area output message
private:
    uint64_t numFacets = 0;                                                              //!< Total number of spacecraft facets
    uint64_t numArticulatedFacets = 0;                                                   //!< Number of articulated facets
    std::vector<ReadFunctor<HingedRigidBodyMsgPayload>> articulatedFacetDataInMsgs;      //!< Articulated facet angle data input message
    std::vector<double> facetArticulationAngleList;                                      //!< [rad] Vector of facet rotation angles
    std::vector<Eigen::Vector3d> facetNHat_BList;                                        //!< Vector of facet normals expressed in B frame components
    FacetedSpacecraftGeometryData scGeometry;                                            //!< Spacecraft facet data structure
    bool facetAngleMsgRead = false;                                                      //!< Boolean variable signaling that the facet articulation messages are read
};

#endif
