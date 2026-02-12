/*
 ISC License

 Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#ifndef FACETED_SPACECRAFT_MODEL_H
#define FACETED_SPACECRAFT_MODEL_H

#include <Eigen/Dense>
#include <vector>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/HingedRigidBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/FacetedSCMsgPayload.h"
#include "architecture/msgPayloadDefC/FacetedSCBodyMsgPayload.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/avsEigenMRP.h"

/*! @brief Faceted Spacecraft model */
class FacetedSpacecraftModel: public SysModel {
public:
    FacetedSpacecraftModel() = default;  //!< Constructor
    ~FacetedSpacecraftModel() = default;  //!< Destructor
    void Reset(uint64_t callTime) override;  //!< Reset method
    void addArticulatedFacet(Message<HingedRigidBodyMsgPayload> *tmpMsg);  //!< Method for adding articulated facets
    void UpdateState(uint64_t callTime) override;  //!< Update method
    void writeOutputMessages(uint64_t callTime);  //!< Method to write output messages

    std::vector<ReadFunctor<HingedRigidBodyMsgPayload>> articulatedFacetDataInMsgs;  //!< Articulated facet angle data input message
    ReadFunctor<FacetedSCMsgPayload> facetedSCInMsg;  //!< Faceted SC input message (Expressed in facet frames)
    Message<FacetedSCBodyMsgPayload> facetedSCBodyOutMsg;  //!< Faceted SC output message (Expressed in hub B frame)

    BSKLogger *bskLogger;  //!< BSK Logging

private:
    /* Facet input message data */
    uint64_t numFacets{};  //!< Total number of spacecraft facets
    uint64_t numArticulatedFacets{}; //!< Number of articulated facets
    std::vector<double> facetAreaList;  //!< [m^2] List of facet areas
    std::vector<Eigen::Vector3d> facetR_CopF_FList;  //!< [m] List of facet center of pressure locations wrt facet frame origin point F
    std::vector<Eigen::Vector3d> facetNHat_FList;  //!< [-] List of facet normal vectors expressed in facet frame F components
    std::vector<Eigen::Vector3d> facetRotHat_FList;  //!< [-] List of facet rotation axes expressed in facet frame F components
    std::vector<Eigen::Matrix3d> facetDcm_F0BList;  //!< [-] List of initial facet attitudes relative to hub B frame
    std::vector<Eigen::Vector3d> facetR_FB_BList;  //!< [m] List of facet locations relative to hub frame origin point B
    std::vector<double> facetDiffuseCoeffList;  //!< [-] List of facet diffuse reflection optical coefficient list
    std::vector<double> facetSpecularCoeffList;  //!< [-] List of facet specular reflection optical coefficient list

    /* Facet output message data */
    std::vector<Eigen::Vector3d> facetR_CopB_BList;  //!< [m] List of facet center of pressure locations wrt point B expressed in B frame components
    std::vector<Eigen::Vector3d> facetNHat_BList;  //!< [-] List of facet normal vectors expressed in hub B frame components
    std::vector<Eigen::Vector3d> facetRotHat_BList;  //!< [-] List of facet rotation axes expressed in hub B frame components
};

#endif
