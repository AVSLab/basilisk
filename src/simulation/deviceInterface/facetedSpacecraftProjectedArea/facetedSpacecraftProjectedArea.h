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

#ifndef FACETED_SPACECRAFT_PROJECTED_AREA_H
#define FACETED_SPACECRAFT_PROJECTED_AREA_H

#include <Eigen/Dense>
#include <vector>
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/FacetElementBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/ProjectedAreaMsgPayload.h"
#include "architecture/msgPayloadDefC/SCStatesMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "architecture/msgPayloadDefC/BodyHeadingMsgPayload.h"


/*! @brief Faceted spacecraft projected area computation module */
class FacetedSpacecraftProjectedArea: public SysModel {
public:
    FacetedSpacecraftProjectedArea() = default;  //!< Constructor
    ~FacetedSpacecraftProjectedArea() override;  //!< Destructor
    void Reset(uint64_t callTime) override;  //!< Reset method
    void UpdateState(uint64_t callTime) override;  //!< Update method
    void readInputMessages(); //!< Method to read input messages
    void writeOutputMessages(uint64_t callTime);  //!< Method to write output messages
    void setNumFacets(const uint64_t numFacets);  //!< Setter method for total number of spacecraft facets
    const uint64_t getNumFacets() const;  //!< Getter method for total number of spacecraft facets

    ReadFunctor<SCStatesMsgPayload> spacecraftStateInMsg;  //!< Spacecraft state input message
    ReadFunctor<SpicePlanetStateMsgPayload> sunStateInMsg;  //!< Sun spice ephemeris input message
    ReadFunctor<BodyHeadingMsgPayload> bodyHeadingInMsg;  //!< Body heading input message
    std::vector<ReadFunctor<FacetElementBodyMsgPayload>> facetElementBodyInMsgs;  //!< List of facet geometry input data (Expressed in hub B frame)
    std::vector<Message<ProjectedAreaMsgPayload>*> facetProjectedAreaOutMsgs; //!< List of facet projected area output messages
    Message<ProjectedAreaMsgPayload> totalProjectedAreaOutMsg;  //!< Total spacecraft projected area output message

    BSKLogger *bskLogger = nullptr;  //!< BSK logging

private:
    uint64_t numFacets{};  //!< Number of spacecraft facets
    Eigen::Vector3d bodyHeadingVec_B{};  //!< Current body heading unit vector

    /* Facet input message data */
    std::vector<double> facetAreaList;  //!< [m^2] List of facet areas
    std::vector<Eigen::Vector3d> facetNHat_BList;  //!< [-] List of facet normal vectors expressed in hub B frame components

    /* Facet output message data */
    std::vector<double> facetProjectedAreaList;  //!< [m^2] List of facet projected areas
    double totalProjectedArea{};  //!< [m^2] Total projected area for all facets
};

#endif
