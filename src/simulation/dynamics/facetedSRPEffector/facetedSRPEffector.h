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

#ifndef FACETED_SRP_EFFECTOR_H
#define FACETED_SRP_EFFECTOR_H

#include <Eigen/Dense>
#include <vector>
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"
#include "architecture/utilities/bskLogging.h"
#include "architecture/messaging/messaging.h"
#include "architecture/msgPayloadDefC/FacetElementBodyMsgPayload.h"
#include "architecture/msgPayloadDefC/ProjectedAreaMsgPayload.h"
#include "architecture/msgPayloadDefC/SpicePlanetStateMsgPayload.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"

/*! @brief Faceted solar radiation pressure dynamic effector */
class FacetedSRPEffector: public SysModel, public DynamicEffector {
public:
    FacetedSRPEffector() = default;  //!< Constructor
    ~FacetedSRPEffector() = default;  //!< Destructor
    void linkInStates(DynParamManager& states) override;  //!< Method for giving the effector access to the hub states
    void computeForceTorque(double callTime, double timeStep) override;  //!< Method for computing the total SRP force and torque about point B
    void Reset(uint64_t currentSimNanos) override;  //!< Reset method
    void setNumFacets(const uint64_t numFacets);  //!< Setter method for total number of spacecraft facets
    const uint64_t getNumFacets() const;  //!< Getter method for total number of spacecraft facets

    std::vector<ReadFunctor<FacetElementBodyMsgPayload>> facetElementBodyInMsgs;  //!< List of facet geometry input data (Expressed in hub B frame)
    std::vector<ReadFunctor<ProjectedAreaMsgPayload>> facetProjectedAreaInMsgs;  //!< List of facet projected area input messages
    ReadFunctor<SpicePlanetStateMsgPayload> sunStateInMsg;  //!< Sun spice ephemeris input message

private:
    /* Facet input message data */
    std::vector<double> facetAreaList;  //!< [m^2] List of facet areas
    std::vector<double> facetProjectedAreaList;  //!< [m^2] List of facet projected areas
    std::vector<Eigen::Vector3d> facetR_CopB_BList;  //!< [m] List of facet center of pressure locations wrt point B expressed in B frame components
    std::vector<Eigen::Vector3d> facetNHat_BList;  //!< [-] List of facet normal vectors expressed in hub B frame components
    std::vector<double> facetDiffuseCoeffList;  //!< [-] List of facet diffuse reflection optical coefficient list
    std::vector<double> facetSpecularCoeffList;  //!< [-] List of facet specular reflection optical coefficient list

    uint64_t numFacets{};  //!< Number of spacecraft facets
    StateData *hubPosition = nullptr;  //!< [m] Hub inertial position vector
    StateData *hubSigma = nullptr;  //!< Hub MRP inertial attitude
};

#endif
