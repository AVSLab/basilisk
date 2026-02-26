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

#include "facetedSpacecraftModel.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <cassert>

FacetedSpacecraftModel::~FacetedSpacecraftModel() {
    for (auto* msg : this->facetElementBodyOutMsgs) { delete msg; }
}

/*! This method resets required module variables and checks the input messages to ensure they are linked.
 @param callTime [ns] Time the method is called
*/
void FacetedSpacecraftModel::Reset(uint64_t callTime) {
    if (this->numArticulatedFacets > this->numFacets) {
        this->bskLogger->bskLog(BSK_ERROR,
                        "FacetedSpacecraftModel: numArticulatedFacets cannot be greater than total numFacets");
        return;
    }
    if (this->facetElementInMsgs.size() != this->numFacets ||
        this->facetElementBodyOutMsgs.size() != this->numFacets ||
        this->articulatedFacetDataInMsgs.size() != this->numArticulatedFacets) {
            this->bskLogger->bskLog(BSK_ERROR,
                                    "FacetedSpacecraftModel: Message vector size mismatch during Reset.");
            return;
        }

    // Clear and allocate data lists
    this->facetAreaList.clear();
    this->facetR_CopF_FList.clear();
    this->facetNHat_FList.clear();
    this->facetRotHat_FList.clear();
    this->facetDcm_F0BList.clear();
    this->facetR_FB_BList.clear();
    this->facetDiffuseCoeffList.clear();
    this->facetSpecularCoeffList.clear();
    this->facetR_CopB_BList.clear();
    this->facetNHat_BList.clear();
    this->facetRotHat_BList.clear();
    this->facetAreaList.reserve(this->numFacets);
    this->facetR_CopF_FList.reserve(this->numFacets);
    this->facetNHat_FList.reserve(this->numFacets);
    this->facetRotHat_FList.reserve(this->numFacets);
    this->facetDcm_F0BList.reserve(this->numFacets);
    this->facetR_FB_BList.reserve(this->numFacets);
    this->facetDiffuseCoeffList.reserve(this->numFacets);
    this->facetSpecularCoeffList.reserve(this->numFacets);
    this->facetR_CopB_BList.reserve(this->numFacets);
    this->facetNHat_BList.reserve(this->numFacets);
    this->facetRotHat_BList.reserve(this->numFacets);

    // Read faceted spacecraft element messages
    for (uint64_t idx = 0; idx < this->numFacets; idx++) {
        if (!this->facetElementInMsgs[idx].isLinked() || !this->facetElementInMsgs[idx].isWritten()) {
            this->bskLogger->bskLog(BSK_ERROR,
                                    "FacetedSpacecraftModel: Input message is not linked or written.");
            return;
        }

        FacetElementMsgPayload facetElementIn{};
        facetElementIn = this->facetElementInMsgs[idx]();

        // Save facet data to lists
        this->facetAreaList.push_back(facetElementIn.area);
        this->facetR_CopF_FList.push_back(cArray2EigenVector3d(facetElementIn.r_CopF_F));
        this->facetNHat_FList.push_back(cArray2EigenVector3d(facetElementIn.nHat_F).normalized());
        this->facetRotHat_FList.push_back(cArray2EigenVector3d(facetElementIn.rotHat_F));
        this->facetDcm_F0BList.push_back(cArray2EigenMatrix3d(*facetElementIn.dcm_F0B));
        this->facetR_FB_BList.push_back(cArray2EigenVector3d(facetElementIn.r_FB_B));
        this->facetDiffuseCoeffList.push_back(facetElementIn.c_diffuse);
        this->facetSpecularCoeffList.push_back(facetElementIn.c_specular);

        // Initialize output data lists
        this->facetR_CopB_BList.push_back(Eigen::Vector3d::Zero());
        this->facetNHat_BList.push_back(Eigen::Vector3d::Zero());
        this->facetRotHat_BList.push_back(Eigen::Vector3d::Zero());
    }

    // Populate output data lists for fixed facets
    for (uint64_t idx = this->numArticulatedFacets; idx < this->numFacets; idx++) {
        this->facetR_CopB_BList[idx] = this->facetDcm_F0BList[idx].transpose() * this->facetR_CopF_FList[idx]
                                          + this->facetR_FB_BList[idx];
        this->facetNHat_BList[idx] = this->facetDcm_F0BList[idx].transpose() * this->facetNHat_FList[idx];
        this->facetRotHat_BList[idx] = this->facetDcm_F0BList[idx].transpose() * this->facetRotHat_FList[idx];
    }
}


/*! Module update method.
 @param callTime [s] Time the method is called
*/
void FacetedSpacecraftModel::UpdateState(uint64_t callTime) {
    // Write module output messages
    this->writeOutputMessages(callTime);
}


/*! Method to write module output messages.
 @param callTime [s] Time the method is called
*/
void FacetedSpacecraftModel::writeOutputMessages(uint64_t callTime) {
}

/*! This method subscribes the articulated facet angle input messages to the module
articulatedFacetDataInMsgs input messages.
 @param tmpMsg hingedRigidBody input message containing facet articulation angle data
*/
void FacetedSpacecraftModel::addArticulatedFacet(Message<HingedRigidBodyMsgPayload> *tmpMsg) {
    // Safety checks
    assert(tmpMsg != nullptr && "addArticulatedFacet() received null msg pointer");
    assert(this->numArticulatedFacets < this->numFacets && "addArticulatedFacet() called more times than total facets");

    this->numArticulatedFacets++;
    this->articulatedFacetDataInMsgs.push_back(tmpMsg->addSubscriber());
}

/*! Setter method for the total number of spacecraft facets.
 @param numFacets [-]
*/
void FacetedSpacecraftModel::setNumTotalFacets(const uint64_t numFacets) {
    this->numFacets = numFacets;

    // Release old output messages if this setter is called multiple times
    for (auto* msg : this->facetElementBodyOutMsgs) {
        delete msg;
    }
    this->facetElementInMsgs.clear();
    this->facetElementBodyOutMsgs.clear();
    this->facetElementInMsgs.reserve(this->numFacets);
    this->facetElementBodyOutMsgs.reserve(this->numFacets);

    // Push back facet message vectors
    for (uint64_t idx = 0; idx < this->numFacets; ++idx) {
        this->facetElementInMsgs.push_back(ReadFunctor<FacetElementMsgPayload>{});
        this->facetElementBodyOutMsgs.push_back(new Message<FacetElementBodyMsgPayload>());
    }
}

/*! Getter method for the total number of spacecraft facets.
 @return const uint64_t
*/
const uint64_t FacetedSpacecraftModel::getNumTotalFacets() const { return this->numFacets; }
