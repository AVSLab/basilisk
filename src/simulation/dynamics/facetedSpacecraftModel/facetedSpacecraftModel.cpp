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
#include "architecture/utilities/rigidBodyKinematics.h"
#include <cmath>

/*! This method resets required module variables and checks the input messages to ensure they are linked.
 @param callTime [ns] Time the method is called
*/
void FacetedSpacecraftModel::Reset(uint64_t callTime) {
    // Read faceted spacecraft element messages
    for (int idx = 0; idx < this->numFacets; idx++) {
        if (this->facetElementInMsgs[idx].isLinked() && this->facetElementInMsgs[idx].isWritten()) {
            FacetElementMsgPayload facetElementIn{};
            facetElementIn = this->facetElementInMsgs[idx]();

            // Save facet data to lists
            this->facetAreaList.push_back(facetElementIn.area);
            this->facetR_CopF_FList.push_back(cArray2EigenVector3d(facetElementIn.r_CopF_F));
            this->facetNHat_FList.push_back(cArray2EigenVector3d(facetElementIn.nHat_F).normalized());
            this->facetRotHat_FList.push_back(cArray2EigenVector3d(facetElementIn.rotHat_F).normalized());
            this->facetDcm_F0BList.push_back(cArray2EigenMatrix3d(*facetElementIn.dcm_F0B));
            this->facetR_FB_BList.push_back(cArray2EigenVector3d(facetElementIn.r_FB_B));
            this->facetDiffuseCoeffList.push_back(facetElementIn.c_diffuse);
            this->facetSpecularCoeffList.push_back(facetElementIn.c_specular);

            // Initialize output data lists
            this->facetR_CopB_BList.push_back(Eigen::Vector3d::Zero());
            this->facetNHat_BList.push_back(Eigen::Vector3d::Zero());
            this->facetRotHat_BList.push_back(Eigen::Vector3d::Zero());
        }
    }

    // Populate output data lists for fixed facets
    for (int idx = this->numArticulatedFacets; idx < this->numFacets; idx++) {
        this->facetR_CopB_BList.at(idx) = this->facetDcm_F0BList.at(idx).transpose() * this->facetR_CopF_FList.at(idx)
                                          + this->facetR_FB_BList.at(idx);
        this->facetNHat_BList.at(idx) = this->facetDcm_F0BList.at(idx).transpose() * this->facetNHat_FList.at(idx);
        this->facetRotHat_BList.at(idx) = this->facetDcm_F0BList.at(idx).transpose() * this->facetRotHat_FList.at(idx);
    }
}

/*! This method subscribes the articulated facet angle input messages to the module
articulatedFacetDataInMsgs input messages.
 @param tmpMsg hingedRigidBody input message containing facet articulation angle data
*/
void FacetedSpacecraftModel::addArticulatedFacet(Message<HingedRigidBodyMsgPayload> *tmpMsg) {
    this->numArticulatedFacets++;
    this->articulatedFacetDataInMsgs.push_back(tmpMsg->addSubscriber());
}


/*! Module update method.
 @param callTime [s] Time the method is called
*/
void FacetedSpacecraftModel::UpdateState(uint64_t callTime) {
    // Read the articulated facet input messages
    HingedRigidBodyMsgPayload articulatedFacetAngleIn{};
    std::vector<double> articulatedFacetAngleList;
    for (int idx = 0; idx < this->numArticulatedFacets; idx++) {
        if (this->articulatedFacetDataInMsgs[idx].isLinked() && this->articulatedFacetDataInMsgs[idx].isWritten()) {
            articulatedFacetAngleIn = this->articulatedFacetDataInMsgs[idx]();
            articulatedFacetAngleList.push_back(articulatedFacetAngleIn.theta);
        }
    }

    double dcm_FF0Array[3][3];
    Eigen::Matrix3d dcm_FF0;
    Eigen::Matrix3d dcm_FB;
    std::vector<Eigen::Vector3d> facetNHat_BList;
    for (int idx = 0; idx < this->numArticulatedFacets; idx++) {
        // Save current facet articulation angle
        double articulationAngle = articulatedFacetAngleList.at(idx);

        // Determine the current facet attitude relative to its initial orientation F0
        double prv_FF0Array[3] = {articulationAngle * this->facetRotHat_FList[idx][0],
                                  articulationAngle * this->facetRotHat_FList[idx][1],
                                  articulationAngle * this->facetRotHat_FList[idx][2]};
        PRV2C(prv_FF0Array, dcm_FF0Array);
        dcm_FF0 = c2DArray2EigenMatrix3d(dcm_FF0Array);

        // Determine the current facet attitude relative to the hub frame B
        dcm_FB = dcm_FF0 * this->facetDcm_F0BList.at(idx);

        // Compute the articulating facet output data in the hub B frame
        this->facetR_CopB_BList.at(idx) = dcm_FB.transpose() * this->facetR_CopF_FList.at(idx)
                                          + this->facetR_FB_BList.at(idx);
        this->facetNHat_BList.at(idx) = dcm_FB.transpose() * this->facetNHat_FList.at(idx);
        this->facetRotHat_BList.at(idx) = dcm_FB.transpose() * this->facetRotHat_FList.at(idx);
    }

    // Write module output messages
    this->writeOutputMessages(callTime);
}


/*! Method to write module output messages.
 @param callTime [s] Time the method is called
*/
void FacetedSpacecraftModel::writeOutputMessages(uint64_t callTime) {
    for (int idx = 0; idx < this->numFacets; idx++) {
        FacetElementBodyMsgPayload facetElementBodyOut = this->facetElementBodyOutMsgs[idx]->zeroMsgPayload;
        facetElementBodyOut.area = facetAreaList.at(idx);
        eigenVector3d2CArray(facetR_CopB_BList.at(idx), facetElementBodyOut.r_CopB_B);
        eigenVector3d2CArray(facetNHat_BList.at(idx), facetElementBodyOut.nHat_B);
        eigenVector3d2CArray(facetRotHat_BList.at(idx), facetElementBodyOut.rotHat_B);
        facetElementBodyOut.c_diffuse = facetDiffuseCoeffList.at(idx);
        facetElementBodyOut.c_specular = facetSpecularCoeffList.at(idx);
        this->facetElementBodyOutMsgs[idx]->write(&facetElementBodyOut, moduleID, callTime);
    }
}

/*! Setter method for the total number of spacecraft facets.
 @param numFacets [-]
*/
void FacetedSpacecraftModel::setNumTotalFacets(const uint64_t numFacets) {
    this->numFacets = numFacets;

    // Push back vectors for the facet messages
    for (uint64_t idx = 0; idx < this->numFacets; idx++) {
        this->facetElementInMsgs.push_back(ReadFunctor<FacetElementMsgPayload>());
        this->facetElementBodyOutMsgs.push_back(new Message<FacetElementBodyMsgPayload>);
    }
}

/*! Getter method for the total number of spacecraft facets.
 @return const uint64_t
*/
const uint64_t FacetedSpacecraftModel::getNumTotalFacets() const { return this->numFacets; }
