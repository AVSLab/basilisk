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

#include "facetedSpacecraftProjectedArea.h"
#include "architecture/utilities/avsEigenSupport.h"

FacetedSpacecraftProjectedArea::~FacetedSpacecraftProjectedArea() {
    for (auto* msg : this->facetProjectedAreaOutMsgs) { delete msg; }
}

/*! This method resets required module variables and checks the input messages to ensure they are linked.
 @param callTime [ns] Time the method is called
*/
void FacetedSpacecraftProjectedArea::Reset(uint64_t callTime) {
    // If general heading is linked, no other messages can be linked
    if (this->bodyHeadingInMsg.isLinked() &&
        (this->spacecraftStateInMsg.isLinked() || this->sunStateInMsg.isLinked())) {
        this->bskLogger->bskLog(BSK_ERROR,
                                "FacetedSpacecraftProjectedArea: Input message error. bodyHeadingInMsg cannot be used with spacecraftStateInMsg and/or sunStateInMsg.");
        return;
    }
    // If Sun heading is linked, the spacecraft state message must also be linked
    if (this->sunStateInMsg.isLinked() && !this->spacecraftStateInMsg.isLinked()) {
        this->bskLogger->bskLog(BSK_ERROR,
                                "FacetedSpacecraftProjectedArea: Input message error. sunStateInMsg requires spacecraftStateInMsg to be linked.");
        return;
    }
    // If no messages are linked, no valid heading is configured
    if (! this->bodyHeadingInMsg.isLinked() && !this->sunStateInMsg.isLinked() && !this->spacecraftStateInMsg.isLinked()) {
        this->bskLogger->bskLog(BSK_ERROR,
                                "FacetedSpacecraftProjectedArea: Input message error. No valid heading source configured.");
        return;
    }

    // Check facet element messages are linked
    for (uint64_t idx = 0; idx < this->facetElementBodyInMsgs.size(); idx++) {
        if (!this->facetElementBodyInMsgs[idx].isLinked()) {
            this->bskLogger->bskLog(BSK_ERROR,
                                    "FacetedSpacecraftProjectedArea: Input message error. facetElementBodyInMsgs is not linked.");
            return;
        }
    }

    // Clear and allocate data lists
    this->facetAreaList.clear();
    this->facetNHat_BList.clear();
    this->facetProjectedAreaList.clear();
    this->facetAreaList.reserve(this->numFacets);
    this->facetNHat_BList.reserve(this->numFacets);
    this->facetProjectedAreaList.reserve(this->numFacets);
    for (uint64_t idx = 0; idx < this->numFacets; idx++) {
        this->facetAreaList.push_back(0.0);
        this->facetNHat_BList.push_back(Eigen::Vector3d::Zero());
        this->facetProjectedAreaList.push_back(0.0);
    }
}

/*! Module update method.
 @param callTime [ns] Time the method is called
*/
void FacetedSpacecraftProjectedArea::UpdateState(uint64_t callTime) {
    // Stop the simulation if Reset() failed
    if (this->facetAreaList.size() != this->numFacets ||
        this->facetNHat_BList.size() != this->numFacets ||
        this->facetProjectedAreaList.size() != this->numFacets ||
        this->facetElementBodyInMsgs.size() != this->numFacets ||
        this->facetProjectedAreaOutMsgs.size() != this->numFacets) {
        this->bskLogger->bskLog(BSK_ERROR,
                                "FacetedSpacecraftProjectedArea: UpdateState() called before successful Reset().");
        return;
    }

    // Read input messages
    if (!this->readInputMessages()) {
        return;
    }

    // Compute the projected area for all facets
    this->totalProjectedArea = 0.0;
    double projectedArea{};
    double cosTheta{};
    for (uint64_t idx = 0; idx < this->numFacets; idx++) {
        cosTheta = facetNHat_BList[idx].dot(this->bodyHeadingVec_B);
        projectedArea = facetAreaList[idx] * std::max(0.0, cosTheta);
        this->facetProjectedAreaList[idx] = projectedArea;
        this->totalProjectedArea += projectedArea;
    }

    // Write module output messages
    this->writeOutputMessages(callTime);
}

/*! This method reads the module input messages and updates the current body heading vector. The method returns a
    boolean indicating whether reading the input messages was successful.
 */
bool FacetedSpacecraftProjectedArea::readInputMessages() {
    // Set body heading vector default to zero
    this->bodyHeadingVec_B.setZero();

    // Read input messages
    if (this->spacecraftStateInMsg.isLinked()) {
        if (!this->spacecraftStateInMsg.isWritten()) {
            this->bskLogger->bskLog(BSK_ERROR,
                                    "FacetedSpacecraftProjectedArea: Input message error. spacecraftStateInMsg is linked but not written.");
            return false;
        }

        SCStatesMsgPayload spacecraftStateIn{};
        spacecraftStateIn = this->spacecraftStateInMsg();
        Eigen::MRPd sigma_BN = cArray2EigenMRPd(spacecraftStateIn.sigma_BN);
        Eigen::Matrix3d dcm_BN = sigma_BN.toRotationMatrix().transpose();

        if (this->sunStateInMsg.isLinked()) {
            // Option 1: Sun direction heading (Application: sunlit area calculation)
            // Also requires spacecraftStateInMsg
            if (!this->sunStateInMsg.isWritten()) {
                this->bskLogger->bskLog(BSK_ERROR,
                                        "FacetedSpacecraftProjectedArea: Input message error. sunStateInMsg is linked but not written.");
                return false;
            }

            SpicePlanetStateMsgPayload sunStateIn{};
            sunStateIn = this->sunStateInMsg();
            Eigen::Vector3d r_SN_N = cArray2EigenVector3d(sunStateIn.PositionVector);

            // Compute the sun direction unit vector in spacecraft body frame components
            Eigen::Vector3d r_BN_N = cArray2EigenVector3d(spacecraftStateIn.r_BN_N);
            Eigen::Vector3d r_SB_B = dcm_BN * (r_SN_N - r_BN_N);
            double norm = r_SB_B.norm();
            if (norm > 1e-12) {
                this->bodyHeadingVec_B = r_SB_B / norm;
            } else {
                this->bskLogger->bskLog(BSK_ERROR,
                                        "FacetedSpacecraftProjectedArea: No unique body heading vector can be resolved.");
                return false;
            }
        } else {
            // Option 2: Spacecraft velocity heading (Application: drag)
            Eigen::Vector3d v_BN_N = cArray2EigenVector3d(spacecraftStateIn.v_BN_N);
            Eigen::Vector3d v_BN_B = dcm_BN * v_BN_N;
            double norm = v_BN_B.norm();
            if (norm > 1e-12) {
                this->bodyHeadingVec_B = v_BN_B / norm;
            } else {
                this->bskLogger->bskLog(BSK_ERROR,
                                        "FacetedSpacecraftProjectedArea: No unique body heading vector can be resolved.");
                return false;
            }
        }
    } else if (this->bodyHeadingInMsg.isLinked()) {
        if (!this->bodyHeadingInMsg.isWritten()) {
            this->bskLogger->bskLog(BSK_ERROR,
                                    "FacetedSpacecraftProjectedArea: Input message error. bodyHeadingInMsg is linked but not written.");
            return false;
        }
        // Option 3: Direct heading
        BodyHeadingMsgPayload bodyHeadingIn{};
        bodyHeadingIn = this->bodyHeadingInMsg();
        this->bodyHeadingVec_B = cArray2EigenVector3d(bodyHeadingIn.rHat_XB_B);
    } else {
        this->bskLogger->bskLog(BSK_ERROR,
                                "FacetedSpacecraftProjectedArea: Input message error. Cannot subscribe multiple direction vectors to the module.");
        return false;
    }

    // Read the articulated facet input messages
    FacetElementBodyMsgPayload facetElementBodyIn{};
    for (uint64_t idx = 0; idx < this->numFacets; idx++) {
        if (this->facetElementBodyInMsgs[idx].isWritten()) {
            facetElementBodyIn = this->facetElementBodyInMsgs[idx]();
            this->facetAreaList[idx] = facetElementBodyIn.area;
            this->facetNHat_BList[idx] = cArray2EigenVector3d(facetElementBodyIn.nHat_B);
        } else {
            this->bskLogger->bskLog(BSK_ERROR,
                                    "FacetedSpacecraftProjectedArea: Input message error. facetElementBodyInMsgs is not written.");
            return false;
        }
    }

    return true;  // Reading input messages was successful
}

/*! Method to write module output messages.
 @param callTime [ns] Time the method is called
*/
void FacetedSpacecraftProjectedArea::writeOutputMessages(uint64_t callTime) {
    // Write total spacecraft projected area output message
    ProjectedAreaMsgPayload totalProjectedAreaOut{};
    totalProjectedAreaOut.area = this->totalProjectedArea;
    this->totalProjectedAreaOutMsg.write(&totalProjectedAreaOut, moduleID, callTime);

    // Write individual facet projected area output messages
    for (uint64_t idx = 0; idx < this->numFacets; idx++) {
        ProjectedAreaMsgPayload facetProjectedAreaOut = this->facetProjectedAreaOutMsgs[idx]->zeroMsgPayload;
        facetProjectedAreaOut.area = facetProjectedAreaList[idx];
        this->facetProjectedAreaOutMsgs[idx]->write(&facetProjectedAreaOut, moduleID, callTime);
    }
}

/*! Setter method for the total number of spacecraft facets.
 @param numFacets [-]
*/
void FacetedSpacecraftProjectedArea::setNumFacets(const uint64_t numFacets) {
    this->numFacets = numFacets;

    // Release old output messages if this setter is called multiple times
    for (auto* msg : this->facetProjectedAreaOutMsgs) { delete msg; }
    this->facetElementBodyInMsgs.clear();
    this->facetProjectedAreaOutMsgs.clear();
    this->facetElementBodyInMsgs.reserve(this->numFacets);
    this->facetProjectedAreaOutMsgs.reserve(this->numFacets);

    // Push back facet message vectors
    for (uint64_t idx = 0; idx < this->numFacets; ++idx) {
        this->facetElementBodyInMsgs.push_back(ReadFunctor<FacetElementBodyMsgPayload>{});
        this->facetProjectedAreaOutMsgs.push_back(new Message<ProjectedAreaMsgPayload>());
    }
}

/*! Getter method for the total number of spacecraft facets.
 @return const uint64_t
*/
const uint64_t FacetedSpacecraftProjectedArea::getNumFacets() const { return this->numFacets; }
