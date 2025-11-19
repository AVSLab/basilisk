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

#include "computeSCSunlitFacetArea.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <cmath>

/*! This method resets required module variables and checks the input messages to ensure they are linked.

 @param currentSimNanos [ns] Time the method is called
*/
void ComputeSCSunlitFacetArea::Reset(uint64_t currentSimNanos) {
    assert(this->sunInMsg.isLinked());
    assert(this->articulatedFacetDataInMsgs.size() == this->numArticulatedFacets);
}

/*! This method populates the spacecraft facet geometry structure with user-input facet information.

 @param area  [m^2] Facet area
 @param dcm_F0B Facet frame F initial attitude DCM relative to the B frame
 @param nHat_F  Facet normal expressed in facet F frame components
 @param rotHat_F  Facet articulation axis expressed in facet F frame components
 @param r_CopB_B  [m] Facet location wrt point B expressed in B frame components
 @param diffuseCoeff  Facet diffuse reflection optical coefficient
 @param specularCoeff  Facet spectral reflection optical coefficient
*/
void ComputeSCSunlitFacetArea::addFacet(double area,
                                       Eigen::Matrix3d dcm_F0B,
                                       Eigen::Vector3d nHat_F,
                                       Eigen::Vector3d rotHat_F,
                                       Eigen::Vector3d r_CopB_B,
                                       double diffuseCoeff,
                                       double specularCoeff) {
    this->scGeometry.facetAreaList.push_back(area);
    this->scGeometry.facetDcm_F0BList.push_back(dcm_F0B);
    this->scGeometry.facetNHat_FList.push_back(nHat_F);
    Eigen::Vector3d nHat_B = dcm_F0B.transpose() * nHat_F;
    this->facetNHat_BList.push_back(nHat_B);
    this->scGeometry.facetRotHat_FList.push_back(rotHat_F);
    this->scGeometry.facetR_CopB_BList.push_back(r_CopB_B);
    this->scGeometry.facetDiffuseCoeffList.push_back(diffuseCoeff);
    this->scGeometry.facetSpecularCoeffList.push_back(specularCoeff);
}

/*! This method subscribes the articulated facet angle input messages to the module
articulatedFacetDataInMsgs input messages.

 @param tmpMsg hingedRigidBody input message containing facet articulation angle data
*/
void ComputeSCSunlitFacetArea::addArticulatedFacet(Message<HingedRigidBodyMsgPayload> *tmpMsg) {
    this->articulatedFacetDataInMsgs.push_back(tmpMsg->addSubscriber());
}

/*! This method computes the total spacecraft sunlit facet area.

 @param currentSimNanos [ns] Time the method is called
*/
void ComputeSCSunlitFacetArea::UpdateState(uint64_t currentSimNanos) {
    SpicePlanetStateMsgPayload sunInMsgBuffer{};
    // Read the Sun state input message
    if (this->sunInMsg.isLinked() && this->sunInMsg.isWritten()) {
        sunInMsgBuffer = this->sunInMsg();
    }
    Eigen::Vector3d r_SN_N = cArray2EigenVector3d(sunInMsgBuffer.PositionVector);

    SCStatesMsgPayload scStatesInMsgBuffer{};
    // Read the spacecraft state input message
    if (this->scStatesInMsg.isLinked() && this->scStatesInMsg.isWritten()) {
        scStatesInMsgBuffer = this->scStatesInMsg();
    }
    Eigen::Vector3d r_BN_N = cArray2EigenVector3d(scStatesInMsgBuffer.r_BN_N);
    Eigen::MRPd sigma_BN = cArray2EigenMRPd(scStatesInMsgBuffer.sigma_BN);
    Eigen::Matrix3d dcm_BN = sigma_BN.toRotationMatrix().transpose();

    // Compute the Sun unit direction vector relative to point B in B frame components
    Eigen::Vector3d r_SB_B = dcm_BN * (r_SN_N - r_BN_N);
    Eigen::Vector3d sHat = r_SB_B / r_SB_B.norm();

    // Read the facet articulation angle data
    HingedRigidBodyMsgPayload facetAngleMsg;
    this->facetArticulationAngleList.clear();
    for (uint64_t i = 0; i < this->numArticulatedFacets; i++) {
        if (this->articulatedFacetDataInMsgs[i].isLinked() && this->articulatedFacetDataInMsgs[i].isWritten()) {
            facetAngleMsg = this->articulatedFacetDataInMsgs[i]();
            this->facetArticulationAngleList.push_back(facetAngleMsg.theta);
            this->facetAngleMsgRead = true;
        } else {
            this->facetAngleMsgRead = false;
        }
    }

    double totalSunlitFacetArea{};

    // Loop through the facets and calculate the total sunlit facet area
    for (uint64_t i = 0; i < this->numFacets; i++) {
        double dcm_FF0Array[3][3];
        Eigen::Matrix3d dcm_FF0;
        Eigen::Matrix3d dcm_FB;

        // Determine the current facet normal vector if the facet articulates
        if ((this->numArticulatedFacets != 0) && (i >= (this->numFacets - this->numArticulatedFacets)) &&
            this->facetAngleMsgRead) {
            uint64_t articulatedIndex = this->numArticulatedFacets - (this->numFacets - i);
            double articulationAngle = facetArticulationAngleList.at(articulatedIndex);

            // Determine the required DCM that rotates the facet normal vector through the articulation angle
            double prv_FF0Array[3] = {articulationAngle * this->scGeometry.facetRotHat_FList[i][0],
                                 articulationAngle * this->scGeometry.facetRotHat_FList[i][1],
                                 articulationAngle * this->scGeometry.facetRotHat_FList[i][2]};
            PRV2C(prv_FF0Array, dcm_FF0Array);
            dcm_FF0 = c2DArray2EigenMatrix3d(dcm_FF0Array);

            // Rotate the facet normal vector through the current articulation angle (Note: this is an active rotation)
            dcm_FB = dcm_FF0 * this->scGeometry.facetDcm_F0BList[i];
            this->facetNHat_BList[i] = dcm_FB.transpose() * this->scGeometry.facetNHat_FList[i];
        }

        // Determine the facet projected area
        double cosTheta = this->facetNHat_BList[i].dot(sHat);
        double projectedArea = this->scGeometry.facetAreaList[i] * cosTheta;

        // Add the facet area to the total spacecraft sunlit facet area
        if (projectedArea > 0.0) {
            totalSunlitFacetArea += projectedArea;
        }
    }

    // Write the output message
    SCSunlitFacetAreaMsgPayload scSunlitFacetAreaOut{};
    scSunlitFacetAreaOut.area = totalSunlitFacetArea;
    this->scSunlitFacetAreaOutMsg.write(&scSunlitFacetAreaOut, moduleID, currentSimNanos);
}

/*! Setter method for the total number of facets used to model the spacecraft structure.

 @param numFacets Total number of spacecraft facets
*/
void ComputeSCSunlitFacetArea::setNumFacets(const uint64_t numFacets) {
    this->numFacets = numFacets;
}

/*! Setter method for the number of articulated facets used to model the spacecraft articulating components.

 @param numArticulatedFacets Number of articulated spacecraft facets
*/
void ComputeSCSunlitFacetArea::setNumArticulatedFacets(const uint64_t numArticulatedFacets) {
    this->numArticulatedFacets = numArticulatedFacets;
}

/*! Getter method for the total number of facets used to model the spacecraft structure.
 @return uint64_t
*/
uint64_t ComputeSCSunlitFacetArea::getNumFacets() const {
    return this->numFacets;
}

/*! Getter method for the number of articulated facets used to model the spacecraft articulating components.
 @return uint64_t
*/
uint64_t ComputeSCSunlitFacetArea::getNumArticulatedFacets() const {
    return this->numArticulatedFacets;
}
