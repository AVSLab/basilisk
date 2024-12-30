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

#include "facetSRPDynamicEffector.h"
#include "architecture/utilities/rigidBodyKinematics.h"
#include "architecture/utilities/avsEigenSupport.h"
#include <cmath>

const double speedLight = 299792458.0;  // [m/s] Speed of light
const double AstU = 149597870700.0;  // [m] Astronomical unit
const double solarRadFlux = 1368.0;  // [W/m^2] Solar radiation flux at 1 AU

/*! This method resets required module variables and checks the input messages to ensure they are linked.

 @param currentSimNanos [ns] Time the method is called
*/
void FacetSRPDynamicEffector::Reset(uint64_t currentSimNanos) {
    if (!this->sunInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "FacetSRPDynamicEffector.sunInMsg was not linked.");
    }
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
void FacetSRPDynamicEffector::addFacet(double area,
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
void FacetSRPDynamicEffector::addArticulatedFacet(Message<HingedRigidBodyMsgPayload> *tmpMsg) {
    this->articulatedFacetDataInMsgs.push_back(tmpMsg->addSubscriber());
}

/*! This method gives the module access to the hub inertial attitude and position.

 @param states Dynamic parameter states
*/
void FacetSRPDynamicEffector::linkInStates(DynParamManager& states) {
    this->hubSigma = states.getStateObject(this->stateNameOfSigma);
    this->hubPosition = states.getStateObject(this->stateNameOfPosition);
}

/*! This method reads the Sun state input message. If time-varying facet articulations are considered,
the articulation angle messages are also read.

*/
void FacetSRPDynamicEffector::ReadMessages() {
    // Read the Sun state input message
    if (this->sunInMsg.isLinked() && this->sunInMsg.isWritten()) {
        SpicePlanetStateMsgPayload sunMsgBuffer;
        sunMsgBuffer = sunInMsg.zeroMsgPayload;
        sunMsgBuffer = this->sunInMsg();
        this->r_SN_N = cArray2EigenVector3d(sunMsgBuffer.PositionVector);
    }

    // Read the facet articulation angle data
    if (this->articulatedFacetDataInMsgs.size() == this->numArticulatedFacets) {
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
    } else {
        bskLogger.bskLog(BSK_ERROR, "NUMBER OF ARTICULATED FACETS DOES NOT MATCH COUNTED VALUE.");
    }
}

/*! This method computes the srp force and torque acting about the hub point B in B frame components.

 @param callTime [s] Time the method is called
 @param timeStep [s] Simulation time step
*/
void FacetSRPDynamicEffector::computeForceTorque(double callTime, double timeStep) {
    // Read the input messages
    ReadMessages();

    // Compute dcm_BN
    Eigen::MRPd sigma_BN;
    sigma_BN = (Eigen::Vector3d)this->hubSigma->getState();
    Eigen::Matrix3d dcm_BN = sigma_BN.toRotationMatrix().transpose();

    // Grab the current spacecraft inertial position
    Eigen::Vector3d r_BN_N = this->hubPosition->getState();

    // Calculate the Sun unit direction vector relative to point B in B frame components
    Eigen::Vector3d r_SB_B = dcm_BN * (this->r_SN_N - r_BN_N);
    Eigen::Vector3d sHat = r_SB_B / r_SB_B.norm();

    // Define local srp force and torque storage vectors
    Eigen::Vector3d facetSRPForcePntB_B;
    Eigen::Vector3d facetSRPTorquePntB_B;
    Eigen::Vector3d totalSRPForcePntB_B;
    Eigen::Vector3d totalSRPTorquePntB_B;

    // Zero storage information
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();
    facetSRPForcePntB_B.setZero();
    facetSRPTorquePntB_B.setZero();
    totalSRPForcePntB_B.setZero();
    totalSRPTorquePntB_B.setZero();
    double projectedArea = 0.0;
    double cosTheta = 0.0;

    // Calculate the SRP pressure acting at the current spacecraft location
    double numAU = AstU / r_SB_B.norm();
    double SRPPressure = (solarRadFlux / speedLight) * numAU * numAU;

    // Loop through the facets and calculate the total SRP force and torque acting on the spacecraft about point B
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
        cosTheta = this->facetNHat_BList[i].dot(sHat);
        projectedArea = this->scGeometry.facetAreaList[i] * cosTheta;

        // Compute the SRP force and torque acting on the facet only if the facet is in view of the Sun
        if (projectedArea > 0.0) {
            facetSRPForcePntB_B = -SRPPressure * projectedArea
                                  * ((1 - this->scGeometry.facetSpecularCoeffList[i])
                                  * sHat + 2 * ((this->scGeometry.facetDiffuseCoeffList[i] / 3)
                                  + this->scGeometry.facetSpecularCoeffList[i] * cosTheta)
                                  * this->facetNHat_BList[i]);
            facetSRPTorquePntB_B = this->scGeometry.facetR_CopB_BList[i].cross(facetSRPForcePntB_B);

            // Add the facet contribution to the total SRP force and torque acting on the spacecraft
            totalSRPForcePntB_B = totalSRPForcePntB_B + facetSRPForcePntB_B;
            totalSRPTorquePntB_B = totalSRPTorquePntB_B + facetSRPTorquePntB_B;
        }
    }

    // Update the force and torque vectors in the dynamic effector base class
    this->forceExternal_B = totalSRPForcePntB_B;
    this->torqueExternalPntB_B = totalSRPTorquePntB_B;
}

/*! Setter method for the total number of facets used to model the spacecraft structure.

 @param numFacets Total number of spacecraft facets
*/
void FacetSRPDynamicEffector::setNumFacets(const uint64_t numFacets) {
    this->numFacets = numFacets;
}

/*! Setter method for the number of articulated facets used to model the spacecraft articulating components.

 @param numArticulatedFacets Number of articulated spacecraft facets
*/
void FacetSRPDynamicEffector::setNumArticulatedFacets(const uint64_t numArticulatedFacets) {
    this->numArticulatedFacets = numArticulatedFacets;
}

/*! Getter method for the total number of facets used to model the spacecraft structure.
 @return uint64_t
*/
uint64_t FacetSRPDynamicEffector::getNumFacets() const {
    return this->numFacets;
}

/*! Getter method for the number of articulated facets used to model the spacecraft articulating components.
 @return uint64_t
*/
uint64_t FacetSRPDynamicEffector::getNumArticulatedFacets() const {
    return this->numArticulatedFacets;
}
