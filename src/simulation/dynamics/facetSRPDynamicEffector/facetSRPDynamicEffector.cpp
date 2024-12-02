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

/*! The constructor */
FacetSRPDynamicEffector::FacetSRPDynamicEffector() {
    this->forceExternal_B.fill(0.0);
    this->torqueExternalPntB_B.fill(0.0);
    this->numFacets = 0;
    this->numArticulatedFacets = 0;
    this->facetAngleMsgRead = false;
}

/*! The destructor */
FacetSRPDynamicEffector::~FacetSRPDynamicEffector() {
}

/*! The reset method
 @return void
 @param currentSimNanos  [ns] Time the method is called
*/
void FacetSRPDynamicEffector::Reset(uint64_t currentSimNanos) {
    if (!this->sunInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "FacetSRPDynamicEffector.sunInMsg was not linked.");
    }
}

/*! This method populates the spacecraft facet geometry structure with user-input facet information
 @return void
 @param area  [m^2] Facet area
 @param specularCoeff  Facet spectral reflection optical coefficient
 @param diffuseCoeff  Facet diffuse reflection optical coefficient
 @param nHat_B  Facet normal expressed in B frame components
 @param r_CopB_B  [m] Facet location wrt point B expressed in B frame components
 @param rotHat_B  Facet articulation axis expressed in B frame components
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
articulatedFacetDataInMsgs input message
 @return void
 @param tmpMsg  hingedRigidBody input message containing facet articulation angle data
*/
void FacetSRPDynamicEffector::addArticulatedFacet(Message<HingedRigidBodyMsgPayload> *tmpMsg) {
    this->articulatedFacetDataInMsgs.push_back(tmpMsg->addSubscriber());
}

/*! This method is used to link the faceted SRP effector to the hub attitude and position,
which are required for calculating SRP forces and torques
 @return void
 @param states  Dynamic parameter states
*/
void FacetSRPDynamicEffector::linkInStates(DynParamManager& states) {
    this->hubSigma = states.getStateObject(this->stateNameOfSigma);
    this->hubPosition = states.getStateObject(this->stateNameOfPosition);
}

/*! This method reads the Sun state input message. If time-varying facet articulations are considered,
the articulation angle messages are also read
 @return void
*/
void FacetSRPDynamicEffector::ReadMessages()
{
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

/*! This method computes the srp force and torque acting about the hub point B in B frame components
 @return void
 @param callTime  [s] Time the method is called
 @param timeStep  [s] Simulation time step
*/
void FacetSRPDynamicEffector::computeForceTorque(double callTime, double timeStep) {
    // Read the articulated facet information
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

    // Loop through the facets and calculate the SRP force and torque acting on the spacecraft about point B
    for (uint64_t i = 0; i < this->numFacets; i++) {
        double dcm_F0FArray[3][3];
        Eigen::Matrix3d dcm_F0F;

        // Determine the current facet normal vector if the facet articulates
        if ((this->numArticulatedFacets != 0) && (i >= (this->numFacets - this->numArticulatedFacets)) &&
            this->facetAngleMsgRead) {
            uint64_t articulatedIndex = this->numArticulatedFacets - (this->numFacets - i);
            double articulationAngle = facetArticulationAngleList.at(articulatedIndex);

            // Determine the required DCM that rotates the facet normal vector through the articulation angle
            double prv_F0FArray[3] = {-articulationAngle * this->scGeometry.facetRotHat_FList[i][0],
                                 -articulationAngle * this->scGeometry.facetRotHat_FList[i][1],
                                 -articulationAngle * this->scGeometry.facetRotHat_FList[i][2]};
            PRV2C(prv_F0FArray, dcm_F0FArray);
            dcm_F0F = c2DArray2EigenMatrix3d(dcm_F0FArray);

            // Rotate the facet normal vector through the current articulation angle
            this->facetNHat_BList[i] = this->scGeometry.facetDcm_F0BList[i].transpose() * dcm_F0F * this->scGeometry.facetNHat_FList[i];
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
