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

#include "facetedSRPEffector.h"
#include "architecture/utilities/avsEigenSupport.h"

const double speedLight = 299792458.0;  // [m/s] Speed of light
const double AstU = 149597870700.0;  // [m] Astronomical unit
const double solarRadFlux = 1368.0;  // [W/m^2] Solar radiation flux at 1 AU

/*! This method resets required module variables and checks the input messages to ensure they are linked.

 @param currentSimNanos [ns] Time the method is called
*/
void FacetedSRPEffector::Reset(uint64_t currentSimNanos) {
    // Check Sun state input message is linked
    if (!this->sunStateInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "FacetedSRPEffector.sunStateInMsg was not linked.");
        return;
    }

    // Check all facet input messages are linked
    for (uint64_t idx = 0; idx < this->facetElementBodyInMsgs.size(); idx++) {
        if (!this->facetElementBodyInMsgs[idx].isLinked()) {
            this->bskLogger.bskLog(BSK_ERROR,
                                    "FacetedSpacecraftProjectedArea: Input message error. facetElementBodyInMsgs is not linked.");
            return;
        }
        if (!this->facetProjectedAreaInMsgs[idx].isLinked()) {
            this->bskLogger.bskLog(BSK_ERROR,
                                    "FacetedSpacecraftProjectedArea: Input message error. facetProjectedAreaInMsgs is not linked.");
            return;
        }
    }

    // Clear, allocate and initialize data lists
    this->facetAreaList.clear();
    this->facetProjectedAreaList.clear();
    this->facetR_CopB_BList.clear();
    this->facetNHat_BList.clear();
    this->facetDiffuseCoeffList.clear();
    this->facetSpecularCoeffList.clear();
    this->facetAreaList.reserve(this->numFacets);
    this->facetProjectedAreaList.reserve(this->numFacets);
    this->facetR_CopB_BList.reserve(this->numFacets);
    this->facetNHat_BList.reserve(this->numFacets);
    this->facetDiffuseCoeffList.reserve(this->numFacets);
    this->facetSpecularCoeffList.reserve(this->numFacets);
    for (uint64_t idx = 0; idx < this->numFacets; idx++) {
        this->facetAreaList.push_back(0.0);
        this->facetProjectedAreaList.push_back(0.0);
        this->facetR_CopB_BList.push_back(Eigen::Vector3d::Zero());
        this->facetNHat_BList.push_back(Eigen::Vector3d::Zero());
        this->facetDiffuseCoeffList.push_back(0.0);
        this->facetSpecularCoeffList.push_back(0.0);
    }
}

/*! This method gives the module access to the hub inertial attitude and position.

 @param states Dynamic parameter states
*/
void FacetedSRPEffector::linkInStates(DynParamManager& states) {
    this->hubSigma = states.getStateObject(this->stateNameOfSigma);
    this->hubPosition = states.getStateObject(this->stateNameOfPosition);
}

/*! This method computes the srp force and torque acting about the hub point B in B frame components.

 @param callTime [s] Time the method is called
 @param timeStep [s] Simulation time step
*/
void FacetedSRPEffector::computeForceTorque(double callTime, double timeStep) {

    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();
    if (this->facetAreaList.size() != this->numFacets) {
        return;
    }

    // Read the sun state input message
    SpicePlanetStateMsgPayload sunStateIn{};
    Eigen::Vector3d r_SN_N = Eigen::Vector3d::Zero();
    if (this->sunStateInMsg.isWritten()) {
        sunStateIn = this->sunStateInMsg();
        r_SN_N = cArray2EigenVector3d(sunStateIn.PositionVector);
    }

    // Compute the sun direction unit vector in spacecraft body frame components
    Eigen::MRPd sigma_BN;
    sigma_BN = (Eigen::Vector3d)this->hubSigma->getState();
    Eigen::Matrix3d dcm_BN = sigma_BN.toRotationMatrix().transpose();
    Eigen::Vector3d r_BN_N = this->hubPosition->getState();
    Eigen::Vector3d r_SB_B = dcm_BN * (r_SN_N - r_BN_N);
    Eigen::Vector3d sunDirHat_B = r_SB_B.normalized();

    // Read the facet geometry and projected area input messages
    FacetElementBodyMsgPayload facetElementBodyIn{};
    ProjectedAreaMsgPayload facetProjectedAreaIn{};
    for (uint64_t idx = 0; idx < this->numFacets; idx++) {
        if (this->facetElementBodyInMsgs[idx].isWritten() && this->facetProjectedAreaInMsgs[idx].isWritten()) {
            facetElementBodyIn = this->facetElementBodyInMsgs[idx]();
            this->facetAreaList[idx] = facetElementBodyIn.area;
            this->facetR_CopB_BList[idx] = cArray2EigenVector3d(facetElementBodyIn.r_CopB_B);
            this->facetNHat_BList[idx] = cArray2EigenVector3d(facetElementBodyIn.nHat_B);
            this->facetDiffuseCoeffList[idx] = facetElementBodyIn.c_diffuse;
            this->facetSpecularCoeffList[idx] = facetElementBodyIn.c_specular;

            facetProjectedAreaIn = this->facetProjectedAreaInMsgs[idx]();
            this->facetProjectedAreaList[idx] = facetProjectedAreaIn.area;
        }
    }

    // Calculate the SRP pressure acting at the current spacecraft location
    double numAU = AstU / r_SB_B.norm();
    double SRPPressure = (solarRadFlux / speedLight) * numAU * numAU;  // [Pa]

    // Compute the per-facet and total SRP force and torque
    Eigen::Vector3d facetSRPForcePntB_B = Eigen::Vector3d::Zero();
    Eigen::Vector3d facetSRPTorquePntB_B = Eigen::Vector3d::Zero();
    Eigen::Vector3d totalSRPForcePntB_B = Eigen::Vector3d::Zero();
    Eigen::Vector3d totalSRPTorquePntB_B = Eigen::Vector3d::Zero();

    for (uint64_t idx = 0; idx < this->numFacets; idx++) {
        if (this->facetProjectedAreaList[idx] > 0.0) {
            double cosTheta = facetProjectedAreaList[idx] / facetAreaList[idx];

            facetSRPForcePntB_B = -SRPPressure * this->facetProjectedAreaList[idx]
                                  * ((1 - this->facetSpecularCoeffList[idx])
                                  * sunDirHat_B + 2 * ((this->facetDiffuseCoeffList[idx] / 3)
                                  + this->facetSpecularCoeffList[idx] * cosTheta)
                                  * this->facetNHat_BList[idx]);  // [N]
            facetSRPTorquePntB_B = this->facetR_CopB_BList[idx].cross(facetSRPForcePntB_B);  // [Nm]

            // Add the facet contribution to the total SRP force and torque acting on the spacecraft
            totalSRPForcePntB_B += facetSRPForcePntB_B;
            totalSRPTorquePntB_B += facetSRPTorquePntB_B;
        }
    }

    // Update the force and torque vectors in the dynamic effector base class
    this->forceExternal_B = totalSRPForcePntB_B;
    this->torqueExternalPntB_B = totalSRPTorquePntB_B;
}

/*! Setter method for the total number of spacecraft facets.
 @param numFacets [-]
*/
void FacetedSRPEffector::setNumFacets(const uint64_t numFacets) {
    this->numFacets = numFacets;

    this->facetElementBodyInMsgs.clear();
    this->facetProjectedAreaInMsgs.clear();
    this->facetElementBodyInMsgs.reserve(this->numFacets);
    this->facetProjectedAreaInMsgs.reserve(this->numFacets);

    // Push back facet message vectors
    for (uint64_t idx = 0; idx < this->numFacets; ++idx) {
        this->facetElementBodyInMsgs.push_back(ReadFunctor<FacetElementBodyMsgPayload>{});
        this->facetProjectedAreaInMsgs.push_back(ReadFunctor<ProjectedAreaMsgPayload>());
    }
}

/*! Getter method for the total number of spacecraft facets.
 @return const uint64_t
*/
const uint64_t FacetedSRPEffector::getNumFacets() const { return this->numFacets; }
