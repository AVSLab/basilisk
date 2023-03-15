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
#include <cmath>

const double speedLight = 299792458.0; //  [m/s] Speed of light
const double AstU = 149597870700.0; // [m] Astronomical unit
const double solarRadFlux = 1368.0; // [W/m^2] Solar radiation flux at 1 AU

/*! The constructor initializes the member variables to zero. */
FacetSRPDynamicEffector::FacetSRPDynamicEffector()
{
    this->forceExternal_B.fill(0.0);
    this->torqueExternalPntB_B.fill(0.0);
    this->numFacets = 0;
}

/*! The destructor. */
FacetSRPDynamicEffector::~FacetSRPDynamicEffector()
{
}

/*! The reset member function. This method checks to ensure the input message is linked.
 @return void
 @param currentSimNanos [ns]  Time the method is called
*/
void FacetSRPDynamicEffector::Reset(uint64_t currentSimNanos)
{
    if (!this->sunInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "FacetSRPDynamicEffector.sunInMsg was not linked.");
    }
}

/*! The SRP dynamic effector does not write any output messages.
 @return void
 @param currentClock [ns] Time the method is called
*/
void FacetSRPDynamicEffector::writeOutputMessages(uint64_t currentClock)
{
}

/*! This member function populates the spacecraft geometry structure with user-input facet information.
 @return void
 @param area  [m^2] Facet area
 @param specCoeff  Facet spectral reflection optical coefficient
 @param diffCoeff  Facet diffuse reflection optical coefficient
 @param normal_B  Facet normal expressed in B frame components
 @param locationPntB_B  [m] Facet location wrt point B in B frame components
*/
void FacetSRPDynamicEffector::addFacet(double area,
                                       double specCoeff,
                                       double diffCoeff,
                                       Eigen::Vector3d normal_B,
                                       Eigen::Vector3d locationPntB_B)
{
    this->scGeometry.facetAreas.push_back(area);
    this->scGeometry.facetSpecCoeffs.push_back(specCoeff);
    this->scGeometry.facetDiffCoeffs.push_back(diffCoeff);
    this->scGeometry.facetNormals_B.push_back(normal_B);
    this->scGeometry.facetLocationsPntB_B.push_back(locationPntB_B);
    this->numFacets = this->numFacets + 1;
}

/*! This method is used to link the faceted SRP effector to the hub attitude and position,
which are required for calculating SRP forces and torques.
 @return void
 @param states  Dynamic parameter states
*/
void FacetSRPDynamicEffector::linkInStates(DynParamManager& states)
{
    this->hubSigma = states.getStateObject("hubSigma");
    this->hubPosition = states.getStateObject("hubPosition");
}

/*! This method computes the body forces and torques for the SRP effector.
 @return void
 @param integTime  [s] Time the method is called
 @param timeStep  [s] Simulation time step
*/
void FacetSRPDynamicEffector::computeForceTorque(double integTime, double timeStep)
{
    // Read the input message
    SpicePlanetStateMsgPayload sunMsgBuffer;
    sunMsgBuffer = sunInMsg.zeroMsgPayload;
    sunMsgBuffer = this->sunInMsg();

    // Calculate the Sun position with respect to the inertial frame, expressed in inertial frame components
    this->r_SN_N = cArray2EigenVector3d(sunMsgBuffer.PositionVector);

    // Compute dcm_BN using MRP transformation
    Eigen::MRPd sigmaBN;
    sigmaBN = (Eigen::Vector3d)this->hubSigma->getState();
    Eigen::Matrix3d dcm_BN = sigmaBN.toRotationMatrix().transpose();

    // Store the hub B frame position with respect to the inertial frame
    Eigen::Vector3d r_BN_N = this->hubPosition->getState();

    // Calculate the vector pointing from point B on the spacecraft to the Sun
    Eigen::Vector3d r_SB_B = dcm_BN * (this->r_SN_N - r_BN_N);

    // Calculate the unit vector pointing from point B on the spacecraft to the Sun
    Eigen::Vector3d sHat = r_SB_B / r_SB_B.norm();

    // Define local vectors for the facet force and torque storage
    Eigen::Vector3d facetSRPForcePntB_B;
    Eigen::Vector3d facetSRPTorquePntB_B;
    Eigen::Vector3d totalSRPForcePntB_B;
    Eigen::Vector3d totalSRPTorquePntB_B;

    // Zero storage information
    double projectedArea = 0.0;
    double projectionTerm = 0.0;
    facetSRPForcePntB_B.setZero();
    facetSRPTorquePntB_B.setZero();
    totalSRPForcePntB_B.setZero();
    totalSRPTorquePntB_B.setZero();
    this->forceExternal_B.setZero();
    this->torqueExternalPntB_B.setZero();
    
    // Calculate the SRP pressure acting on point B
    double numAU = AstU / r_SB_B.norm();
    double SRPPressure = (solarRadFlux / speedLight) * numAU * numAU;

    // Loop through the facets and calculate the SRP force and torque acting on point B
    for(int i = 0; i < this->numFacets; i++)
    {
        projectionTerm = this->scGeometry.facetNormals_B[i].dot(sHat);
        projectedArea = this->scGeometry.facetAreas[i] * projectionTerm;

        if(projectedArea > 0.0){

            // Calculate the incidence angle theta between the facet normal vector and the Sun-direction vector
            double cosTheta = projectionTerm;
            Eigen::Vector3d intermediate = sHat.cross(this->scGeometry.facetNormals_B[i]);
            double sinTheta = intermediate.norm();
            double theta = atan2(sinTheta, cosTheta);

            // Compute the SRP force acting on the ith facet
            facetSRPForcePntB_B = -SRPPressure * projectedArea * cos(theta)
                                  * ( (1-this->scGeometry.facetSpecCoeffs[i])
                                  * sHat + 2 * ( (this->scGeometry.facetDiffCoeffs[i] / 3)
                                  + this->scGeometry.facetSpecCoeffs[i] * cos(theta))
                                  * this->scGeometry.facetNormals_B[i] );

            // Compute the SRP torque acting on the ith facet
            facetSRPTorquePntB_B = this->scGeometry.facetLocationsPntB_B[i].cross(facetSRPForcePntB_B);

            // Compute the total SRP force and torque acting on the spacecraft
            totalSRPForcePntB_B = totalSRPForcePntB_B + facetSRPForcePntB_B;
            totalSRPTorquePntB_B = totalSRPTorquePntB_B + facetSRPTorquePntB_B;
        }
    }

    // Write the total SRP force and torque local variables to the dynamic effector variables
    this->forceExternal_B = totalSRPForcePntB_B;
    this->torqueExternalPntB_B = totalSRPTorquePntB_B;
}

/*! This is the UpdateState() method
 @return void
 @param currentSimNanos [ns] Time the method is called
*/
void FacetSRPDynamicEffector::UpdateState(uint64_t currentSimNanos)
{
}
