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

#include "stripLocation.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include <algorithm>
#include <iostream>

/*! @brief create an instance of the Strip Location Class
 */
StripLocation::StripLocation()
{   //! - Set some default initial conditions:
    this->minimumElevation = 10.*D2R; // [rad] Minimum elevation above the target point; defaults to 10 degrees.
    this->maximumRange = -1; // [m] Maximum range with the target point; defaults no maximum range (-1).
    this->smallAngle = 1e-12; // [rad] Numerical tolerance when evaluating near-zero strip central angles.

    this->currentStripStateBuffer = this->currentStripStateOutMsg.zeroMsgPayload;

    this->planetRadius = REQ_EARTH*1e3;
    this->planetState = this->planetInMsg.zeroMsgPayload;
    this->planetState.J20002Pfix[0][0] = 1;
    this->planetState.J20002Pfix[1][1] = 1;
    this->planetState.J20002Pfix[2][2] = 1;
    this->r_North_N << 0, 0, 1;

    this->r_LP_P_Start.fill(0.0); // [m] Starting point of the strip relative to PCPF.
    this->r_LP_P_End.fill(0.0); // [m] Ending point of the strip relative to PCPF.

    this->OldSimNanos = 0; // [ns] Currrent time when the simulation starts, used to compute the time already spent to image the strip.

    this->acquisitionSpeed = 3*1e-6; // [m/ns] Required acquisition speed.

    this->preImagingTime = 0; // [ns] Pre-imaging time used to artificially modify r_LP_P_Start.
    this->isStartPositionUpdated = false; // [-] Flag indicating if the starting point has already been updated.
}

/*! empty destructor method.
 */
StripLocation::~StripLocation()
{
    for (long unsigned int c=0; c<this->accessOutMsgs.size(); c++) {
        delete this->accessOutMsgs.at(c);
    }
    return;
}

/*! reset the module.*/
void StripLocation::Reset(uint64_t currentSimNanos)
{
    this->isStartPositionUpdated = false;
    this->durationStripImaging = 0;
    this->OldSimNanos = currentSimNanos;

    if (this->smallAngle <= 0.0) {
        this->smallAngle = 1e-12;
        bskLogger.bskLog(BSK_WARNING, "StripLocation module smallAngle must be positive. Resetting to 1e-12 rad.");
    }

    if (this->planetRadius < 0) {
        bskLogger.bskLog(BSK_ERROR, "StripLocation module must have planetRadius set.");
    }
}

/*! specify the start location of the strip from planet-centered latitude, longitude, altitude position
 *
 * @param lat
 * @param longitude
 * @param alt
  */
void StripLocation::specifyLocationStart(double lat, double longitude, double alt)
{
    Eigen::Vector3d tmpLLAPosition(lat, longitude, alt);
    this->r_LP_P_Start = LLA2PCPF(tmpLLAPosition, this->planetRadius);
}

/*! specify the end location of the strip from planet-centered latitude, longitude, altitude position
 *
 * @param lat
 * @param longitude
 * @param alt
  */
void StripLocation::specifyLocationEnd(double lat, double longitude, double alt)
{
    Eigen::Vector3d tmpLLAPosition(lat, longitude, alt);
    this->r_LP_P_End = LLA2PCPF(tmpLLAPosition, this->planetRadius);
}

/*! update the starting point of the strip to take into account the specified pre-imaging time
 */
void StripLocation::newpstart()
{
    this->pStart = this->r_LP_P_Start.normalized() * this->planetRadius; //[m] Making sure the location of the starting point is on the Earth surface for the interpolation (Assuming Spherical Earth).
    this->pEnd = this->r_LP_P_End.normalized() * this->planetRadius; //[m] Making sure the location of the ending point is on the Earth surface for the interpolation (Assuming Spherical Earth).
    double dotVal = pStart.dot(pEnd) / (this->planetRadius * this->planetRadius);
    dotVal = std::max(-1.0, std::min(1.0, dotVal));
    this->theta = std::acos(dotVal);  // [rad]

    // If the acquisition speed is equal to zero, or pre-imaging time is non-positive, no pre-imaging time is applied.
    if (this->acquisitionSpeed <= 0.0 || this->preImagingTime <= 0) {
        this->pStartUpdated = this->pStart;
        this->isStartPositionUpdated = true;
        return;
    }

    this->lengthCentralLine = this->theta * this->planetRadius; // [m] Length of the central line of the strip.
    double line_speed_ratio = this->lengthCentralLine / this->acquisitionSpeed;
    double t = this->preImagingTime * -1 / line_speed_ratio;

    this->pStartUpdated = this->PositionCentralLine(t, this->theta, this->pStart, this->pEnd); //[m] Updated starting point of the strip relative to PCPF, taking into account the pre-imaging time (Spherical Extrapolation).
    this->isStartPositionUpdated = true;
}

/*! add a scState message name to the vector of names to be subscribed to (also creates a corresponding access message output name)
*/
void StripLocation::addSpacecraftToModel(Message<SCStatesMsgPayload> *tmpScMsg)
{
    this->scStateInMsgs.push_back(tmpScMsg->addSubscriber());

    /* create output message */
    Message<AccessMsgPayload> *msg;
    msg = new Message<AccessMsgPayload>;
    this->accessOutMsgs.push_back(msg);

    /* expand the buffer vector */
    AccessMsgPayload accMsg;
    this->accessMsgBuffer.push_back(accMsg);
}

/*! read module messages
*/
bool StripLocation::ReadMessages()
{
    SCStatesMsgPayload scMsg;

    /* clear out the vector of spacecraft states.  This is created freshly below. */
    this->scStatesBuffer.clear();

    //! - read in the spacecraft state messages
    bool scRead;
    if(!this->scStateInMsgs.empty())
    {
        scRead = true;
        for (long unsigned int c = 0; c < this->scStateInMsgs.size(); c++) {
            scMsg = this->scStateInMsgs.at(c)();
            scRead = scRead && this->scStateInMsgs.at(c).isWritten();
            this->scStatesBuffer.push_back(scMsg);
        }
    } else {
        bskLogger.bskLog(BSK_ERROR, "Ground location has no spacecraft to track.");
        scRead = false;
    }
    //! - Read in the optional planet message.  if no planet message is set, then a zero planet position, velocity and orientation is assumed
    bool planetRead = true;
    if(this->planetInMsg.isLinked())
    {
        planetRead = this->planetInMsg.isWritten();
        this->planetState = this->planetInMsg();
    }

    return(planetRead && scRead);
}

/*! write module messages
*/
void StripLocation::WriteMessages(uint64_t currentClock)
{
    //! - write access message for each spacecraft
    for (long unsigned int c=0; c< this->accessMsgBuffer.size(); c++) {
        this->accessOutMsgs.at(c)->write(&this->accessMsgBuffer.at(c), this->moduleID, currentClock);
    }
    this->currentStripStateOutMsg.write(&this->currentStripStateBuffer, this->moduleID, currentClock);
}

/*! compute inertial position and velocity of the target point on the strip
*/
void StripLocation::updateInertialPosition()
{
    // Rotation matrix from the inertial to planet frame from SPICE:
    this->dcm_PN = cArray2EigenMatrix3d(*this->planetState.J20002Pfix);
    this->dcm_PN_dot = cArray2EigenMatrix3d(*this->planetState.J20002Pfix_dot);
    this->r_PN_N = cArray2EigenVector3d(this->planetState.PositionVector);

    // Position of the target in the inertial frame
    this->r_LP_N = this->dcm_PN.transpose() * this->r_LP_P;
    this->rhat_LP_N = this->r_LP_N/this->r_LP_N.norm();
    this->r_LN_N = this->r_PN_N + this->r_LP_N;

    // Get planet frame angular velocity vector
    Eigen::Matrix3d w_tilde_PN = - this->dcm_PN_dot * this->dcm_PN.transpose();
    this->w_PN << w_tilde_PN(2,1), w_tilde_PN(0,2), w_tilde_PN(1,0);

    // Transform the velocity to the inertial frame
    // Include planet rotation to obtain inertial target-track velocity.
    this->v_LP_N = this->dcm_PN.transpose() * (this->v_LP_P + this->w_PN.cross(this->r_LP_P));

    // Compute full inertial velocity: v_LN_N = v_PN_N + v_LP_N
    Eigen::Vector3d v_PN_N = cArray2EigenVector3d(this->planetState.VelocityVector);
    this->v_LN_N = v_PN_N + this->v_LP_N;

    //  Stash updated position in the StripState message
    eigenVector3d2CArray(this->r_LN_N, this->currentStripStateBuffer.r_LN_N);
    eigenVector3d2CArray(this->r_LP_N, this->currentStripStateBuffer.r_LP_N);

    //  Stash updated velocity in the StripState message
    eigenVector3d2CArray(this->v_LP_N, this->currentStripStateBuffer.v_LP_N);
    eigenVector3d2CArray(this->v_LN_N, this->currentStripStateBuffer.v_LN_N);
}

/*! return target point on the central line of the strip relative to PCPF */
Eigen::Vector3d StripLocation::PositionCentralLine(double t, double teta, const Eigen::Vector3d& pStart, const Eigen::Vector3d& pEnd) const {
    // If points are too close (theta ~ 0), return pStart to avoid division by zero
    if (std::abs(teta) < this->smallAngle) {
        return pStart;
    }
    // Perform spherical linear interpolation
    double sinTheta = std::sin(teta);
    double coeff1 = std::sin((1 - t) * teta) / sinTheta;
    double coeff2 = std::sin(t * teta) / sinTheta;
    Eigen::Vector3d interpolatedPoint = coeff1 * pStart + coeff2 * pEnd;

    // Return the interpolated point on the Earth's surface
    return interpolatedPoint.normalized() * this->planetRadius;
}

/*! return the tangent velocity vector to the central line path of the target point relative to PCPF */
Eigen::Vector3d StripLocation::VelocityCentralLine(double t, double teta, const Eigen::Vector3d& pStart, const Eigen::Vector3d& pEnd) const {
    // If points are too close (theta ~ 0), return zero velocity to avoid division by zero
    if (std::abs(teta) < this->smallAngle) {
        return Eigen::Vector3d::Zero();
    }

    double sinTheta = std::sin(teta);
    double coeff1 = std::cos((1 - t) * teta) * (-teta) / sinTheta;
    double coeff2 = std::cos(t * teta) * teta / sinTheta;

    Eigen::Vector3d velocity = coeff1 * pStart + coeff2 * pEnd;
    double velocityNorm = velocity.norm(); // Not zero thanks to the check of teta
    return velocity / velocityNorm * (this->acquisitionSpeed * 1e9);
}

/*! update the target point on the central line in PCPF */
void StripLocation::updateTargetPositionPCPF(uint64_t currentClock)
    {
        if (!this->isStartPositionUpdated) {
            this->newpstart();
        }

        double dotVal = pStartUpdated.dot(pEnd) / (this->planetRadius * this->planetRadius);
        dotVal = std::max(-1.0, std::min(1.0, dotVal));
        this->thetaUpdated = std::acos(dotVal);  // [rad] Angle between pStartUpdated and pEnd

        // Time already spent to image the strip
        this->lengthCentralLineUpdated = this->thetaUpdated * this->planetRadius;
        this->durationStripImaging = this->durationStripImaging + (currentClock-this->OldSimNanos);

        if (this->acquisitionSpeed <= 0.0) {
            this->r_LP_P = this->pStartUpdated;
            this->v_LP_P.setZero();
            Eigen::Vector3d tmpLLAPosition = PCPF2LLA(this->r_LP_P, this->planetRadius);
            this->dcm_LP = C_PCPF2SEZ(tmpLLAPosition[0], tmpLLAPosition[1]);
            return;
        }

        double line_speed_ratio = this->lengthCentralLineUpdated / this->acquisitionSpeed;
        double tau = this->durationStripImaging / line_speed_ratio;
        double tauClamped = std::max(0.0, std::min(1.0, tau));

        //Update of the position vector of the target point on the central line an update the dcm_LP
        this->r_LP_P = this->PositionCentralLine(tauClamped, this->thetaUpdated, this->pStartUpdated, this->pEnd);
        Eigen::Vector3d tmpLLAPosition = PCPF2LLA(this->r_LP_P, this->planetRadius);
        this->dcm_LP = C_PCPF2SEZ(tmpLLAPosition[0], tmpLLAPosition[1]);

        // Update the velocity vector
        this->v_LP_P = this->VelocityCentralLine(tauClamped, this->thetaUpdated, this->pStartUpdated, this->pEnd);

        }

/*! compute access to the target point on the central line */
void StripLocation::computeAccess()
{
    // Update the groundLocation's inertial position
    this->updateInertialPosition();

    // Iterate over spacecraft position messages and compute the access for each one
    std::vector<AccessMsgPayload>::iterator accessMsgIt;
    std::vector<SCStatesMsgPayload>::iterator scStatesMsgIt;
    for(scStatesMsgIt = this->scStatesBuffer.begin(), accessMsgIt = accessMsgBuffer.begin(); scStatesMsgIt != scStatesBuffer.end(); scStatesMsgIt++, accessMsgIt++){
        //! Compute the relative position of each spacecraft to the site in the planet-centered inertial frame
        Eigen::Vector3d r_BP_N = cArray2EigenVector3d(scStatesMsgIt->r_BN_N) - this->r_PN_N;
        Eigen::Vector3d r_BL_N = r_BP_N - this->r_LP_N;
        auto r_BL_mag = r_BL_N.norm();
        if (r_BL_mag < 1e-12) {
            accessMsgIt->slantRange = 0.0;
            accessMsgIt->elevation = -M_PI_2;
            accessMsgIt->azimuth = 0.0;
            accessMsgIt->range_dot = 0.0;
            accessMsgIt->az_dot = 0.0;
            accessMsgIt->el_dot = 0.0;
            accessMsgIt->hasAccess = 0;
            Eigen::Vector3d zeroVec = Eigen::Vector3d::Zero();
            eigenVector3d2CArray(zeroVec, accessMsgIt->r_BL_L);
            eigenVector3d2CArray(zeroVec, accessMsgIt->v_BL_L);
            continue;
        }
        Eigen::Vector3d relativeHeading_N = r_BL_N / r_BL_mag;

        double viewAngle = (M_PI_2-safeAcos(this->rhat_LP_N.dot(relativeHeading_N)));

        accessMsgIt->slantRange = r_BL_mag;
        accessMsgIt->elevation = viewAngle;
        Eigen::Vector3d r_BL_L = this->dcm_LP * this->dcm_PN * r_BL_N;
        eigenVector3d2CArray(r_BL_L, accessMsgIt->r_BL_L);
        Eigen::Vector3d v_PN_N = cArray2EigenVector3d(this->planetState.VelocityVector);
        Eigen::Vector3d v_BP_P = this->dcm_PN * (cArray2EigenVector3d(scStatesMsgIt->v_BN_N)- v_PN_N - this->w_PN.cross(r_BP_N));
        Eigen::Vector3d v_BL_L = this->dcm_LP * (v_BP_P - this->v_LP_P);
        accessMsgIt->range_dot = v_BL_L.dot(r_BL_L)/r_BL_mag;
        double xy_norm = sqrt(pow(r_BL_L[0],2)+pow(r_BL_L[1],2));
        if (xy_norm < 1e-12) {
            accessMsgIt->azimuth = 0.0;
            accessMsgIt->az_dot = 0.0;
            accessMsgIt->el_dot = 0.0;
        } else {
            double cos_az = -r_BL_L[0] / xy_norm;
            double sin_az = r_BL_L[1] / xy_norm;
            accessMsgIt->azimuth = atan2(sin_az, cos_az);
            accessMsgIt->az_dot = (-r_BL_L[0]*v_BL_L[1] + r_BL_L[1]*v_BL_L[0])/pow(xy_norm,2);
            accessMsgIt->el_dot = (v_BL_L[2]/xy_norm - r_BL_L[2]*(r_BL_L[0]*v_BL_L[0] + r_BL_L[1]*v_BL_L[1])/pow(xy_norm,3))/(1+pow(r_BL_L[2]/xy_norm,2));
        }

        if( (viewAngle > this->minimumElevation) && (r_BL_mag <= this->maximumRange || this->maximumRange < 0)&& (this->durationStripImaging >= this->preImagingTime)){
            accessMsgIt->hasAccess = 1;
        }
        else
        {
            accessMsgIt->hasAccess = 0;
        }
    }
}

/*!
 update module
 @param currentSimNanos
 */
void StripLocation::UpdateState(uint64_t currentSimNanos)
{
    // Update the target point
    this->ReadMessages();
    this->updateTargetPositionPCPF(currentSimNanos);
    this->computeAccess();
    this->WriteMessages(currentSimNanos);
    this->OldSimNanos = currentSimNanos;

}
