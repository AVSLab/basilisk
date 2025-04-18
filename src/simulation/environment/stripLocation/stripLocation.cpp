/*
 ISC License

 Copyright (c) 2024, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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

#include "simulation/environment/stripLocation/stripLocation.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/linearAlgebra.h"
#include <iostream>

/*! @brief Creates an instance of the GroundLocation class with a minimum elevation of 10 degrees,
 @return void
 */
StripLocation::StripLocation()
{
    //! - Set some default initial conditions:
    
    this->minimumElevation = 10. *D2R; // [rad] Minimum elevation above each point of the central line of the strip; defaults to 10 degrees
    this->maximumRange = -1; // [m] Maximum range for the groundLocation to compute access; defaults no maximum range
    
    this->currentStripStateBuffer = this->currentStripStateOutMsg.zeroMsgPayload; // Initialize the currentGroundStateBuffer (NavTransMsgPayload structure)
    
    this->planetRadius = REQ_EARTH*1e3; // [m] Earth's equatorial radius
    this->planetState = this->planetInMsg.zeroMsgPayload; // Initialize the planetState
    this->planetState.J20002Pfix[0][0] = 1;
    this->planetState.J20002Pfix[1][1] = 1;
    this->planetState.J20002Pfix[2][2] = 1;
    this->r_North_N << 0, 0, 1; // Set the vector r_North_N to point along the z-axis
    
    this->r_LP_P_Start.fill(0.0); // [m] Ground location of the first point to image on the central line of the strip relative to PCPF
    
    this->r_LP_P_End.fill(0.0); // [m] Ground location of the last point to image on the central line of the strip relative to PCPF
    
    
    this->lenght_central_line=this->theta * this->planetRadius; // [m] Lenght of the central line
    
    this->r_LP_P = this->r_LP_P_Start; // [m] Ground location of the current target point on the central line of the strip relative to PCPF
    
    this->duration_strip_imaging = 0;  // Time already spent to image the strip
    this->OldSimNanos = 0; // Previous CurrentSimNanos
    
    this->acquisition_speed = 3*1e-6; // [m/ns] Constant acquisition speed of the camera; defaults 3 km/s
    this->pre_imaging_time = 0; // [ns] Pre_imaging time used to artificially modify r_LP_P_Start
    
}

/*! Empty destructor method.
 @return void
 */
StripLocation::~StripLocation()
{
    for (long unsigned int c=0; c<this->accessOutMsgs.size(); c++) {
        delete this->accessOutMsgs.at(c);
    }
    return;
}

/*! Resets the internal position to the specified initial position.*/
void StripLocation::Reset(uint64_t CurrentSimNanos)
{
    this->r_LP_P = this->r_LP_P_Start;
    
    if (this->planetRadius < 0) {
        bskLogger.bskLog(BSK_ERROR, "GroundLocation module must have planetRadius set.");
    }
}

/*! Specifies the ground location from planet-centered latitude, longitude, altitude position o
 *
 * @param lat
 * @param longitude
 * @param alt
 * @return
 */
void StripLocation::specifyLocationStart(double lat, double longitude, double alt)
{
    Eigen::Vector3d tmpLLAPosition(lat, longitude, alt);
    this->r_LP_P_Start = LLA2PCPF(tmpLLAPosition, this->planetRadius);  
}

void StripLocation::specifyLocationEnd(double lat, double longitude, double alt)
{
    Eigen::Vector3d tmpLLAPosition(lat, longitude, alt);
    this->r_LP_P_End = LLA2PCPF(tmpLLAPosition, this->planetRadius);
}

void StripLocation::newpstart()
{
    this->p_start = this->r_LP_P_Start.normalized() * this->planetRadius; //[m] Making sure the location of the first point is on the Earth surface for the interpolation
    this->p_end = this->r_LP_P_End.normalized() * this->planetRadius; //[m] Making sure the location of the last point is on the Earth surface for the interpolation
    this->theta = std::acos(p_start.dot(p_end) / (this->planetRadius * this->planetRadius));  // [rad]
    
    this->lenght_central_line=this->theta* this->planetRadius;
    double line_speed_ratio = this->lenght_central_line / this->acquisition_speed;
    double imaging_time = this->pre_imaging_time*-1;
    double t= imaging_time / line_speed_ratio;
    double sinTheta = std::sin(this->theta);
    double coeff1 = std::sin((1 - t) * this->theta) / sinTheta;
    double coeff2 = std::sin(t * this->theta) / sinTheta;
    
    this->r_LP_P_Start = coeff1 * this->p_start + coeff2 * this->p_end;
    
}

void StripLocation::lenght_line()
{
    this->p_start = this->r_LP_P_Start.normalized() * this->planetRadius; //[m] Making sure the location of the first point is on the Earth surface for the interpolation
    this->p_end = this->r_LP_P_End.normalized() * this->planetRadius; //[m] Making sure the location of the last point is on the Earth surface for the interpolation
    this->theta = std::acos(p_start.dot(p_end) / (this->planetRadius * this->planetRadius));  // [rad] Angle between p_start and p_end
    // Time already spent to image the strip
    this->lenght_central_line=this->theta* this->planetRadius;
    
}

/*! Adds a scState message name to the vector of names to be subscribed to. Also creates a corresponding access message output name.
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

/*! Read module messages
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
void StripLocation::WriteMessages(uint64_t CurrentClock)
{
    //! - write access message for each spacecraft
    for (long unsigned int c=0; c< this->accessMsgBuffer.size(); c++) {
        this->accessOutMsgs.at(c)->write(&this->accessMsgBuffer.at(c), this->moduleID, CurrentClock);
    }
    this->currentStripStateOutMsg.write(&this->currentStripStateBuffer, this->moduleID, CurrentClock);
}

/*! Inertial position and velocity of the target point on the strip 
*/
void StripLocation::updateInertialPosition()
{
    // First, get the rotation matrix from the inertial to planet frame from SPICE:
    this->dcm_PN = cArray2EigenMatrix3d(*this->planetState.J20002Pfix);
    this->dcm_PN_dot = cArray2EigenMatrix3d(*this->planetState.J20002Pfix_dot);
    this->r_PN_N = cArray2EigenVector3d(this->planetState.PositionVector);

    // Position of the target in the inertial frame 
    this->r_LP_N = this->dcm_PN.transpose() * this->r_LP_P;
    this->rhat_LP_N = this->r_LP_N/this->r_LP_N.norm();
    this->r_LN_N = this->r_PN_N + this->r_LP_N;

    // Transform the velocity to the inertial frame 
    this->v_LP_N = this->dcm_PN.transpose() * this->v_LP_P;
    
    // Get planet frame angular velocity vector
    Eigen::Matrix3d w_tilde_PN = - this->dcm_PN_dot * this->dcm_PN.transpose();
    this->w_PN << w_tilde_PN(2,1), w_tilde_PN(0,2), w_tilde_PN(1,0);

    //  Stash updated position in the groundState message
    eigenVector3d2CArray(this->r_LN_N, this->currentStripStateBuffer.r_LN_N);
    eigenVector3d2CArray(this->r_LP_N, this->currentStripStateBuffer.r_LP_N);

    //  Stash updated velocity in the groundState message
    eigenVector3d2CArray(this->v_LP_N, this->currentStripStateBuffer.v_LP_N);
}

/*! Interpolate the trajectory points on the earth surface to define the continuous central line of the strip. It is assumed that the Earth is spherical */
Eigen::Vector3d StripLocation::PositionCentralLine(double t) {
        
        // If points are too close (theta ~ 0), return p_start to avoid division by zero
        if (std::abs(this->theta) < 1e-6) {
            this->duration_strip_imaging = 0;
            return p_start;
        }

        // Perform spherical linear interpolation 
        double sinTheta = std::sin(this->theta);
        double coeff1 = std::sin((1 - t) * this->theta) / sinTheta;
        double coeff2 = std::sin(t * this->theta) / sinTheta;

        Eigen::Vector3d interpolatedPoint = coeff1 * this->p_start + coeff2 * this->p_end;
        
        // Return the interpolated point on the Earth's surface
        return interpolatedPoint.normalized() * this->planetRadius;
    }

Eigen::Vector3d StripLocation::VelocityCentralLine(double t) {
    if (std::abs(this->theta) < 1e-6) {
        return Eigen::Vector3d::Zero();
    }

    double sinTheta = std::sin(this->theta);
    //double cosTheta = std::cos(this->theta);
    double coeff1 = std::cos((1 - t) * this->theta) * (-this->theta) / sinTheta;
    double coeff2 = std::cos(t * this->theta) * this->theta / sinTheta;

    Eigen::Vector3d velocity = coeff1 * this->p_start + coeff2 * this->p_end;
    return velocity;
}

/*! Update the target point on the central line */
void StripLocation::updateTargetPositionPCPF(uint64_t CurrentClock)
    {
        this->p_start = this->r_LP_P_Start.normalized() * this->planetRadius; //[m] Making sure the location of the first point is on the Earth surface for the interpolation
        this->p_end = this->r_LP_P_End.normalized() * this->planetRadius; //[m] Making sure the location of the last point is on the Earth surface for the interpolation
        this->theta = std::acos(p_start.dot(p_end) / (this->planetRadius * this->planetRadius));  // [rad] Angle between p_start and p_end
        // Time already spent to image the strip
        this->lenght_central_line=this->theta* this->planetRadius;
        this->duration_strip_imaging = this->duration_strip_imaging + (CurrentClock-this->OldSimNanos);
        double imaging_time = static_cast<double>(this->duration_strip_imaging);
        double line_speed_ratio = this->lenght_central_line / this->acquisition_speed;

        //Update of the position vector of the target point on the central line 
        this->r_LP_P = this->StripLocation::PositionCentralLine(imaging_time / line_speed_ratio);
        /* Convert to LLA */
        Eigen::Vector3d tmpLLAPosition = PCPF2LLA(this->r_LP_P, this->planetRadius);
        /* Compute dcm_LP */
        this->dcm_LP = C_PCPF2SEZ(tmpLLAPosition[0], tmpLLAPosition[1]);

        // Calculate velocity
        this->v_LP_P = this->StripLocation::VelocityCentralLine(imaging_time / line_speed_ratio);

        }

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
        Eigen::Vector3d relativeHeading_N = r_BL_N / r_BL_mag;

        double viewAngle = (M_PI_2-safeAcos(this->rhat_LP_N.dot(relativeHeading_N)));

        accessMsgIt->slantRange = r_BL_mag;
        accessMsgIt->elevation = viewAngle;
        Eigen::Vector3d r_BL_L = this->dcm_LP * this->dcm_PN * r_BL_N;
        eigenVector3d2CArray(r_BL_L, accessMsgIt->r_BL_L);
        double cos_az = -r_BL_L[0]/(sqrt(pow(r_BL_L[0],2) + pow(r_BL_L[1],2)));
        double sin_az = r_BL_L[1]/(sqrt(pow(r_BL_L[0],2) + pow(r_BL_L[1],2)));
        accessMsgIt->azimuth = atan2(sin_az, cos_az);
        
        Eigen::Vector3d v_BL_L = this->dcm_LP * this->dcm_PN * (cArray2EigenVector3d(scStatesMsgIt->v_BN_N) - this->w_PN.cross(r_BP_N)); // V observed from gL wrt P frame, expressed in L frame coords (SEZ)
        eigenVector3d2CArray(v_BL_L, accessMsgIt->v_BL_L);
        accessMsgIt->range_dot = v_BL_L.dot(r_BL_L)/r_BL_mag;
        double xy_norm = sqrt(pow(r_BL_L[0],2)+pow(r_BL_L[1],2));
        accessMsgIt->az_dot = (-r_BL_L[0]*v_BL_L[1] + r_BL_L[1]*v_BL_L[0])/pow(xy_norm,2);
        accessMsgIt->el_dot = (v_BL_L[2]/xy_norm - r_BL_L[2]*(r_BL_L[0]*v_BL_L[0] + r_BL_L[1]*v_BL_L[1])/pow(xy_norm,3))/(1+pow(r_BL_L[2]/xy_norm,2));
        
        if( (viewAngle > this->minimumElevation) && (r_BL_mag <= this->maximumRange || this->maximumRange < 0)&& (this->duration_strip_imaging >= this->pre_imaging_time)){
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
 @param CurrentSimNanos
 */
void StripLocation::UpdateState(uint64_t CurrentSimNanos)
{   
    // Update the target point
    this->ReadMessages();
    this->updateTargetPositionPCPF(CurrentSimNanos);
    this->computeAccess();
    this->WriteMessages(CurrentSimNanos);
    this->OldSimNanos = CurrentSimNanos;

}

 
